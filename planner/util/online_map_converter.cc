#include "onboard/planner/util/online_map_converter.h"

#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <queue>
#include <utility>
#include <vector>

#include "onboard/lite/check.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/types/span.h"

#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/online_map_constants.h"
#include "onboard/maps/online_map_data_types.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

constexpr int kMaxNearbyLanesGetNum = 3;
const double kLMaxNearbyLanesGetRadius =
    kMaxNearbyLanesGetNum * mapping::kMaxLaneWidth * 0.5;
constexpr double kEpsilon = 1e-2;

absl::Status CheckOptionValidity(
    const OnlineSemanticMapConverterOption& option) {
  if (option.lane_sample_interval < kEpsilon ||
      option.boundary_sample_interval < kEpsilon) {
    return absl::InvalidArgumentError(
        "Lane or boundary sample interval are invalid.");
  }
  return absl::OkStatus();
}

absl::StatusOr<const mapping::LaneInfo*> FindNearestLaneInfo(
    absl::Span<const mapping::LaneInfo* const> lane_infos, const Vec2d& pos) {
  const mapping::LaneInfo* nearest_lane_info = nullptr;
  double distance = std::numeric_limits<double>::max();

  for (const auto* lane_info : lane_infos) {
    const auto sl = lane_info->SmoothXYToSL(pos);
    if (sl.s < 0.0 || sl.s > lane_info->length()) continue;

    if (std::fabs(sl.l) < distance) {
      distance = std::fabs(sl.l);
      nearest_lane_info = lane_info;
    }
  }

  if (nearest_lane_info == nullptr) {
    return absl::NotFoundError("Can not found nearest lane info.");
  }

  return nearest_lane_info;
}

std::vector<const mapping::LaneInfo*> FindLaneInfoByRange(
    absl::Span<const mapping::LaneInfo* const> lane_infos, const Vec2d& pos,
    const double perception_lateral_range) {
  std::vector<const mapping::LaneInfo*> lane_info_vec;
  lane_info_vec.reserve(std::size(lane_infos));

  for (const auto* lane_info : lane_infos) {
    if (lane_info->is_in_intersection) continue;
    const auto sl = lane_info->SmoothXYToSL(pos);
    if (sl.s < 0.0 || sl.s > lane_info->length()) continue;

    if (std::fabs(sl.l) < perception_lateral_range) {
      lane_info_vec.push_back(lane_info);
    }
  }
  return lane_info_vec;
}

template <typename T>
std::vector<double> CalculateCumulativeLengths(const T& points) {
  const auto calculate_smooth_point_distance = [](const auto& p1,
                                                  const auto& p2) -> double {
    const double dx = p1.x() - p2.x();
    const double dy = p1.y() - p2.y();
    return Hypot(dx, dy);
  };

  std::vector<double> cumulative_lengths(points.size());
  for (int i = 1; i < points.size(); ++i) {
    cumulative_lengths[i] =
        cumulative_lengths[i - 1] +
        calculate_smooth_point_distance(points[i], points[i - 1]);
  }
  return cumulative_lengths;
}

std::vector<Vec2d> ResamplePoints(const std::vector<Vec2d>& points,
                                  const std::vector<double>& cumulative_lengths,
                                  double resample_step, double start_fraction,
                                  double end_fraction) {
  QCHECK_EQ(points.size(), cumulative_lengths.size());
  const int resample_size =
      CeilToInt(cumulative_lengths.back() * (end_fraction - start_fraction) /
                resample_step) +
      1;
  std::vector<Vec2d> resample_points;
  resample_points.reserve(resample_size);
  PiecewiseLinearFunction<Vec2d> plf(cumulative_lengths, points);

  const double start_s = cumulative_lengths.back() * start_fraction;
  for (int i = 0; i < resample_size; ++i) {
    resample_points.push_back(plf(start_s + i * resample_step));
  }
  return resample_points;
}

mapping::OnlineLaneProto ToOnlineLaneProto(const mapping::LaneInfo& lane_info,
                                           double start_fraction,
                                           double end_fraction,
                                           double lane_sample_interval) {
  QCHECK_LE(start_fraction, end_fraction);
  mapping::OnlineLaneProto online_lane_proto;
  online_lane_proto.set_id(lane_info.id.value());

  // Resample lane points.
  const int size =
      CeilToInt(lane_info.length() * (end_fraction - start_fraction) /
                lane_sample_interval) +
      1;
  online_lane_proto.mutable_smooth_points()->Reserve(size);

  double start_s = lane_info.length() * start_fraction;
  for (int i = 0; i < size; ++i) {
    const double fraction = std::clamp(
        (start_s + i * lane_sample_interval) / lane_info.length(), 0.0, 1.0);
    const auto lerp_pt = lane_info.LerpPointFromFraction(fraction);
    auto* online_lane_pt = online_lane_proto.add_smooth_points();
    online_lane_pt->set_x(lerp_pt.x());
    online_lane_pt->set_y(lerp_pt.y());
  }

  // Add incoming lane ids.
  for (const auto id : lane_info.incoming_lanes()) {
    online_lane_proto.add_incoming_lane_ids(id.value());
  }

  // Add outgoing lane ids.
  for (const auto id : lane_info.outgoing_lanes()) {
    online_lane_proto.add_outgoing_lane_ids(id.value());
  }

  // Add left neighbor lane and boundary.
  if (!lane_info.lane_neighbors_on_left.empty()) {
    const auto& neighbor_info = lane_info.lane_neighbors_on_left.front();
    online_lane_proto.set_left_lane_id(neighbor_info.other_id.value());
    if (neighbor_info.lane_boundary_id != mapping::kInvalidElementId) {
      online_lane_proto.set_left_boundary_id(
          neighbor_info.lane_boundary_id.value());
    }
  }

  // Add right neighbor lane and boundary.
  if (!lane_info.lane_neighbors_on_right.empty()) {
    const auto& neighbor_info = lane_info.lane_neighbors_on_right.front();
    online_lane_proto.set_right_lane_id(neighbor_info.other_id.value());
    if (neighbor_info.lane_boundary_id != mapping::kInvalidElementId) {
      online_lane_proto.set_right_boundary_id(
          neighbor_info.lane_boundary_id.value());
    }
  }
  return online_lane_proto;
}

mapping::OnlineLaneBoundaryProto ToOnlineLaneBoundaryProto(
    const PlannerSemanticMapManager& psmm, mapping::ElementId boundary_id,
    double start_fraction, double end_fraction,
    double boundary_sample_interval) {
  QCHECK_LE(start_fraction, end_fraction);
  mapping::OnlineLaneBoundaryProto online_boundary;
  online_boundary.set_id(boundary_id.value());

  // Resample lane boundary points.
  const auto* boundary_info = psmm.FindLaneBoundaryByIdOrNull(boundary_id);
  if (boundary_info == nullptr) {
    QLOG(ERROR) << "Can not find lane boundary info, id: " << boundary_id;
    return online_boundary;
  }
  const auto& points_smooth = boundary_info->points_smooth;

  const auto lane_boundary_points = ResamplePoints(
      points_smooth, CalculateCumulativeLengths(points_smooth),
      /*resample_step=*/boundary_sample_interval, start_fraction, end_fraction);

  online_boundary.mutable_points()->Reserve(lane_boundary_points.size());
  for (const auto& pt : lane_boundary_points) {
    auto* boundary_pt = online_boundary.add_points();
    boundary_pt->set_type(boundary_info->type);
    boundary_pt->mutable_smooth_point()->set_x(pt.x());
    boundary_pt->mutable_smooth_point()->set_y(pt.y());
  }

  return online_boundary;
}

const mapping::OnlineLaneProto* AddLaneAndBoundariesToOnlineMap(
    const PlannerSemanticMapManager& psmm, const mapping::LaneInfo& lane_info,
    double start_fraction, double end_fraction, double lane_sample_interval,
    double boundary_sample_interval,
    mapping::OnlineSemanticMapProto* online_map_proto,
    absl::flat_hash_set<mapping::ElementId>* visited_boundary_ids) {
  auto* online_lane = online_map_proto->add_lanes();
  *online_lane = ToOnlineLaneProto(lane_info, start_fraction, end_fraction,
                                   lane_sample_interval);

  // Add left boundary to online map, if exist and has not been visited.
  if (online_lane->has_left_boundary_id()) {
    const auto left_boundary_id =
        mapping::ElementId(online_lane->left_boundary_id());
    if (visited_boundary_ids->insert(left_boundary_id).second) {
      *online_map_proto->add_boundaries() =
          ToOnlineLaneBoundaryProto(psmm, left_boundary_id, start_fraction,
                                    end_fraction, boundary_sample_interval);
    }
  }

  // Add right boundary to online map, if exist and has not been visited.
  if (online_lane->has_right_boundary_id()) {
    const auto right_boundary_id =
        mapping::ElementId(online_lane->right_boundary_id());
    if (visited_boundary_ids->insert(right_boundary_id).second) {
      *online_map_proto->add_boundaries() =
          ToOnlineLaneBoundaryProto(psmm, right_boundary_id, start_fraction,
                                    end_fraction, boundary_sample_interval);
    }
  }
  return online_lane;
}

struct ExtendOnlineMapInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const mapping::LaneInfo* start_lane_info = nullptr;
  double extend_length = 0.0;
  bool is_forward_extend = true;
  double lane_sample_interval = 1.0;      // meters
  double boundary_sample_interval = 1.0;  // meters
};

void ExtendOnlineMap(
    const ExtendOnlineMapInput& input,
    mapping::OnlineSemanticMapProto* online_map_proto,
    absl::flat_hash_set<mapping::ElementId>* visited_lane_ids,
    absl::flat_hash_set<mapping::ElementId>* visited_boundary_ids) {
  const auto& psmm = *input.psmm;
  const auto& start_lane_info = *input.start_lane_info;
  const bool is_forward_extend = input.is_forward_extend;

  struct SearchData {
    mapping::ElementId lane_id;
    double extend_length;
  };
  std::queue<SearchData> search_queue;

  if (input.extend_length > 0.0) {
    const auto search_lane_ids = is_forward_extend
                                     ? start_lane_info.outgoing_lanes()
                                     : start_lane_info.incoming_lanes();
    for (const auto id : search_lane_ids) {
      search_queue.push(SearchData{id, input.extend_length});
    }
  }

  while (!search_queue.empty()) {
    const auto data = search_queue.front();
    search_queue.pop();

    if (!visited_lane_ids->insert(data.lane_id).second) continue;

    const auto* lane_info = psmm.FindLaneInfoOrNull(data.lane_id);
    if (lane_info == nullptr) continue;

    const double start_fraction =
        is_forward_extend
            ? 0.0
            : std::max(0.0, (lane_info->length() - data.extend_length) /
                                lane_info->length());
    const double end_fraction =
        is_forward_extend
            ? std::min(1.0, data.extend_length / lane_info->length())
            : 1.0;

    const auto* online_lane = AddLaneAndBoundariesToOnlineMap(
        psmm, *lane_info, start_fraction, end_fraction,
        input.lane_sample_interval, input.boundary_sample_interval,
        online_map_proto, visited_boundary_ids);

    const double rest_extend_length = data.extend_length - lane_info->length();
    if (rest_extend_length > 0.0) {
      const auto& search_lane_ids = is_forward_extend
                                        ? online_lane->outgoing_lane_ids()
                                        : online_lane->incoming_lane_ids();
      for (const auto id : search_lane_ids) {
        search_queue.push(
            SearchData{mapping::ElementId(id), rest_extend_length});
      }
    }
  }
}

void ExtendOnlineMapFromLane(
    const PlannerSemanticMapManager& psmm, const mapping::LaneInfo& lane_info,
    const Vec2d& pos, double look_ahead_distance, double look_back_distance,
    double lane_sample_interval, double boundary_sample_interval,
    mapping::OnlineSemanticMapProto* online_map_proto,
    absl::flat_hash_set<mapping::ElementId>* visited_lane_ids,
    absl::flat_hash_set<mapping::ElementId>* visited_boundary_ids) {
  const double start_s_on_lane = lane_info.SmoothXYToSL(pos).s;
  const double start_fraction = std::max(
      0.0, (start_s_on_lane - look_back_distance) / lane_info.length());
  const double end_fraction = std::min(
      1.0, (start_s_on_lane + look_ahead_distance) / lane_info.length());

  // Return if lane id is repeated.
  if (const auto iter = std::find_if(online_map_proto->lanes().begin(),
                                     online_map_proto->lanes().end(),
                                     [&lane_info](const auto& lane) {
                                       return lane.id() == lane_info.id.value();
                                     });
      iter != online_map_proto->lanes().end()) {
    return;
  }

  // Add lane and boundaries to online map.
  const auto* online_lane = AddLaneAndBoundariesToOnlineMap(
      psmm, lane_info, start_fraction, end_fraction, lane_sample_interval,
      boundary_sample_interval, online_map_proto, visited_boundary_ids);

  online_map_proto->add_lane_ids_at_ego_pos(online_lane->id());

  const double start_s_on_online_lane =
      std::min(start_s_on_lane, look_back_distance);
  online_map_proto->add_point_index_of_lane_at_ego_pos(
      mapping::FindNearestPointIndexAtS(
          CalculateCumulativeLengths(online_lane->smooth_points()),
          start_s_on_online_lane));

  // Forward extend online map.
  const double forward_extend_length =
      start_s_on_lane + look_ahead_distance - lane_info.length();
  ExtendOnlineMap(
      ExtendOnlineMapInput{&psmm, &lane_info, forward_extend_length,
                           /*is_forward_extend=*/true, lane_sample_interval,
                           boundary_sample_interval},
      online_map_proto, visited_lane_ids, visited_boundary_ids);

  // Backward extend online map.
  const double backward_extend_length = look_back_distance - start_s_on_lane;
  ExtendOnlineMap(
      ExtendOnlineMapInput{&psmm, &lane_info, backward_extend_length,
                           /*is_forward_extend=*/false, lane_sample_interval,
                           boundary_sample_interval},
      online_map_proto, visited_lane_ids, visited_boundary_ids);
}
}  // namespace

absl::StatusOr<mapping::OnlineSemanticMapProto> RunOnlineSemanticMapConverter(
    const PlannerSemanticMapManager& psmm,
    const OnlineSemanticMapConverterOption& option) {
  RETURN_IF_ERROR(CheckOptionValidity(option));

  const Vec2d smooth_pos(option.smooth_x, option.smooth_y);
  const auto& coordinate_converter = psmm.coordinate_converter();
  const auto level_id = coordinate_converter.GetLevel();
  const double global_yaw =
      coordinate_converter.SmoothYawToGlobal(option.smooth_yaw);

  ASSIGN_OR_RETURN(const auto nearest_lane,
                   FindNearestLaneInfo(psmm.GetLanesInfoWithHeadingAtLevel(
                                           level_id, smooth_pos, global_yaw,
                                           kLMaxNearbyLanesGetRadius,
                                           /*max_heading_diff=*/M_PI_4),
                                       smooth_pos));

  mapping::OnlineSemanticMapProto online_map_proto;
  absl::flat_hash_set<mapping::ElementId> visited_lane_ids;
  absl::flat_hash_set<mapping::ElementId> visited_boundary_ids;

  // Bidirectionally extend online map start from left neighbor lane.
  if (!nearest_lane->lane_neighbors_on_left.empty()) {
    const auto& left_neighbor = nearest_lane->lane_neighbors_on_left.front();
    SMM_ASSIGN_LANE_OR_ERROR_ISSUE(left_lane_info, psmm,
                                   left_neighbor.other_id);
    ExtendOnlineMapFromLane(
        psmm, left_lane_info, smooth_pos, option.look_ahead_distance,
        option.look_back_distance, option.lane_sample_interval,
        option.boundary_sample_interval, &online_map_proto, &visited_lane_ids,
        &visited_boundary_ids);
  }

  // Bidirectionally extend online map start from nearest lane.
  ExtendOnlineMapFromLane(psmm, *nearest_lane, smooth_pos,
                          option.look_ahead_distance, option.look_back_distance,
                          option.lane_sample_interval,
                          option.boundary_sample_interval, &online_map_proto,
                          &visited_lane_ids, &visited_boundary_ids);

  // Bidirectionally extend online map start from right neighbor lane.
  if (!nearest_lane->lane_neighbors_on_right.empty()) {
    const auto& right_neighbor = nearest_lane->lane_neighbors_on_right.front();
    SMM_ASSIGN_LANE_OR_ERROR_ISSUE(right_lane_info, psmm,
                                   right_neighbor.other_id);
    ExtendOnlineMapFromLane(
        psmm, right_lane_info, smooth_pos, option.look_ahead_distance,
        option.look_back_distance, option.lane_sample_interval,
        option.boundary_sample_interval, &online_map_proto, &visited_lane_ids,
        &visited_boundary_ids);
  }

  static int64_t unique_id = 1;
  online_map_proto.set_update_id(unique_id++);
  online_map_proto.set_timestamp_s(option.timestamp_s);
  *online_map_proto.mutable_localization_transform() =
      coordinate_converter.localization_transform();
  online_map_proto.mutable_ego_pos()->set_x(smooth_pos.x());
  online_map_proto.mutable_ego_pos()->set_y(smooth_pos.y());
  online_map_proto.set_ego_yaw(option.smooth_yaw);

  return online_map_proto;
}

absl::StatusOr<mapping::OnlineSemanticMapProto>
RunOnlineSemanticMapPredictionConverter(
    const PlannerSemanticMapManager& psmm,
    const OnlineSemanticMapConverterOption& option) {
  RETURN_IF_ERROR(CheckOptionValidity(option));

  const Vec2d smooth_pos(option.smooth_x, option.smooth_y);
  const auto& coordinate_converter = psmm.coordinate_converter();
  const auto level_id = coordinate_converter.GetLevel();
  const double global_yaw =
      coordinate_converter.SmoothYawToGlobal(option.smooth_yaw);

  const auto& lane_info_vec = FindLaneInfoByRange(
      psmm.GetLanesInfoWithHeadingAtLevel(level_id, smooth_pos, global_yaw,
                                          kLMaxNearbyLanesGetRadius,
                                          /*max_heading_diff=*/M_PI_4),
      smooth_pos, option.perception_lateral_range);

  mapping::OnlineSemanticMapProto online_map_proto;
  absl::flat_hash_set<mapping::ElementId> visited_lane_ids;
  absl::flat_hash_set<mapping::ElementId> visited_boundary_ids;

  for (const auto* lane : lane_info_vec) {
    ExtendOnlineMapFromLane(psmm, *lane, smooth_pos, option.look_ahead_distance,
                            option.look_back_distance,
                            option.lane_sample_interval,
                            option.boundary_sample_interval, &online_map_proto,
                            &visited_lane_ids, &visited_boundary_ids);
  }

  static int64_t unique_id = 1;
  online_map_proto.set_update_id(unique_id++);
  online_map_proto.set_timestamp_s(option.timestamp_s);
  *online_map_proto.mutable_localization_transform() =
      coordinate_converter.localization_transform();
  online_map_proto.mutable_ego_pos()->set_x(smooth_pos.x());
  online_map_proto.mutable_ego_pos()->set_y(smooth_pos.y());
  online_map_proto.set_ego_yaw(option.smooth_yaw);

  return online_map_proto;
}
}  // namespace qcraft::planner
