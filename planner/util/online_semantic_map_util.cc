#include "onboard/planner/util/online_semantic_map_util.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include "onboard/lite/qissue_trans.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

absl::StatusOr<std::vector<Vec2d>> FindClosestLanePathPoints(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map,
    const FrenetFrame& frenet_frame, const Vec2d& ego_xy,
    double valid_lane_length, double max_lat_offset_thres,
    double avg_lat_offset_thres) {
  FUNC_QTRACE();

  const auto starting_lane_ids =
      ComputeStartLanesByPos(psmm, online_map, ego_xy, /*proj_thres=*/1.0,
                             /*max_backward_thres=*/1.0);

  std::vector<mapping::LanePath> lane_path_vec;
  for (const auto id : starting_lane_ids) {
    mapping::LanePath lane_path(psmm.semantic_map_manager(),
                                {mapping::ElementId(id)},
                                /*start_fraction=*/0.0, /*end_fraction=*/1.0);
    lane_path =
        BackwardExtendLanePath(psmm, lane_path, kDrivePassageKeepBehindLength);
    ASSIGN_OR_CONTINUE(lane_path,
                       ClampLanePathFromPos(psmm, lane_path, ego_xy));
    lane_path = lane_path.BeforeArclength(valid_lane_length);

    auto new_lane_path_vec =
        CollectAllLanePathFromStartLane(psmm, lane_path, valid_lane_length);
    lane_path_vec.insert(lane_path_vec.end(),
                         std::make_move_iterator(new_lane_path_vec.begin()),
                         std::make_move_iterator(new_lane_path_vec.end()));
  }

  constexpr double kSampleLaneStep = 1.0;  // m.
  double min_abs_lat_offset = avg_lat_offset_thres;
  std::vector<Vec2d> closest_points;
  for (const auto& lane_path : lane_path_vec) {
    ASSIGN_OR_CONTINUE(auto points,
                       SampleLanePathByStep(psmm, lane_path, kSampleLaneStep));

    int valid_pt_cnt = 0;
    double lat_offset = 0.0, max_lat_offset = 0.0;
    for (const auto& pt : points) {
      const auto sl = frenet_frame.XYToSL(pt);
      if (sl.s < frenet_frame.start_s()) continue;
      if (sl.s > frenet_frame.end_s()) break;

      const double abs_offset = std::fabs(sl.l);
      max_lat_offset = std::max(max_lat_offset, abs_offset);
      lat_offset += abs_offset;
      valid_pt_cnt++;
    }
    if (valid_pt_cnt < 2 || max_lat_offset > max_lat_offset_thres) continue;

    lat_offset /= valid_pt_cnt;
    if (lat_offset < min_abs_lat_offset) {
      min_abs_lat_offset = lat_offset;
      closest_points = std::move(points);
    }
  }

  if (closest_points.empty()) {
    return absl::NotFoundError(
        "Failed to find closest lp from online semantic map.");
  }
  return closest_points;
}

std::vector<mapping::ElementId> ComputeStartLanesByPos(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map, const Vec2d& ego_xy,
    double proj_thres, double max_backward_thres) {
  FUNC_QTRACE();

  std::vector<mapping::ElementId> start_lane_ids;
  absl::flat_hash_set<mapping::ElementId> skipped_ids;
  for (const auto& lane : online_map.lanes()) {
    const auto lane_id = mapping::ElementId(lane.id());
    if (skipped_ids.contains(lane_id)) continue;

    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, lane_id);

    const auto& incomings = lane_info.incoming_lanes();
    const auto& outgoings = lane_info.outgoing_lanes();

    const auto ego_sl = lane_info.SmoothXYToSL(ego_xy);
    if (ego_sl.s > lane_info.length() + proj_thres ||
        ego_sl.s < -max_backward_thres ||
        (!incomings.empty() && ego_sl.s < -proj_thres)) {
      continue;
    }

    start_lane_ids.push_back(lane_id);
    skipped_ids.insert(incomings.begin(), incomings.end());
    skipped_ids.insert(outgoings.begin(), outgoings.end());
  }

  return start_lane_ids;
}

absl::StatusOr<std::vector<mapping::ElementId>> FindCloseOnlineLaneIds(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map,
    const FrenetFrame& frenet_frame, double valid_lane_length,
    double max_check_length, double max_lat_offset_thres,
    double avg_lat_offset_thres) {
  FUNC_QTRACE();

  constexpr double kSampleLaneStep = 1.0;  // m.
  std::vector<mapping::ElementId> close_lane_ids;
  for (const auto& lane : online_map.lanes()) {
    const auto id = mapping::ElementId(lane.id());
    mapping::LanePath lane_path(psmm.semantic_map_manager(), {id},
                                /*start_fraction=*/0.0, /*end_fraction=*/1.0);

    if (lane_path.length() < valid_lane_length) continue;

    int valid_pt_cnt = 0;
    double lat_offset = 0.0, max_lat_offset = 0.0;
    std::optional<double> start_check_s;
    for (double sample_s = 0.0; sample_s <= lane_path.length();
         sample_s += kSampleLaneStep) {
      const auto pt =
          ComputeLanePointPos(psmm, lane_path.ArclengthToLanePoint(sample_s));

      const auto sl = frenet_frame.XYToSL(pt);
      if (sl.s < frenet_frame.start_s()) continue;
      if (sl.s > frenet_frame.end_s()) break;

      const double abs_offset = std::fabs(sl.l);
      max_lat_offset = std::max(max_lat_offset, abs_offset);
      lat_offset += abs_offset;
      valid_pt_cnt++;

      if (!start_check_s.has_value()) {
        start_check_s = sample_s;
      }

      if (sample_s - *start_check_s >= max_check_length) break;
    }
    if (valid_pt_cnt < 2 || max_lat_offset > max_lat_offset_thres) continue;

    lat_offset /= valid_pt_cnt;
    if (lat_offset < avg_lat_offset_thres) {
      close_lane_ids.push_back(id);
    }
  }

  if (close_lane_ids.empty()) {
    return absl::NotFoundError(
        "Failed to find any close lanes from online semantic map.");
  }

  return close_lane_ids;
}

}  // namespace qcraft::planner
