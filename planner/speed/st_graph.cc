#include "onboard/planner/speed/st_graph.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <stddef.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <tuple>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/aabox3d.pb.h"
#include "onboard/math/geometry/proto/halfplane.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/path_approx_overlap.h"
#include "onboard/planner/decision/traffic_gap_result.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_finder_flags.h"
#include "onboard/planner/speed/st_close_trajectory_generator.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

DEFINE_bool(planner_use_path_approx_based_st_mapping, true,
            "Whether to path-approx-based st boundaries mapping.");

namespace qcraft::planner {

namespace {

constexpr double kSearchBuffer = 0.2;  // m.
constexpr double kMaxLatDist = 1.5;    // m.
constexpr double kEps = 1e-4;

using ::qcraft::prediction::PredictedTrajectory;

std::optional<int> FindFinalPathPointIndexInRealDrivePassage(
    const SegmentMatcherKdtree& path_kd_tree,
    const DrivePassage* drive_passage) {
  if (drive_passage == nullptr) return std::nullopt;
  const auto& last_real_station =
      drive_passage->station(drive_passage->last_real_station_index());
  const auto& last_real_station_boundaries = last_real_station.boundaries();
  const double search_radius =
      std::max(std::abs(last_real_station_boundaries.front().lat_offset),
               std::abs(last_real_station_boundaries.back().lat_offset));
  int final_index_in_real_drive_passage;
  if (path_kd_tree.GetNearestSegmentIndexWithHeading(
          last_real_station.xy().x(), last_real_station.xy().y(),
          last_real_station.tangent().FastAngle(), search_radius, M_PI_2,
          &final_index_in_real_drive_passage)) {
    return final_index_in_real_drive_passage;
  }
  return std::nullopt;
}

std::vector<DistanceInfo> CalcDistanceInfoToImpassableBoundaries(
    const DiscretizedPath& path_points,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const PlannerSemanticMapManager& psman_mgr, double search_radius,
    double min_mirrors_height, bool consider_mirrors_by_default, double buffer,
    std::optional<int> last_path_point_index) {
  FUNC_QTRACE();

  int considered_path_point_num = av_shapes.size();
  if (last_path_point_index.has_value()) {
    considered_path_point_num = std::max(
        1, std::min(considered_path_point_num, *last_path_point_index));
  }
  std::vector<DistanceInfo> res;
  res.reserve(considered_path_point_num);
  constexpr std::array<const char*, 3> kPartName{"B", "LM", "RM"};
  for (int i = 0; i < considered_path_point_num; ++i) {
    const auto& path_point = path_points[i];
    const auto& av_shape = av_shapes[i];
    const auto segs_info = psman_mgr.GetImpassableBoundariesInfoAtLevel(
        psman_mgr.GetLevel(), av_shape->center(), search_radius);
    double min_dist = std::numeric_limits<double>::infinity();
    int part_idx = -1;
    const std::string* id_ptr = nullptr;
    for (const auto& seg_info : segs_info) {
      if (double dist = av_shape->MainBodyDistanceTo(seg_info.segment);
          dist < min_dist) {
        min_dist = dist;
        part_idx = 0;  // MainBody.
        id_ptr = &seg_info.id;
      }
      const bool has_height = seg_info.height.has_value();
      const bool consider_mirrors =
          (consider_mirrors_by_default && !has_height) ||
          (has_height && *seg_info.height > min_mirrors_height);
      if (consider_mirrors) {
        if (const double dist =
                av_shape->LeftMirrorDistanceTo(seg_info.segment);
            dist < min_dist) {
          min_dist = dist;
          part_idx = 1;  // LeftMirror.
          id_ptr = &seg_info.id;
        }
        if (const double dist =
                av_shape->RightMirrorDistanceTo(seg_info.segment);
            dist < min_dist) {
          min_dist = dist;
          part_idx = 2;  // RightMirror.
          id_ptr = &seg_info.id;
        }
      }
    }
    if (std::isfinite(min_dist)) {
      res.push_back({.s = path_point.s(),
                     .dist = min_dist,
                     .id = *id_ptr,
                     .info = absl::StrFormat(
                         "%s|s: %.2f|id: {%s}|dist: %.2f", kPartName[part_idx],
                         path_point.s(), *id_ptr, min_dist)});
    } else {
      res.push_back(
          {.s = path_point.s(), .dist = min_dist, .id = "", .info = ""});
    }
    if (min_dist <= buffer) break;
  }
  return res;
}

absl::Status CheckStBoundary(const StBoundary& st_boundary) {
  const auto& id = st_boundary.id();
  absl::Status status = absl::OkStatus();
  if (st_boundary.lower_points().size() < 2) {
    status = absl::InternalError(absl::StrFormat(
        "st_boundary: %s lower points size is less than 2.", id));
    QEVENT("pingshi", "st_graph_invalid_st_boundary",
           [&status](QEvent* qevent) {
             qevent->AddField("error_reason", status.message());
           });
    return status;
  }

  if (st_boundary.max_s() < 0.0) {
    status = absl::InternalError(
        absl::StrFormat("st_boundary: %s max_s() < 0.0.", id));
  }

  if (st_boundary.max_t() < 0.0) {
    status = absl::InternalError(
        absl::StrFormat("st_boundary: %s max_t() < 0.0.", id));
  }

  if (!status.ok()) {
    VLOG(2) << status.ToString();
    QEVENT("pingshi", "st_graph_invalid_st_boundary",
           [&status](QEvent* qevent) {
             qevent->AddField("error_reason", status.message());
           });
  }

  return absl::OkStatus();
}

void CheckAndRemoveInvalidStBoundaryPoints(
    std::vector<StBoundaryPoints>* boundaries_points) {
  QCHECK_NOTNULL(boundaries_points);
  QCHECK(!boundaries_points->empty());
  const auto& points = boundaries_points->back();
  QCHECK(!points.lower_points.empty());
  QCHECK_EQ(points.lower_points.size(), points.upper_points.size());
  if (points.lower_points.size() < 2) boundaries_points->pop_back();
}

bool IsMappableSpacetimeObject(
    const SpacetimeObjectTrajectory& spacetime_object) {
  if (spacetime_object.states().empty()) {
    VLOG(2) << "Skip mapping spacetime object " << spacetime_object.traj_id()
            << " because its states are empty.";
    return false;
  }
  if (ToStBoundaryObjectType(spacetime_object.planner_object().type()) ==
      StBoundaryProto::IGNORABLE) {
    VLOG(2) << "Skip mapping spacetime object " << spacetime_object.traj_id()
            << " because its object type "
            << ObjectType_Name(spacetime_object.planner_object().type())
            << " is ignorable.";
    return false;
  }
  return true;
}

double ComputeRelativeSpeed(double v_heading, double v, double ref_heading) {
  return v * Vec2d::FastUnitFromAngle(v_heading).dot(
                 Vec2d::FastUnitFromAngle(ref_heading));
}

std::string MakeStBoundaryId(absl::string_view traj_id, int idx) {
  return absl::StrFormat("%s|%d", traj_id, idx);
}

std::string MakeProtectiveStBoundaryId(absl::string_view traj_id) {
  return absl::StrFormat("%s|%c", traj_id, 'p');
}

std::optional<std::pair<double, double>> ConvertToOverlapRange(
    absl::Span<const AgentOverlap> agent_overlaps) {
  if (agent_overlaps.empty()) return std::nullopt;
  std::optional<std::pair<double, double>> overlap_range;
  for (const auto& agent_overlap : agent_overlaps) {
    if (agent_overlap.lat_dist != 0.0) continue;
    if (!overlap_range.has_value()) {
      overlap_range =
          std::make_pair(agent_overlap.first_ra_s, agent_overlap.last_ra_s);
    } else {
      overlap_range->first =
          std::min(overlap_range->first, agent_overlap.first_ra_s);
      overlap_range->second =
          std::max(overlap_range->second, agent_overlap.last_ra_s);
    }
  }
  if (overlap_range.has_value() &&
      overlap_range->first >= overlap_range->second) {
    return std::nullopt;
  }
  return overlap_range;
}

using StNearestPoint = StCloseTrajectory::StNearestPoint;
using CloseTrajPoints = std::vector<StNearestPoint>;

std::vector<CloseTrajPoints> GenerateMovingStCloseTrajPoints(
    absl::Span<const StGraph::AgentNearestPoint> agent_nearest_points,
    bool is_cut_in_vehicle) {
  std::vector<CloseTrajPoints> close_trajs_points;
  bool prev_state_is_close = false;
  for (int i = 0; i < agent_nearest_points.size(); ++i) {
    const auto& nearest_point = agent_nearest_points[i];
    const auto ra_heading = nearest_point.ra_heading;
    const auto obj_theta = nearest_point.obj_theta;
    const double abs_lat_dist = std::abs(nearest_point.lat_dist);
    if (is_cut_in_vehicle) {
      constexpr double kParallelHeadingThreshold = M_PI / 9.0;
      const double theta_diff =
          std::fabs(NormalizeAngle(ra_heading - obj_theta));
      constexpr double kSmallAngleCutInLateralBuffer = 0.6;  // m.
      if (abs_lat_dist > kSmallAngleCutInLateralBuffer ||
          theta_diff > kParallelHeadingThreshold) {
        prev_state_is_close = false;
        continue;
      }
    } else if (abs_lat_dist > kMaxLatDist) {
      prev_state_is_close = false;
      continue;
    }
    if (!prev_state_is_close) {
      close_trajs_points.emplace_back();
      close_trajs_points.back().reserve(agent_nearest_points.size() - i);
    }
    prev_state_is_close = true;
    auto& close_traj_points = close_trajs_points.back();
    close_traj_points.push_back(
        {.s = nearest_point.ra_s,
         .t = nearest_point.t,
         .v = nearest_point.obj_v * Vec2d::FastUnitFromAngle(obj_theta).dot(
                                        Vec2d::FastUnitFromAngle(ra_heading)),
         .lat_dist = nearest_point.lat_dist,
         .obj_idx = nearest_point.obj_idx});
  }
  return close_trajs_points;
}

void GenerateCutInProtectiveStBoundary(
    absl::Span<const CloseTrajPoints> curr_close_trajs, double path_length,
    double plan_start_v, double max_decel, double vehicle_length,
    std::vector<StBoundaryPoints>* st_boundaries_points) {
  QCHECK_NOTNULL(st_boundaries_points);
  const auto& fisrt_overlap_boundary_points = st_boundaries_points->front();
  const double fisrt_overlap_lower_s =
      fisrt_overlap_boundary_points.lower_points.front().s();

  constexpr double kNearDistanceThres = 3.0;  // m.
  if (!curr_close_trajs.empty()) {
    size_t cnt = 0;
    auto end_iter = curr_close_trajs.rbegin();
    for (; end_iter != curr_close_trajs.rend(); ++end_iter) {
      if (end_iter->back().s + kNearDistanceThres < fisrt_overlap_lower_s) {
        break;
      }
      cnt += end_iter->size();
    }
    StBoundaryPoints boundary_points;
    boundary_points.protection_type = StBoundaryProto::SMALL_ANGLE_CUT_IN;
    boundary_points.Reserve(cnt);
    // Emplace the first true overlap point to make st boundaries continous.
    const double fisrt_overlap_upper_s =
        fisrt_overlap_boundary_points.upper_points.front().s();
    const double fisrt_overlap_delta_s =
        std::max(vehicle_length, fisrt_overlap_upper_s - fisrt_overlap_lower_s);
    const double fisrt_overlap_time =
        fisrt_overlap_boundary_points.speed_points.front().t();
    boundary_points.speed_points.push_back(
        fisrt_overlap_boundary_points.speed_points.front());
    boundary_points.lower_points.push_back(
        fisrt_overlap_boundary_points.lower_points.front());
    boundary_points.upper_points.push_back(
        fisrt_overlap_boundary_points.upper_points.front());
    boundary_points.overlap_infos.push_back(
        fisrt_overlap_boundary_points.overlap_infos.front());

    for (auto curr_close_traj_iter = curr_close_trajs.rbegin();
         curr_close_traj_iter != end_iter; ++curr_close_traj_iter) {
      for (auto pt_iter = curr_close_traj_iter->rbegin();
           pt_iter != curr_close_traj_iter->rend(); ++pt_iter) {
        const auto& pt = *pt_iter;
        constexpr double kSpeedBuffer = 2.0;      // m/s.
        constexpr double kMaxExtendedTime = 3.0;  // s.
        const double last_mid_s =
            boundary_points.lower_points.back().s() * 0.5 +
            boundary_points.upper_points.back().s() * 0.5;
        const double last_time = boundary_points.lower_points.back().t();
        const double cur_v = pt.v + kSpeedBuffer;
        if (fisrt_overlap_time - pt.t > kMaxExtendedTime || last_time < pt.t) {
          continue;
        }
        const double lower_s = last_mid_s - fisrt_overlap_delta_s * 0.5 -
                               (last_time - pt.t) * cur_v;
        constexpr double kMaxLonDist = 30.0;  // m.
        constexpr double kMinLowerS = 1.0;    // m.
        if (lower_s < kMinLowerS ||
            lower_s < fisrt_overlap_lower_s - kMaxLonDist) {
          break;
        }
        const double max_brake_v =
            std::max(0.0, pt.t * max_decel + plan_start_v);
        const double max_brake_s = (Sqr(max_brake_v) - Sqr(plan_start_v)) /
                                   std::min(2.0 * max_decel, -kEps);
        if (lower_s < max_brake_s) {
          break;
        }
        boundary_points.speed_points.emplace_back(cur_v, pt.t);
        boundary_points.lower_points.emplace_back(lower_s, pt.t);
        boundary_points.upper_points.emplace_back(
            std::min(path_length, lower_s + fisrt_overlap_delta_s), pt.t);
        boundary_points.overlap_infos.push_back(OverlapInfo{
            .time = pt.t,
            .obj_idx = pt.obj_idx,
            .av_start_idx = FloorToInt(pt.s / kPathSampleInterval),
            .av_end_idx = FloorToInt(
                (std::min(path_length, pt.s + fisrt_overlap_delta_s)) /
                kPathSampleInterval)});
      }
    }
    std::reverse(boundary_points.speed_points.begin(),
                 boundary_points.speed_points.end());
    std::reverse(boundary_points.lower_points.begin(),
                 boundary_points.lower_points.end());
    std::reverse(boundary_points.upper_points.begin(),
                 boundary_points.upper_points.end());
    std::reverse(boundary_points.overlap_infos.begin(),
                 boundary_points.overlap_infos.end());
    if (boundary_points.speed_points.size() > 2) {
      st_boundaries_points->emplace(st_boundaries_points->begin(),
                                    std::move(boundary_points));
    }
  }
}

bool FindOverlapRangeOrNearestPointUsingPathApprox(
    const DiscretizedPath& path_points, const PathApprox* path_approx,
    const PathApprox* path_approx_for_mirrors, double search_radius,
    double search_radius_for_mirrors, const SpacetimeObjectState& obj_state,
    int obj_idx, bool consider_mirrors, double lat_buffer, double lon_buffer,
    int* low_idx, int* high_idx,
    std::vector<StGraph::AgentNearestPoint>* agent_nearest_points) {
  QCHECK_NOTNULL(low_idx);
  QCHECK_NOTNULL(high_idx);
  // Use path aprrox.
  const auto& obj_shape = obj_state.contour;
  const auto& traj_point = obj_state.traj_point;
  const double path_step_length = path_points[1].s() - path_points[0].s();
  double first_s = std::numeric_limits<double>::infinity(), last_s = 0.0;
  const auto agent_overlaps = ComputeAgentOverlapsWithBuffer(
      *path_approx, path_step_length, /*first_index=*/0,
      /*last_index=*/path_points.size() - 1, obj_shape,
      /*max_lat_dist=*/kMaxLatDist, lat_buffer, lon_buffer, search_radius);
  if (!agent_overlaps.empty()) {
    if (agent_overlaps.front().lat_dist == 0.0) {
      const auto overlap_range = ConvertToOverlapRange(agent_overlaps);
      if (overlap_range.has_value()) {
        std::tie(first_s, last_s) = *overlap_range;
      }
    } else if (agent_nearest_points != nullptr) {
      const auto agent_overlap = agent_overlaps.front();

      agent_nearest_points->push_back(
          StGraph::AgentNearestPoint{.ra_s = agent_overlap.first_ra_s,
                                     .ra_heading = agent_overlap.ra_heading,
                                     .t = traj_point->t(),
                                     .obj_v = traj_point->v(),
                                     .obj_theta = traj_point->theta(),
                                     .lat_dist = agent_overlap.lat_dist,
                                     .obj_idx = obj_idx});
    }
  }
  if (path_approx_for_mirrors != nullptr && consider_mirrors) {
    const auto agent_overlaps_with_mirrors = ComputeAgentOverlapsWithBuffer(
        *path_approx_for_mirrors, path_step_length, /*first_index=*/0,
        /*last_index=*/path_points.size() - 1, obj_shape,
        /*max_lat_dist=*/kSearchBuffer, lat_buffer, lon_buffer,
        search_radius_for_mirrors);
    const auto overlap_range_for_mirrors =
        ConvertToOverlapRange(agent_overlaps_with_mirrors);
    if (overlap_range_for_mirrors.has_value()) {
      first_s = std::min(first_s, overlap_range_for_mirrors->first);
      last_s = std::max(last_s, overlap_range_for_mirrors->second);
    }
  }
  if (first_s > last_s) {
    VLOG(3) << "First collision s is larger than last collision s";
    return false;
  }
  *low_idx = std::max(0, CeilToInt(first_s / path_step_length - kEps));
  *high_idx = std::min(static_cast<int>(path_points.size() - 1),
                       FloorToInt(last_s / path_step_length + kEps));
  return true;
}

bool FindOverlapRangeOrNearestPointUsingAvShapes(
    const DiscretizedPath& path_points,
    // The SDC shapes on path points.
    const absl::Span<const VehicleShapeBasePtr> av_shape_on_path_points,
    const SegmentMatcherKdtree& path_kd_tree, const Vec2d& search_point,
    double search_radius, double search_radius_for_mirrors,
    const SpacetimeObjectState& obj_state, int obj_idx, bool consider_mirrors,
    double lat_buffer, double lon_buffer, int* low_idx, int* high_idx,
    std::vector<StGraph::AgentNearestPoint>* agent_nearest_points) {
  QCHECK_NOTNULL(low_idx);
  QCHECK_NOTNULL(high_idx);
  // Use path aprrox.
  const auto& obj_shape = obj_state.contour;
  const auto& traj_point = obj_state.traj_point;
  const double required_max_gap = std::max(lat_buffer, lon_buffer);

  const double radius = std::max(search_radius, search_radius_for_mirrors);
  auto indices = path_kd_tree.GetSegmentIndexInRadius(search_point.x(),
                                                      search_point.y(), radius);
  if (indices.empty()) {
    VLOG(3) << absl::StrFormat("No point found within %.2f meter radius.",
                               radius);
    return false;
  }
  // Use av shapes.
  std::stable_sort(indices.begin(), indices.end());

  bool updated = false;
  StGraph::AgentNearestPoint agent_nearest_point{
      .ra_s = 0.0,
      .ra_heading = 0.0,
      .t = traj_point->t(),
      .obj_v = traj_point->v(),
      .obj_theta = traj_point->theta(),
      .lat_dist = std::numeric_limits<double>::infinity(),
      .obj_idx = obj_idx};
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& raw_av_shape = av_shape_on_path_points[*it];
    if (agent_nearest_points == nullptr) {
      if (raw_av_shape->HasOverlapWithBuffer(obj_shape, lat_buffer, lon_buffer,
                                             consider_mirrors)) {
        *low_idx = *it;
        updated = true;
        break;
      }
    } else {
      const double dist_to_main_body =
          raw_av_shape->MainBodyDistanceTo(obj_shape);
      if (dist_to_main_body < agent_nearest_point.lat_dist) {
        agent_nearest_point.ra_s = path_points[*it].s();
        agent_nearest_point.ra_heading = path_points[*it].theta();
        agent_nearest_point.lat_dist = dist_to_main_body;
      }
      if (dist_to_main_body <= required_max_gap) {
        *low_idx = *it;
        updated = true;
        break;
      }
      if (consider_mirrors) {
        if (const double dist = raw_av_shape->LeftMirrorDistanceTo(obj_shape);
            dist <= required_max_gap) {
          *low_idx = *it;
          updated = true;
          break;
        }
        if (const double dist = raw_av_shape->RightMirrorDistanceTo(obj_shape);
            dist <= required_max_gap) {
          *low_idx = *it;
          updated = true;
          break;
        }
      }
    }
  }
  if (!updated) {
    if (agent_nearest_points != nullptr) {
      agent_nearest_points->push_back(agent_nearest_point);
    }
    VLOG(3) << "search_point has no overlap with path.";
    return false;
  }
  for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
    const auto& raw_av_shape = av_shape_on_path_points[*it];
    if (raw_av_shape->HasOverlapWithBuffer(obj_shape, lat_buffer, lon_buffer,
                                           consider_mirrors)) {
      *high_idx = *it;
      break;
    }
  }

  return true;
}

}  // namespace

// StGraph.
StGraph::StGraph(
    const DiscretizedPath* path_points, int traj_steps, double plan_start_v,
    double max_decel, const VehicleGeometryParamsProto* vehicle_geo_params,
    const SpeedFinderParamsProto::StGraphParamsProto* st_graph_params,
    const std::vector<VehicleShapeBasePtr>* av_shape_on_path_points,
    const SegmentMatcherKdtree* path_kd_tree, const PathApprox* path_approx,
    const PathApprox* path_approx_for_mirrors)
    : total_plan_time_(kTrajectoryTimeStep * traj_steps),
      plan_start_v_(plan_start_v),
      max_decel_(max_decel),
      path_points_(QCHECK_NOTNULL(path_points)),
      av_shape_on_path_points_(QCHECK_NOTNULL(av_shape_on_path_points)),
      path_kd_tree_(QCHECK_NOTNULL(path_kd_tree)),
      vehicle_geo_params_(QCHECK_NOTNULL(vehicle_geo_params)),
      st_graph_params_(QCHECK_NOTNULL(st_graph_params)),
      path_approx_(path_approx),
      path_approx_for_mirrors_(path_approx_for_mirrors) {
  SCOPED_QTRACE("StGraph::StGraph");

  ego_radius_ = Hypot(std::max(vehicle_geo_params_->front_edge_to_center(),
                               vehicle_geo_params_->back_edge_to_center()),
                      vehicle_geo_params_->right_edge_to_center());
  if (vehicle_geo_params_->has_left_mirror() &&
      vehicle_geo_params_->has_right_mirror()) {
    const auto& mirror = vehicle_geo_params_->left_mirror();
    ego_radius_for_mirrors_ =
        Hypot(mirror.x(), mirror.y()) + 0.5 * mirror.length();
  }
  std::tie(min_mirror_height_avg_, max_mirror_height_avg_) =
      ComputeMinMaxMirrorAverageHeight(*vehicle_geo_params_);
}

std::vector<StBoundaryRef> StGraph::GetStBoundaries(
    const SpacetimeTrajectoryManager& traj_mgr,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    const ConstraintManager& constraint_mgr,
    const PlannerSemanticMapManager* psman_mgr,
    const DrivePassage* drive_passage, const PathSlBoundary* path_sl_boundary,
    ThreadPool* thread_pool) {
  FUNC_QTRACE();

  std::vector<StBoundaryRef> st_boundaries;

  if (path_points_->size() < 2) {
    QLOG(WARNING) << "Fail to get st_boundary because of too few path points.";
    return st_boundaries;
  }

  // Map spacetime objects on st_graph.
  const auto& traffic_gap = constraint_mgr.TrafficGap();
  const auto& moving_spacetime_objects = traj_mgr.moving_object_trajs();
  const int moving_obj_num = moving_spacetime_objects.size();
  std::vector<std::vector<StBoundaryRef>> st_boundaries_for_workers(
      moving_obj_num);
  ParallelFor(0, moving_obj_num, thread_pool, [&](int i) {
    const SpacetimeObjectTrajectory* st_traj = moving_spacetime_objects[i];
    const std::string traj_id(st_traj->traj_id());
    const std::string object_id(st_traj->object_id());
    const bool generate_lane_change_gap =
        ((traffic_gap.leader_id.has_value() &&
          *traffic_gap.leader_id == object_id) ||
         (traffic_gap.follower_id.has_value() &&
          *traffic_gap.follower_id == object_id));
    if (IsMappableSpacetimeObject(*st_traj) &&
        leading_objs.find(traj_id) == leading_objs.end()) {
      auto moving_st_boundaries = MapMovingSpacetimeObject(
          *st_traj, generate_lane_change_gap, /*calc_moving_close_traj=*/
          FLAGS_planner_enable_moving_close_traj_speed_limit);
      for (int j = 0; j < moving_st_boundaries.size(); ++j) {
        if (CheckStBoundary(*moving_st_boundaries[j]).ok()) {
          st_boundaries_for_workers[i].push_back(
              std::move(moving_st_boundaries[j]));
        }
      }
    }
  });

  // Collect all st_boundaries from each workers.
  for (auto& single_worker_st_boundaries : st_boundaries_for_workers) {
    st_boundaries.insert(
        st_boundaries.end(),
        std::make_move_iterator(single_worker_st_boundaries.begin()),
        std::make_move_iterator(single_worker_st_boundaries.end()));
  }

  // Map nearest stationary object on st_graph.
  auto stationary_st_boundaries = MapStationarySpacetimeObjects(
      traj_mgr.stationary_object_trajs(), leading_objs);
  for (int i = 0; i < stationary_st_boundaries.size(); ++i) {
    if (CheckStBoundary(*stationary_st_boundaries[i]).ok()) {
      VLOG(3) << "Map stationary objects on st_graph, id: "
              << stationary_st_boundaries[i]->id();
      st_boundaries.push_back(std::move(stationary_st_boundaries[i]));
    }
  }

  // Map stop_line on st_graph.
  const int num_stop_lines = constraint_mgr.StopLine().size();
  std::vector<StBoundaryRef> st_boundaries_from_stop_lines(num_stop_lines);
  ParallelFor(0, num_stop_lines, thread_pool, [&](int i) {
    const auto& stop_line = constraint_mgr.StopLine()[i];
    if (StBoundaryRef st_boundary = MapStopLine(stop_line)) {
      if (CheckStBoundary(*st_boundary).ok()) {
        st_boundaries_from_stop_lines[i] = std::move(st_boundary);
        VLOG(3) << "Map stop line on st_graph, id: " << stop_line.id();
      }
    }
  });
  for (auto& st_boundary : st_boundaries_from_stop_lines) {
    if (st_boundary != nullptr) {
      st_boundaries.push_back(std::move(st_boundary));
    }
  }

  // Map path stop_line on st_graph.
  const int num_path_stop_lines = constraint_mgr.PathStopLine().size();
  std::vector<StBoundaryRef> st_boundaries_from_path_stop_lines;
  st_boundaries_from_path_stop_lines.reserve(num_path_stop_lines);
  for (const auto& path_stop_line : constraint_mgr.PathStopLine()) {
    if (StBoundaryRef st_boundary = MapPathStopLine(path_stop_line)) {
      if (CheckStBoundary(*st_boundary).ok()) {
        st_boundaries_from_path_stop_lines.emplace_back(std::move(st_boundary));
        VLOG(3) << "Map path stop line on st_graph, id: "
                << path_stop_line.id();
      }
    }
  }
  for (auto& st_boundary : st_boundaries_from_path_stop_lines) {
    st_boundaries.push_back(std::move(st_boundary));
  }

  if (psman_mgr != nullptr) {
    // Map nearest collision impassable boundary on st_graph.
    if (auto nearest_impassable_st_boundary =
            MapNearestImpassableBoundary(*psman_mgr, drive_passage)) {
      if (CheckStBoundary(*nearest_impassable_st_boundary).ok()) {
        VLOG(3) << "Map impassable boundary on st_graph, id: "
                << nearest_impassable_st_boundary->id();
        st_boundaries.push_back(std::move(nearest_impassable_st_boundary));
      }
    }
  }
  // Map nearest path boundary on st_graph.
  if (drive_passage != nullptr && path_sl_boundary != nullptr) {
    if (auto nearest_path_boundary_st_boundary =
            MapNearestPathBoundary(*drive_passage, *path_sl_boundary)) {
      if (CheckStBoundary(*nearest_path_boundary_st_boundary).ok()) {
        VLOG(3) << "Map neareast path boundary on st_graph.";
        st_boundaries.push_back(std::move(nearest_path_boundary_st_boundary));
      }
    }
  }

  // Map leading object on st_graph.
  if (drive_passage != nullptr && path_sl_boundary != nullptr &&
      !leading_objs.empty()) {
    for (auto& leading_obj_st_boundary : MapLeadingSpacetimeObjects(
             traj_mgr, leading_objs, *drive_passage, *path_sl_boundary)) {
      if (CheckStBoundary(*leading_obj_st_boundary).ok()) {
        st_boundaries.push_back(std::move(leading_obj_st_boundary));
      }
    }
  }

  return st_boundaries;
}

StBoundaryRef StGraph::MapNearestImpassableBoundary(
    const PlannerSemanticMapManager& psman_mgr,
    const DrivePassage* drive_passage) {
  FUNC_QTRACE();
  const double search_radius_for_impassable_boundaries =
      std::max(ego_radius_, ego_radius_for_mirrors_) +
      st_graph_params_->impassable_boundaries_search_radius_buffer();
  const auto last_path_point_index =
      FindFinalPathPointIndexInRealDrivePassage(*path_kd_tree_, drive_passage);
  constexpr double kExtendBuffer = 0.1;  // m.
  distance_info_to_impassable_boundaries_ =
      CalcDistanceInfoToImpassableBoundaries(
          *path_points_, *av_shape_on_path_points_, psman_mgr,
          search_radius_for_impassable_boundaries, min_mirror_height_avg_,
          st_graph_params_->consider_mirrors_by_default(), kExtendBuffer,
          last_path_point_index);
  const auto& distance_info = distance_info_to_impassable_boundaries_.back();
  if (distance_info.dist > kExtendBuffer) return nullptr;

  const double s_max = path_points_->back().s();
  const double s_min = std::min(s_max, std::max(0.0, distance_info.s));
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(s_min, 0.0),
                                     StPoint(s_min, total_plan_time_)};
  st_boundary_points.upper_points = {StPoint(s_max, 0.0),
                                     StPoint(s_max, total_plan_time_)};
  st_boundary_points.speed_points = {VtPoint(0.0, 0.0),
                                     VtPoint(0.0, total_plan_time_)};
  return QCHECK_NOTNULL(StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::IMPASSABLE_BOUNDARY,
      distance_info.id,
      /*probability=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false));
}

StBoundaryRef StGraph::MapNearestPathBoundary(
    const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary) const {
  FUNC_QTRACE();

  // The conditions are in line with
  // ValidateDrivePassageAndPathBoundaryViolation in trajectory_validation.
  constexpr double kPathBoundaryViolationLimit = 1.2;  // meter.
  std::optional<double> low_s = std::nullopt;
  const double offset =
      0.5 * vehicle_geo_params_->width() - kPathBoundaryViolationLimit;
  for (const auto& path_point : *path_points_) {
    const auto av_sl = drive_passage.QueryFrenetCoordinateAt(
        Vec2d(path_point.x(), path_point.y()));
    if (!av_sl.ok()) {
      low_s = path_point.s();
      break;
    }
    const auto boundary_l_pair = path_sl_boundary.QueryBoundaryL(av_sl->s);
    if (boundary_l_pair.second - av_sl->l - offset < 0.0 ||
        boundary_l_pair.first - av_sl->l + offset > 0.0) {
      low_s = path_point.s();
      break;
    }
  }
  if (low_s.has_value()) {
    const double s_min = std::max(0.0, *low_s);
    const double s_max = path_points_->back().s();
    StBoundaryPoints st_boundary_points;
    st_boundary_points.lower_points = {StPoint(s_min, 0.0),
                                       StPoint(s_min, total_plan_time_)};
    st_boundary_points.upper_points = {StPoint(s_max, 0.0),
                                       StPoint(s_max, total_plan_time_)};
    st_boundary_points.speed_points = {VtPoint(0.0, 0.0),
                                       VtPoint(0.0, total_plan_time_)};

    return QCHECK_NOTNULL(StBoundary::CreateInstance(
        st_boundary_points, StBoundaryProto::PATH_BOUNDARY, "PATH_BOUNDARY",
        /*probability=*/1.0,
        /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
        /*is_large_vehicle=*/false));
  } else {
    return nullptr;
  }
}

std::optional<std::pair<int, int>> StGraph::FindSegment2dOverlapRange(
    const Segment2d& segment) const {
  std::pair<int, int> overlap_range_idx;  // {low, high}
  const auto center = segment.center();
  const double radius = segment.length() * 0.5 + ego_radius_ + kSearchBuffer;
  const double driving_direction =
      (-segment.unit_direction().Perp()).FastAngle();

  // Use path aprrox.
  if (FLAGS_planner_use_path_approx_based_st_mapping &&
      path_approx_ != nullptr) {
    const double path_step_length =
        (*path_points_)[1].s() - (*path_points_)[0].s();

    double first_s = std::numeric_limits<double>::infinity(), last_s = 0.0;
    // We use a tiny box to represent segment.
    constexpr double kSmallBoxWidth = 0.01;  // m.
    const auto agent_overlaps = ComputeAgentOverlapsWithBufferAndHeading(
        *path_approx_, path_step_length, /*first_index=*/0,
        /*last_index=*/path_points_->size() - 1,
        Polygon2d(Box2d(segment, kSmallBoxWidth)),
        /*max_lat_dist=*/kSearchBuffer, /*lat_buffer=*/0.0, /*lon_buffer=*/0.0,
        radius, driving_direction, /*max_heading_diff=*/M_PI_2);
    const auto overlap_range = ConvertToOverlapRange(agent_overlaps);
    if (overlap_range.has_value()) {
      std::tie(first_s, last_s) = *overlap_range;
    }
    if (first_s > last_s) {
      return std::nullopt;
    }
    overlap_range_idx.first =
        std::max(0, CeilToInt(first_s / path_step_length - kEps));
    overlap_range_idx.second =
        std::min(static_cast<int>(path_points_->size() - 1),
                 FloorToInt(last_s / path_step_length + kEps));
    return overlap_range_idx;
  }
  // Use av shapes.
  auto indices = path_kd_tree_->GetSegmentIndexInRadiusWithHeading(
      center.x(), center.y(), driving_direction, radius, M_PI_2);
  if (indices.empty()) return std::nullopt;
  std::stable_sort(indices.begin(), indices.end());

  bool updated = false;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& av_shape = (*av_shape_on_path_points_)[*it];
    if (av_shape->MainBodyHasOverlapWithBuffer(segment,
                                               /*lat_buffer=*/0.0,
                                               /*lon_buffer=*/0.0)) {
      overlap_range_idx.first = *it;
      updated = true;
      break;
    }
  }
  if (!updated) return std::nullopt;
  for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
    const auto& av_shape = (*av_shape_on_path_points_)[*it];
    if (av_shape->MainBodyHasOverlapWithBuffer(segment,
                                               /*lat_buffer=*/0.0,
                                               /*lon_buffer=*/0.0)) {
      overlap_range_idx.second = *it;
      break;
    }
  }
  return overlap_range_idx;
}

std::optional<std::pair<int, int>> StGraph::FindStopLineOverlapRange(
    const ConstraintProto::StopLineProto& stop_line) const {
  const Segment2d stop_line_seg(Vec2dFromProto(stop_line.half_plane().start()),
                                Vec2dFromProto(stop_line.half_plane().end()));
  return FindSegment2dOverlapRange(stop_line_seg);
}

StBoundaryRef StGraph::MapStopLine(
    const ConstraintProto::StopLineProto& stop_line) const {
  FUNC_QTRACE();

  const auto overlap_range = FindStopLineOverlapRange(stop_line);
  if (!overlap_range.has_value()) return nullptr;
  const double s_min = std::max(0.0, (*path_points_)[overlap_range->first].s());
  const double s_max = std::max(s_min, path_points_->back().s());
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(s_min, 0.0),
                                     StPoint(s_min, total_plan_time_)};
  st_boundary_points.upper_points = {StPoint(s_max, 0.0),
                                     StPoint(s_max, total_plan_time_)};
  st_boundary_points.speed_points = {VtPoint(0.0, 0.0),
                                     VtPoint(0.0, total_plan_time_)};

  return QCHECK_NOTNULL(StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VIRTUAL, stop_line.id(),
      /*probability=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false));
}

StBoundaryRef StGraph::MapPathStopLine(
    const ConstraintProto::PathStopLineProto& path_stop_line) const {
  FUNC_QTRACE();

  double s_min = 0.0;
  double s_max = 0.0;

  QCHECK_GE(path_stop_line.s(), 0.0);
  QCHECK_LE(path_stop_line.s(), path_points_->back().s());
  s_min = path_stop_line.s();
  s_max = path_points_->back().s();

  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(s_min, 0.0),
                                     StPoint(s_min, total_plan_time_)};
  st_boundary_points.upper_points = {StPoint(s_max, 0.0),
                                     StPoint(s_max, total_plan_time_)};
  st_boundary_points.speed_points = {VtPoint(0.0, 0.0),
                                     VtPoint(0.0, total_plan_time_)};

  return QCHECK_NOTNULL(StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VIRTUAL, path_stop_line.id(),
      /*probability=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false));
}

std::vector<StBoundaryRef> StGraph::MapMovingSpacetimeObject(
    const SpacetimeObjectTrajectory& spacetime_object,
    bool generate_lane_change_gap, bool calc_moving_close_traj) const {
  FUNC_QTRACE();

  QCHECK(!spacetime_object.is_stationary());

  std::vector<StBoundaryRef> st_boundaries;

  const auto st_boundaries_points = GetMovingObjStBoundaryPoints(
      spacetime_object, generate_lane_change_gap, calc_moving_close_traj);
  if (st_boundaries_points.empty()) {
    return st_boundaries;
  }

  const int size = st_boundaries_points.size();
  st_boundaries.reserve(size);
  int protective_boundary_cnt = 0;
  for (int i = 0; i < size; ++i) {
    std::string st_boundary_id;
    if (st_boundaries_points[i].protection_type !=
        StBoundaryProto::NON_PROTECTIVE) {
      st_boundary_id = MakeProtectiveStBoundaryId(spacetime_object.traj_id());
      // There is at most 1 protective boundary now and it must be the 0th
      // element.
      QCHECK_EQ(i, 0);
      ++protective_boundary_cnt;
    } else {
      st_boundary_id = (size - protective_boundary_cnt) == 1
                           ? std::string(spacetime_object.traj_id())
                           : MakeStBoundaryId(spacetime_object.traj_id(), i);
    }
    QCHECK_EQ(st_boundaries_points[i].lower_points.size(),
              st_boundaries_points[i].upper_points.size());
    if (st_boundaries_points[i].lower_points.size() < 2) continue;
    auto st_boundary = StBoundary::CreateInstance(
        st_boundaries_points[i],
        ToStBoundaryObjectType(spacetime_object.planner_object().type()),
        std::move(st_boundary_id), spacetime_object.trajectory().probability(),
        /*is_stationary=*/false, st_boundaries_points[i].protection_type,
        spacetime_object.planner_object().is_large_vehicle());
    st_boundaries.push_back(std::move(st_boundary));
  }
  if (st_boundaries.size() >= 2 && st_boundaries[0]->is_protective()) {
    // Protective st-boundary protect the first (by min t) original st boundary.
    st_boundaries[0]->set_protected_st_boundary_id(st_boundaries[1]->id());
  }
  if (calc_moving_close_traj) {
    absl::MutexLock lock(&mutex_);
    std::sort(moving_close_trajs_.begin(), moving_close_trajs_.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.min_t() < rhs.min_t();
              });
  }
  return st_boundaries;
}

std::vector<StBoundaryRef> StGraph::MapStationarySpacetimeObjects(
    absl::Span<const SpacetimeObjectTrajectory* const>
        stationary_spacetime_objs,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs) const {
  std::vector<StBoundaryRef> st_boundaries;
  st_boundaries.reserve(stationary_spacetime_objs.size());
  for (int i = 0; i < stationary_spacetime_objs.size(); ++i) {
    const SpacetimeObjectTrajectory* obj = stationary_spacetime_objs[i];
    const std::string traj_id(obj->traj_id());
    QCHECK(obj->is_stationary());
    if (!IsMappableSpacetimeObject(*obj)) continue;
    if (leading_objs.find(traj_id) != leading_objs.end()) continue;
    auto st_boundary_points_or = GetStationaryObjStBoundaryPoints(*obj);
    if (!st_boundary_points_or) {
      VLOG(3) << "GetStationaryObjStBoundaryPoints fails. Id: " << traj_id;
      continue;
    }
    st_boundaries.push_back(StBoundary::CreateInstance(
        *st_boundary_points_or, ToStBoundaryObjectType(obj->object_type()),
        traj_id, obj->trajectory().probability(),
        /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
        obj->planner_object().is_large_vehicle()));
  }

  return st_boundaries;
}

std::vector<StBoundaryRef> StGraph::MapLeadingSpacetimeObjects(
    const SpacetimeTrajectoryManager& traj_mgr,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary) const {
  SCOPED_QTRACE_ARG1("StGraph::MapLeadingSpacetimeObjects", "num_obj",
                     leading_objs.size());

  std::vector<StBoundaryRef> st_boundaries;
  st_boundaries.reserve(leading_objs.size());
  for (const auto& [traj_id, leading_obj] : leading_objs) {
    const SpacetimeObjectTrajectory* st_traj =
        traj_mgr.FindTrajectoryById(traj_id);
    if (st_traj == nullptr) continue;
    if (!IsMappableSpacetimeObject(*st_traj)) continue;
    if (st_traj->is_stationary()) {
      const auto lower_index = FindLeadingObjectLowerIndex(
          st_traj->contour(), drive_passage, path_sl_boundary);
      if (!lower_index.has_value()) continue;
      const double low_s = (*path_points_)[*lower_index].s();
      const double high_s = path_points_->back().s();
      StBoundaryPoints st_boundary_points;
      st_boundary_points.lower_points = {{low_s, 0.0},
                                         {low_s, total_plan_time_}};
      st_boundary_points.upper_points = {{high_s, 0.0},
                                         {high_s, total_plan_time_}};
      st_boundary_points.speed_points = {{0.0, 0.0}, {0.0, total_plan_time_}};
      st_boundary_points.overlap_infos = {OverlapInfo{
          .time = 0.0,
          .obj_idx = 0,
          .av_start_idx = *lower_index,
          .av_end_idx = static_cast<int>(path_points_->size() - 1)}};
      st_boundaries.push_back(StBoundary::CreateInstance(
          st_boundary_points,
          ToStBoundaryObjectType(st_traj->planner_object().type()), traj_id,
          st_traj->trajectory().probability(),
          /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
          st_traj->planner_object().is_large_vehicle()));
    } else {
      const auto& object_states = st_traj->states();
      QCHECK_GT(object_states.size(), 0);
      const Box2d& obj_box = st_traj->bounding_box();
      const double required_lateral_gap = st_traj->required_lateral_gap();
      const double obj_radius = obj_box.diagonal() * 0.5 + required_lateral_gap;
      const double search_radius = obj_radius + ego_radius_ + kSearchBuffer;
      const double search_radius_for_mirrors =
          obj_radius + ego_radius_for_mirrors_ + kSearchBuffer;

      const bool consider_mirrors = IsConsiderMirrorObject(
          st_traj->planner_object().object_proto(), min_mirror_height_avg_,
          max_mirror_height_avg_);
      StBoundaryPoints st_boundary_points;
      st_boundary_points.Reserve(object_states.size());
      const int st_constraints_length = leading_obj.st_constraints_size();
      const double leading_time =
          st_constraints_length == 0
              ? std::numeric_limits<double>::infinity()
              : leading_obj.st_constraints(st_constraints_length - 1).t();
      for (int i = 0; i < object_states.size(); ++i) {
        const auto& curr_state = object_states[i];
        const auto* traj_point = curr_state.traj_point;
        const double t = traj_point->t();
        int lower_index = 0, upper_index = path_points_->size() - 1;
        double ra_theta = 0.0;
        if (t <= leading_time) {
          const auto lower_index_or = FindLeadingObjectLowerIndex(
              curr_state.contour, drive_passage, path_sl_boundary);
          if (!lower_index_or.has_value()) break;
          lower_index = *lower_index_or;
          ra_theta = (*path_points_)[lower_index].theta();
        } else {
          if (!FindOverlapRangeOrNearestPoint(
                  curr_state.box.center(), search_radius,
                  search_radius_for_mirrors, curr_state, i, consider_mirrors,
                  required_lateral_gap, required_lateral_gap, &lower_index,
                  &upper_index, /*agent_nearest_points=*/nullptr)) {
            break;
          }
          const double middle_s = ((*path_points_)[upper_index].s() +
                                   (*path_points_)[lower_index].s()) *
                                  0.5;
          const auto middle_point = path_points_->Evaluate(middle_s);
          ra_theta = middle_point.theta();
        }
        const double speed = ComputeRelativeSpeed(traj_point->theta(),
                                                  traj_point->v(), ra_theta);

        st_boundary_points.speed_points.emplace_back(speed, t);
        st_boundary_points.lower_points.emplace_back(
            (*path_points_)[lower_index].s(), t);
        st_boundary_points.upper_points.emplace_back(
            (*path_points_)[upper_index].s(), t);
        st_boundary_points.overlap_infos.push_back(
            OverlapInfo{.time = t,
                        .obj_idx = i,
                        .av_start_idx = lower_index,
                        .av_end_idx = upper_index});
      }
      if (st_boundary_points.lower_points.size() >= 2) {
        st_boundaries.push_back(StBoundary::CreateInstance(
            st_boundary_points,
            ToStBoundaryObjectType(st_traj->planner_object().type()), traj_id,
            st_traj->trajectory().probability(),
            /*is_stationary=*/false, StBoundaryProto::NON_PROTECTIVE,
            st_traj->planner_object().is_large_vehicle()));
      }
    }
  }
  return st_boundaries;
}

std::optional<StBoundaryPoints> StGraph::GetStationaryObjStBoundaryPoints(
    const SpacetimeObjectTrajectory& spacetime_object) const {
  FUNC_QTRACE();

  QCHECK(spacetime_object.is_stationary());
  StBoundaryPoints st_boundary_points;

  const Box2d& obj_box = spacetime_object.bounding_box();
  const double obj_radius =
      obj_box.diagonal() * 0.5 + spacetime_object.required_lateral_gap();
  const double search_radius = obj_radius + ego_radius_ + kSearchBuffer;
  const double search_radius_for_mirrors =
      obj_radius + ego_radius_for_mirrors_ + kSearchBuffer;

  int low_idx = 0;
  int high_idx = 0;
  const bool consider_mirrors =
      IsConsiderMirrorObject(spacetime_object.planner_object().object_proto(),
                             min_mirror_height_avg_, max_mirror_height_avg_);
  // No need to calc nearest point for stationary objects.
  if (!FindOverlapRangeOrNearestPoint(
          obj_box.center(), search_radius, search_radius_for_mirrors,
          spacetime_object.states()[0], /*obj_idx=*/0, consider_mirrors,
          spacetime_object.required_lateral_gap(),
          spacetime_object.required_lateral_gap(), &low_idx, &high_idx,
          /*agent_nearest_points=*/nullptr)) {
    return std::nullopt;
  }
  if (low_idx == 0) {
    if (!FindOverlapRangeOrNearestPoint(
            obj_box.center(), search_radius, search_radius_for_mirrors,
            spacetime_object.states()[0], /*obj_idx=*/0, consider_mirrors,
            /*lat_buffer=*/0.0,
            /*lon_buffer=*/0.0, &low_idx, &high_idx,
            /*agent_nearest_points=*/nullptr)) {
      return std::nullopt;
    }
  }
  const double low_s = (*path_points_)[low_idx].s();
  // Set high_s to be path length.
  const double high_s = path_points_->back().s();
  st_boundary_points.lower_points = {{low_s, 0.0}, {low_s, total_plan_time_}};
  st_boundary_points.upper_points = {{high_s, 0.0}, {high_s, total_plan_time_}};
  st_boundary_points.speed_points = {{0.0, 0.0}, {0.0, total_plan_time_}};
  st_boundary_points.overlap_infos = {OverlapInfo{.time = 0.0,
                                                  .obj_idx = 0,
                                                  .av_start_idx = low_idx,
                                                  .av_end_idx = high_idx}};
  return st_boundary_points;
}

std::vector<StBoundaryPoints> StGraph::GetMovingObjStBoundaryPoints(
    const SpacetimeObjectTrajectory& spacetime_object,
    bool generate_lane_change_gap, bool calc_moving_close_traj) const {
  QCHECK(!spacetime_object.is_stationary());
  std::vector<StBoundaryPoints> st_boundaries_points;
  const Box2d& obj_box = spacetime_object.bounding_box();
  const double obj_radius =
      obj_box.diagonal() * 0.5 + spacetime_object.required_lateral_gap();
  const double search_radius = obj_radius + ego_radius_ + kSearchBuffer;
  const double search_radius_for_mirrors =
      obj_radius + ego_radius_for_mirrors_ + kSearchBuffer;

  const auto& object_states = spacetime_object.states();
  QCHECK_GT(object_states.size(), 0);

  bool prev_state_has_overlap = false;
  constexpr double kNegtiveTimeThreshold = -1e-6;
  int low_idx = 0;
  int high_idx = 0;
  const bool consider_mirrors =
      IsConsiderMirrorObject(spacetime_object.planner_object().object_proto(),
                             min_mirror_height_avg_, max_mirror_height_avg_);
  const auto& path_points = *path_points_;
  std::vector<AgentNearestPoint> agent_nearest_points;
  for (int i = 0; i < object_states.size(); ++i) {
    const auto& cur_state = object_states[i];
    const auto* traj_point = cur_state.traj_point;
    QCHECK_GT(traj_point->t(), kNegtiveTimeThreshold);
    if (!FindOverlapRangeOrNearestPoint(
            cur_state.box.center(), search_radius, search_radius_for_mirrors,
            cur_state, i, consider_mirrors,
            spacetime_object.required_lateral_gap(),
            spacetime_object.required_lateral_gap(), &low_idx, &high_idx,
            &agent_nearest_points)) {
      if (prev_state_has_overlap) {
        CheckAndRemoveInvalidStBoundaryPoints(&st_boundaries_points);
      }
      prev_state_has_overlap = false;
      continue;
    }
    if (!prev_state_has_overlap) {
      st_boundaries_points.emplace_back();
      st_boundaries_points.back().Reserve(object_states.size() - i);
    }
    prev_state_has_overlap = true;

    auto& boundary_points = st_boundaries_points.back();
    const double middle_s =
        (path_points[high_idx].s() + path_points[low_idx].s()) * 0.5;
    const auto middle_point = path_points.Evaluate(middle_s);
    const double middle_speed = ComputeRelativeSpeed(
        traj_point->theta(), traj_point->v(), middle_point.theta());
    const double low_s = (*path_points_)[low_idx].s();
    const double high_s = (*path_points_)[high_idx].s();
    boundary_points.speed_points.emplace_back(middle_speed, traj_point->t());
    boundary_points.lower_points.emplace_back(low_s, traj_point->t());
    boundary_points.upper_points.emplace_back(high_s, traj_point->t());
    boundary_points.overlap_infos.push_back(
        OverlapInfo{.time = traj_point->t(),
                    .obj_idx = i,
                    .av_start_idx = low_idx,
                    .av_end_idx = high_idx});
  }
  constexpr double kMaxOverlapTimeThres = 7.0;  // s.
  if (st_boundaries_points.empty() ||
      st_boundaries_points.front().overlap_infos.front().time >
          kMaxOverlapTimeThres) {
    if (calc_moving_close_traj) {
      auto st_close_traj_points = GenerateMovingStCloseTrajPoints(
          agent_nearest_points, /*is_cut_in_vehicle=*/false);
      auto st_close_trajs = GenerateMovingStCloseTrajectories(
          spacetime_object, std::move(st_close_traj_points));
      absl::MutexLock lock(&mutex_);
      moving_close_trajs_.insert(
          moving_close_trajs_.end(),
          std::make_move_iterator(st_close_trajs.begin()),
          std::make_move_iterator(st_close_trajs.end()));
    }
    // Ignore the st-boundaries that first overlap time is after 7s. This
    // feature is upgraded from a bug.
    st_boundaries_points.clear();
  }

  GenerateProtectiveStBoundaries(spacetime_object, agent_nearest_points,
                                 obj_box.center(), generate_lane_change_gap,
                                 &st_boundaries_points);

  return st_boundaries_points;
}

std::optional<int> StGraph::FindLeadingObjectLowerIndex(
    const Polygon2d& obj_shape, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary) const {
  const absl::StatusOr<FrenetBox> frenet_box_or =
      drive_passage.QueryFrenetBoxAtContour(obj_shape);
  if (!frenet_box_or.ok() || frenet_box_or->s_min > path_sl_boundary.end_s()) {
    return std::nullopt;
  }

  const auto [right, left] =
      path_sl_boundary.QueryBoundaryXY(frenet_box_or->s_min);
  const Segment2d leading_seg(right, left);

  const auto overlap_range = FindSegment2dOverlapRange(leading_seg);
  return overlap_range.has_value() ? std::optional(overlap_range->first)
                                   : std::nullopt;
}

bool StGraph::FindOverlapRangeOrNearestPoint(
    const Vec2d& search_point, double search_radius,
    double search_radius_for_mirrors, const SpacetimeObjectState& obj_state,
    int obj_idx, bool consider_mirrors, double lat_buffer, double lon_buffer,
    int* low_idx, int* high_idx,
    std::vector<AgentNearestPoint>* agent_nearest_points) const {
  QCHECK_NOTNULL(low_idx);
  QCHECK_NOTNULL(high_idx);

  if (FLAGS_planner_use_path_approx_based_st_mapping &&
      path_approx_ != nullptr) {
    return FindOverlapRangeOrNearestPointUsingPathApprox(
        *path_points_, path_approx_, path_approx_for_mirrors_, search_radius,
        search_radius_for_mirrors, obj_state, obj_idx, consider_mirrors,
        lat_buffer, lon_buffer, low_idx, high_idx, agent_nearest_points);
  }
  return FindOverlapRangeOrNearestPointUsingAvShapes(
      *path_points_, *av_shape_on_path_points_, *path_kd_tree_, search_point,
      search_radius, search_radius_for_mirrors, obj_state, obj_idx,
      consider_mirrors, lat_buffer, lon_buffer, low_idx, high_idx,
      agent_nearest_points);
}

void StGraph::GenerateLatInflatedStBoundary(
    const SpacetimeObjectTrajectory& spacetime_object,
    StBoundaryProto::ProtectionType protection_type, double lat_buffer,
    std::vector<StBoundaryPoints>* st_boundaries_points) const {
  constexpr double kStBoundaryMinT = 5.0;  // s.
  constexpr double kMaxAcc = 0.6;          // m/ss.
  constexpr double kMinAcc = -0.8;         // m/ss.

  const double max_decel_time = std::abs(plan_start_v_ / kMinAcc);
  double max_time = total_plan_time_;
  if (!st_boundaries_points->empty()) {
    max_time = std::min(max_time,
                        st_boundaries_points->front().speed_points.front().t());
  }

  const Box2d& obj_box = spacetime_object.bounding_box();
  const double obj_radius = obj_box.diagonal() * 0.5 + lat_buffer;
  const double search_radius = obj_radius + ego_radius_ + kSearchBuffer;

  const auto& object_states = spacetime_object.states();
  const auto& path_points = *path_points_;
  int low_idx = 0;
  int high_idx = 0;

  QCHECK_NOTNULL(st_boundaries_points);
  constexpr double kMaxAccLimit = 0.5;  // m/ss.
  const double current_v = spacetime_object.planner_object().pose().v();
  const bool use_prediction =
      current_v < kEps ||
      std::abs(spacetime_object.planner_object().pose().a()) > kMaxAccLimit;
  const int current_size = st_boundaries_points->size();
  for (int i = 0; i < object_states.size(); ++i) {
    const auto& cur_state = object_states[i];
    const auto* traj_point = cur_state.traj_point;
    const double t =
        use_prediction ? traj_point->t() : traj_point->s() / current_v;
    if (t < kStBoundaryMinT) {
      continue;
    }
    if (t > max_time) {
      break;
    }
    if (!FindOverlapRangeOrNearestPoint(
            cur_state.box.center(), search_radius,
            /*search_radius_for_mirrors=*/0.0, cur_state, i,
            /*consider_mirrors=*/false, /*lat_buffer=*/lat_buffer,
            /*lon_buffer=*/0.0, &low_idx, &high_idx,
            /*agent_nearest_points=*/nullptr)) {
      if (st_boundaries_points->size() > current_size) {
        CheckAndRemoveInvalidStBoundaryPoints(st_boundaries_points);
        if (st_boundaries_points->size() > current_size) return;
      }
      continue;
    }

    double low_s = (*path_points_)[low_idx].s();
    double high_s = (*path_points_)[high_idx].s();
    // Clip the st boundary by acceleration/deceleration capability of ego
    // vehicle to avoid unreasonable st boudnary.
    const double max_s = plan_start_v_ * t + 0.5 * kMaxAcc * Sqr(t);
    const double decel_time = std::min(max_decel_time, t);
    const double min_s =
        plan_start_v_ * decel_time + 0.5 * kMinAcc * Sqr(decel_time);
    if (max_s < low_s || min_s > high_s) continue;
    if (max_s < high_s) {
      while ((*path_points_)[high_idx].s() > max_s) {
        --high_idx;
      }
      high_s = (*path_points_)[high_idx].s();
    }
    if (min_s > low_s) {
      while ((*path_points_)[low_idx].s() < min_s) {
        ++low_idx;
      }
      low_s = (*path_points_)[low_idx].s();
    }
    if (high_idx <= low_idx) continue;

    if (st_boundaries_points->size() == current_size) {
      StBoundaryPoints boundary_points;
      boundary_points.Reserve(object_states.size() - i);
      boundary_points.protection_type = protection_type;
      st_boundaries_points->emplace(st_boundaries_points->begin(),
                                    std::move(boundary_points));
    }

    auto& boundary_points = st_boundaries_points->front();
    const double middle_s =
        (path_points[high_idx].s() + path_points[low_idx].s()) * 0.5;
    const auto middle_point = path_points.Evaluate(middle_s);
    const double middle_speed = ComputeRelativeSpeed(
        traj_point->theta(), traj_point->v(), middle_point.theta());
    boundary_points.speed_points.emplace_back(middle_speed, t);
    boundary_points.lower_points.emplace_back(low_s, t);
    boundary_points.upper_points.emplace_back(high_s, t);
    boundary_points.overlap_infos.push_back(
        OverlapInfo{.time = t,
                    .obj_idx = i,
                    .av_start_idx = low_idx,
                    .av_end_idx = high_idx});
  }
}

void StGraph::GenerateProtectiveStBoundaries(
    const SpacetimeObjectTrajectory& spacetime_object,
    absl::Span<const StGraph::AgentNearestPoint> agent_nearest_points,
    const Vec2d& obj_center, bool generate_lane_change_gap,
    std::vector<StBoundaryPoints>* st_boundaries_points) const {
  QCHECK_NOTNULL(st_boundaries_points);
  const auto& path_points = *path_points_;
  if (generate_lane_change_gap) {
    constexpr double kLaneChangeGapLatBuffer = 2.5;  // m.
    GenerateLatInflatedStBoundary(
        spacetime_object, StBoundaryProto::LANE_CHANGE_GAP,
        kLaneChangeGapLatBuffer, st_boundaries_points);
  } else if (st_boundaries_points->empty()) {
    constexpr double kMaxRelVel = 3.0;  // m/s.
    constexpr double kMinVel = 10.0;    // m/s.
    if (st_graph_params_->consider_large_vehicle_blind_spot() &&
        spacetime_object.planner_object().is_large_vehicle() &&
        plan_start_v_ > kMinVel &&
        std::abs(spacetime_object.planner_object().pose().v() - plan_start_v_) <
            kMaxRelVel) {
      constexpr double kPreviewDistance = 10.0;             // m.
      constexpr double kParallelHeadingDiff = M_PI / 36.0;  // 5 degree.

      const auto& obj_contour = spacetime_object.contour();
      const auto& current_path_point = path_points.front();
      const Vec2d current_pos = ToVec2d(current_path_point);
      const Vec2d cur_heading_dir =
          Vec2d::FastUnitFromAngle(current_path_point.theta());
      const double theta_diff = NormalizeAngle(current_path_point.theta() -
                                               spacetime_object.pose().theta());
      Vec2d front_most, back_most;
      obj_contour.ExtremePoints(cur_heading_dir, &back_most, &front_most);

      const double front_dis = (front_most - current_pos).Dot(cur_heading_dir) +
                               vehicle_geo_params_->back_edge_to_center();
      const double back_dis = (back_most - current_pos).Dot(cur_heading_dir) -
                              vehicle_geo_params_->front_edge_to_center();
      if (std::abs(theta_diff) < kParallelHeadingDiff &&
          back_dis < kPreviewDistance && front_dis > 0.0) {
        constexpr double kBlindSpotAreaLatBuffer = 2.5;  // m.
        GenerateLatInflatedStBoundary(
            spacetime_object, StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT,
            kBlindSpotAreaLatBuffer, st_boundaries_points);
      }
    }
  } else if (path_approx_ != nullptr &&
             ToStBoundaryObjectType(spacetime_object.object_type()) ==
                 StBoundaryProto::VEHICLE &&
             Vec2d::FastUnitFromAngle((*path_points_)[0].theta())
                     .Dot(obj_center - ToVec2d(path_points[0])) > 0.0) {
    // Ignore vehicles behinde AV.
    const auto st_close_trajs =
        GenerateMovingStCloseTrajPoints(agent_nearest_points,
                                        /*is_cut_in_vehicle=*/true);
    GenerateCutInProtectiveStBoundary(
        st_close_trajs, path_points.back().s(), plan_start_v_, max_decel_,
        vehicle_geo_params_->length(), st_boundaries_points);
  }
}

bool StGraph::GetStDistancePointInfo(const SpacetimeObjectState& state,
                                     double slow_down_radius,
                                     StDistancePoint* st_distance_point) const {
  QCHECK_NOTNULL(st_distance_point);
  const Polygon2d& obj_shape = state.contour;
  const double search_radius =
      obj_shape.CircleRadius() + ego_radius_ + slow_down_radius;

  // Use path apporx.
  if (FLAGS_planner_use_path_approx_based_st_mapping &&
      path_approx_ != nullptr) {
    const double path_step_length =
        (*path_points_)[1].s() - (*path_points_)[0].s();
    const auto agent_overlaps = ComputeAgentOverlapsWithBuffer(
        *path_approx_, path_step_length, /*first_index=*/0,
        /*last_index=*/path_points_->size() - 1, obj_shape,
        /*max_lat_dist=*/slow_down_radius, /*lat_buffer=*/0.0,
        /*lon_buffer=*/0.0, search_radius);
    if (agent_overlaps.empty()) return false;
    const auto agent_overlap = agent_overlaps.front();
    const double dist = std::fabs(agent_overlap.lat_dist);
    if (dist > slow_down_radius) return false;

    st_distance_point->path_s = agent_overlap.first_ra_s;
    st_distance_point->distance = dist;
    st_distance_point->relative_v =
        ComputeRelativeSpeed((*state.traj_point).theta(),
                             (*state.traj_point).v(), agent_overlap.ra_heading);
    return true;
  }

  // Use brute force.
  const auto indices = path_kd_tree_->GetSegmentIndexInRadius(
      obj_shape.CircleCenter().x(), obj_shape.CircleCenter().y(),
      search_radius);
  if (indices.empty()) {
    return false;
  }
  double min_dist = std::numeric_limits<double>::max();
  PathPoint nearest_pt;
  for (const auto index : indices) {
    const auto& pt = (*path_points_)[index];
    const double dist = obj_shape.DistanceTo(ToVec2d(pt));
    if (dist < min_dist) {
      min_dist = dist;
      nearest_pt = pt;
    }
  }

  // The distance to the object is approximate.
  min_dist -= vehicle_geo_params_->width() * 0.5;
  if (min_dist > slow_down_radius) {
    return false;
  }

  st_distance_point->path_s = nearest_pt.s();
  st_distance_point->distance = min_dist;
  st_distance_point->relative_v = ComputeRelativeSpeed(
      (*state.traj_point).theta(), (*state.traj_point).v(), nearest_pt.theta());
  return true;
}

std::vector<CloseSpaceTimeObject> StGraph::GetCloseSpaceTimeObjects(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    absl::Span<const SpacetimeObjectTrajectory* const> spacetime_object_trajs,
    double slow_down_radius) const {
  absl::flat_hash_set<std::string> traj_id_set;
  traj_id_set.reserve(st_boundaries_with_decision.size());
  for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
    if (const auto& traj_id = st_boundary_with_decision.traj_id();
        traj_id.has_value()) {
      traj_id_set.insert(*traj_id);
    }
  }
  std::vector<CloseSpaceTimeObject> close_space_time_objects;
  close_space_time_objects.reserve(spacetime_object_trajs.size());
  for (const SpacetimeObjectTrajectory* obj_traj : spacetime_object_trajs) {
    if (traj_id_set.contains(obj_traj->traj_id()) ||
        obj_traj->states().empty()) {
      continue;
    }
    if (ToStBoundaryObjectType(obj_traj->planner_object().type()) ==
        StBoundaryProto::IGNORABLE) {
      continue;
    }

    StDistancePoint st_dis_point;
    if (!GetStDistancePointInfo(obj_traj->states()[0], slow_down_radius,
                                &st_dis_point)) {
      continue;
    }

    auto& object = close_space_time_objects.emplace_back();
    object.st_distance_points.push_back(st_dis_point);
    object.box = obj_traj->states()[0].box;
    object.object_type =
        ToStBoundaryObjectType(obj_traj->planner_object().type());
    object.id = obj_traj->traj_id();
  }
  return close_space_time_objects;
}

}  // namespace qcraft::planner
