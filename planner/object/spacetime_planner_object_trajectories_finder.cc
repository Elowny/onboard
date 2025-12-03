#include "onboard/planner/object/spacetime_planner_object_trajectories_finder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <string_view>
#include <utility>
#include <vector>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

// Desired lateral distance for nudge moving objects.
constexpr double kLateralDistanceNudgeMovingObs = 0.4;  // m.
// Following params are used for hysteresis to ensure decisions stability.
// Minimal lateral distance for nudge moving objects.
constexpr double kHysteresisLateralDistanceNudgeMovingObs = 0.1;  // m.
constexpr double kCloseMovingObjThreshold = 0.75;                 // m.

struct LateralRelationInfo {
  double obj_lmin = std::numeric_limits<double>::lowest();
  double obj_lmax = std::numeric_limits<double>::max();
  double ref_lmin = std::numeric_limits<double>::lowest();
  double ref_lmax = std::numeric_limits<double>::max();
};

bool IsTrajEffectiveAndFillTrajBoxes(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const std::vector<ApolloTrajectoryPointProto>& time_aligned_prev_traj,
    const VehicleGeometryParamsProto& veh_geo, double effective_check_time,
    double fill_boxes_time, std::vector<Box2d>* time_aligned_prev_traj_boxes) {
  const int comfortable_nudge_lat_speed_check_count =
      static_cast<int>(fill_boxes_time / kTrajectoryTimeStep + 0.5);
  time_aligned_prev_traj_boxes->reserve(
      comfortable_nudge_lat_speed_check_count);
  for (const auto& pt : time_aligned_prev_traj) {
    if (pt.relative_time() > effective_check_time) break;
    auto av_box = ComputeAvBox(Vec2d(pt.path_point().x(), pt.path_point().y()),
                               pt.path_point().theta(), veh_geo);

    const auto frenet_box_or = drive_passage.QueryFrenetBoxAt(av_box);
    if (!frenet_box_or.ok()) {
      time_aligned_prev_traj_boxes->clear();
      return false;
    }
    const auto [l_min, l_max] = path_sl_boundary.QueryBoundaryL(
        0.5 * (frenet_box_or->s_min + frenet_box_or->s_max));
    const bool is_box_inside_boundary =
        frenet_box_or->l_min > l_min && frenet_box_or->l_max < l_max;
    if (!is_box_inside_boundary) {
      time_aligned_prev_traj_boxes->clear();
      return false;
    }
    if (pt.relative_time() < fill_boxes_time) {
      time_aligned_prev_traj_boxes->push_back(std::move(av_box));
    }
  }
  return true;
}

bool IsFrontTrajectory(const DrivePassage& drive_passage,
                       const FrenetBox& av_sl_box, const Polygon2d& contour,
                       const VehicleGeometryParamsProto& veh_geo,
                       bool prev_st_planner_obj) {
  ASSIGN_OR_RETURN(const auto obj_frenet_box,
                   drive_passage.QueryFrenetBoxAtContour(contour), false);
  // If the object is not considered as a spacetime planner object previously,
  // its head must pass AV head minus an offset for it to be considered;
  // otherwise, it will be ignored by spacetime planners only if AV has passed
  // its head for a larger offset.
  constexpr double kFrontRelativeSOffset = 1.0;  // m.
  const double hysteresis_rel_s_offset = veh_geo.front_edge_to_center();
  if (prev_st_planner_obj &&
      av_sl_box.s_max < obj_frenet_box.s_max + hysteresis_rel_s_offset) {
    return true;
  }
  if (av_sl_box.s_max < obj_frenet_box.s_max + kFrontRelativeSOffset) {
    return true;
  }

  return false;
}

// NOLINTBEGIN(readability-function-cognitive-complexity)

struct NudgeInfo {
  double t;
  double obj_l_min;
  double obj_l_max;
  double path_l_min;
  double path_l_max;
  bool lon_overlapped;
  double s_min;
  double s_max;
  const Box2d* box;
  double theta;
};

// Check if AV can decelerate to position behind object state that first has
// no nudge space. Accurate check is unnecessary.
bool CanComfortablyDecel(const DrivePassage& drive_passage,
                         const std::vector<NudgeInfo>& nudge_info_vec,
                         double av_speed, const FrenetBox& av_sl_box,
                         int index) {
  constexpr double kComfortableDecel = -2.0;

  if (index < nudge_info_vec.size()) {
    // must at same direction with refline.
    const auto& nudge_info = nudge_info_vec[index];
    const auto passage_theta_at_object_s =
        drive_passage.QueryTangentAngleAtS(nudge_info.s_min);
    if (passage_theta_at_object_s.ok()) {
      if (std::abs(NormalizeAngle(*passage_theta_at_object_s -
                                  nudge_info.theta)) < M_PI_2) {
        const double decelerate_time =
            std::min(nudge_info.t, av_speed / (-kComfortableDecel));
        const double decelerate_s =
            av_speed * decelerate_time +
            0.5 * kComfortableDecel * Sqr(decelerate_time);
        VLOG(2) << "Av can decelerate with constant acc(" << kComfortableDecel
                << "m/s^2) for " << decelerate_time << "s, distance is "
                << decelerate_s << ", object_min_s is " << nudge_info.s_min
                << ", av_max_s is " << av_sl_box.s_max;
        if ((decelerate_s + av_sl_box.s_max) < nudge_info.s_min) {
          VLOG(2) << "AV may comfortably decelerate to safe position behind "
                     "object at time "
                  << nudge_info.t << "(s).";
          return true;
        }
      }
    }
  }
  return false;
}

bool IsComfortablyNudgeableTrajectory(
    const DrivePassage& drive_passage,
    const std::vector<ApolloTrajectoryPointProto>& time_aligned_prev_traj,
    const PathSlBoundary& path_sl_boundary, const Box2d& av_box,
    const FrenetBox& av_sl_box, double av_speed, double av_width,
    const SpacetimeObjectTrajectory& traj, double lat_nudge_dist,
    const VehicleGeometryParamsProto& veh_geo,
    double comfortable_nudge_check_time,
    double comfortable_nudge_lat_speed_check_time,
    std::optional<bool>* is_prev_traj_effective,
    std::vector<Box2d>* time_aligned_prev_traj_boxes) {
  // In case of undesired small negative speed.
  av_speed = std::max(av_speed, 0.0);
  // If object is too close to AV, we can't nudge it comfortably for sure. The
  // distance threshold is related to AV speed.
  const PiecewiseLinearFunction<double, double> av_speed_close_moving_dist_plf =
      {{0.0, 10.0, 20.0}, {0.2, 0.3, 0.4}};
  const auto& states = traj.states();
  const double av_to_obj_dist = av_box.DistanceTo(states[0].box);
  if (av_to_obj_dist < av_speed_close_moving_dist_plf(av_speed)) {
    VLOG(2) << "Object is too close to AV to be nudged. Distance: "
            << av_to_obj_dist;
    return false;
  }

  std::vector<NudgeInfo> nudge_info_vec;
  nudge_info_vec.reserve(states.size());
  constexpr double kAvMaxAccel = 0.2;   // m/s^2.
  constexpr double kAvMinAccel = -0.2;  // m/s^2.
  const double brake_to_stop_time = -av_speed / kAvMinAccel;
  const double brake_to_stop_dist = -0.5 * Sqr(av_speed) / kAvMinAccel;
  for (int i = 0; i < states.size(); ++i) {
    const auto& state = states[i];
    if (state.traj_point->t() > comfortable_nudge_check_time) break;
    ASSIGN_OR_CONTINUE(const auto frenet_box,
                       drive_passage.QueryFrenetBoxAt(state.box));
    const auto [l_min, l_max] = path_sl_boundary.QueryBoundaryL(
        0.5 * (frenet_box.s_min + frenet_box.s_max));
    // Use min/max const acceleration model to estimate AV longitudinal
    // reachable range.
    const double half_t_sqr = 0.5 * Sqr(state.traj_point->t());
    const double av_max_progress =
        av_speed * state.traj_point->t() + half_t_sqr * kAvMaxAccel;
    const double av_min_progress =
        state.traj_point->t() < brake_to_stop_time
            ? av_speed * state.traj_point->t() + half_t_sqr * kAvMinAccel
            : brake_to_stop_dist;
    const double av_s_min = av_sl_box.s_min + av_min_progress;
    const double av_s_max = av_sl_box.s_max + av_max_progress;
    constexpr double kLonOverlapExtent = 1.0;  // m.
    const bool lon_overlapped =
        frenet_box.s_min < av_s_max + kLonOverlapExtent &&
        frenet_box.s_max > av_s_min - kLonOverlapExtent;
    VLOG(3) << "At point " << i << " obj_s_min " << frenet_box.s_min
            << " obj_s_max " << frenet_box.s_max << " obj_l_min "
            << frenet_box.l_min << " obj_l_max " << frenet_box.l_max
            << " path_l_min " << l_min << " path_l_max " << l_max
            << " av_s_min " << av_s_min << " av_s_max " << av_s_max
            << " lon_overlapped " << int(lon_overlapped) << " t "
            << state.traj_point->t();
    nudge_info_vec.push_back({.t = state.traj_point->t(),
                              .obj_l_min = frenet_box.l_min,
                              .obj_l_max = frenet_box.l_max,
                              .path_l_min = l_min,
                              .path_l_max = l_max,
                              .lon_overlapped = lon_overlapped,
                              .s_min = frenet_box.s_min,
                              .s_max = frenet_box.s_max,
                              .box = &state.box,
                              .theta = state.traj_point->theta()});
  }

  // If object is moving along AV direction fast, is currently has no
  // longitudinal overlap with AV and only has a small period of longitudinal
  // overlap with AV, no need to nudge.
  constexpr double kRelSpeedThres = 2.5;  // m/s.
  constexpr double kObjectSExtent = 2.0;  // m
  const auto& nudge_info_first = nudge_info_vec.front();
  if (traj.planner_object().velocity().Dot(av_box.tangent()) - av_speed >
      kRelSpeedThres) {
    const bool cur_lon_overlapped =
        (av_sl_box.s_max > (nudge_info_first.s_min - kObjectSExtent) &&
         av_sl_box.s_max < (nudge_info_first.s_max + kObjectSExtent)) ||
        (av_sl_box.s_min > (nudge_info_first.s_min - kObjectSExtent) &&
         av_sl_box.s_min < (nudge_info_first.s_max + kObjectSExtent));
    if (!cur_lon_overlapped) {
      double last_lon_overlap_time = 0.0;
      for (auto it = nudge_info_vec.rbegin(); it != nudge_info_vec.rend();
           it++) {
        if (it->lon_overlapped) {
          last_lon_overlap_time = it->t;
        }
      }
      constexpr double kLonOverlapTimeThres = 2.0;  // s.
      if (last_lon_overlap_time < kLonOverlapTimeThres) {
        VLOG(2) << "last_lon_overlap_time: " << last_lon_overlap_time;
        return false;
      }
    }
  }

  std::optional<bool> is_prev_traj_close_to_object_traj;

  constexpr double kMinimumNudgeBuffer = 0.5;  // m.
  constexpr double kTimeEps = 1e-3;            // m.
  const PiecewiseLinearFunction<double, double> av_speed_lat_speed_thres_plf = {
      {0.0, 5.0, 10.0, 15.0, 20}, {3.0, 2.0, 1.2, 0.6, 0.3}};
  const double av_lat_speed_thres = av_speed_lat_speed_thres_plf(av_speed);
  const double av_center_l = 0.5 * (av_sl_box.l_max + av_sl_box.l_min);
  const double av_half_width = 0.5 * av_width;
  bool can_comfortably_nudge_from_left = true, left_have_space = true;
  int first_left_no_space_index = std::numeric_limits<int>::infinity();
  VLOG(2) << "Check comfortable nudge from left.";
  for (int i = 0; i < nudge_info_vec.size(); ++i) {
    // Check if AV can comfortably nudge from left.
    const auto& nudge_info = nudge_info_vec[i];
    if (!nudge_info.lon_overlapped) {
      VLOG(3) << "Not lon overlapped at point " << i;
      continue;
    }
    if (left_have_space) {
      const double space =
          nudge_info.path_l_max - nudge_info.obj_l_max - av_width;
      if (space < lat_nudge_dist) {
        VLOG(2) << "Not enough left nudge space at point " << i
                << ", path_l_max: " << nudge_info.path_l_max
                << ", obj_l_max: " << nudge_info.obj_l_max
                << ", av_width: " << av_width
                << ", lat_nudge_dist: " << lat_nudge_dist;
        left_have_space = false;
        first_left_no_space_index = i;
      }
    }
    // Don't check lateral speed for pedestrian.
    if (traj.object_type() == ObjectType::OT_PEDESTRIAN) continue;
    if (nudge_info.t < kTimeEps) continue;
    if (nudge_info.t < comfortable_nudge_lat_speed_check_time) {
      // Use AV center to estimate nudge lateral movement and speed.
      const double estimate_nudge_lat_movement =
          std::max(nudge_info.obj_l_max + kMinimumNudgeBuffer + av_half_width -
                       av_center_l,
                   0.0);
      const double estimate_nudge_lat_speed =
          estimate_nudge_lat_movement / nudge_info.t;
      VLOG(2) << "Left nudge at point " << i << ", estimate_nudge_lat_movement "
              << estimate_nudge_lat_movement
              << ", estimate_nudge_lat_speed: " << estimate_nudge_lat_speed;
      if (estimate_nudge_lat_speed > av_lat_speed_thres) {
        VLOG(2) << "Uncomfortable left nudge lat speed! Threshold: "
                << av_lat_speed_thres;
        can_comfortably_nudge_from_left = false;
      }
    }
  }

  // Maybe estimate_nudge_lat_speed is not accurate, so we check av
  // position based on prev traj.
  constexpr double kDistanceToPrevTrajBuffer = 0.6;  // m.
  const auto evaluate_prev_traj_and_object_traj =
      [&time_aligned_prev_traj_boxes, &nudge_info_vec]() -> bool {
    for (int idx = 0; idx < time_aligned_prev_traj_boxes->size() &&
                      idx < nudge_info_vec.size();
         ++idx) {
      const auto& av_box = (*time_aligned_prev_traj_boxes)[idx];
      const double dist_to_prev_traj =
          av_box.DistanceTo(*nudge_info_vec[idx].box);
      if (dist_to_prev_traj < kDistanceToPrevTrajBuffer) {
        return true;
      }
    }
    return false;
  };

  if (!can_comfortably_nudge_from_left) {
    if (!is_prev_traj_effective->has_value()) {
      *is_prev_traj_effective = IsTrajEffectiveAndFillTrajBoxes(
          drive_passage, path_sl_boundary, time_aligned_prev_traj, veh_geo,
          comfortable_nudge_check_time, comfortable_nudge_lat_speed_check_time,
          time_aligned_prev_traj_boxes);
    }
    if (*is_prev_traj_effective) {
      is_prev_traj_close_to_object_traj = evaluate_prev_traj_and_object_traj();
      if (!*is_prev_traj_close_to_object_traj) {
        can_comfortably_nudge_from_left = true;
      }
    }
  }
  if (can_comfortably_nudge_from_left && left_have_space) {
    return true;
  }

  if (can_comfortably_nudge_from_left && (!left_have_space) &&
      CanComfortablyDecel(drive_passage, nudge_info_vec, av_speed, av_sl_box,
                          first_left_no_space_index)) {
    return true;
  }

  bool can_comfortably_nudge_from_right = true, right_have_space = true;
  int first_right_no_space_index = std::numeric_limits<int>::infinity();
  VLOG(2) << "Check comfortable nudge from right.";
  for (int i = 0; i < nudge_info_vec.size(); ++i) {
    // Check if AV can comfortably nudge from right.
    const auto& nudge_info = nudge_info_vec[i];
    if (!nudge_info.lon_overlapped) continue;
    if (right_have_space) {
      const double space =
          nudge_info.obj_l_min - nudge_info.path_l_min - av_width;
      if (space < lat_nudge_dist) {
        VLOG(2) << "Not enough right nudge space at point " << i
                << ", obj_l_min: " << nudge_info.obj_l_min
                << ", path_l_min: " << nudge_info.path_l_min
                << ", av_width: " << av_width
                << ", lat_nudge_dist: " << lat_nudge_dist;
        right_have_space = false;
        first_right_no_space_index = i;
      }
    }
    // Don't check lateral speed for pedestrian.
    if (traj.object_type() == ObjectType::OT_PEDESTRIAN) continue;
    if (nudge_info.t < kTimeEps) continue;
    if (nudge_info.t < comfortable_nudge_lat_speed_check_time) {
      // Use AV center to estimate nudge lateral movement and speed.
      const double estimate_nudge_lat_movement =
          std::max(av_center_l - nudge_info.obj_l_min + kMinimumNudgeBuffer +
                       av_half_width,
                   0.0);
      const double estimate_nudge_lat_speed =
          estimate_nudge_lat_movement / nudge_info.t;
      VLOG(2) << "Right nudge at point " << i
              << ", estimate_nudge_lat_movement " << estimate_nudge_lat_movement
              << ", estimate_nudge_lat_speed: " << estimate_nudge_lat_speed;
      if (estimate_nudge_lat_speed > av_lat_speed_thres) {
        VLOG(2) << "Uncomfortable right nudge lat speed! Threshold: "
                << av_lat_speed_thres;
        can_comfortably_nudge_from_right = false;
      }
    }
  }

  if (!can_comfortably_nudge_from_right) {
    if (!is_prev_traj_effective->has_value()) {
      *is_prev_traj_effective = IsTrajEffectiveAndFillTrajBoxes(
          drive_passage, path_sl_boundary, time_aligned_prev_traj, veh_geo,
          comfortable_nudge_check_time, comfortable_nudge_lat_speed_check_time,
          time_aligned_prev_traj_boxes);
    }
    if (*is_prev_traj_effective) {
      if (!is_prev_traj_close_to_object_traj.has_value()) {
        is_prev_traj_close_to_object_traj =
            evaluate_prev_traj_and_object_traj();
      }
      if (!*is_prev_traj_close_to_object_traj) {
        can_comfortably_nudge_from_right = true;
      }
    }
  }
  if (can_comfortably_nudge_from_right && right_have_space) {
    return true;
  }
  if (can_comfortably_nudge_from_right && (!right_have_space) &&
      CanComfortablyDecel(drive_passage, nudge_info_vec, av_speed, av_sl_box,
                          first_right_no_space_index)) {
    return true;
  }

  return false;
}  // NOLINTEND(readability-function-cognitive-complexity)

bool IsFrontOrSideObject(const DrivePassage& drive_passage,
                         const FrenetCoordinate& av_sl, double av_length,
                         const Box2d& bbox) {
  ASSIGN_OR_RETURN(const auto obj_frenet_box,
                   drive_passage.QueryFrenetBoxAt(bbox), false);
  // Object's current front s is less than ego s (behind ego vehicle), do not
  // consider in space time planner.
  if (av_sl.s + av_length * 0.5 < obj_frenet_box.s_max) {
    return true;
  }
  return false;
}

bool IsFrontOrSideObjectForEmergencySituation(const DrivePassage& drive_passage,
                                              const FrenetBox& av_sl_box,
                                              const Box2d& bbox) {
  ASSIGN_OR_RETURN(const auto obj_frenet_box,
                   drive_passage.QueryFrenetBoxAt(bbox), false);
  // Object's current front s is less than ego s (behind ego vehicle), do not
  // consider in space time planner.
  if (av_sl_box.s_max < obj_frenet_box.s_max) {
    return true;
  }
  return false;
}

bool IsLeftBoundaryBrokenLine(const mapping::LaneInfo& lane_info,
                              const PlannerSemanticMapManager& psmm) {
  const auto& boundary_infos = lane_info.lane_boundaries_on_left;
  if (!boundary_infos.empty()) {
    const auto* boundary = psmm.FindLaneBoundaryByIdOrNull(
        boundary_infos.front().lane_boundary_id);
    if (boundary != nullptr &&
        !(boundary->type == mapping::LaneBoundaryProto::BROKEN_WHITE ||
          boundary->type == mapping::LaneBoundaryProto::BROKEN_YELLOW ||
          boundary->type == mapping::LaneBoundaryProto::BROKEN_DOUBLE_YELLOW ||
          boundary->type ==
              mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE)) {
      return false;
    }
  }
  return true;
}

bool IsAllIncomingLanesStraight(const mapping::LaneInfo& lane_info,
                                const PlannerSemanticMapManager& psmm) {
  const auto& lanes_incoming = lane_info.incoming_lanes();
  for (const auto& lane_id : lanes_incoming) {
    const auto incoming_lane_ptr = psmm.FindLaneInfoOrNull(lane_id);
    if (incoming_lane_ptr != nullptr &&
        incoming_lane_ptr->direction != mapping::LaneProto::STRAIGHT) {
      return false;
    }
  }
  return true;
}

void ComputeDriveInSRange(const mapping::LaneInfo& lane_info,
                          const mapping::LaneInfo& right_first_lane_neighbor,
                          const std::optional<Station>& start_dp_station,
                          const Station& station, double dp_end_s,
                          const PlannerSemanticMapManager& psmm,
                          std::optional<double>* s_merge_range_min,
                          std::optional<double>* s_merge_range_max) {
  if (start_dp_station.has_value() &&
      lane_info.id == start_dp_station->lane_id()) {
    if (!IsLeftBoundaryBrokenLine(lane_info, psmm)) {
      return;
    }
    *s_merge_range_min = 0.0;
    *s_merge_range_max =
        std::min(lane_info.length() *
                     (1.0 - start_dp_station->GetLanePoint().fraction()),
                 dp_end_s);
  } else {
    *s_merge_range_max =
        std::min(station.accumulated_s() + lane_info.length(), dp_end_s);
    auto* current_lane = &lane_info;
    auto* right_lane = &right_first_lane_neighbor;
    double current_s = *(*s_merge_range_max) - current_lane->length();
    // Loop to get drive in lane range.
    while (true) {
      if (current_s <= 0.0) {
        *s_merge_range_min = 0.0;
        break;
      }
      if (!IsLeftBoundaryBrokenLine(*current_lane, psmm)) {
        s_merge_range_min->reset();
        s_merge_range_max->reset();
        return;
      }
      const auto& current_incomming = current_lane->incoming_lanes();
      const auto& merging_incomming = right_lane->incoming_lanes();
      if (current_incomming.empty() || merging_incomming.empty()) {
        *s_merge_range_min = current_s;
        break;
      }
      const auto current_incomming_lane_info =
          psmm.FindLaneInfoOrNull(current_incomming.front());
      const bool driving_in_lane_cut =
          current_incomming_lane_info == nullptr ||
          current_incomming_lane_info->lane_neighbors_on_right.empty() ||
          current_incomming_lane_info->lane_neighbors_on_right.front()
                  .other_id != merging_incomming.front();
      if (driving_in_lane_cut) {
        *s_merge_range_min = current_s;
        break;
      }
      right_lane = psmm.FindLaneInfoOrNull(merging_incomming.front());
      if (right_lane == nullptr) {
        *s_merge_range_min = current_s;
        break;
      }
      current_lane = current_incomming_lane_info;
      current_s -= current_lane->length();
    }
  }
}

}  // namespace

StationarySpacetimePlannerObjectTrajectoriesFinder::
    StationarySpacetimePlannerObjectTrajectoriesFinder(
        const PlannerSemanticMapManager* psmm,
        const mapping::LanePath& lane_path) {
  // BANDAID(runbing): This is a hack to identify a gate boom barrier.
  constexpr double kForwardDistance = 50.0;      // m.
  constexpr double kBoomBarrierBoxLength = 1.0;  // m.
  constexpr double kBoomBarrierBoxWidth = 2.0;   // m.
  const auto lanes_info = GetLanesInfoContinueIfNotFound(
      *psmm, lane_path.BeforeArclength(kForwardDistance));
  for (const auto* lane_info : lanes_info) {
    if (lane_info->endpoint_toll) {
      QCHECK_GE(lane_info->points_smooth.size(), 2);
      const auto& points = lane_info->points_smooth;
      const Vec2d end_vec(points.back() - points[points.size() - 2]);
      bool_barrier_box_or_ = Box2d(points.back(), end_vec.FastAngle(),
                                   kBoomBarrierBoxLength, kBoomBarrierBoxWidth);
    }
  }
}

SpacetimePlannerObjectTrajectoryReason::Type
StationarySpacetimePlannerObjectTrajectoriesFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  if (traj.is_stationary()) {
    if (!bool_barrier_box_or_.has_value() ||
        !traj.contour().HasOverlap(*bool_barrier_box_or_)) {
      return SpacetimePlannerObjectTrajectoryReason::STATIONARY;
    }
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  return SpacetimePlannerObjectTrajectoryReason::NONE;
}

FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder::
    FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder(
        const Box2d& av_box, const DrivePassage* drive_passage,
        const PathSlBoundary* sl_boundary, double av_speed,
        const SpacetimePlannerObjectTrajectoriesProto* prev_st_trajs,
        const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
        const VehicleGeometryParamsProto* veh_geo)
    : av_box_(av_box),
      drive_passage_(QCHECK_NOTNULL(drive_passage)),
      path_sl_boundary_(QCHECK_NOTNULL(sl_boundary)),
      veh_geo_(QCHECK_NOTNULL(veh_geo)),
      av_speed_(av_speed),
      time_aligned_prev_traj_(time_aligned_prev_traj) {
  QCHECK_NOTNULL(time_aligned_prev_traj);
  const auto av_sl_box_or = drive_passage_->QueryFrenetBoxAt(av_box_);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = *av_sl_box_or;
  }
  for (const auto& st_traj_proto : prev_st_trajs->trajectory()) {
    prev_st_planner_obj_id_.insert(st_traj_proto.id());
  }
}

SpacetimePlannerObjectTrajectoryReason::Type
FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  VLOG(2) << "Find trajectory " << traj.traj_id()
          << " by front-side spacetime planner object trajectory finder.";

  if (!av_sl_box_.has_value()) {
    VLOG(2) << "AV box can't be mapped on drive passage, skip.";
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }

  // Don't consider stationary trajectories.
  if (traj.is_stationary()) {
    VLOG(2) << "Trajectory is stationary, skip.";
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }

  // Don't consider trajectories whose current position is largely behind AV.
  const auto& states = traj.states();
  const bool prev_st_planner_obj = prev_st_planner_obj_id_.contains(
      traj.planner_object().is_sim_agent() ? traj.planner_object().base_id()
                                           : traj.planner_object().id());

  if (!IsFrontTrajectory(*drive_passage_, *av_sl_box_, states[0].contour,
                         *veh_geo_, prev_st_planner_obj)) {
    VLOG(2) << "Trajectory is not in front, skip.";
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }

  // Don't consider trajectories which don't leave a comfortable nudge space
  // for AV.
  const double lat_nudge_dist = prev_st_planner_obj
                                    ? kHysteresisLateralDistanceNudgeMovingObs
                                    : kLateralDistanceNudgeMovingObs;
  if (!IsComfortablyNudgeableTrajectory(
          *drive_passage_, *time_aligned_prev_traj_, *path_sl_boundary_,
          av_box_, *av_sl_box_, av_speed_, veh_geo_->width(), traj,
          lat_nudge_dist, *veh_geo_, kComfortableNudgeCheckTime,
          kComfortableNudgeLatSpeedCheckTime, &is_prev_traj_effective_,
          &time_aligned_prev_traj_boxes_)) {
    VLOG(2) << "Trajectory can't be nudged comfortably.";
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }

  return SpacetimePlannerObjectTrajectoryReason::SIDE;
}

DrivingInSpacetimePlannerObjectTrajectoriesFinder::
    DrivingInSpacetimePlannerObjectTrajectoriesFinder(
        const Box2d& av_box, double av_speed,
        const LaneChangeStateProto* lane_change_state,
        const PlannerSemanticMapManager* psmm,
        const DrivePassage* drive_passage)
    : drive_passage_(QCHECK_NOTNULL(drive_passage)), av_speed_(av_speed) {
  // Recognize drive in lane range on drive passage. Using merging lane as drive
  // in lane.
  if (lane_change_state->stage() != LCS_NONE) return;
  const auto av_sl_box_or = drive_passage_->QueryFrenetBoxAt(av_box);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = *av_sl_box_or;
  }
  const auto& last_real_station =
      drive_passage_->station(drive_passage_->last_real_station_index());
  const auto current_speed_limit = drive_passage_->QuerySpeedLimitAtS(0.0);
  if (!current_speed_limit.ok()) return;
  constexpr double kDriveInLookAheadTime = 5.0;  // s
  const double merge_look_ahead_s =
      std::min(last_real_station.accumulated_s(),
               *current_speed_limit * kDriveInLookAheadTime);
  std::optional<Station> start_dp_station;
  for (const auto& station : drive_passage_->stations()) {
    if (station.accumulated_s() >= 0.0 &&
        station.accumulated_s() < merge_look_ahead_s) {
      if (!start_dp_station.has_value()) {
        start_dp_station = station;
      }
      const auto lane_info = psmm->FindLaneInfoOrNull(station.lane_id());
      if (lane_info != nullptr) {
        if (!lane_info->lane_neighbors_on_right.empty()) {
          const auto right_first_lane_neighbor_ptr = psmm->FindLaneInfoOrNull(
              lane_info->lane_neighbors_on_right.front().other_id);
          if (right_first_lane_neighbor_ptr != nullptr &&
              right_first_lane_neighbor_ptr->IsMerging()) {
            // Make sure current lane and right lane have the same outgoing
            // lane.
            const auto& current_outgoing = lane_info->outgoing_lanes();
            const auto& merging_outgoint =
                right_first_lane_neighbor_ptr->outgoing_lanes();
            if (!current_outgoing.empty() && !merging_outgoint.empty() &&
                current_outgoing.front() == merging_outgoint.front()) {
              const auto outgoing_lane_ptr =
                  psmm->FindLaneInfoOrNull(current_outgoing.front());
              if (outgoing_lane_ptr == nullptr) {
                return;
              }
              if (!IsAllIncomingLanesStraight(*outgoing_lane_ptr, *psmm)) {
                return;
              }
              // If right lane of current lane is merging, compute drive in s
              // range on this lane.
              ComputeDriveInSRange(*lane_info, *right_first_lane_neighbor_ptr,
                                   start_dp_station, station,
                                   drive_passage->end_s(), *psmm,
                                   &s_merge_range_min_, &s_merge_range_max_);
              break;
            }
          }
        }
      }
    }
  }
}

SpacetimePlannerObjectTrajectoryReason::Type
DrivingInSpacetimePlannerObjectTrajectoriesFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  if (traj.object_type() != ObjectType::OT_VEHICLE &&
      traj.object_type() != ObjectType::OT_LARGE_VEHICLE) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  if (av_sl_box_.has_value() && s_merge_range_min_.has_value() &&
      s_merge_range_max_.has_value()) {
    const auto object_frenet_box =
        drive_passage_->QueryFrenetBoxAtContour(traj.contour());
    if (!object_frenet_box.ok()) {
      return SpacetimePlannerObjectTrajectoryReason::NONE;
    }
    constexpr double kLookAheadTime = 2.0;  // s.
    const double look_forward_dist =
        std::max(0.0, (av_speed_ - traj.pose().v()) * kLookAheadTime);
    if (object_frenet_box->s_min < (av_sl_box_->s_max + look_forward_dist) &&
        object_frenet_box->s_max > *s_merge_range_min_ &&
        object_frenet_box->s_min < *s_merge_range_max_ &&
        object_frenet_box->l_max < 0.0) {
      return SpacetimePlannerObjectTrajectoryReason::DRIVE_IN;
    }
  }
  return SpacetimePlannerObjectTrajectoryReason::NONE;
}

DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder::
    DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder(
        const Box2d& av_box, const DrivePassage* drive_passage,
        double av_velocity)
    : av_box_(av_box),
      drive_passage_(drive_passage),
      av_tangent_(av_box.tangent()),
      av_velocity_(av_velocity) {
  auto av_sl_box_or = drive_passage_->QueryFrenetBoxAt(av_box);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = *av_sl_box_or;
  }
}
SpacetimePlannerObjectTrajectoryReason::Type
DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  if (!av_sl_box_.has_value()) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  // Don't consider stationary object here.
  if (traj.is_stationary()) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  // Don't consider objects much faster along AV heading direction.
  constexpr double kRelSpeedThres = 3.5;  // m/s.
  const Vec2d obj_v =
      traj.planner_object().pose().v() *
      Vec2d::FastUnitFromAngle(traj.planner_object().pose().theta());
  if (obj_v.Dot(av_tangent_) - av_velocity_ > kRelSpeedThres) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  if (av_box_.DistanceTo(traj.states()[0].box) > kCloseMovingObjThreshold) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  if (IsFrontOrSideObjectForEmergencySituation(*drive_passage_, *av_sl_box_,
                                               traj.states()[0].box)) {
    return SpacetimePlannerObjectTrajectoryReason::EMERGENCY_AVOIDANCE;
  }
  return SpacetimePlannerObjectTrajectoryReason::NONE;
}

FrontMovingSpacetimePlannerObjectTrajectoriesFinder::
    FrontMovingSpacetimePlannerObjectTrajectoriesFinder(
        const DrivePassage* drive_passage,
        const ApolloTrajectoryPointProto* plan_start_point, double av_length)
    : drive_passage_(drive_passage),
      plan_start_point_(plan_start_point),
      av_length_(av_length) {
  const Vec2d av_pos(plan_start_point_->path_point().x(),
                     plan_start_point_->path_point().y());
  auto av_sl_or = drive_passage_->QueryFrenetCoordinateAt(av_pos);
  QCHECK(av_sl_or.ok());
  av_sl_ = *av_sl_or;
}

SpacetimePlannerObjectTrajectoryReason::Type
FrontMovingSpacetimePlannerObjectTrajectoriesFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  // Don't consider stationary obstacle here.
  if (traj.is_stationary()) {
    return SpacetimePlannerObjectTrajectoryReason::NONE;
  }
  if (IsFrontOrSideObject(*drive_passage_, av_sl_, av_length_,
                          traj.states()[0].box)) {
    return SpacetimePlannerObjectTrajectoryReason::FRONT;
  }
  return SpacetimePlannerObjectTrajectoryReason::NONE;
}

}  // namespace planner
}  // namespace qcraft
