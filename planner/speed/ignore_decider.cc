#include "onboard/planner/speed/ignore_decider.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/decider/pre_brake_util.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEpsilon = 0.01;

std::vector<StPoint> GenerateAvEmergencyStopStPoints(double init_s,
                                                     double init_v,
                                                     double init_a,
                                                     int traj_steps) {
  QCHECK_GE(init_v, 0.0);
  constexpr double kMaxInitAccel = -0.6;           // m/s^2.
  constexpr double kMinEmergencyStopAccel = -4.0;  // m/s^2.
  constexpr double kEmergencyStopJerk = -2.0;      // m/s^3.
  constexpr double kDeltaAccel =
      kEmergencyStopJerk * kTrajectoryTimeStep;  // m/s^2.
  // In line with path resoluation but can differ from it.
  constexpr double kCheckCollisionTime = 4.0;  // s.
  double curr_s = init_s;
  double curr_v = init_v;
  double curr_a = std::min(init_a, kMaxInitAccel);
  std::vector<StPoint> emergency_stop_points;
  emergency_stop_points.reserve(traj_steps);
  double t = 0.0;
  for (int i = 0; i < traj_steps && t < kCheckCollisionTime;
       ++i, t += kTrajectoryTimeStep) {
    emergency_stop_points.emplace_back(curr_s, t);
    curr_s += std::max(0.0, curr_v * kTrajectoryTimeStep +
                                0.5 * curr_a * Sqr(kTrajectoryTimeStep));
    curr_v = std::max(0.0, curr_v + curr_a * kTrajectoryTimeStep);
    curr_a = std::max(kMinEmergencyStopAccel, curr_a + kDeltaAccel);
  }
  return emergency_stop_points;
}

bool IgnoreHitAvEmergencyStopTrajectoryStBoundary(
    const std::vector<StPoint>& emergency_stop_points,
    const StBoundary& st_boundary) {
  // Don't ignore st-boundaries that would hit path in a short time.
  constexpr double kShortTime = 1.0;        // s.
  constexpr double kShortTimeForPed = 0.8;  // s.
  const double short_time =
      st_boundary.object_type() == StBoundaryProto::PEDESTRIAN
          ? kShortTimeForPed
          : kShortTime;
  if (st_boundary.min_t() < short_time) return false;

  constexpr double kCheckCollisionSBuffer = 0.5;  // m.
  bool st_boundary_passed = false;
  const auto start_iter = std::lower_bound(
      emergency_stop_points.begin(), emergency_stop_points.end(),
      st_boundary.min_t(),
      [](const StPoint& point, double t) { return point.t() < t; });
  for (auto it = start_iter; it < emergency_stop_points.end(); ++it) {
    const auto& st_point = *it;
    const double curr_s = st_point.s();
    const double t = st_point.t();
    const auto s_range = st_boundary.GetBoundarySRange(t);
    if (!s_range.has_value()) {
      if (!st_boundary_passed) {
        continue;
      } else {
        return false;
      }
    }
    st_boundary_passed = true;
    if (curr_s > s_range->second + kCheckCollisionSBuffer) {
      VLOG(2) << "St-boundary " << st_boundary.id()
              << " is ignored because AV would hit it at t = " << t
              << " s = " << curr_s << " by emergency stop.";
      return true;
    }
  }
  return false;
}

bool IsCollisionHead(const VehicleShapeBasePtr& av_shape,
                     const Polygon2d& obj_contour, const PathPoint& path_point,
                     const VehicleGeometryParamsProto& vehicle_geometry_params,
                     double lat_buffer, double lon_buffer) {
  const Polygon2d av_polygon = Polygon2d(
      av_shape->GetCornersWithBufferCounterClockwise(lat_buffer, lon_buffer),
      /*is_convex=*/true);
  Polygon2d overlap_polygon;
  if (!av_polygon.ComputeOverlap(obj_contour, &overlap_polygon)) {
    return false;
  } else {
    const Vec2d av_tangent = Vec2d::FastUnitFromAngle(path_point.theta());
    Vec2d front, back;
    overlap_polygon.ExtremePoints(av_tangent, &back, &front);
    constexpr double kVehicleHeadRatio = 0.25;
    const double head_lower_limit =
        vehicle_geometry_params.front_edge_to_center() -
        vehicle_geometry_params.length() * kVehicleHeadRatio + lon_buffer;

    if ((front - Vec2d(path_point.x(), path_point.y())).dot(av_tangent) >
        head_lower_limit) {
      return true;
    }
  }
  return false;
}

bool IsCollisionMirrors(
    const VehicleShapeBasePtr& av_shape, const Polygon2d& obj_contour,
    const SpacetimeObjectTrajectory& st_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const double buffer = st_traj.required_lateral_gap();
  const auto [min_mirror_height, max_mirror_height] =
      ComputeMinMaxMirrorAverageHeight(vehicle_geometry_params);
  const bool consider_mirrors =
      IsConsiderMirrorObject(st_traj.planner_object().object_proto(),
                             min_mirror_height, max_mirror_height);
  return consider_mirrors &&
         (av_shape->LeftMirrorHasOverlapWithBuffer(obj_contour, buffer) ||
          av_shape->RightMirrorHasOverlapWithBuffer(obj_contour, buffer));
}

bool IgnoreHitAvCurrentPositionStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const PathPoint& current_path_point, const StBoundary& st_boundary,
    double current_v) {
  const double first_overlap_time = st_boundary.bottom_left_point().t();

  constexpr double kQuickStopMaxVel = 1.2;  // m/s.
  constexpr double kNudgeMaxTime = 2.5;     // s.
  // If AV could fully stop in a short distance and the object will invade AV's
  // path in a short time, the object may want to nudge and overtake AV and we
  // need check the first overlap area to determine if this object can be
  // ignored safely.
  const bool has_nudge_intention =
      current_v < kQuickStopMaxVel && first_overlap_time <= kNudgeMaxTime;

  if (st_boundary.bottom_left_point().s() <= kEpsilon &&
      first_overlap_time >= kEpsilon && !has_nudge_intention) {
    return true;
  } else {
    QCHECK(st_boundary.traj_id().has_value());
    const auto& traj_id = *st_boundary.traj_id();
    const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

    const auto& overlap_infos = st_boundary.overlap_infos();
    QCHECK(!overlap_infos.empty());
    constexpr double kCheckTime = 0.5;  // s.
    const double check_time_limit = first_overlap_time + kCheckTime;
    for (auto& overlap_info : overlap_infos) {
      if (overlap_info.time > check_time_limit) {
        break;
      }
      if (overlap_info.av_start_idx == 0) {
        // NOTE: Buffer may be time-varying in the future.
        const bool is_stationary_object = traj->is_stationary();
        const double buffer =
            is_stationary_object ? 0.0 : traj->required_lateral_gap();
        const auto& obj_contour = traj->states()[overlap_info.obj_idx].contour;
        if (is_stationary_object) {
          return av_shapes[0]->MainBodyHasOverlapWithBuffer(obj_contour, buffer,
                                                            buffer) &&
                 !IsCollisionHead(av_shapes[0], obj_contour, current_path_point,
                                  vehicle_geometry_params, buffer, buffer);
        } else {
          return !IsCollisionMirrors(av_shapes[0], obj_contour, *traj,
                                     vehicle_geometry_params) &&
                 !IsCollisionHead(av_shapes[0], obj_contour, current_path_point,
                                  vehicle_geometry_params, buffer, buffer);
        }
      }
    }
  }
  return false;
}

bool IgnoreBackCutInStBoundary(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PathPointSemantic& current_path_semantic,
    bool on_left_turn_waiting_lane, const StBoundary& st_boundary) {
  if (!st_boundary.overlap_meta().has_value()) return false;
  const auto current_lane_semantic = current_path_semantic.lane_semantic;
  if (current_lane_semantic == LaneSemantic::NONE ||
      current_lane_semantic == LaneSemantic::INTERSECTION_UTURN) {
    return false;
  }
  if (st_boundary.overlap_meta()->source() !=
          StOverlapMetaProto::OBJECT_CUTIN &&
      st_boundary.overlap_meta()->source() != StOverlapMetaProto::LANE_MERGE &&
      st_boundary.overlap_meta()->source() != StOverlapMetaProto::LANE_CROSS) {
    return false;
  }
  if (!st_boundary.overlap_meta()->has_front_most_projection_distance()) {
    return false;
  }
  const auto& front_most_projection_distance =
      st_boundary.overlap_meta()->front_most_projection_distance();
  // To imitate human drivers, objects that not pass the center of AV will be
  // ignored.
  constexpr double kDriverAwarenessFactor = 0.5;
  const double driver_awareness_area =
      vehicle_geometry_params.front_edge_to_center() -
      vehicle_geometry_params.length() * kDriverAwarenessFactor;
  const bool behind_current_path_point_center =
      front_most_projection_distance < driver_awareness_area;

  if (current_lane_semantic == LaneSemantic::ROAD ||
      current_lane_semantic == LaneSemantic::INTERSECTION_STRAIGHT) {
    if (st_boundary.overlap_meta()->source() !=
        StOverlapMetaProto::OBJECT_CUTIN) {
      return false;
    }
    return behind_current_path_point_center;
  } else if (current_lane_semantic == LaneSemantic::INTERSECTION_LEFT_TURN ||
             current_lane_semantic == LaneSemantic::INTERSECTION_RIGHT_TURN) {
    // If AV is turning left/right at intersection, check if object current
    // contour is behind current path point. If so, ignore back cut-in object.
    // Not valid for LOW priority LANE-CROSS/LANE-MERGE & AV merging & crossing
    // straight lane.
    if ((st_boundary.overlap_meta()->source() !=
             StOverlapMetaProto::OBJECT_CUTIN &&
         st_boundary.overlap_meta()->priority() == StOverlapMetaProto::LOW) ||
        (st_boundary.overlap_meta()->has_obj_lane_direction() &&
         st_boundary.overlap_meta()->obj_lane_direction() ==
             mapping::LaneProto::STRAIGHT)) {
      return false;
    }
    if (behind_current_path_point_center) return true;
    // For left-turn waiting area.
    if (on_left_turn_waiting_lane) {
      // If AV is turning to non-innermost lane or on left turn waiting lane, we
      // ignore those behind AV front.
      const bool behind_current_path_point_front =
          front_most_projection_distance <
          vehicle_geometry_params.front_edge_to_center();
      if (behind_current_path_point_front) {
        return true;
      }
    }
    // Other cases, don't ignore front cut-in objects.
    return false;
  }

  QLOG(FATAL) << "Should not be here.";
  return false;
}

bool IgnoreOnRoadParallelCutInStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const FrenetBox& av_frenet_box, bool av_in_drive_passage_lane,
    const DrivePassage& drive_passage, double current_v,
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& upcoming_merge_lane_ids,
    bool ignore_late_parallel_cut_in_vehicle, const StBoundary& st_boundary) {
  FUNC_QTRACE();
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
      StOverlapMetaProto::OBJECT_CUTIN) {
    return false;
  }
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE &&
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }
  // Check if AV is on road and entirely enclosed by drive passage lane
  // boundaries.
  const auto& overlap_infos = st_boundary.overlap_infos();
  QCHECK(!overlap_infos.empty());
  const auto& fo_info = overlap_infos.front();

  if (path_semantics[fo_info.av_start_idx].lane_semantic !=
      LaneSemantic::ROAD) {
    return false;
  }
  if (!av_in_drive_passage_lane) {
    return false;
  }

  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto obj_frenet_box_or =
      drive_passage.QueryFrenetBoxAtContour(traj->contour());
  if (!obj_frenet_box_or.ok()) {
    return false;
  }
  const auto& obj_frenet_box = *obj_frenet_box_or;

  constexpr double kLateCutInTimeThres = 3.5;  // s.
  if (!ignore_late_parallel_cut_in_vehicle ||
      st_boundary.object_type() != StBoundaryProto::VEHICLE ||
      st_boundary.min_t() < kLateCutInTimeThres) {
    // Condition 1): Object s_max is among a relative speed related AV frenet s
    // range. The assumption here is: if object is faster than AV, it would
    // possibly cut in AV when not fully surpass it; otherwise, it would not cut
    // in AV if not fully supass it. The parameter here can be further fine
    // tuned.
    const double object_length = obj_frenet_box.s_max - obj_frenet_box.s_min;
    // TODO(tianbo): Use different params for vehicle & cyclist.
    const PiecewiseLinearFunction rel_speed_surpass_dist_plf(
        std::vector<double>({-2.0, 0.0, 2.0}),
        std::vector<double>({0.8 * object_length, 0.3 * object_length, 0.0}));
    constexpr double kRearSRangeOffset = 0.5;  // m.
    if (obj_frenet_box.s_max < av_frenet_box.s_min - kRearSRangeOffset ||
        obj_frenet_box.s_max >
            av_frenet_box.s_max +
                rel_speed_surpass_dist_plf(traj->planner_object().pose().v() -
                                           current_v)) {
      return false;
    }
  }

  // Condition 2): Object heading is around its nearby drive passage center.
  if (!st_boundary.overlap_meta()->has_theta_diff()) return false;
  const auto& theta_diff = st_boundary.overlap_meta()->theta_diff();
  constexpr double kParallelHeadingDiff = 0.17453292519943295;  // 10 degree.
  if (std::abs(theta_diff) > kParallelHeadingDiff) {
    return false;
  }

  // Condition 3): Object is largely out of the lane boundaries that enclose
  // AV.
  const double obj_s_mean = 0.5 * (obj_frenet_box.s_min + obj_frenet_box.s_max);
  const auto obj_s_min_boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(obj_frenet_box.s_min);
  const auto obj_s_mean_boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(obj_s_mean);
  const auto obj_s_max_boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(obj_frenet_box.s_max);
  constexpr double kOutOfLaneOffset = 0.8;  // m.
  const double obj_l_min = obj_frenet_box.l_min + kOutOfLaneOffset;
  const double obj_l_max = obj_frenet_box.l_max - kOutOfLaneOffset;
  const bool left_of_left_boundary =
      obj_s_min_boundary.left.has_value() &&
      obj_s_mean_boundary.left.has_value() &&
      obj_s_max_boundary.left.has_value() &&
      obj_l_min > std::max({obj_s_min_boundary.left->lat_offset,
                            obj_s_mean_boundary.left->lat_offset,
                            obj_s_max_boundary.left->lat_offset});
  const bool right_of_right_boundary =
      obj_s_min_boundary.right.has_value() &&
      obj_s_mean_boundary.right.has_value() &&
      obj_s_max_boundary.right.has_value() &&
      obj_l_max < std::min({obj_s_min_boundary.right->lat_offset,
                            obj_s_mean_boundary.right->lat_offset,
                            obj_s_max_boundary.right->lat_offset});
  const bool out_of_lane = left_of_left_boundary || right_of_right_boundary;
  if (!out_of_lane) return false;

  // Condition 4): Object is not near an upcoming merge lane.
  double fraction = 0.0;
  double min_dist = 0.0;
  bool obj_near_upcoming_merge_lane = false;
  constexpr double kNearLaneDist = 2.0;  // m.
  for (const auto& merge_lane_id : upcoming_merge_lane_ids) {
    if (psmm.GetLaneProjection(traj->pose().pos(), merge_lane_id, &fraction,
                               /*point=*/nullptr, &min_dist,
                               /*segment=*/nullptr) &&
        min_dist < kNearLaneDist) {
      obj_near_upcoming_merge_lane = true;
      break;
    }
  }
  if (obj_near_upcoming_merge_lane) return false;

  return true;
}

bool IgnoreCutInStBoundaryOutOfLateralBound(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    bool av_in_drive_passage_lane, const DrivePassage& drive_passage,
    const StBoundary& st_boundary) {
  FUNC_QTRACE();
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
      StOverlapMetaProto::OBJECT_CUTIN) {
    return false;
  }
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE &&
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }
  if (!av_in_drive_passage_lane) {
    return false;
  }

  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto obj_frenet_center =
      drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
          traj->planner_object().bounding_box().center());
  if (!obj_frenet_center.ok()) {
    return false;
  }

  // Appoximately witdh of two and a half lanes.
  constexpr double kLateralBoundThres = 7.0;  // m.
  if (std::abs(obj_frenet_center->l) < kLateralBoundThres) {
    return false;
  }

  const auto& obj_proj_station =
      drive_passage.FindNearestStationAtS(obj_frenet_center->s);
  return !obj_proj_station.is_in_intersection();
}

bool IgnoreReverseDrivingStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const StBoundary& st_boundary) {
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
          StOverlapMetaProto::OBJECT_CUTIN &&
      st_boundary.overlap_meta()->source() !=
          StOverlapMetaProto::UNKNOWN_SOURCE) {
    return false;
  }
  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return false;
  }
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE &&
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }

  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  if (traj->planner_object().pose().v() >= -kEpsilon &&
      !traj->trajectory().is_reversed()) {
    return false;
  }
  const auto& overlap_infos = st_boundary.overlap_infos();
  QCHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      traj->states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kParallelDrivingThreshold = M_PI / 6.0;  // 30 deg.
  if (std::abs(NormalizeAngle(first_overlap_obj_heading -
                              first_overlap_av_middle_heading)) >
      kParallelDrivingThreshold) {
    return false;
  }
  return true;
}

inline bool IsCutInStBoundary(const StBoundary& st_boundary) {
  constexpr double kMinOverlapTimeThres = 1.0;  // s.
  return st_boundary.bottom_left_point().t() > kMinOverlapTimeThres;
}

bool IsOncomingStBoundary(const SpacetimeTrajectoryManager& st_traj_mgr,
                          const DiscretizedPath& path,
                          const StBoundary& st_boundary,
                          double oncoming_angle_threshold) {
  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return false;
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const auto& first_overlap_info = overlap_infos.front();
  const int av_mid_idx =
      (first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) / 2;
  const auto* first_overlap_obj_point =
      traj->states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading = path[av_mid_idx].theta();
  return std::abs(NormalizeAngle(first_overlap_obj_heading -
                                 first_overlap_av_middle_heading)) >
         oncoming_angle_threshold;
}

bool IgnoreStBoundaryOutOfRealRange(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage& drive_passage, const DiscretizedPath& path,
    const StBoundary& st_boundary, double current_v) {
  const auto& overlap_infos = st_boundary.overlap_infos();
  if (overlap_infos.empty()) {
    return false;
  }

  constexpr double kOnComingThreshold = d2r(160.0);  // 160 deg.
  if (!IsCutInStBoundary(st_boundary) &&
      !IsOncomingStBoundary(st_traj_mgr, path, st_boundary,
                            kOnComingThreshold)) {
    return false;
  }

  constexpr double kMinFilterTimeHeadway = 2.0;  // s.
  constexpr double kMinEgoVel = 2.0;             // m/s.
  if (const double min_filter_dist =
          kMinFilterTimeHeadway * std::max(kMinEgoVel, current_v);
      st_boundary.bottom_left_point().s() < min_filter_dist) {
    return false;
  }

  const auto& first_overlap_info = overlap_infos.front();
  const auto& first_overlap_path_point =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2];
  const auto first_overlap_station_idx = drive_passage.FindNearestStationIndex(
      Vec2d(first_overlap_path_point.x(), first_overlap_path_point.y()));

  const auto& last_real_station_idx = drive_passage.last_real_station_index();
  return first_overlap_station_idx > last_real_station_idx;
}

inline bool IgnoreLateCutinStBoundary(const StBoundary& st_boundary) {
  const auto& overlap_infos = st_boundary.overlap_infos();
  if (overlap_infos.empty()) {
    return false;
  }

  if (st_boundary.object_type() == StBoundaryProto::PEDESTRIAN) {
    constexpr double kPedestrianCutinTimeThreshold = 4.0;  // s.
    return st_boundary.min_t() > kPedestrianCutinTimeThreshold;
  }
  constexpr double kCutinTimeThreshold = 2.5;  // s.
  return st_boundary.min_t() > kCutinTimeThreshold;
}

bool IgnoreOncomingStBoundaryNotOnPath(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const SegmentMatcherKdtree& path_kd_tree, const StBoundary& st_boundary) {
  const auto& overlap_infos = st_boundary.overlap_infos();
  if (overlap_infos.empty()) {
    return false;
  }

  constexpr double kOnComingThreshold = d2r(100.0);  // 100 deg.
  if (!IsOncomingStBoundary(st_traj_mgr, path, st_boundary,
                            kOnComingThreshold)) {
    return false;
  }

  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
  const Box2d& obj_box = traj->planner_object().bounding_box();
  const auto& obj_contour = traj->planner_object().contour();

  constexpr double kSearchBuffer = 0.2;  // m.
  const double obj_radius = obj_box.diagonal() * 0.5;
  const double ego_radius =
      Hypot(std::max(vehicle_geometry_params.front_edge_to_center(),
                     vehicle_geometry_params.back_edge_to_center()),
            vehicle_geometry_params.right_edge_to_center());
  const double search_radius = obj_radius + ego_radius + kSearchBuffer;
  const auto indices = path_kd_tree.GetSegmentIndexInRadius(
      obj_box.center().x(), obj_box.center().y(), search_radius);
  for (const auto idx : indices) {
    if (av_shapes[idx]->HasOverlapWithBuffer(obj_contour, /*lat_buffer=*/0.0,
                                             /*lon_buffer=*/0.0,
                                             /*consider_mirrors=*/false)) {
      return false;
    }
  }
  return true;
}

bool IgnoreUncomfortableBrakeOncomingStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    double current_v, const StBoundary& st_boundary) {
  // The method to identify uncomfortable brake oncoming st-boundary here is
  // much similar with that in ModifyOncomingStBoundary of
  // pre_st_boundary_modifier but the parameters are a little different.
  // TODO(renjie): Refactor this part to reduce duplicate code with
  // ModifyOncomingStBoundary,
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
      StOverlapMetaProto::OBJECT_CUTIN) {
    return false;
  }
  constexpr double kMinTimeLimit = 0.5;  // s.
  if (st_boundary.bottom_left_point().t() < kMinTimeLimit) {
    return false;
  }

  constexpr double kOnComingThreshold = d2r(170.0);  // 170 deg.
  if (!IsOncomingStBoundary(st_traj_mgr, path, st_boundary,
                            kOnComingThreshold)) {
    return false;
  }

  // Only ignore the oncoming prediction if it would cause uncomfortable brake.
  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (st_boundary.bottom_right_point().s() > const_speed_s) {
    // No brake is needed.
    return false;
  }
  const double estimated_av_decel =
      2.0 * (const_speed_s - st_boundary.bottom_right_point().s()) /
      Sqr(st_boundary.bottom_right_point().t());
  constexpr double kUncomfortableDecel = 0.3;  // m/s^2.
  if (estimated_av_decel < kUncomfortableDecel) {
    return false;
  }
  return true;
}

bool IgnoreOncomingStBoundaryWithoutObviousCutInIntention(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const DrivePassage& drive_passage, double current_v,
    const StBoundary& st_boundary) {
  // Ignore the oncoming objects without obvious intention to cut in or crossing
  // AV's trajectory.
  if (!st_boundary.overlap_meta().has_value()) return false;
  if (st_boundary.overlap_meta()->source() !=
          StOverlapMetaProto::OBJECT_CUTIN &&
      st_boundary.overlap_meta()->source() != StOverlapMetaProto::LANE_CROSS) {
    return false;
  }
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE &&
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  QCHECK(!overlap_infos.empty());
  const auto& fo_info = overlap_infos.front();

  const auto fo_lane_semantic =
      path_semantics[fo_info.av_start_idx].lane_semantic;
  if (fo_lane_semantic != LaneSemantic::ROAD &&
      fo_lane_semantic != LaneSemantic::INTERSECTION_STRAIGHT) {
    return false;
  }

  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return false;
  }
  constexpr double kMinTimeLimit = 0.1;  // s.
  if (st_boundary.bottom_left_point().t() < kMinTimeLimit) {
    return false;
  }

  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = *st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

  // Here we use object info from planner object to recognize current object's
  // intention.
  const Vec2d& obj_pos = traj->planner_object().pose().pos();
  const auto obj_frenet_coord = drive_passage.QueryFrenetCoordinateAt(obj_pos);
  if (!obj_frenet_coord.ok()) {
    return false;
  }
  const auto obj_frenet_theta =
      drive_passage.QueryTangentAngleAtS(obj_frenet_coord->s);
  if (!obj_frenet_theta.ok()) {
    return false;
  }
  const double ref_theta = *obj_frenet_theta;
  const double theta_diff =
      NormalizeAngle(traj->planner_object().pose().theta() - ref_theta);

  // Larger threshold for cyclist.
  constexpr double kOncomingThresholdVehicle = 3.0543261909900768;  // 175 deg.
  constexpr double kOncomingThresholdCyclist = 2.9670597283903604;  // 170 deg.
  const double oncoming_threshold =
      st_boundary.object_type() == StBoundaryProto::VEHICLE
          ? kOncomingThresholdVehicle
          : kOncomingThresholdCyclist;
  if (std::abs(theta_diff) < oncoming_threshold) {
    return false;
  }

  // Only ignore the oncoming prediction if it would cause uncomfortable brake.
  constexpr double kFollowDistance = 3.0;  // m.
  const double brake_dis = st_boundary.min_s() - kFollowDistance;
  const double brake_time = st_boundary.bottom_right_point().t();
  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (brake_dis < kEpsilon) {
    // Not enough distance to brake.
    return true;
  }
  if (brake_dis > const_speed_s) {
    // No brake is needed.
    return false;
  }
  const bool brake_to_stop =
      0.5 * brake_dis / std::max(current_v, kEpsilon) < brake_time;
  const double estimated_av_decel =
      brake_to_stop
          ? 0.5 * Sqr(current_v) / brake_dis
          : 2.0 * (const_speed_s - st_boundary.bottom_right_point().s()) /
                Sqr(st_boundary.bottom_right_point().t());
  constexpr double kUncomfortableDecel = 1.0;  // m/s^2.
  if (estimated_av_decel < kUncomfortableDecel) {
    return false;
  }
  return true;
}

std::optional<VtSpeedLimit> MakeOncomingPreBrakeDecisionForStBoundary(
    double current_v, double max_v, double time_step, int step_num,
    const StBoundaryWithDecision& st_boundary_wd) {
  constexpr double kMinTimeLimit = 0.1;  // s.
  constexpr double kMaxTimeLimit = 3.5;  // s.
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  const double pre_brake_time = st_boundary.bottom_left_point().t();
  if (pre_brake_time < kMinTimeLimit || pre_brake_time > kMaxTimeLimit) {
    return std::nullopt;
  }

  constexpr double kEmergencyStopAccel = -3.0;  // m/s^2.
  const double s_limit_lower = current_v * pre_brake_time +
                               0.5 * kEmergencyStopAccel * Sqr(pre_brake_time);
  if (st_boundary.bottom_left_point().s() < s_limit_lower) {
    return std::nullopt;
  }

  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id();
  constexpr double kMildDecel = -0.6;  // m/s^2.
  constexpr double kMinVel = 4.0;      // m/s.
  constexpr double kExtraTime = 1.0;   // s.
  const auto info = absl::StrCat("Pre brake for oncoming object ", *traj_id);
  return GenerateConstAccSpeedLimit(
      /*start_t=*/0.0, pre_brake_time + kExtraTime, current_v, kMinVel, max_v,
      kMildDecel, time_step, step_num, info);
}

inline void MergeOptionalSpeedLimit(std::optional<VtSpeedLimit> source,
                                    std::optional<VtSpeedLimit>* target) {
  if (source.has_value()) {
    if (target->has_value()) {
      MergeVtSpeedLimit(source.value(), &target->value());
    } else {
      *target = std::move(source);
    }
  }
}

void MakeIgnoreAndPreBrakeDecisionForStBoundary(
    const SpeedFinderParamsProto::IgnoreDeciderParamsProto& params,
    const DiscretizedPath& path,
    const std::vector<PathPointSemantic>& path_semantics,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage* drive_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const SegmentMatcherKdtree& path_kd_tree, double current_v, double max_v,
    double time_step, double trajectory_steps,
    const PlannerSemanticMapManager& psmm,
    const std::vector<StPoint>& emergency_stop_points,
    bool on_left_turn_waiting_lane,
    const std::optional<FrenetBox>& av_frenet_box,
    bool av_in_drive_passage_lane,
    const std::vector<mapping::ElementId>& upcoming_merge_lane_ids,
    const std::optional<std::string>& nearest_on_path_object_id,
    StBoundaryWithDecision* st_boundary_wd,
    std::optional<VtSpeedLimit>* speed_limit) {
  QCHECK_NOTNULL(st_boundary_wd);

  if (st_boundary_wd->decision_type() != StBoundaryProto::UNKNOWN) {
    return;
  }
  const auto& st_boundary = *st_boundary_wd->raw_st_boundary();
  if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
    return;
  }

  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         StBoundaryWithDecision* st_boundary_wd) {
        st_boundary_wd->set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd->set_decision_reason(StBoundaryProto::IGNORE_DECIDER);
        st_boundary_wd->set_ignore_reason(ignore_reason);
        st_boundary_wd->set_decision_info(decision_info);
      };

  // 1. Ignore st-boundaries hit AV current position.
  if (IgnoreHitAvCurrentPositionStBoundary(st_traj_mgr, vehicle_geometry_params,
                                           av_shapes, path.front(), st_boundary,
                                           current_v)) {
    make_ignore_decision("ignore hit current AV position",
                         StBoundaryProto::HIT_CURRENT_POS, st_boundary_wd);
    return;
  }

  if (st_boundary.is_stationary()) return;

  // 2. Ignore st-boundaries hit AV emergency stop trajectory.
  if (IgnoreHitAvEmergencyStopTrajectoryStBoundary(emergency_stop_points,
                                                   st_boundary)) {
    make_ignore_decision("ignore hit AV emergency stop trajectory",
                         StBoundaryProto::HIT_EMERGENCY_TRAJ, st_boundary_wd);
    return;
  }

  // 3. Ignore back cut-in st-boundaries.
  if (!path_semantics.empty() &&
      IgnoreBackCutInStBoundary(vehicle_geometry_params, path_semantics.front(),
                                on_left_turn_waiting_lane, st_boundary)) {
    make_ignore_decision("ignore back cutin", StBoundaryProto::BACK_CUT_IN,
                         st_boundary_wd);
    return;
  }

  // 4. Ignore on-road parallel cut-in st-boundaries.
  if (!path_semantics.empty() && av_frenet_box.has_value() &&
      drive_passage != nullptr &&
      IgnoreOnRoadParallelCutInStBoundary(
          st_traj_mgr, path_semantics, *av_frenet_box, av_in_drive_passage_lane,
          *drive_passage, current_v, psmm, upcoming_merge_lane_ids,
          params.ignore_late_parallel_cut_in_vehicle(), st_boundary)) {
    make_ignore_decision("ignore on-road parallel cutin",
                         StBoundaryProto::PARALLEL_CUT_IN, st_boundary_wd);
    return;
  }

  // 5. Ignore cut-in objects that out of lateral bound.
  if (drive_passage != nullptr &&
      params.ignore_objects_out_of_lateral_bound() &&
      IgnoreCutInStBoundaryOutOfLateralBound(
          st_traj_mgr, av_in_drive_passage_lane, *drive_passage, st_boundary)) {
    make_ignore_decision("ignore cut-in objects out of lateral bound",
                         StBoundaryProto::OBJECT_OUT_OF_LAT_BOUND,
                         st_boundary_wd);
    return;
  }

  // 6. Ignore cut-in & oncoming objects that overlap out of map range.
  if (drive_passage != nullptr &&
      IgnoreStBoundaryOutOfRealRange(st_traj_mgr, *drive_passage, path,
                                     st_boundary, current_v)) {
    make_ignore_decision("ignore cut-in/oncoming objects out of real range",
                         StBoundaryProto::OUT_OF_REAL_RANGE, st_boundary_wd);
    return;
  }

  // 7. HACK(bo): Ignore all OBJECT_CUTIN & UNKNOWN_SOURCE reverse-driving
  // vehicles/cyclists for vision-only demo.
  const bool is_first_on_path_object =
      st_boundary.object_id().has_value() &&
      nearest_on_path_object_id.has_value() &&
      *st_boundary.object_id() == *nearest_on_path_object_id;
  if (params.ignore_reverse_driving() && !is_first_on_path_object &&
      IgnoreReverseDrivingStBoundary(st_traj_mgr, path, st_boundary)) {
    make_ignore_decision("ignore reverse-driving object",
                         StBoundaryProto::REVERSE_DRIVING, st_boundary_wd);
  }

  // 8. Ignore late cut-in object.
  if (params.ignore_late_cut_in_objects() &&
      IgnoreLateCutinStBoundary(st_boundary)) {
    make_ignore_decision("ignore late cut-in object",
                         StBoundaryProto::LATE_CUT_IN, st_boundary_wd);
    return;
  }

  // 9. Ignore oncoming objects not on path.
  if (params.ignore_oncoming_objects_not_on_path() &&
      IgnoreOncomingStBoundaryNotOnPath(st_traj_mgr, path,
                                        vehicle_geometry_params, av_shapes,
                                        path_kd_tree, st_boundary)) {
    make_ignore_decision("ignore oncoming object not on path",
                         StBoundaryProto::ONCOMING_OBJECT_NOT_ON_PATH,
                         st_boundary_wd);
    return;
  }

  // 10. Ignore uncomfortable brake incoming cut-in st-boundaries.
  if (IgnoreUncomfortableBrakeOncomingStBoundary(st_traj_mgr, path, current_v,
                                                 st_boundary)) {
    make_ignore_decision("ignore uncomfortable brake oncoming",
                         StBoundaryProto::ONCOMING_OBJECT, st_boundary_wd);
    auto speed_limit_opt = MakeOncomingPreBrakeDecisionForStBoundary(
        current_v, max_v, time_step, trajectory_steps, *st_boundary_wd);
    MergeOptionalSpeedLimit(std::move(speed_limit_opt), speed_limit);
    return;
  }

  // 11. Ignore oncoming cut-in/crossing st-boundaries without obvious cut-in
  // intention.
  if (!path_semantics.empty() && drive_passage != nullptr &&
      IgnoreOncomingStBoundaryWithoutObviousCutInIntention(
          st_traj_mgr, path_semantics, *drive_passage, current_v,
          st_boundary)) {
    make_ignore_decision(
        "ignore oncoming objects without obvious cut-in intention",
        StBoundaryProto::ONCOMING_OBJECT, st_boundary_wd);
    auto speed_limit_opt = MakeOncomingPreBrakeDecisionForStBoundary(
        current_v, max_v, time_step, trajectory_steps, *st_boundary_wd);
    MergeOptionalSpeedLimit(std::move(speed_limit_opt), speed_limit);
    return;
  }
}

void MakeIgnoreDecisionForNonNearestStationaryStBoundaries(
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  QCHECK_NOTNULL(st_boundaries_wd);

  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         StBoundaryWithDecision* st_boundary_wd) {
        st_boundary_wd->set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd->set_decision_reason(StBoundaryProto::IGNORE_DECIDER);
        st_boundary_wd->set_ignore_reason(ignore_reason);
        st_boundary_wd->set_decision_info(decision_info);
      };

  StBoundaryWithDecision* nearest_stationary_st_boundary = nullptr;
  for (auto& st_boundary_wd : *st_boundaries_wd) {
    const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (!st_boundary.is_stationary()) continue;
    if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) continue;
    if (nullptr == nearest_stationary_st_boundary) {
      nearest_stationary_st_boundary = &st_boundary_wd;
    } else if (st_boundary.min_s() <
               nearest_stationary_st_boundary->raw_st_boundary()->min_s()) {
      make_ignore_decision("ignore non-nearest stationary st-boundary",
                           StBoundaryProto::NON_NEAREST_STATIONARY,
                           nearest_stationary_st_boundary);
      nearest_stationary_st_boundary = &st_boundary_wd;
    } else {
      make_ignore_decision("ignore non-nearest stationary st-boundary",
                           StBoundaryProto::NON_NEAREST_STATIONARY,
                           &st_boundary_wd);
    }
  }
  return;
}

void MakeIgnoreDecisionForNonNearestOnPathOccludedStBoundaries(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::optional<std::string>& nearest_on_path_stay_object_id,
    const absl::flat_hash_map<std::string, double>& on_path_object_id_s_map,
    double nearest_on_path_stay_object_s,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  QCHECK_NOTNULL(st_boundaries_wd);

  if (!nearest_on_path_stay_object_id.has_value()) {
    return;
  }

  const auto insert_occluded_object_id_to_set =
      [](const SpacetimeTrajectoryManager& st_traj_mgr,
         const StBoundary& st_boundary,
         absl::flat_hash_set<std::string>* occluded_object_id_set) {
        if (ContainsKey(*occluded_object_id_set, *st_boundary.object_id())) {
          return;
        }
        QCHECK(st_boundary.traj_id().has_value());
        const auto& traj_id = *st_boundary.traj_id();
        const auto* traj =
            QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
        const auto& object_proto = traj->planner_object().object_proto();
        if (IsOccludedCameraObject(object_proto) ||
            IsOccludedLidarObject(object_proto)) {
          occluded_object_id_set->insert(*st_boundary.object_id());
        }
      };

  absl::flat_hash_set<std::string> occluded_object_id_set;
  for (auto& st_boundary_wd : *st_boundaries_wd) {
    const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) continue;
    if (st_boundary.min_t() > 0.0) continue;
    if (!st_boundary_wd.object_id().has_value()) continue;
    if (*st_boundary_wd.object_id() == *nearest_on_path_stay_object_id) {
      continue;
    }
    if (const auto* object_s =
            FindOrNull(on_path_object_id_s_map, *st_boundary_wd.object_id());
        nullptr != object_s && *object_s > nearest_on_path_stay_object_s) {
      insert_occluded_object_id_to_set(st_traj_mgr, st_boundary,
                                       &occluded_object_id_set);
    }
  }

  for (auto& st_boundary_wd : *st_boundaries_wd) {
    const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) continue;
    if (ContainsKey(occluded_object_id_set, *st_boundary.object_id())) {
      st_boundary_wd.set_decision_type(StBoundaryProto::IGNORE);
      st_boundary_wd.set_decision_reason(StBoundaryProto::IGNORE_DECIDER);
      st_boundary_wd.set_ignore_reason(StBoundaryProto::NON_NEAREST_OCCLUDED);
      st_boundary_wd.set_decision_info(
          "ignore non-nearest on-path occluded st-boundary");
    }
  }
  return;
}

void GetNearestOnPathObjectInfo(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const SegmentMatcherKdtree& path_kd_tree,
    const std::vector<StBoundaryWithDecision>& st_boundaries_wd,
    std::optional<std::string>* nearest_on_path_object_id,
    std::optional<std::string>* nearest_on_path_stay_object_id,
    double* nearest_on_path_stay_object_s,
    absl::flat_hash_map<std::string, double>* on_path_object_id_s_map) {
  double nearest_on_path_object_s;
  for (const auto& st_boundary_wd : st_boundaries_wd) {
    const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) continue;
    if (st_boundary_wd.raw_st_boundary()->min_t() > 0.0) continue;
    if (!st_boundary_wd.object_id().has_value()) continue;
    if (ContainsKey(*on_path_object_id_s_map, *st_boundary_wd.object_id())) {
      continue;
    }
    QCHECK(st_boundary.traj_id().has_value());
    const auto& traj_id = *st_boundary.traj_id();
    const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
    const auto& pos = traj->planner_object().pose().pos();
    double obj_s, obj_l;
    if (!path_kd_tree.GetProjection(pos.x(), pos.y(), /*is_clamp=*/false,
                                    &obj_s, &obj_l)) {
      continue;
    }
    if (!nearest_on_path_object_id->has_value() ||
        obj_s < nearest_on_path_object_s) {
      *nearest_on_path_object_id = st_boundary_wd.object_id();
      nearest_on_path_object_s = obj_s;
    }
    if (st_boundary.overlap_meta()->pattern() == StOverlapMetaProto::STAY &&
        (!nearest_on_path_stay_object_id->has_value() ||
         obj_s < *nearest_on_path_stay_object_s)) {
      *nearest_on_path_stay_object_id = st_boundary_wd.object_id();
      *nearest_on_path_stay_object_s = obj_s;
    }
    (*on_path_object_id_s_map)[*st_boundary_wd.object_id()] = obj_s;
  }
}

}  // namespace

void MakeIgnoreAndPreBrakeDecisionForStBoundaries(
    const IgnoreDeciderInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::optional<VtSpeedLimit>* speed_limit) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(input.params);
  QCHECK_NOTNULL(input.path);
  QCHECK_NOTNULL(input.path_semantics);
  QCHECK_NOTNULL(input.psmm);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.vehicle_geometry_params);
  QCHECK_NOTNULL(input.av_shapes);
  QCHECK_NOTNULL(input.path_kd_tree);
  QCHECK_GT(input.max_v, 0.0);
  QCHECK_GT(input.time_step, 0.0);
  QCHECK_GT(input.trajectory_steps, 0);

  // Prepare some common input.
  // Calculate the AV emergency stop st points.
  const auto emergency_stop_points =
      GenerateAvEmergencyStopStPoints(input.path->front().s(), input.current_v,
                                      input.current_a, input.trajectory_steps);
  // Calculate the travel length for left turn to be used in ignore decider.
  const auto& path_semantics = *input.path_semantics;
  bool on_left_turn_waiting_lane = false;
  if (!path_semantics.empty() && path_semantics.front().lane_semantic ==
                                     LaneSemantic::INTERSECTION_LEFT_TURN) {
    QCHECK_NOTNULL(path_semantics.front().lane_info);
    const auto& current_lane_info = *path_semantics.front().lane_info;
    QCHECK(current_lane_info.direction == mapping::LaneProto::LEFT_TURN);
    // Check if on left turn waiting lane.
    on_left_turn_waiting_lane = true;
    if (!current_lane_info.outgoing_lanes().empty()) {
      for (const auto& lane_id : current_lane_info.outgoing_lanes()) {
        const auto* outgoing_lane_info_ptr =
            input.psmm->FindLaneInfoOrNull(lane_id);
        if (outgoing_lane_info_ptr == nullptr) continue;
        if (outgoing_lane_info_ptr->direction !=
            mapping::LaneProto::LEFT_TURN) {
          on_left_turn_waiting_lane = false;
          break;
        }
      }
    } else {
      on_left_turn_waiting_lane = false;
    }
  }

  // Calculate AV frenet box info on drive passage.
  bool av_in_drive_passage_lane = false;
  std::optional<FrenetBox> av_frenet_box = std::nullopt;
  if (input.drive_passage != nullptr) {
    const auto& curr_path_point = input.path->front();
    const auto av_box =
        ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                     curr_path_point.theta(), *input.vehicle_geometry_params);
    const auto av_frenet_box_or = input.drive_passage->QueryFrenetBoxAt(av_box);
    if (av_frenet_box_or.ok()) {
      av_frenet_box = *av_frenet_box_or;
      const auto av_s_min_boundary =
          input.drive_passage->QueryEnclosingLaneBoundariesAtS(
              av_frenet_box_or->s_min);
      const auto av_s_max_boundary =
          input.drive_passage->QueryEnclosingLaneBoundariesAtS(
              av_frenet_box_or->s_max);
      // Check if AV is entirely in drive passage lane.
      av_in_drive_passage_lane =
          av_s_min_boundary.left.has_value() &&
          av_s_max_boundary.left.has_value() &&
          av_s_min_boundary.right.has_value() &&
          av_s_max_boundary.right.has_value() &&
          av_frenet_box->l_max < std::min(av_s_min_boundary.left->lat_offset,
                                          av_s_max_boundary.left->lat_offset) &&
          av_frenet_box->l_min > std::max(av_s_min_boundary.right->lat_offset,
                                          av_s_max_boundary.right->lat_offset);
    }
  }

  // Get upcoming merge lane ids.
  std::vector<mapping::ElementId> upcoming_merge_lane_ids;
  if (!path_semantics.empty() &&
      path_semantics.front().lane_semantic == LaneSemantic::ROAD &&
      av_in_drive_passage_lane) {
    const auto& current_closest_lane_point =
        path_semantics.front().closest_lane_point;
    QCHECK_NOTNULL(path_semantics.front().lane_info);
    const auto& current_lane_info = *path_semantics.front().lane_info;
    const auto& current_lane_proto = *current_lane_info.proto;
    for (const auto& interaction : current_lane_proto.interactions()) {
      // Only consider lane interactions beyond current AV position.
      if (interaction.this_lane_fraction() <
          current_closest_lane_point.fraction()) {
        continue;
      }
      constexpr double kUpcomingMergeThres = 30.0;  // m.
      if (interaction.geometric_configuration() ==
              mapping::LaneInteractionProto::MERGE &&
          current_lane_info.length() * (interaction.this_lane_fraction() -
                                        current_closest_lane_point.fraction()) <
              kUpcomingMergeThres) {
        upcoming_merge_lane_ids.push_back(
            mapping::ElementId(interaction.other_lane_id()));
      }
    }
  }

  // <id of the corresponding original st-boundary, protective st-boundary
  // pointer>
  absl::flat_hash_map<std::string, StBoundaryWithDecision*>
      protective_st_boundary_wd_map;
  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if (!st_boundary_wd.raw_st_boundary()->is_protective() ||
        st_boundary_wd.raw_st_boundary()->protection_type() ==
            StBoundaryProto::LANE_CHANGE_GAP) {
      continue;
    }
    const auto& protected_st_boundary_id =
        st_boundary_wd.raw_st_boundary()->protected_st_boundary_id();
    if (!protected_st_boundary_id.has_value()) continue;
    protective_st_boundary_wd_map.emplace(*protected_st_boundary_id,
                                          &st_boundary_wd);
  }

  std::optional<std::string> nearest_on_path_object_id;
  std::optional<std::string> nearest_on_path_stay_object_id;
  double nearest_on_path_stay_object_s;
  // <on-path object id, first overlap distance>
  absl::flat_hash_map<std::string, double> on_path_object_id_s_map;
  GetNearestOnPathObjectInfo(
      *input.st_traj_mgr, *input.path_kd_tree, *st_boundaries_wd,
      &nearest_on_path_object_id, &nearest_on_path_stay_object_id,
      &nearest_on_path_stay_object_s, &on_path_object_id_s_map);

  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if (st_boundary_wd.raw_st_boundary()->is_protective()) {
      continue;
    }
    MakeIgnoreAndPreBrakeDecisionForStBoundary(
        *input.params, *input.path, path_semantics, *input.st_traj_mgr,
        input.drive_passage, *input.vehicle_geometry_params, *input.av_shapes,
        *input.path_kd_tree, input.current_v, input.max_v, input.time_step,
        input.trajectory_steps, *input.psmm, emergency_stop_points,
        on_left_turn_waiting_lane, av_frenet_box, av_in_drive_passage_lane,
        upcoming_merge_lane_ids, nearest_on_path_object_id, &st_boundary_wd,
        speed_limit);
    if (const auto protective_st_boundary_wd_ptr =
            FindOrNull(protective_st_boundary_wd_map, st_boundary_wd.id());
        nullptr != protective_st_boundary_wd_ptr) {
      auto& protective_st_boundary_wd = **protective_st_boundary_wd_ptr;
      // The decision of protective st-boundary follows the decision of the
      // corresponding original st-boundary.
      if (protective_st_boundary_wd.decision_type() !=
          st_boundary_wd.decision_type()) {
        protective_st_boundary_wd.set_decision_type(
            st_boundary_wd.decision_type());
        protective_st_boundary_wd.set_decision_reason(
            StBoundaryProto::FOLLOW_PROTECTED);
        protective_st_boundary_wd.set_ignore_reason(
            st_boundary_wd.ignore_reason());
        protective_st_boundary_wd.set_decision_info(
            st_boundary_wd.decision_info());
      }
    }
  }
  // Only keep the nearest non-ignored stationary st-boundary.
  MakeIgnoreDecisionForNonNearestStationaryStBoundaries(st_boundaries_wd);

  // Ignore all on-path occluded objects except the nearest one.
  if (input.params->ignore_non_nearest_on_path_occluded_objects()) {
    MakeIgnoreDecisionForNonNearestOnPathOccludedStBoundaries(
        *input.st_traj_mgr, nearest_on_path_stay_object_id,
        on_path_object_id_s_map, nearest_on_path_stay_object_s,
        st_boundaries_wd);
  }
}
}  // namespace planner
}  // namespace qcraft
