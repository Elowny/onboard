#include "onboard/planner/freespace/freespace_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <ostream>
#include <string>

#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/vehicle_shape.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {

namespace {

bool CheckGoalValidity(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const std::vector<FreespaceBoundary>& freespace_boundaries,
    const std::vector<FreespaceObject>& stationary_objects, const Vec2d& pos,
    double theta) {
  const double offset = 0.5 * (veh_geo_params.front_edge_to_center() -
                               veh_geo_params.back_edge_to_center());
  const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
  const Vec2d center = pos + offset * tangent;
  const VehicleOctagonShape av_shape(
      veh_geo_params, vehicle_model_params, pos, center, tangent, theta,
      0.5 * veh_geo_params.length(), 0.5 * veh_geo_params.width());
  const auto should_consider_mirror =
      [&path_finder_params, &vehicle_model_params](const double object_height) {
        return vehicle_model_params.consider_mirror() &&
               (object_height + path_finder_params.mirror_height_buffer()) >
                   vehicle_model_params.mirror_height();
      };

  for (const auto& object : stationary_objects) {
    if (av_shape.HasOverlapWithBuffer(
            object.contour, path_finder_params.object_lateral_buffer(),
            path_finder_params.object_longitudinal_buffer(),
            should_consider_mirror(object.height))) {
      return false;
    }
  }
  for (const auto& boundary : freespace_boundaries) {
    const auto buffers =
        GetVehicleBufferForBoundary(path_finder_params, boundary);
    for (int i = 0; i + 1 < boundary.points.size(); ++i) {
      const Segment2d segment(boundary.points[i], boundary.points[i + 1]);
      if (av_shape.HasOverlapWithBuffer(
              segment, buffers.first, buffers.second,
              should_consider_mirror(boundary.height)))
        return false;
    }
  }
  return true;
}

}  // namespace

MinStopSInfo ComputeMinStopSInfo(
    const std::vector<StBoundaryWithDecision>& st_boundaries_wd) {
  MinStopSInfo min_stop_s_info;
  for (const StBoundaryWithDecision& st_boundary_wd : st_boundaries_wd) {
    const StBoundary& st_boundary = *st_boundary_wd.raw_st_boundary();
    QCHECK(st_boundary.source_type() != StBoundarySourceTypeProto::UNKNOWN);
    if (st_boundary_wd.decision_type() != StBoundaryProto::FOLLOW) {
      continue;
    }
    if (st_boundary.is_stationary()) {
      const double stop_s =
          std::max(0.0, st_boundary.min_s() -
                            st_boundary_wd.follow_standstill_distance());
      if (!min_stop_s_info.min_stop_s.has_value() ||
          stop_s < *min_stop_s_info.min_stop_s) {
        min_stop_s_info.min_stop_s = stop_s;
      }
      if (st_boundary.source_type() == StBoundarySourceTypeProto::ST_OBJECT &&
          (!min_stop_s_info.min_stationary_object_stop_s.has_value() ||
           stop_s < *min_stop_s_info.min_stationary_object_stop_s)) {
        min_stop_s_info.min_stationary_object_stop_s = stop_s;
        min_stop_s_info.nearest_stationary_object_id =
            SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
                st_boundary.id());
      }
    } else {
      constexpr double kMinTThres = 2.0;  // s.
      if (st_boundary.min_t() > kMinTThres) continue;
      constexpr double kDeltaT = 0.1;  // s.
      for (double t = st_boundary.min_t();
           t <= std::min(kMinTThres, st_boundary.max_t()); t += kDeltaT) {
        const auto min_t_s_range = st_boundary.GetBoundarySRange(t);
        if (!min_t_s_range.has_value()) continue;
        const double stop_s =
            std::max(0.0, min_t_s_range->second -
                              st_boundary_wd.follow_standstill_distance());
        if (!min_stop_s_info.min_stop_s.has_value() ||
            stop_s < *min_stop_s_info.min_stop_s) {
          min_stop_s_info.min_stop_s = stop_s;
        }
      }
    }
  }
  return min_stop_s_info;
}

bool HasGoal(const GlobalGoalReferenceProto& global_goal_ref) {
  switch (global_goal_ref.reference_case()) {
    case GlobalGoalReferenceProto::kNoneRef:
      return global_goal_ref.none_ref().has_smooth_goal();
    case GlobalGoalReferenceProto::kHdMapRef:
      return global_goal_ref.hd_map_ref().has_global_goal();
    case GlobalGoalReferenceProto::kParkingSpotRef:
      return global_goal_ref.parking_spot_ref().has_spot_local_goal();
    case GlobalGoalReferenceProto::REFERENCE_NOT_SET:
      return false;
  }
}

TrajectoryProto CreateFreespaceTrajectoryProto(
    absl::Time plan_time,
    const std::vector<ApolloTrajectoryPointProto>& planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto>& past_points,
    const Chassis::GearPosition& gear_position,
    const DrivingStateProto& driving_state, bool low_speed_freespace,
    bool enable_stationary_steering,
    const DirectionalPath& smooth_directional_path, double stop_s,
    const std::vector<PathPoint>& past_directional_path_points) {
  TrajectoryProto trajectory;
  trajectory.set_trajectory_start_timestamp(ToUnixDoubleSeconds(plan_time));
  for (int i = 0; i < planned_trajectory.size(); ++i) {
    *trajectory.add_trajectory_point() = planned_trajectory[i];
  }

  // NOTE: past_points are designed for controller.
  for (const auto& past_point : past_points) {
    *trajectory.add_past_points() = past_point;
  }

  trajectory.set_gear(gear_position);

  trajectory.mutable_driving_state()->CopyFrom(driving_state);

  trajectory.set_low_speed_freespace(low_speed_freespace);

  trajectory.set_enable_stationary_steering(enable_stationary_steering);

  smooth_directional_path.ToProto(trajectory.mutable_directional_path());

  trajectory.set_stop_s(stop_s);

  *trajectory.mutable_past_directional_path_points() = {
      past_directional_path_points.begin(), past_directional_path_points.end()};

  return trajectory;
}

std::pair<double, double> GetVehicleBufferForBoundary(
    const FreespacePathFinderParamsProto& path_finder_params,
    const FreespaceBoundary& boundary) {
  constexpr double kDefaultBuffer = -0.5;
  switch (boundary.type) {
    case FreespaceMapProto::CURB:
      if (boundary.near_parking_spot) {
        return std::make_pair(
            path_finder_params.near_spot_curb_lateral_buffer(),
            path_finder_params.near_spot_curb_longitudinal_buffer());
      } else {
        return std::make_pair(path_finder_params.curb_lateral_buffer(),
                              path_finder_params.curb_longitudinal_buffer());
      }
    case FreespaceMapProto::BARRIER:
      return std::make_pair(path_finder_params.barrier_lateral_buffer(),
                            path_finder_params.barrier_longitudinal_buffer());
    case FreespaceMapProto::YELLOW_SOLID_LANE:
      return std::make_pair(
          path_finder_params.solid_lane_lateral_buffer(),
          path_finder_params.solid_lane_longitudinal_buffer());
    case FreespaceMapProto::YELLOW_DASHED_LANE:
      return std::make_pair(kDefaultBuffer, kDefaultBuffer);
    case FreespaceMapProto::WHITE_SOLID_LANE:
      return std::make_pair(
          path_finder_params.solid_lane_lateral_buffer(),
          path_finder_params.solid_lane_longitudinal_buffer());
    case FreespaceMapProto::WHITE_DASHED_LANE:
      return std::make_pair(kDefaultBuffer, kDefaultBuffer);
    case FreespaceMapProto::PARKING_SPOT:
      return std::make_pair(path_finder_params.spot_line_lateral_buffer(),
                            path_finder_params.spot_line_longitudinal_buffer());
    case FreespaceMapProto::PARKING_STOPPER:
      return std::make_pair(
          path_finder_params.parking_stopper_lateral_buffer(),
          path_finder_params.parking_stopper_longitudinal_buffer());
    case FreespaceMapProto::VIRTUAL:
      return std::make_pair(
          path_finder_params.virtual_boundary_lateral_buffer(),
          path_finder_params.virtual_boundary_longitudinal_buffer());
    case FreespaceMapProto::OTHER:
      return std::make_pair(kDefaultBuffer, kDefaultBuffer);
  }
  return std::make_pair(kDefaultBuffer, kDefaultBuffer);
}

PathPoint MaybeAdjustGoal(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const PlannerSemanticMapManager* psmm,
    const std::vector<FreespaceObject>& stationary_objects,
    const PathPoint& goal, bool is_parking_task, char adjust_dir,
    double max_adjust_dist, double adjust_step, double planner_buffer) {
  // Get boundaries near goal.
  std::vector<FreespaceBoundary> freespace_boundaries;
  if (psmm != nullptr) {
    constexpr double kNearbyDistance = 10.0;  // m.
    const std::vector<const mapping::LaneBoundaryInfo*> boundaries_info =
        psmm->GetLaneBoundariesInfoAtLevel(
            psmm->GetLevel(), Vec2d(goal.x(), goal.y()), kNearbyDistance);
    for (const mapping::LaneBoundaryInfo* lane_boundary_info :
         boundaries_info) {
      switch (lane_boundary_info->type) {
        case mapping::LaneBoundaryProto::CURB:
          freespace_boundaries.push_back(
              {.id = std::to_string(lane_boundary_info->id),
               .type = FreespaceMapProto::CURB,
               .points = lane_boundary_info->points_smooth,
               .near_parking_spot = is_parking_task,
               .height = lane_boundary_info->proto->height()});
          break;
        case mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW:
        case mapping::LaneBoundaryProto::SOLID_YELLOW:
          freespace_boundaries.push_back(
              {.id = std::to_string(lane_boundary_info->id),
               .type = FreespaceMapProto::YELLOW_SOLID_LANE,
               .points = lane_boundary_info->points_smooth,
               .near_parking_spot = is_parking_task,
               .height = 0.0});
          break;
        case mapping::LaneBoundaryProto::SOLID_DOUBLE_WHITE:
        case mapping::LaneBoundaryProto::BROKEN_DOUBLE_YELLOW:
        case mapping::LaneBoundaryProto::BROKEN_DOUBLE_WHITE:
        case mapping::LaneBoundaryProto::BROKEN_YELLOW:
        case mapping::LaneBoundaryProto::SOLID_WHITE:
        case mapping::LaneBoundaryProto::BROKEN_WHITE:
        case mapping::LaneBoundaryProto::UNKNOWN_TYPE:
        case mapping::LaneBoundaryProto::VIRTUAL:
        case mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE:
        case mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE:
          break;
      }
    }
  }
  // Adjust goal.
  Vec2d adjust_tangent;
  switch (adjust_dir) {
    case 'F':
      adjust_tangent = Vec2d::FastUnitFromAngle(goal.theta());
      break;
    case 'B':
      adjust_tangent = -Vec2d::FastUnitFromAngle(goal.theta());
      break;
    case 'L':
      adjust_tangent = Vec2d::FastUnitFromAngle(goal.theta()).Perp();
      break;
    case 'R':
      adjust_tangent = -Vec2d::FastUnitFromAngle(goal.theta()).Perp();
      break;
  }

  PathPoint res = goal;
  const Vec2d goal_pos(goal.x(), goal.y());
  double adjust_dist = 0.0;
  while (adjust_dist <= max_adjust_dist) {
    const auto new_goal_pos = goal_pos + adjust_dist * adjust_tangent;
    res.set_x(new_goal_pos.x());
    res.set_y(new_goal_pos.y());
    if (CheckGoalValidity(path_finder_params, veh_geo_params,
                          vehicle_model_params, freespace_boundaries,
                          stationary_objects, new_goal_pos, goal.theta())) {
      // If parallel parking goal has been adjusted, we need to adjust for a
      // short distance again to reserve enough space for planning.
      if (adjust_dist > 0.0 && planner_buffer > 0.0) {
        const auto res_pos =
            goal_pos + (adjust_dist + planner_buffer) * adjust_tangent;
        res.set_x(res_pos.x());
        res.set_y(res_pos.y());
      }
      return res;
    }
    adjust_dist += adjust_step;
  }
  // Return result anyway.
  return goal;
}

PathPoint MaybeAdjustGoal(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const PlannerSemanticMapManager* psmm,
    const PlannerObjectManager* object_manager,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const PathPoint& goal, bool is_parking_task, char adjust_dir,
    double max_adjust_dist, double adjust_step, double planner_buffer) {
  // Get stationary objects.
  std::vector<FreespaceObject> stationary_objects;
  stationary_objects.reserve(stalled_objects.size());
  for (const auto& id : stalled_objects) {
    const auto obj_ptr = object_manager->FindObjectById(id);
    QCHECK(obj_ptr != nullptr);
    FreespaceObject obj = {.contour = obj_ptr->contour(),
                           .height = obj_ptr->object_proto().max_z() -
                                     obj_ptr->object_proto().ground_z()};
    stationary_objects.push_back(std::move(obj));
  }

  return MaybeAdjustGoal(path_finder_params, veh_geo_params,
                         vehicle_model_params, psmm, stationary_objects, goal,
                         is_parking_task, adjust_dir, max_adjust_dist,
                         adjust_step, planner_buffer);
}

bool CheckPoseValidityWithKDTree(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const Vec2d& pos, double heading, const Vec2d& tangent) {
  const double offset = 0.5 * (veh_geo_params.front_edge_to_center() -
                               veh_geo_params.back_edge_to_center());
  const Vec2d av_geo_center = pos + offset * tangent;
  const VehicleOctagonShape av_shape(
      veh_geo_params, vehicle_model_params, pos, av_geo_center, tangent,
      heading, 0.5 * veh_geo_params.length(), 0.5 * veh_geo_params.width());
  const auto should_consider_mirror =
      [&path_finder_params, &vehicle_model_params](const double object_height) {
        return vehicle_model_params.consider_mirror() &&
               (object_height + path_finder_params.mirror_height_buffer()) >
                   vehicle_model_params.mirror_height();
      };

  const auto nearby_named_segments = segments_kd_tree.GetNamedSegmentsInRadius(
      av_geo_center.x(), av_geo_center.y(), veh_geo_params.length());
  for (const auto& named_segment : nearby_named_segments) {
    const auto iter = objects_map.find(named_segment.second);
    if (iter != objects_map.end()) {
      if (av_shape.HasOverlapWithBuffer(
              iter->second->contour, path_finder_params.object_lateral_buffer(),
              path_finder_params.object_longitudinal_buffer(),
              should_consider_mirror(iter->second->height))) {
        return false;
      }
    } else {
      const auto boundary_iter = boundaries_map.find(named_segment.second);
      QCHECK(boundary_iter != boundaries_map.end());
      const auto buffers = GetVehicleBufferForBoundary(path_finder_params,
                                                       *boundary_iter->second);
      if (av_shape.HasOverlapWithBuffer(
              *named_segment.first, buffers.first, buffers.second,
              should_consider_mirror(boundary_iter->second->height))) {
        return false;
      }
    }
  }
  return true;
}

double GetPoseDistanceToObstacle(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const Vec2d& pos, double heading, const Vec2d& tangent) {
  const double offset = 0.5 * (veh_geo_params.front_edge_to_center() -
                               veh_geo_params.back_edge_to_center());
  const Vec2d av_geo_center = pos + offset * tangent;
  const VehicleOctagonShape av_shape(
      veh_geo_params, vehicle_model_params, pos, av_geo_center, tangent,
      heading, 0.5 * veh_geo_params.length(), 0.5 * veh_geo_params.width());
  const auto should_consider_mirror =
      [&path_finder_params, &vehicle_model_params](const double object_height) {
        return vehicle_model_params.consider_mirror() &&
               (object_height + path_finder_params.mirror_height_buffer()) >
                   vehicle_model_params.mirror_height();
      };

  double min_dis = std::numeric_limits<double>::infinity();
  const auto nearby_named_segments = segments_kd_tree.GetNamedSegmentsInRadius(
      av_geo_center.x(), av_geo_center.y(), veh_geo_params.length());
  for (const auto& named_segment : nearby_named_segments) {
    const auto iter = objects_map.find(named_segment.second);
    if (iter != objects_map.end()) {
      min_dis = std::min(
          min_dis,
          av_shape.DistanceTo(iter->second->contour,
                              should_consider_mirror(iter->second->height)));
    } else {
      const auto boundary_iter = boundaries_map.find(named_segment.second);
      QCHECK(boundary_iter != boundaries_map.end());
      min_dis = std::min(
          min_dis, av_shape.DistanceTo(
                       *named_segment.first,
                       should_consider_mirror(boundary_iter->second->height)));
    }
  }
  return min_dis;
}

PathPoint RestoreSmoothGoalFromGlobalRef(
    const GlobalGoalReferenceProto& global_goal_ref,
    const CoordinateConverter* nullable_coordinate_converter,
    const mapping::ParkingSpotInfo* nullable_parking_spot_info) {
  PathPoint goal;
  switch (global_goal_ref.reference_case()) {
    case GlobalGoalReferenceProto::kNoneRef: {
      QCHECK(global_goal_ref.none_ref().has_smooth_goal());
      const auto& smooth_goal = global_goal_ref.none_ref().smooth_goal();
      goal.set_x(smooth_goal.pos().x());
      goal.set_y(smooth_goal.pos().y());
      goal.set_theta(smooth_goal.theta());
    } break;
    case GlobalGoalReferenceProto::kHdMapRef: {
      QCHECK(global_goal_ref.hd_map_ref().has_global_goal());
      QCHECK_NOTNULL(nullable_coordinate_converter);
      // Convert global goal to current smooth.
      const Vec2d cur_goal_pos = nullable_coordinate_converter->GlobalToSmooth(
          Vec2d(global_goal_ref.hd_map_ref().global_goal().pos()));
      goal.set_x(cur_goal_pos.x());
      goal.set_y(cur_goal_pos.y());
      goal.set_theta(nullable_coordinate_converter->GlobalYawToSmooth(
          global_goal_ref.hd_map_ref().global_goal().theta()));
    } break;
    case GlobalGoalReferenceProto::kParkingSpotRef: {
      QCHECK(global_goal_ref.parking_spot_ref().has_spot_local_goal());
      QCHECK_NOTNULL(nullable_parking_spot_info);
      QCHECK_EQ(nullable_parking_spot_info->id(),
                mapping::ElementId(
                    global_goal_ref.parking_spot_ref().parking_spot_id()));
      const Vec2d spot_dir = nullable_parking_spot_info->unit_direction();
      const Vec2d spot_centroid =
          nullable_parking_spot_info->polygon().centroid();
      const auto& proto = global_goal_ref.parking_spot_ref().spot_local_goal();
      const Vec2d cur_goal_pos =
          Vec2d(proto.pos()).Rotate(spot_dir) + spot_centroid;
      goal.set_x(cur_goal_pos.x());
      goal.set_y(cur_goal_pos.y());
      goal.set_theta(NormalizeAngle(proto.theta() + spot_dir.Angle()));
    } break;
    case GlobalGoalReferenceProto::REFERENCE_NOT_SET:
      QLOG(FATAL) << "Global goal reference is not set.";
      break;
  }
  return goal;
}

std::vector<VehicleShapeBasePtr> BuildFreespaceAvShapes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const DiscretizedPath& path_points, bool forward,
    const VehicleOctagonModelParamsProto& vehicle_model_params) {
  const double half_length = vehicle_geom.length() * 0.5;
  const double half_width = vehicle_geom.width() * 0.5;
  const double rac_to_center = half_length - vehicle_geom.back_edge_to_center();
  std::vector<VehicleShapeBasePtr> av_shapes;
  const int num_points = path_points.size();
  av_shapes.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    const auto& pt = path_points[i];
    const double theta =
        forward ? pt.theta() : NormalizeAngle(pt.theta() + M_PI);
    const Vec2d rac(pt.x(), pt.y());
    const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
    const Vec2d center = rac + tangent * rac_to_center;
    av_shapes.push_back(std::make_unique<VehicleOctagonShape>(
        vehicle_geom, vehicle_model_params, rac, center, tangent, theta,
        half_length, half_width));
  }
  return av_shapes;
}

}  // namespace qcraft::planner
