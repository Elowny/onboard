#include "onboard/planner/planner_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "glog/logging.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/maps/v2/semantic_map_definition.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {

namespace {

void UpdateToMinSpeedLimit(std::map<mapping::ElementId, double>* map,
                           mapping::ElementId lane_id, double speed_limit) {
  auto pair_it = map->insert({lane_id, speed_limit});
  if (pair_it.second == false) {
    pair_it.first->second = std::min(pair_it.first->second, speed_limit);
  }
}

}  // namespace

double ComputeLongitudinalJerk(const TrajectoryPoint& traj_point) {
  return traj_point.j() - Cube(traj_point.v()) * Sqr(traj_point.kappa());
}

double ComputeLateralAcceleration(const TrajectoryPoint& traj_point) {
  return Sqr(traj_point.v()) * traj_point.kappa();
}

double ComputeLateralJerk(const TrajectoryPoint& traj_point) {
  return 3.0 * traj_point.v() * traj_point.a() * traj_point.kappa() +
         Sqr(traj_point.v()) * traj_point.psi();
}

bool IsVulnerableRoadUserType(ObjectType type) {
  return type == OT_PEDESTRIAN || type == OT_CYCLIST || type == OT_TRICYCLIST ||
         type == OT_MOTORCYCLIST;
}

bool IsStaticObjectType(ObjectType type) {
  return type == OT_UNKNOWN_STATIC || type == OT_VEGETATION || type == OT_FOD ||
         type == OT_BARRIER || type == OT_CONE;
}

PlannerSemanticMapModification CreateSemanticMapModification(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const mapping::SemanticMapModifierProto& modifier) {
  std::map<mapping::ElementId, double> lane_speed_limit_map;
  double max_speed_limit = std::numeric_limits<double>::max();

  if (modifier.has_speed_limit_modifier()) {
    if (modifier.speed_limit_modifier().has_max_speed_limit()) {
      max_speed_limit = modifier.speed_limit_modifier().max_speed_limit();
    }

    for (const auto& lane_id_mod :
         modifier.speed_limit_modifier().lane_id_modifier()) {
      UpdateToMinSpeedLimit(&lane_speed_limit_map,
                            mapping::ElementId(lane_id_mod.lane_id()),
                            lane_id_mod.override_speed_limit());
    }

    for (const auto& region_mod :
         modifier.speed_limit_modifier().region_modifier()) {
      const Polygon2d polygon =
          Polygon2d(mapping::ConverterGeoPoints(region_mod.region().points()));
      for (const auto& lane_ptr : semantic_map_manager.semantic_map().lanes) {
        bool in_polygon = true;
        for (const auto& point : lane_ptr->segment_points()) {
          if (!polygon.IsPointIn(point)) {
            in_polygon = false;
            break;
          }
        }
        if (in_polygon) {
          UpdateToMinSpeedLimit(&lane_speed_limit_map,
                                mapping::ElementId(lane_ptr->proto().id()),
                                region_mod.override_speed_limit());
        }
      }
    }
  }

  return PlannerSemanticMapModification{
      .lane_speed_limit_map = std::move(lane_speed_limit_map),
      .max_speed_limit = max_speed_limit};
}

mapping::SemanticMapModifierProto PlannerSemanticMapModificationToProto(
    const PlannerSemanticMapModification& modifier) {
  mapping::SemanticMapModifierProto modifier_proto;
  modifier_proto.mutable_speed_limit_modifier()->set_max_speed_limit(
      modifier.max_speed_limit);

  for (const auto& it : modifier.lane_speed_limit_map) {
    auto* lane_id_modifier =
        modifier_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
    lane_id_modifier->set_lane_id(it.first.value());
    lane_id_modifier->set_override_speed_limit(it.second);
  }

  return modifier_proto;
}

std::vector<ApolloTrajectoryPointProto> CreatePastPointsList(
    absl::Time plan_time, const TrajectoryProto& prev_traj, bool reset,
    int max_past_point_num) {
  std::vector<ApolloTrajectoryPointProto> past_points;
  const double curr_plan_time = ToUnixDoubleSeconds(plan_time);
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  if (prev_traj.trajectory_point().empty() ||
      curr_plan_time >
          prev_traj_start_time +
              prev_traj.trajectory_point().rbegin()->relative_time() ||
      curr_plan_time < prev_traj_start_time || reset) {
    return past_points;
  }
  past_points.reserve(max_past_point_num);
  const int relative_time_index =
      RoundToInt((curr_plan_time - prev_traj_start_time) / kTrajectoryTimeStep);
  QCHECK_LT(relative_time_index, prev_traj.trajectory_point_size());
  const double relative_s =
      -prev_traj.trajectory_point(relative_time_index).path_point().s();
  for (int i = max_past_point_num; i > 0; --i) {
    const int index = relative_time_index - i;
    if (index + prev_traj.past_points_size() < 0) {
      continue;
    }
    auto point =
        index < 0 ? prev_traj.past_points(index + prev_traj.past_points_size())
                  : prev_traj.trajectory_point(index);
    point.set_relative_time(-i * kTrajectoryTimeStep);
    point.mutable_path_point()->set_s(point.path_point().s() + relative_s);
    past_points.push_back(point);
  }
  past_points.shrink_to_fit();

  return past_points;
}

std::vector<PathPoint> CreatePastDirectionalPathPoints(
    const TrajectoryProto& prev_traj, bool reset,
    double start_s_on_prev_dir_path, bool freespace_reset) {
  std::vector<PathPoint> past_points;
  // Return empty past points if reset or no previous directional path.
  if (reset || freespace_reset || prev_traj.directional_path().path().empty()) {
    return past_points;
  }

  constexpr double kMaxPastPathLength = 2.0;  // m.
  past_points.reserve(prev_traj.directional_path().path().size() +
                      prev_traj.past_directional_path_points().size());
  const auto insert_past_points =
      [&past_points, start_s_on_prev_dir_path](const auto& path_points) {
        for (auto p : path_points) {
          if (p.s() < start_s_on_prev_dir_path) {
            if (p.s() < start_s_on_prev_dir_path - kMaxPastPathLength) continue;
            p.set_s(p.s() - start_s_on_prev_dir_path);
            past_points.push_back(p);
            continue;
          }
          break;
        }
      };
  insert_past_points(prev_traj.past_directional_path_points());
  insert_past_points(prev_traj.directional_path().path());

  return past_points;
}

ApolloTrajectoryPointProto ComputePlanStartPointAfterReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const std::optional<bool>& is_forward_task) {
  ApolloTrajectoryPointProto plan_start_point;
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_s(0.0);
  plan_start_point.mutable_path_point()->set_theta(pose.yaw());
  const double pose_v = pose.vel_body().x();
  const double abs_pose_v = std::abs(pose_v);
  if (is_forward_task.has_value()) {
    plan_start_point.set_v(*is_forward_task ? std::max(0.0, pose_v)
                                            : std::min(0.0, pose_v));
  } else {
    plan_start_point.set_v(pose_v);
  }

  if (prev_reset_planned_point.has_value()) {
    constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
    const bool full_stop = prev_reset_planned_point->v() == 0.0 &&
                           abs_pose_v < kFullStopSpeedThreshold;
    if (full_stop) {
      plan_start_point.mutable_path_point()->set_kappa(
          prev_reset_planned_point->path_point().kappa());
      plan_start_point.mutable_path_point()->set_lambda(
          prev_reset_planned_point->path_point().lambda());
      plan_start_point.set_a(0.0);
      plan_start_point.set_j(0.0);
      return plan_start_point;
    }
  }

  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  if (abs_pose_v < kLowSpeedThreshold) {
    // At speed lower than this, we don't trust the measured acceleration and
    // angular velocity. Reset like this are mostly when we're stopped anyway.
    const double steer_angle =
        chassis.has_steering_percentage()
            ? chassis.steering_percentage() * 0.01 *
                  vehicle_drive_params.max_steer_angle() /
                  vehicle_drive_params.steer_ratio()
            : 0.0;  // rad.
    const double kappa =
        std::tan(steer_angle) / vehicle_geom_params.wheel_base();
    plan_start_point.mutable_path_point()->set_kappa(kappa);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
    plan_start_point.set_a(0.0);
    plan_start_point.set_j(0.0);
  } else {
    plan_start_point.mutable_path_point()->set_kappa(pose.ar_smooth().z() /
                                                     pose_v);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
    plan_start_point.set_a(std::clamp(
        pose.accel_body().x(), motion_constraint_params.max_deceleration(),
        motion_constraint_params.max_acceleration()));
    plan_start_point.set_j(0.0);
  }
  return plan_start_point;
}

ApolloTrajectoryPointProto ComputePlanStartPointAfterLateralReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, const Chassis& chassis,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  ApolloTrajectoryPointProto plan_start_point;
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_s(0.0);
  plan_start_point.mutable_path_point()->set_theta(pose.yaw());
  // Keep longitudinal quantities.
  QCHECK(prev_reset_planned_point.has_value());
  plan_start_point.set_v(prev_reset_planned_point->v());
  plan_start_point.set_a(prev_reset_planned_point->a());
  plan_start_point.set_j(prev_reset_planned_point->j());

  const double pose_v = pose.vel_body().x();
  const double abs_pose_v = std::abs(pose_v);
  constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
  const bool full_stop = prev_reset_planned_point->v() == 0.0 &&
                         abs_pose_v < kFullStopSpeedThreshold;
  if (full_stop) {
    plan_start_point.mutable_path_point()->set_kappa(
        prev_reset_planned_point->path_point().kappa());
    plan_start_point.mutable_path_point()->set_lambda(
        prev_reset_planned_point->path_point().lambda());
    return plan_start_point;
  }

  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  if (abs_pose_v < kLowSpeedThreshold) {
    // At speed lower than this, we don't trust the measured acceleration and
    // angular velocity. Reset like this are mostly when we're stopped anyway.
    const double steer_angle =
        chassis.has_steering_percentage()
            ? chassis.steering_percentage() * 0.01 *
                  vehicle_drive_params.max_steer_angle() /
                  vehicle_drive_params.steer_ratio()
            : 0.0;  // rad.
    const double kappa =
        std::tan(steer_angle) / vehicle_geom_params.wheel_base();
    plan_start_point.mutable_path_point()->set_kappa(kappa);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
  } else {
    plan_start_point.mutable_path_point()->set_kappa(pose.ar_smooth().z() /
                                                     pose_v);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
  }
  return plan_start_point;
}

ApolloTrajectoryPointProto ComputeACCPlanStartPointAfterLateralReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, const Chassis& chassis,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, double t_diff,
    double prev_longitudinal_error) {
  QCHECK(prev_reset_planned_point.has_value());
  ApolloTrajectoryPointProto plan_start_point;
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  const double cur_v = prev_reset_planned_point->v();
  // When av diverges from prev trajectory, pose.yaw() could better represent
  // moving intention.
  const double heading = pose.yaw();
  // Take prev longitudinal error into consideration to keep longitudinal
  // consecutiveness between frames.
  const double predicted_lon_distance =
      std::max(cur_v * t_diff - prev_longitudinal_error, 0.0);
  double cos_sin[2];
  fast_math::CosAndSin<7>(heading, cos_sin);
  VLOG(2) << "predicted lon distance: " << predicted_lon_distance;
  plan_start_point.mutable_path_point()->set_x(
      pose_pos.x() + cos_sin[0] * predicted_lon_distance);
  plan_start_point.mutable_path_point()->set_y(
      pose_pos.y() + cos_sin[1] * predicted_lon_distance);
  plan_start_point.mutable_path_point()->set_s(0.0);
  plan_start_point.mutable_path_point()->set_theta(pose.yaw());
  // Keep longitudinal quantities.
  plan_start_point.set_v(prev_reset_planned_point->v());
  plan_start_point.set_a(prev_reset_planned_point->a());
  plan_start_point.set_j(prev_reset_planned_point->j());

  const double abs_pose_v = std::abs(pose.vel_body().x());

  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  if (abs_pose_v < kLowSpeedThreshold) {
    // At speed lower than this, we don't trust the measured acceleration and
    // angular velocity. Reset like this are mostly when we're stopped anyway.
    const double steer_angle =
        chassis.has_steering_percentage()
            ? chassis.steering_percentage() * 0.01 *
                  vehicle_drive_params.max_steer_angle() /
                  vehicle_drive_params.steer_ratio()
            : 0.0;  // rad.
    const double kappa =
        std::tan(steer_angle) / vehicle_geom_params.wheel_base();
    plan_start_point.mutable_path_point()->set_kappa(kappa);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
  } else {
    plan_start_point.mutable_path_point()->set_kappa(pose.curvature());
    plan_start_point.mutable_path_point()->set_lambda(0.0);
  }
  return plan_start_point;
}

ApolloTrajectoryPointProto
ComputePlanStartPointAfterLongitudinalResetFromPrevTrajectory(
    const TrajectoryProto& prev_traj, const PoseProto& pose,
    const Chassis& chassis,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  ApolloTrajectoryPointProto plan_start_point;
  // Reset longitudinal quantities from pose.
  plan_start_point.set_v(std::max(0.0, pose.vel_body().x()));
  plan_start_point.set_a(pose.accel_body().x());
  plan_start_point.set_j(0.0);

  if (prev_traj.trajectory_point_size() < 2) {
    // Previous trajectory too short or empty, reset path point from pose.
    plan_start_point.mutable_path_point()->set_s(0.0);
    plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
    plan_start_point.mutable_path_point()->set_theta(pose.yaw());
    constexpr double kLowSpeedThreshold = 1.0;  // m/s.
    if (std::abs(pose.vel_body().x()) < kLowSpeedThreshold) {
      // At speed lower than this, we don't trust the measured angular velocity.
      const double steer_angle =
          chassis.has_steering_percentage()
              ? chassis.steering_percentage() * 0.01 *
                    vehicle_drive_params.max_steer_angle() /
                    vehicle_drive_params.steer_ratio()
              : 0.0;  // rad.
      const double kappa =
          std::tan(steer_angle) / vehicle_geom_params.wheel_base();
      plan_start_point.mutable_path_point()->set_kappa(kappa);
      plan_start_point.mutable_path_point()->set_lambda(0.0);
    } else {
      plan_start_point.mutable_path_point()->set_kappa(pose.ar_smooth().z() /
                                                       pose.vel_body().x());
      plan_start_point.mutable_path_point()->set_lambda(0.0);
    }
    return plan_start_point;
  }

  // Reset path point from previous trajectory.
  std::vector<PathPoint> prev_traj_path_points;
  prev_traj_path_points.reserve(prev_traj.trajectory_point_size());
  for (int i = 0; i < prev_traj.trajectory_point_size(); ++i) {
    prev_traj_path_points.push_back(prev_traj.trajectory_point(i).path_point());
    // Make sure s start from zero.
    prev_traj_path_points.back().set_s(
        prev_traj.trajectory_point(i).path_point().s() -
        prev_traj.trajectory_point(0).path_point().s());
  }
  DiscretizedPath prev_traj_path(std::move(prev_traj_path_points));
  const auto pose_sl = prev_traj_path.XYToSL(
      Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()));
  *plan_start_point.mutable_path_point() = prev_traj_path.Evaluate(pose_sl.s);
  plan_start_point.mutable_path_point()->set_s(0.0);
  return plan_start_point;
}

SelectorParamsProto LoadSelectorParamsFromFile(
    const std::string& file_address) {
  SelectorParamsProto selector_params_proto;
  if (!file_util::TextFileToProto(file_address, &selector_params_proto)) {
    QCHECK(false) << "Read auto tuned selector params as text file failed!!!!";
  }
  QLOG(INFO) << "New auto tuned selector params are used.";
  return selector_params_proto;
}

}  // namespace planner
}  // namespace qcraft
