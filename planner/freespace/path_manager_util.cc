#include "onboard/planner/freespace/path_manager_util.h"

#include <cmath>
#include <iterator>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/vehicle_shape.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace planner {
namespace {

inline bool IsGoalReached(const DirectionalPathProto& path,
                          const PoseProto& av_pose) {
  const Vec2d av_pos(av_pose.pos_smooth().x(), av_pose.pos_smooth().y());
  const DiscretizedPath discrete_path({path.path().begin(), path.path().end()});
  const auto current_sl = discrete_path.XYToSL(av_pos);
  // Distance to goal can be negtive.
  const double distance_to_goal = discrete_path.back().s() - current_sl.s;
  const double cur_speed = av_pose.vel_body().x();
  VLOG(1) << "Distance to goal: " << distance_to_goal << "m";
  constexpr double kMaxDistanceToGoal = 0.2;   // m.
  constexpr double kMinDistanceToGoal = 0.05;  // m.
  constexpr double kMaxStopSpeed = 0.2;        // m/s.
  constexpr double kMinStopSpeed = 0.02;       // m/s.
  if (distance_to_goal < kMaxDistanceToGoal &&
      std::abs(cur_speed) < kMinStopSpeed) {
    return true;
  }
  if (distance_to_goal < kMinDistanceToGoal &&
      std::abs(cur_speed) < kMaxStopSpeed) {
    return true;
  }
  return false;
}

inline bool IsGearShiftedToRefDirection(
    const Chassis::GearPosition& gear_position, bool is_next_path_forward) {
  return (is_next_path_forward && gear_position == Chassis::GEAR_DRIVE) ||
         (!is_next_path_forward && gear_position == Chassis::GEAR_REVERSE);
}

inline double Kappa2FrontWheelAngle(double kappa, double wheel_base) {
  return std::atan(kappa * wheel_base);
}

inline double GetFrontWheelAngle(double steering_percentage,
                                 double max_steer_angle, double steer_ratio) {
  return steering_percentage * 0.01 * max_steer_angle / steer_ratio;  // rad.
}

inline bool IsSteerAngleReached(double steering_wheel_speed, double steer_angle,
                                double target_steer_angle) {
  // TODO(shijun): Modify this value from control side.
  constexpr double kSteerAngleThreshold = 0.1;          // rad.
  constexpr double kSteeringWheelSpeedThreshold = 4.0;  // deg/s.
  return std::abs(target_steer_angle - steer_angle) < kSteerAngleThreshold &&
         std::abs(steering_wheel_speed) < kSteeringWheelSpeedThreshold;
}

// Currently freespace planner has two type tasks: 1. Stop at a place we
// want. 2. Jointly driving with other planner. So gear should be GEAR_DRIVE
// or GEAR_PARKING.
inline bool IsGearMatchedTask(const Chassis::GearPosition& gear_position,
                              const FreespaceTaskProto::TaskType& task_type) {
  bool is_matched = false;
  switch (task_type) {
    case FreespaceTaskProto::UNKNOWN_TASK:
      QLOG(FATAL) << "Unexpected UNKNOWN_TASK.";
    case FreespaceTaskProto::PERPENDICULAR_PARKING:
    case FreespaceTaskProto::PARALLEL_PARKING:
    case FreespaceTaskProto::CUSTOM_PARKING: {
      if (gear_position == Chassis::GEAR_PARKING) {
        is_matched = true;
      }
      break;
    }
    case FreespaceTaskProto::THREE_POINT_TURN:
    case FreespaceTaskProto::DRIVING_TO_LANE:
    case FreespaceTaskProto::FREE_DRIVING: {
      if (gear_position == Chassis::GEAR_DRIVE) {
        is_matched = true;
      }
      break;
    }
  }
  return is_matched;
}

inline bool IsSteeringWheelBackToCenter(
    const VehicleGeometryParamsProto& /*vehicle_geom*/,
    const VehicleDriveParamsProto& vehicle_drive, double steering_wheel_speed,
    double steering_percentage, const PathPoint& /*av_path_point*/) {
  const double steer_angle =
      GetFrontWheelAngle(steering_percentage, vehicle_drive.max_steer_angle(),
                         vehicle_drive.steer_ratio());
  return IsSteerAngleReached(steering_wheel_speed, steer_angle,
                             /*target_steer_angle=*/0.0);
}

}  // namespace

void UpdatePathManagerState(const VehicleGeometryParamsProto& vehicle_geom,
                            const VehicleDriveParamsProto& vehicle_drive,
                            const FreespaceTaskProto::TaskType& task_type,
                            const PoseProto& av_pose, const Chassis& chassis,
                            PathManagerStateProto* state,
                            bool* switched_to_new_path) {
  *switched_to_new_path = false;
  QCHECK_LT(state->curr_path_idx(), state->paths().size());
  VLOG(2) << "Drive_state: "
          << PathManagerStateProto::DriveState_Name(state->drive_state());

  const auto& paths = state->paths();
  switch (state->drive_state()) {
    case PathManagerStateProto::UNKNOWN: {
      QLOG(FATAL) << "Unexpected UNKNOWN state.";
    }
    case PathManagerStateProto::DRIVING: {
      const PathPoint& current_goal =
          *(paths[state->curr_path_idx()].path().rbegin());
      const bool is_last_path_segment =
          (state->curr_path_idx() == paths.size() - 1) ? true : false;
      const bool is_goal_reached =
          IsGoalReached(paths[state->curr_path_idx()], av_pose);
      if (is_goal_reached) {
        if (is_last_path_segment) {
          if (IsSteeringWheelBackToCenter(
                  vehicle_geom, vehicle_drive, chassis.steering_speed(),
                  chassis.steering_percentage(), current_goal) &&
              IsGearMatchedTask(chassis.gear_location(), task_type)) {
            state->set_drive_state(PathManagerStateProto::REACH_FINAL_GOAL);
          } else {
            state->set_drive_state(PathManagerStateProto::CENTER_STEER);
          }
        } else {
          state->set_curr_path_idx(state->curr_path_idx() + 1);
          state->set_drive_state(PathManagerStateProto::SWITCHING_TO_NEXT);
          *switched_to_new_path = true;
        }
      }
    } break;
    case PathManagerStateProto::SWITCHING_TO_NEXT: {
      const PathPoint& current_start_point =
          *(paths[state->curr_path_idx()].path().begin());
      double current_start_kappa = current_start_point.kappa();
      if (!paths[state->curr_path_idx()].forward()) {
        current_start_kappa = -current_start_kappa;
      }
      const double target_front_wheel_angle =
          Kappa2FrontWheelAngle(current_start_kappa, vehicle_geom.wheel_base());
      const double front_wheel_angle = GetFrontWheelAngle(
          chassis.steering_percentage(), vehicle_drive.max_steer_angle(),
          vehicle_drive.steer_ratio());
      const bool is_current_forward = paths[state->curr_path_idx()].forward();
      const bool is_steer_angle_reached =
          IsSteerAngleReached(chassis.steering_speed(), front_wheel_angle,
                              target_front_wheel_angle);
      const bool is_gear_shifted = IsGearShiftedToRefDirection(
          chassis.gear_location(), is_current_forward);

      VLOG(2) << "Target front wheel angle: " << target_front_wheel_angle
              << ". Front wheel angle: " << front_wheel_angle;
      VLOG(2) << "Steer angle reached: "
              << (is_steer_angle_reached ? "true" : "false");
      VLOG(2) << "Gear shifted: " << (is_gear_shifted ? "true" : "false");

      if (is_steer_angle_reached && is_gear_shifted) {
        state->set_drive_state(PathManagerStateProto::DRIVING);
      }
    } break;
    case PathManagerStateProto::CENTER_STEER: {
      const PathPoint& current_goal =
          *(paths[state->curr_path_idx()].path().rbegin());
      if (IsSteeringWheelBackToCenter(
              vehicle_geom, vehicle_drive, chassis.steering_speed(),
              chassis.steering_percentage(), current_goal) &&
          IsGearMatchedTask(chassis.gear_location(), task_type)) {
        state->set_drive_state(PathManagerStateProto::REACH_FINAL_GOAL);
      }
    } break;
    case PathManagerStateProto::REACH_FINAL_GOAL:
      break;
  }
}

absl::StatusOr<PathPoint> GetPathPointFromGlobalIndex(
    absl::Span<const DirectionalPath* const> paths, const int global_index) {
  if (global_index < 0) return absl::InternalError("global_index is negative.");

  int count = 0;
  for (int i = 0; i < paths.size(); ++i) {
    count += paths[i]->path.size();
    if (count >= global_index) {
      int current_seg_index = global_index - (count - paths[i]->path.size());
      return paths[i]->path[current_seg_index];
    }
  }
  return absl::InternalError("global_index overflow.");
}

absl::Status PathSafetyCheck(
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const PoseProto& ego_pose,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const FreespaceMap& freespace_map,
    const google::protobuf::RepeatedPtrField<DirectionalPathProto>& paths,
    int current_index) {
  if (paths.empty()) {
    return absl::OkStatus();
  }
  // Build AV shapes.
  std::vector<VehicleOctagonShape> av_shapes;
  const double offset = 0.5 * (vehicle_geo_params.front_edge_to_center() -
                               vehicle_geo_params.back_edge_to_center());
  for (int i = current_index; i < paths.size(); ++i) {
    // Ignore the excuted part of path.
    double start_s = -1.0;
    if (i == current_index) {
      const DiscretizedPath discrete_path(
          {paths[i].path().begin(), paths[i].path().end()});
      const auto cur_sl = discrete_path.XYToSL(
          Vec2d(ego_pose.pos_smooth().x(), ego_pose.pos_smooth().y()));
      start_s = cur_sl.s;
    }
    for (const auto& pt : paths[i].path()) {
      if (pt.s() < start_s) continue;
      const Vec2d rac(pt.x(), pt.y());
      const double theta =
          paths[i].forward() ? pt.theta() : NormalizeAngle(pt.theta() + M_PI);
      const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
      av_shapes.emplace_back(vehicle_geo_params, vehicle_model_params, rac,
                             rac + offset * tangent, tangent, pt.theta(),
                             0.5 * vehicle_geo_params.length(),
                             0.5 * vehicle_geo_params.width());
    }
  }

  const auto should_consider_mirror =
      [&path_finder_params, &vehicle_model_params](const double object_height) {
        return vehicle_model_params.consider_mirror() &&
               (object_height + path_finder_params.mirror_height_buffer()) >
                   vehicle_model_params.mirror_height();
      };

  constexpr double kExtraBuffer = -0.15;  // m.
  for (const auto& av_shape : av_shapes) {
    for (const auto& traj : stalled_object_trajs) {
      const auto& object_proto = traj->planner_object().object_proto();
      const double object_height =
          object_proto.max_z() - object_proto.ground_z();
      if (av_shape.HasOverlapWithBuffer(
              traj->contour(),
              path_finder_params.object_lateral_buffer() + kExtraBuffer,
              path_finder_params.object_longitudinal_buffer() + kExtraBuffer,
              should_consider_mirror(object_height))) {
        return absl::InternalError(absl::StrFormat(
            "Av has overlap with object %s", traj->object_id()));
      }
    }
    for (const auto& boundary : freespace_map.boundaries) {
      const bool consider_mirror = should_consider_mirror(boundary.height);
      const auto buffers =
          GetVehicleBufferForBoundary(path_finder_params, boundary);
      for (int i = 0; i + 1 < boundary.points.size(); ++i)
        if (av_shape.HasOverlapWithBuffer(
                Segment2d(boundary.points[i], boundary.points[i + 1]),
                buffers.first + kExtraBuffer, buffers.second + kExtraBuffer,
                consider_mirror)) {
          return absl::InternalError(absl::StrFormat(
              "Av has overlap with boundary %s at index %d", boundary.id, i));
        }
    }
  }
  return absl::OkStatus();
}

}  // namespace planner
}  // namespace qcraft
