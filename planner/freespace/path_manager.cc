#include "onboard/planner/freespace/path_manager.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <cmath>
#include <iterator>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/planner/freespace/geometry_method/geometry_parking.h"
#include "onboard/planner/freespace/hybrid_a_star/hybrid_a_star.h"
#include "onboard/planner/freespace/path_manager_util.h"
#include "onboard/planner/freespace/sqp_global_path_smoother/sqp_global_path_smoother.h"

DEFINE_bool(enable_geometry_parking_firstly, true,
            "Whether try geometry method firstly in parking task.");

namespace qcraft {
namespace planner {
namespace {

PathPoint ComputeReplanStartPoint(
    const PoseProto& pose, const Chassis& chassis,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  PathPoint start_point;
  start_point.set_x(pose.pos_smooth().x());
  start_point.set_y(pose.pos_smooth().y());
  start_point.set_s(0.0);
  start_point.set_theta(pose.yaw());
  const double pose_v = pose.vel_body().x();

  // At speed lower than this, we don't trust the measured acceleration and
  // angular velocity. Reset like this are mostly when we're stopped anyway.
  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  if (pose_v < kLowSpeedThreshold) {
    const double steer_angle =
        chassis.has_steering_percentage()
            ? chassis.steering_percentage() * 0.01 *
                  vehicle_drive_params.max_steer_angle() /
                  vehicle_drive_params.steer_ratio()
            : 0.0;  // rad.
    const double kappa =
        std::tan(steer_angle) / vehicle_geom_params.wheel_base();
    start_point.set_kappa(kappa);
    start_point.set_lambda(0.0);
  } else {
    start_point.set_kappa(pose.ar_smooth().z() / pose_v);
    start_point.set_lambda(0.0);
  }
  return start_point;
}

void ConvertPathsToCurrentSmooth(const PathPoint& current_smooth_goal,
                                 PathManagerStateProto* path_mgr_state) {
  const auto iter = path_mgr_state->paths().rbegin();
  const double x_offset = current_smooth_goal.x() - iter->path().rbegin()->x();
  const double y_offset = current_smooth_goal.y() - iter->path().rbegin()->y();
  const double prev_goal_theta = iter->forward()
                                     ? iter->path().rbegin()->theta()
                                     : iter->path().rbegin()->theta() + M_PI;
  const double theta_offset =
      NormalizeAngle(current_smooth_goal.theta() - prev_goal_theta);
  double theta_cos_sin[2];
  fast_math::CosAndSin<7>(theta_offset, theta_cos_sin);
  const double sin = theta_cos_sin[1];
  const double cos = theta_cos_sin[0];
  // Convert path by goal.
  for (auto& path : *(path_mgr_state->mutable_paths())) {
    for (auto& point : *(path.mutable_path())) {
      // Move points to make prev goal meet current goal.
      const double x = point.x() + x_offset;
      const double y = point.y() + y_offset;
      // Rotate path around current smooth goal.
      point.set_x(cos * (x - current_smooth_goal.x()) -
                  sin * (y - current_smooth_goal.y()) +
                  current_smooth_goal.x());
      point.set_y(sin * (x - current_smooth_goal.x()) +
                  cos * (y - current_smooth_goal.y()) +
                  current_smooth_goal.y());
      point.set_theta(NormalizeAngle(point.theta() + theta_offset));
    }
  }
}

}  // namespace

absl::StatusOr<PathManagerOutput> GeneratePath(
    FreespaceReplanReasonProto::ReplanReason replan_reason,
    FreespaceTaskProto::TaskType task_type, const PoseProto& ego_pose,
    const Chassis& chassis,
    const FreespacePathFinderParamsProto& path_finder_params,
    const SqpSmootherParamsProto& sqp_smoother_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const PlannerClusterObjectManager& /*cluster_obj_mgr*/,
    const absl::flat_hash_set<PlannerClusterObject::Id>&
    /*stalled_cluster_object_ids*/,
    const FreespaceMap& freespace_map, const PathPoint& goal,
    const mapping::ParkingSpotInfo* nullable_parking_spot_info,
    PathManagerStateProto* path_mgr_state,
    PathFinderDebugProto* path_finder_debug) {
  SCOPED_QTRACE("GeneratePath");
  // Convert paths to current smooth origin if there is old path.
  if (path_mgr_state->paths().size() > 0) {
    ConvertPathsToCurrentSmooth(goal, path_mgr_state);
  }

  bool is_new_path = false;
  if (replan_reason != FreespaceReplanReasonProto::NONE) {
    std::vector<DirectionalPath> new_path;

    const auto start_point = ComputeReplanStartPoint(
        ego_pose, chassis, veh_geo_params, veh_drive_params);

    const auto start_time = absl::Now();
    bool geometry_path_ok = false;
    if (FLAGS_enable_geometry_parking_firstly &&
        (task_type == FreespaceTaskProto::PARALLEL_PARKING ||
         task_type == FreespaceTaskProto::PERPENDICULAR_PARKING)) {
      QCHECK_NOTNULL(nullable_parking_spot_info);
      if (auto geometry_paths = FindSinglePath(
              path_finder_params, veh_geo_params, veh_drive_params,
              vehicle_model_params, task_type, replan_reason, freespace_map,
              stalled_object_trajs, nullable_parking_spot_info, start_point,
              goal, path_finder_debug);
          geometry_paths.ok()) {
        path_finder_debug->set_finder_type(PathFinderDebugProto::GEOMETRY);
        new_path = std::move(*geometry_paths);
        is_new_path = true;
        geometry_path_ok = true;
      }
    }

    if (!geometry_path_ok) {
      path_finder_debug->set_finder_type(PathFinderDebugProto::HYBRID_A_STAR);
      if (auto paths = FindPath(path_finder_params, veh_geo_params,
                                veh_drive_params, vehicle_model_params,
                                task_type, freespace_map, stalled_object_trajs,
                                start_point, goal, path_finder_debug);
          paths.ok()) {
        new_path = std::move(*paths);
        is_new_path = true;
      } else {
        // Always use prev path When replan failed.
        if (path_mgr_state->paths().size() > 0) {
          is_new_path = false;
        } else {
          return paths.status();
        }
      }
    }
    path_finder_debug->set_find_path_time_consuming_ms(
        absl::ToDoubleMilliseconds(absl::Now() - start_time));

    // Smooth the coarse path globally into denser second-order continuous path.
    const bool suitable_for_global_smoother = !geometry_path_ok && is_new_path;
    if (suitable_for_global_smoother) {
      auto smooth_paths = sqp_global_smoother::SmoothGlobalPath(
          new_path, freespace_map, stalled_object_trajs, veh_geo_params,
          veh_drive_params, sqp_smoother_params, vehicle_model_params,
          path_finder_params);
      if (!smooth_paths.ok()) {
        QLOG(WARNING) << "SQP global path smoother fails: "
                      << smooth_paths.status();
      } else {
        new_path = std::move(*smooth_paths);
        is_new_path = true;
      }
    }

    // Update path in path manager state.
    if (is_new_path) {
      path_mgr_state->clear_paths();
      for (const auto& path : new_path) {
        path.ToProto(path_mgr_state->add_paths());
      }
      path_mgr_state->set_curr_path_idx(0);
      path_mgr_state->set_drive_state(PathManagerStateProto::SWITCHING_TO_NEXT);
    }
  }

  bool switched_to_new_path = false;
  UpdatePathManagerState(veh_geo_params, veh_drive_params, task_type, ego_pose,
                         chassis, path_mgr_state, &switched_to_new_path);
  is_new_path |= switched_to_new_path;

  DirectionalPath output_path;
  output_path.FromProto(
      path_mgr_state->paths().at(path_mgr_state->curr_path_idx()));
  return PathManagerOutput(
      {.path = std::move(output_path), .is_new_path = is_new_path});
}

}  // namespace planner
}  // namespace qcraft
