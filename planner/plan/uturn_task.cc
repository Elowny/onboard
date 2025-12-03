#include "onboard/planner/plan/uturn_task.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <ostream>
#include <string>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/driving_state.h"
#include "onboard/planner/emergency_stop.h"
#include "onboard/planner/freespace/directional_path.h"
#include "onboard/planner/freespace/freespace_constraint_builder.h"
#include "onboard/planner/freespace/freespace_planner.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/object/planner_cluster_object.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/scene/off_road_scene_reasoning.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

absl::Status UTurnTaskMain(const UTurnTaskInput& input,
                           FreespacePlannerStateProto* state,
                           UTurnTaskOutput* result, ThreadPool* thread_pool) {
  FreespaceTaskProto::TaskType task_type = FreespaceTaskProto::THREE_POINT_TURN;
  state->set_task_type(task_type);
  result->debug_proto.mutable_freespace_task_debug()->set_task_type(task_type);
  // Currently uturn task only support HD_MAP as global source.
  state->mutable_global_goal_ref()->set_source_type(
      GlobalGoalReferenceProto::HD_MAP);
  QCHECK_NOTNULL(input.coordinate_converter);
  QCHECK_NOTNULL(input.goal);
  const auto mutable_ref =
      state->mutable_global_goal_ref()->mutable_hd_map_ref();
  mutable_ref->mutable_global_goal()->mutable_pos()->set_x(input.goal->pos.x());
  mutable_ref->mutable_global_goal()->mutable_pos()->set_y(input.goal->pos.y());
  mutable_ref->mutable_global_goal()->set_theta(input.goal->heading);

  // Scene reasoning in unstructured road. Currently output stalled objects
  // only.
  absl::flat_hash_set<std::string> stalled_objects;
  ASSIGN_OR_RETURN(const auto scene_reasoning_output,
                   RunOffRoadSceneReasoning(OffRoadSceneReasoningInput{
                       .object_mgr = input.object_manager}));
  for (const auto& object_annotation :
       scene_reasoning_output.objects_annotation()) {
    // Check stalled object probability, default threshold value is 0.6.
    constexpr double kStalledProbThreshold = 0.6;
    if (object_annotation.stalled_vehicle_likelyhood() >
        kStalledProbThreshold) {
      stalled_objects.insert(object_annotation.object_id());
      result->debug_proto.add_stalled_object_ids(object_annotation.object_id());
    }
  }

  // TODO(yunfeng): Scene reasoning for stalled cluster objects.
  absl::flat_hash_set<PlannerClusterObject::Id> stalled_cluster_object_ids;

  // Reset goal
  if (input.reset) {
    const PathPoint smooth_goal = RestoreSmoothGoalFromGlobalRef(
        state->global_goal_ref(), input.coordinate_converter,
        /*nullable_parking_spot_info=*/nullptr);
    constexpr double kMaxAdjustDist = 1.0;  // m.
    constexpr double kAdjustStep = 0.2;     // m.
    constexpr double kPlannerBuffer = 0.0;  // m.
    const auto new_smooth_goal = MaybeAdjustGoal(
        input.freespace_params->path_finder_params(), *input.veh_geo_params,
        input.vehicle_models_params->freespace_vehicle_octagon_model_params(),
        input.psmm, input.object_manager, stalled_objects, smooth_goal,
        /*is_parking_task=*/false, /*adjust_dir=*/'L', kMaxAdjustDist,
        kAdjustStep, kPlannerBuffer);
    // Update gobal goal in state.
    const auto mutable_ref =
        state->mutable_global_goal_ref()->mutable_hd_map_ref();
    const auto global_pos = input.coordinate_converter->SmoothToGlobal(
        Vec2d(new_smooth_goal.x(), new_smooth_goal.y()));
    global_pos.ToProto(mutable_ref->mutable_global_goal()->mutable_pos());
    mutable_ref->mutable_global_goal()->set_theta(
        input.coordinate_converter->SmoothYawToGlobalNoNormalize(
            new_smooth_goal.theta()));
  }

  // Freespace map
  const auto goal = RestoreSmoothGoalFromGlobalRef(
      state->global_goal_ref(), input.coordinate_converter,
      /*nullable_parking_spot_info=*/nullptr);
  ASSIGN_OR_RETURN(
      auto freespace_map,
      ConstructFreespaceMap(
          task_type,
          input.freespace_params->path_finder_params().region_half_width(),
          *input.veh_geo_params, input.psmm, *input.pose,
          /*parking_spot_info=*/nullptr, goal));

  // Add additional boundary.
  if (input.lane_path != nullptr) {
    AddUTurnBoundary(*input.psmm, input.lane_path, *input.veh_geo_params,
                     &freespace_map);
  }

  // ----------------------------------------------------------
  // -------------------- Freespace Planner -------------------
  // ----------------------------------------------------------
  FreespacePlannerInput freespace_input{
      .new_task = input.reset,
      .force_stop = false,
      .safe_stop = false,
      .autonomy_state = input.autonomy_state,
      .ego_pose = input.pose,
      .coordinate_converter = input.coordinate_converter,
      .chassis = input.chassis,
      .obj_mgr = input.object_manager,
      .cluster_obj_mgr = input.cluster_object_manager,
      .psmm = input.psmm,
      .stalled_object_ids = &stalled_objects,
      .stalled_cluster_object_ids = &stalled_cluster_object_ids,
      .plan_start_point = &input.plan_start_point_info->start_point,
      .start_point_reset = input.plan_start_point_info->reset,
      .reset_reason = input.plan_start_point_info->reset_reason,
      .plan_time = input.plan_time,
      .freespace_map = &freespace_map,
      .freespace_params = input.freespace_params,
      .vehicle_models_params = input.vehicle_models_params,
      .veh_geo_params = input.veh_geo_params,
      .veh_drive_params = input.veh_drive_params,
      .time_interval = input.time_interval};

  ASSIGN_OR_RETURN(
      const auto freespace_planner_output,
      RunFreespacePlanner(freespace_input, state, &result->debug_proto,
                          &result->chart_data, thread_pool));

  const auto driving_state = GetOffRoadDrivingState(
      state->task_type(), state->path_manager_state().drive_state(),
      input.plan_start_point_info->full_stop);
  const auto past_points = CreatePastPointsList(
      input.plan_time, *input.prev_trajectory_proto,
      input.plan_start_point_info->reset, kMaxPastPointNum);
  const auto past_directional_path_points = CreatePastDirectionalPathPoints(
      *input.prev_trajectory_proto, input.plan_start_point_info->reset,
      input.plan_start_point_info->path_s_increment_from_previous_frame,
      freespace_planner_output.reset);

  result->trajectory_info = CreateFreespaceTrajectoryProto(
      input.plan_time, freespace_planner_output.traj_points, past_points,
      freespace_planner_output.gear_position, driving_state,
      freespace_planner_output.low_speed_freespace,
      freespace_planner_output.enable_stationary_steering,
      freespace_planner_output.smooth_directional_path,
      freespace_planner_output.stop_s, past_directional_path_points);

  result->reset = freespace_planner_output.reset;
  result->reset_reason = freespace_planner_output.reset_reason;

  return absl::OkStatus();
}

}  // namespace

absl::Status RunUTurnTask(const UTurnTaskInput& input,
                          FreespacePlannerStateProto* state,
                          UTurnTaskOutput* result, ThreadPool* thread_pool) {
  const auto status = UTurnTaskMain(input, state, result, thread_pool);
  result->debug_proto.set_planner_status(status.ToString());
  if (!status.ok()) {
    QLOG(ERROR) << "Freespace planner fails: " << status.ToString();
    // ----------------------------------------------------------
    // --------------------- AEB Planner-------------------------
    // ----------------------------------------------------------
    result->planner_type = PlannerType::AEB_PLANNER;
    constexpr double kStopDeceleration = -1.0;
    const bool forward =
        input.chassis->gear_location() != Chassis::GEAR_REVERSE;
    const auto output_traj = aeb::GenerateStopTrajectory(
        input.plan_start_point_info->path_s_increment_from_previous_frame,
        input.plan_start_point_info->reset, forward, kStopDeceleration,
        input.freespace_params->motion_constraint_params(),
        TrajectoryPoint(input.plan_start_point_info->start_point),
        {input.prev_trajectory_proto->trajectory_point().begin(),
         input.prev_trajectory_proto->trajectory_point().end()});
    const auto gear_position = input.chassis->gear_location();
    DrivingStateProto driving_state;
    driving_state.set_type(DrivingStateProto::STOPPED_WHEN_PARKING);
    const auto past_points = CreatePastPointsList(
        input.plan_time, *input.prev_trajectory_proto,
        input.plan_start_point_info->reset, kMaxPastPointNum);

    result->trajectory_info = CreateFreespaceTrajectoryProto(
        input.plan_time, output_traj, past_points, gear_position, driving_state,
        /*low_speed_freespace=*/false, /*enable_stationary_steering=*/false,
        DirectionalPath(), /*stop_s=*/0.0, /*past_directional_path_points=*/{});
  }
  result->planner_type = PlannerType::FREESPACE_PLANNER;
  return absl::OkStatus();
}

}  // namespace  qcraft::planner
