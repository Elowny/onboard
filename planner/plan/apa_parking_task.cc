#include "onboard/planner/plan/apa_parking_task.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <cmath>
#include <ostream>
#include <string>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "gflags/gflags.h"

#include "onboard/lite/logging.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
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
#include "onboard/proto/assist_driving_system_state.pb.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"

DEFINE_int64(
    apa_task_global_reference_type, 0,
    "Source type of golbal path reference, 0-NONE, 1-HD_MAP, 2-PARKING_SPOT");

namespace qcraft::planner {
namespace {

absl::Status ApaTaskMain(const ApaParkingTaskInput& input,
                         FreespacePlannerStateProto* state,
                         ApaParkingTaskOutput* result,
                         ThreadPool* thread_pool) {
  FreespaceTaskProto::TaskType task_type;
  switch (input.parking_spot_info.type()) {
    case mapping::ParkingSpotInfo::Type::PERPENDICULAR:
      task_type = FreespaceTaskProto::PERPENDICULAR_PARKING;
      break;
    case mapping::ParkingSpotInfo::Type::PARALLEL:
      task_type = FreespaceTaskProto::PARALLEL_PARKING;
      break;
    case mapping::ParkingSpotInfo::Type::CUSTOM:
      task_type = FreespaceTaskProto::CUSTOM_PARKING;
      break;
  }
  state->set_task_type(task_type);

  // APA task only support NONE and PARKING_SPOT as global ref.
  if (FLAGS_apa_task_global_reference_type == 0) {
    state->mutable_global_goal_ref()->set_source_type(
        GlobalGoalReferenceProto::NONE);
  } else if (FLAGS_apa_task_global_reference_type == 2) {
    state->mutable_global_goal_ref()->set_source_type(
        GlobalGoalReferenceProto::PARKING_SPOT);
  } else {
    QLOG(FATAL) << "Unexpected FLAGS_apa_task_global_reference_type: "
                << FLAGS_apa_task_global_reference_type;
  }

  result->debug_proto.mutable_freespace_task_debug()->set_parking_spot_id(
      input.parking_spot_info.id().value());
  result->debug_proto.mutable_freespace_task_debug()->set_task_type(task_type);
  // Scene reasoning in unstructured road. Currently output stalled objects
  // only.
  absl::flat_hash_set<std::string> stalled_objects;
  ASSIGN_OR_RETURN(const auto scene_reasoning_output,
                   RunOffRoadSceneReasoning(OffRoadSceneReasoningInput{
                       .object_mgr = input.object_manager}));
  for (const auto& object_annotation :
       scene_reasoning_output.objects_annotation()) {
    // Check stalled object probability, default threshold value is 0.6.
    constexpr double kStalledProbThreshold = 0.4;
    if (object_annotation.stalled_vehicle_likelyhood() >
        kStalledProbThreshold) {
      stalled_objects.insert(object_annotation.object_id());
      result->debug_proto.add_stalled_object_ids(object_annotation.object_id());
    }
  }

  // TODO(yunfeng): Scene reasoning for stalled cluster objects.
  absl::flat_hash_set<PlannerClusterObject::Id> stalled_cluster_object_ids;

  // Reset goal
  PathPoint path_point_goal;
  if (input.reset || !HasGoal(state->global_goal_ref())) {
    // Construct goal.
    const Vec2d goal_tangent = input.parking_spot_info.unit_direction();
    const double offset = input.veh_geo_params->front_edge_to_center() -
                          input.veh_geo_params->length() * 0.5;
    Vec2d goal_pos =
        input.parking_spot_info.polygon().centroid() - goal_tangent * offset;
    // BANDAID(fengzhuang): This is a hack, adjust goal of parallel parking to
    // eliminate the error of local smoother and control.
    if (task_type == FreespaceTaskProto::PARALLEL_PARKING) {
      constexpr double kAdjustDist = 0.05;  // m.
      const Vec2d start_pos(input.pose->pos_smooth().x(),
                            input.pose->pos_smooth().y());
      goal_pos += goal_tangent.Perp() *
                  std::copysign(kAdjustDist,
                                (start_pos - goal_pos).CrossProd(goal_tangent));
    }

    path_point_goal.set_x(goal_pos.x());
    path_point_goal.set_y(goal_pos.y());
    path_point_goal.set_theta(goal_tangent.Angle());

    // Adjust goal if needed.
    const auto maybe_adjust_goal_for_perpendicular_parking =
        [&input, &stalled_objects](const PathPoint& goal) {
          constexpr double kAdjustStep = 0.1;     // m.
          constexpr double kMaxAdjustDist = 2.0;  // m.
          constexpr double kPlannerBuffer = 0.0;  // m.
          return MaybeAdjustGoal(input.freespace_params->path_finder_params(),
                                 *input.veh_geo_params,
                                 input.vehicle_models_params
                                     ->freespace_vehicle_octagon_model_params(),
                                 /*psmm=*/nullptr, input.object_manager,
                                 stalled_objects, goal,
                                 /*is_parking_task=*/true, /*adjust_dir=*/'F',
                                 kMaxAdjustDist, kAdjustStep, kPlannerBuffer);
        };
    const auto maybe_adjust_goal_for_parallel_parking =
        [&input, &stalled_objects](const PathPoint& goal, char adjust_dir) {
          constexpr double kAdjustStep = 0.05;     // m.
          constexpr double kMaxAdjustDist = 0.8;   // m.
          constexpr double kPlannerBuffer = 0.05;  // m.
          return MaybeAdjustGoal(input.freespace_params->path_finder_params(),
                                 *input.veh_geo_params,
                                 input.vehicle_models_params
                                     ->freespace_vehicle_octagon_model_params(),
                                 /*psmm=*/nullptr, input.object_manager,
                                 stalled_objects, goal,
                                 /*is_parking_task=*/true, adjust_dir,
                                 kAdjustStep, kMaxAdjustDist, kPlannerBuffer);
        };

    const Vec2d start_pos(input.pose->pos_smooth().x(),
                          input.pose->pos_smooth().y());
    const char parallel_adjust_dir =
        (start_pos - goal_pos).CrossProd(goal_tangent) < 0.0 ? 'L' : 'R';

    if (task_type == FreespaceTaskProto::PERPENDICULAR_PARKING) {
      path_point_goal =
          maybe_adjust_goal_for_perpendicular_parking(path_point_goal);
    } else if (task_type == FreespaceTaskProto::PARALLEL_PARKING) {
      path_point_goal = maybe_adjust_goal_for_parallel_parking(
          path_point_goal, parallel_adjust_dir);
    } else if (task_type == FreespaceTaskProto::CUSTOM_PARKING) {
      // For custom parking spot, we can adjust for both direction, and adjust
      // for parallel first if it's likely.
      const bool is_parallel_likely =
          std::abs(NormalizeAngle(path_point_goal.theta() -
                                  input.pose->yaw())) < M_PI_2;
      if (is_parallel_likely) {
        path_point_goal = maybe_adjust_goal_for_parallel_parking(
            path_point_goal, parallel_adjust_dir);
        path_point_goal =
            maybe_adjust_goal_for_perpendicular_parking(path_point_goal);
      } else {
        path_point_goal =
            maybe_adjust_goal_for_perpendicular_parking(path_point_goal);
        path_point_goal = maybe_adjust_goal_for_parallel_parking(
            path_point_goal, parallel_adjust_dir);
      }
    }
  } else {
    path_point_goal = RestoreSmoothGoalFromGlobalRef(
        state->global_goal_ref(),
        /*nullable_coordinate_converter=*/nullptr, &input.parking_spot_info);
  }

  // Set global reference.
  switch (state->global_goal_ref().source_type()) {
    case GlobalGoalReferenceProto::NONE: {
      const auto mutable_ref =
          state->mutable_global_goal_ref()->mutable_none_ref();
      mutable_ref->mutable_smooth_goal()->mutable_pos()->set_x(
          path_point_goal.x());
      mutable_ref->mutable_smooth_goal()->mutable_pos()->set_y(
          path_point_goal.y());
      mutable_ref->mutable_smooth_goal()->set_theta(path_point_goal.theta());
    } break;
    case GlobalGoalReferenceProto::HD_MAP:
      QLOG(FATAL) << "HD_MAP is not allowed as global ref in APA task.";
      break;
    case GlobalGoalReferenceProto::PARKING_SPOT: {
      const auto mutable_ref =
          state->mutable_global_goal_ref()->mutable_parking_spot_ref();
      mutable_ref->set_parking_spot_id(input.parking_spot_info.id().value());
      const Vec2d spot_dir = input.parking_spot_info.unit_direction();
      const Vec2d spot_centroid = input.parking_spot_info.polygon().centroid();
      const Vec2d local_pos =
          (Vec2d(path_point_goal.x(), path_point_goal.y()) - spot_centroid)
              .Rotate(spot_dir.x(), -spot_dir.y());
      local_pos.ToProto(mutable_ref->mutable_spot_local_goal()->mutable_pos());
      mutable_ref->mutable_spot_local_goal()->set_theta(
          NormalizeAngle(path_point_goal.theta() - spot_dir.Angle()));
    } break;
  }

  // Freespace map
  ASSIGN_OR_RETURN(
      const auto freespace_map,
      ConstructFreespaceMap(
          task_type,
          input.freespace_params->path_finder_params().region_half_width(),
          *input.veh_geo_params, /*psmm=*/nullptr, *input.pose,
          &input.parking_spot_info, path_point_goal));

  // ----------------------------------------------------------
  // -------------------- Freespace Planner -------------------
  // ----------------------------------------------------------
  const auto autonomy_apa_state =
      input.autonomy_state->assist_state().assist_apa_state().state();
  const bool force_stop =
      (autonomy_apa_state ==
           AssistApaStateProto::APA_STATE_PARKING_ACTIVE_PAUSE ||
       autonomy_apa_state ==
           AssistApaStateProto::APA_STATE_PARKING_ACTIVE_STANDBY);
  const bool safe_stop =
      (autonomy_apa_state == AssistApaStateProto::APA_STATE_PARKING_SAFE_STOP);
  FreespacePlannerInput freespace_input{
      .new_task = input.reset,
      .force_stop = force_stop,
      .safe_stop = safe_stop,
      .autonomy_state = input.autonomy_state,
      .ego_pose = input.pose,
      .coordinate_converter = nullptr,
      .chassis = input.chassis,
      .obj_mgr = input.object_manager,
      .cluster_obj_mgr = input.cluster_object_manager,
      .psmm = nullptr,
      .stalled_object_ids = &stalled_objects,
      .stalled_cluster_object_ids = &stalled_cluster_object_ids,
      .plan_start_point = &input.plan_start_point_info->start_point,
      .start_point_reset = input.plan_start_point_info->reset,
      .reset_reason = input.plan_start_point_info->reset_reason,
      .plan_time = input.plan_time,
      .freespace_map = &freespace_map,
      .parking_spot_info = &input.parking_spot_info,
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

  // Set APA related trajectory output.
  result->trajectory_info.set_apa_parking_task_finished(
      state->path_manager_state().drive_state() ==
      PathManagerStateProto::REACH_FINAL_GOAL);
  result->trajectory_info.set_apa_path_blocked(
      freespace_planner_output.is_path_blocked);

  return absl::OkStatus();
}

}  // namespace

absl::Status RunApaParkingTask(const ApaParkingTaskInput& input,
                               FreespacePlannerStateProto* state,
                               ApaParkingTaskOutput* result,
                               ThreadPool* thread_pool) {
  // Fill default APA output.
  result->trajectory_info.set_apa_parking_task_finished(false);
  result->trajectory_info.set_apa_plan_failed(false);
  result->trajectory_info.set_apa_path_blocked(false);

  const auto autonomy_apa_state =
      input.autonomy_state->assist_state().assist_apa_state().state();
  switch (autonomy_apa_state) {
    case AssistApaStateProto::APA_STATE_OFF:
    case AssistApaStateProto::APA_STATE_PASSIVE:
    case AssistApaStateProto::APA_STATE_PARKING_FINISH:
    case AssistApaStateProto::APA_STATE_FAULT:
    case AssistApaStateProto::APA_STATE_PARKING_OUT_STANDBY:
    case AssistApaStateProto::APA_STATE_PARKING_OUT_ACTIVE_ON:
    case AssistApaStateProto::APA_STATE_PARKING_OUT_ACTIVE_PAUSE:
    case AssistApaStateProto::APA_STATE_PARKING_OUT_FINISH:
    case AssistApaStateProto::APA_STATE_PARKING_OUT_SAFE_STOP:
    case AssistApaStateProto::APA_STATE_PARKING_OUT_ACTIVE_STANDBY:
      state->Clear();
      return absl::OkStatus();
    case AssistApaStateProto::APA_STATE_SEARCHING:
    case AssistApaStateProto::APA_STATE_PARKING_STANDBY:
    case AssistApaStateProto::APA_STATE_PARKING_ACTIVE_ON:
    case AssistApaStateProto::APA_STATE_PARKING_ACTIVE_PAUSE:
    case AssistApaStateProto::APA_STATE_PARKING_ACTIVE_STANDBY:
    case AssistApaStateProto::APA_STATE_PARKING_SAFE_STOP:
      break;
  }

  const auto status = ApaTaskMain(input, state, result, thread_pool);
  result->debug_proto.set_planner_status(status.ToString());
  if (!status.ok()) {
    QLOG(ERROR) << "Freespace planner fails: " << status.ToString();
    // ----------------------------------------------------------
    // --------------------- AEB Planner-------------------------
    // ----------------------------------------------------------
    result->planner_type = PlannerType::AEB_PLANNER;
    constexpr double kStopDeceleration = -1.0;
    const bool forward =
        (input.chassis->gear_location() != Chassis::GEAR_REVERSE);
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

    result->trajectory_info.set_apa_plan_failed(true);
    state->set_path_unsafe_time(0.0);
    state->set_path_blocked_time(0.0);
  }
  result->planner_type = PlannerType::FREESPACE_PLANNER;
  return absl::OkStatus();
}

}  // namespace  qcraft::planner
