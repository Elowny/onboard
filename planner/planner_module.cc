#include "onboard/planner/planner_module.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <google/protobuf/descriptor.h>
// IWYU pragma: no_include <cxxabi.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <optional>
#include <queue>
#include <string_view>
#include <tuple>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "common/proto/semantic_map_modifier.pb.h"

#include "onboard/async/async_util.h"
#include "onboard/autonomy_state/assist_state_util.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/counter.h"
#include "onboard/global/run_context.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/hmi/states/run_event_states_util.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/semantic_map_io.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/params/v2/proto/vehicle/installation.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/assist_util.h"
#include "onboard/planner/assist/proto/external_command_status.pb.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/manual_trajectory_util.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/ml/planner_ml_defs.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/plan/plan_task.h"
#include "onboard/planner/plan/plan_task_dispatcher.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_params_builder.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_sim_flags.h"
#include "onboard/planner/planner_state_util.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/standby_state.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/prediction_message_compressor.h"
#include "onboard/proto/assist_driving_system_state.pb.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/proto/parking_spot_finder.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/q_assist_settings.pb.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/remote_assist_common.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/history_buffer.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

#include "offboard/vis/ark/ark_server/ark_client_man.h"
#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

// Example saved trajectory:
// onboard/control/testdata/test_trajectory_garage_circle.pb.txt
DEFINE_string(planner_load_saved_trajectory, "",
              "If not null, publish a saved trajectory once instead of "
              "planning online.");

// Ref: https://qcraft.atlassian.net/wiki/spaces/WEIT/pages/1224015930/snapshot
DEFINE_bool(restore_from_snapshot, false,
            "Restore planner module state from planner_state_proto of playback "
            "source when engage is triggered in simulation.");

DEFINE_bool(planner_enable_objects_histories_buffer, false,
            "Whether to record object's histories for aeb planner");

namespace qcraft::planner {
namespace {

PlannerStatus LoadSavedTrajectory(
    const std::string& saved_traj_path,
    const CoordinateConverter& coordinate_converter,
    const AutonomyStateProto& autonomy_state,
    TrajectoryProto* prev_trajectory_in_global,
    TrajectoryProto* trajectory_in_smooth) {
  if (prev_trajectory_in_global->trajectory_point_size() == 0 &&
      prev_trajectory_in_global->past_points_size() == 0) {
    if (!file_util::FileToProto(saved_traj_path, prev_trajectory_in_global)) {
      return PlannerStatus(
          PlannerStatusProto::INPUT_INCORRECT,
          absl::StrCat("Unable to load trajectory file: ", saved_traj_path));
    }

    CompleteTrajectoryPastPoints(kTrajectoryTimeStep,
                                 prev_trajectory_in_global);
    UpdateTrajectoryPointAccel(prev_trajectory_in_global);
  }

  if (IS_AUTO_DRIVE(autonomy_state.autonomy_state())) {
    ShiftPreviousTrajectory(FLAGS_planner_main_loop_interval,
                            prev_trajectory_in_global);
  }

  *trajectory_in_smooth = *prev_trajectory_in_global;

  ConvertGlobalTrajectoryToSmooth(coordinate_converter, trajectory_in_smooth);

  trajectory_in_smooth->set_trajectory_start_timestamp(
      ToUnixDoubleSeconds(Clock::Now()));

  trajectory_in_smooth->mutable_header()->set_seq_number(2);
  return OkPlannerStatus();
}

void FillInputIterationNumToPlannerState(
    const PlannerInput& input, int64_t run_event_state_seq_num,
    PlannerStateProto* planner_state_proto) {
  FUNC_QTRACE();

  auto* input_seq_num = planner_state_proto->mutable_input_seq_num();
#define SET_SEQ_NUM(NAME)                                           \
  do {                                                              \
    if (input.NAME != nullptr) {                                    \
      input_seq_num->set_##NAME(input.NAME->header().seq_number()); \
    }                                                               \
  } while (false)

  SET_SEQ_NUM(pose);
  SET_SEQ_NUM(av_objects);
  SET_SEQ_NUM(real_objects);
  SET_SEQ_NUM(virtual_objects);
  SET_SEQ_NUM(autonomy_state);
  SET_SEQ_NUM(traffic_light_states);
  SET_SEQ_NUM(driver_action);
  SET_SEQ_NUM(remote_assist_to_car);
  SET_SEQ_NUM(chassis);
  SET_SEQ_NUM(localization_transform);
  SET_SEQ_NUM(prediction);
  SET_SEQ_NUM(route_mgr_output);
  SET_SEQ_NUM(sensor_fovs);
  SET_SEQ_NUM(online_semantic_map);
#undef SET_SEQ_NUM

  input_seq_num->set_run_event_states(run_event_state_seq_num);
}

void ParsePlannerInputToDebugProto(PlannerDebugProto* mutable_debug,
                                   const PlannerInput& planner_input) {
  if (planner_input.pose != nullptr) {
    *mutable_debug->mutable_planner_input()->mutable_pose() =
        *planner_input.pose;
  }
  if (planner_input.chassis != nullptr) {
    *mutable_debug->mutable_planner_input()->mutable_chassis() =
        *planner_input.chassis;
  }
  if (planner_input.localization_transform != nullptr) {
    *mutable_debug->mutable_planner_input()->mutable_loc_transform() =
        *planner_input.localization_transform;
  }
  if (planner_input.real_objects != nullptr) {
    *mutable_debug->mutable_planner_input()->mutable_real_objects() =
        *planner_input.real_objects;
  }
  if (planner_input.prediction != nullptr) {
    *mutable_debug->mutable_planner_input()->mutable_prediction() =
        *planner_input.prediction;
  }
}

void FillAvLtHistories(const PoseProto& pose,
                       const LocalizationTransformProto& lt,
                       HistoryTransformProto* history_transform) {
  if (history_transform->av_seq_size() == ml::kAvHistoryBufferSize) {
    // All elemens' size are equal.Improve performance by use circular_buffer
    auto erase_first_fn = [](auto* repeated_field) {
      std::rotate(repeated_field->begin(), repeated_field->begin() + 1,
                  repeated_field->end());
      repeated_field->erase(repeated_field->begin() + repeated_field->size() -
                            1);
    };
    erase_first_fn(history_transform->mutable_av_seq());
    erase_first_fn(history_transform->mutable_lt_seq());
  }
  history_transform->mutable_av_seq()->Add(pose.header().seq_number());
  history_transform->mutable_lt_seq()->Add(lt.header().seq_number());
}

void UpdateObjectHistoryBuffer(
    const std::shared_ptr<const ObjectsProto>& real_objects,
    const std::shared_ptr<const ObjectsProto>& virtual_objects,
    PlannerState::ObjectsHistoryMap* object_history_map) {
  const auto generate_object_motion_state = [](const ObjectProto& object) {
    PlannerState::ObjectMotionState object_motion_state;
    object_motion_state.heading = object.yaw();
    object_motion_state.accel.FromProto(object.accel());
    object_motion_state.vel.FromProto(object.vel());
    object_motion_state.pos.FromProto(object.pos());
    return object_motion_state;
  };
  if (real_objects != nullptr) {
    for (const auto& obj : real_objects->objects()) {
      (*object_history_map)[obj.id()].push_back(
          absl::FromUnixMicros(real_objects->header().timestamp()),
          generate_object_motion_state(obj));
    }
  }
  if (virtual_objects != nullptr) {
    for (const auto& obj : virtual_objects->objects()) {
      (*object_history_map)[obj.id()].push_back(
          absl::FromUnixMicros(virtual_objects->header().timestamp()),
          generate_object_motion_state(obj));
    }
  }
  constexpr auto kObjectHistoryStaleTime = absl::Seconds(1);  // s.
  const auto now = Clock::Now();
  for (auto& [_, buffer] : *object_history_map) {
    buffer.ClearOlderThanRefTime(now, kObjectHistoryStaleTime);
  }
}

inline bool CheckPreEnterStandbyState(
    const RouteManagerOutputProto* route_mgr_output,
    const RouteSections& prev_route_sections, const PoseProto* pose) {
  return IsRunModeL4() &&
         (route_mgr_output == nullptr ||
          IsResetRouteOutputProto(*route_mgr_output) ||
          (!IsValidRouteOutputProto(*route_mgr_output) &&
           prev_route_sections.empty())) &&
         pose != nullptr;
}

inline bool CheckPostEnterStandbyState(const PlanTask& current_plan_task,
                                       const PoseProto& pose,
                                       const Chassis& chassis,
                                       const TrajectoryProto& trajectory) {
  constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
  return !current_plan_task.IsFreespacePlanTask() &&
         std::abs(pose.vel_body().x()) < kFullStopSpeedThreshold &&
         chassis.gear_location() != trajectory.gear();
}

}  // namespace

absl::Status PlannerModule::CheckIfDriverCanEngage(
    const TrajectoryProto& trajectory) {
  if (trajectory.trajectory_point_size() == 0) {
    if (trajectory.low_speed_freespace() &&
        FLAGS_planner_freespace_path_stop_mode) {
      return absl::OkStatus();
    } else {
      QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
              QIssueSubType::QIST_PLANNER_PROTO_TRAJECTORY_EMPTY,
              "Trajectory points are empty.");
      return absl::NotFoundError(
          "Empty trajectory, no trajectory points found.");
    }
  }

  // The following checks assume non-empty trajectory points.
  // When vehicle is stopped, allow to engage.
  constexpr double kLowSpeedToAllowEngage = 2.0;  // m/s.
  if (std::abs(trajectory.trajectory_point(0).v()) < kLowSpeedToAllowEngage) {
    return absl::OkStatus();
  }

  // Look forward check if the curvature is too high.
  // Find the point near kLookForwardTime.
  int i = 0;
  while (i < trajectory.trajectory_point_size() &&
         trajectory.trajectory_point(i).relative_time() <
             FLAGS_planner_check_trajectory_engage_condition_duration) {
    ++i;
  }
  if (i >= trajectory.trajectory_point_size()) {
    QISSUEX(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_PREVENT_ENGAGE,
        QIssueSubType::QIST_PLANNER_TRAJECTORY_TOO_SHORT,
        absl::StrCat("Trajectory's duration is less than ",
                     FLAGS_planner_check_trajectory_engage_condition_duration));
    return absl::NotFoundError("Trajectory duration is too short.");
  }

  // See https://qcraft.atlassian.net/browse/DEVTEST-719
  const PiecewiseLinearFunction<double> speed_to_max_kappa(
      /*x=*/{5.0, 10.0, 30.0},      // Speed.
      /*y=*/{0.015, 0.01, 0.004});  // Max kappa.

  // Do not allow engage if kappa is larger than a threshold.
  for (int j = 0; j < i; ++j) {
    const double abs_speed = std::abs(trajectory.trajectory_point(j).v());
    const double abs_kappa =
        std::abs(trajectory.trajectory_point(j).path_point().kappa());
    const double allowed_kappa = speed_to_max_kappa(abs_speed);
    if (abs_kappa > allowed_kappa) {
      const std::string args_msg = absl::StrFormat(
          "The trajectory point[%d]'s kappa %f (abs value) is larger than "
          "allowed_kappa [%f]. Change it to a larger value if you want to "
          "engage in this situation.",
          j, abs_kappa, allowed_kappa);
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL,
                        QIssueType::QIT_PREVENT_ENGAGE,
                        QIssueSubType::QIST_PLANNER_TRAJECTORY_KAPPA_TOO_LARGE,
                        "Trajectory's kappa is too large", args_msg);
      return absl::FailedPreconditionError(args_msg);
    }
  }

  // TODO(all): Add other cases that we should not allow engage.

  return absl::OkStatus();
}

void PlannerModule::OnInit() {
  last_iteration_time_ = absl::FromUnixMicros(0L);
  ark_client_man::CreateArkClientMan();
  RunParamsProtoV2 run_params;
  param_manager().GetRunParams(&run_params);
  onboard_input_.vehicle_params = run_params.vehicle_params();
  onboard_input_.prediction_conflict_resolver_params.LoadParams();
  ASSIGN_OR_DIE(onboard_input_.planner_params,
                BuildPlannerParams(
                    onboard_input_.vehicle_params.vehicle_geometry_params(),
                    onboard_input_.vehicle_params.vehicle_params().model(),
                    onboard_input_.vehicle_params.vehicle_params()
                        .installation()
                        .vehicle_plan()));

  if (FLAGS_planner_thread_pool_size > 0) {
    thread_pool_ = std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);
  }
  pub_thread_pool_ = std::make_unique<ThreadPool>(1);

  onboard_input_.planner_model_pool =
      std::make_unique<ModelPool>(param_manager(), param_finder());

  onboard_input_.av_context = std::make_unique<prediction::AvContext>(
      ml::kAvHistoryBufferSize, ml::kAvHistoryBufferLen);

  onboard_input_.traffic_light_states =
      std::make_shared<TrafficLightStatesProto>();

  planner_state_.planner_frame_seq_num = 0;
  planner_state_.lane_change_state = MakeNoneLaneChangeState();

  planner_state_.last_audio_alert_time = absl::UnixEpoch();
  planner_state_.parking_brake_release_time = absl::InfinitePast();

  planner_state_to_proto_future_ = ScheduleFuture(
      static_cast<ThreadPool*>(nullptr),
      [this]() -> std::shared_ptr<PlannerStateProto> {
        auto planner_state_proto = std::make_shared<PlannerStateProto>();
        planner_state_.ToProto(planner_state_proto.get());
        return planner_state_proto;
      });

  // Initialize the first output state proto from context_state. The reason is
  // that the first output_state proto will be used to initialize the first
  // input.state_proto at the beginning of the MainLoop function.
  {
    absl::MutexLock lock(&output_mutex_);
    planner_state_.ToProto(onboard_output_->mutable_planner_state_proto());
  }

  onboard_input_.semantic_map_manager = nullptr;
  onboard_input_.semantic_map_multilevel_spatial_index = nullptr;

  if (!IsPlannerSnapshotMode()) {
    onboard_input_.planner_semantic_map_manager = nullptr;
  }

  hd_semantic_map_listener_ =
      mapping::v2::SemanticMapMultilevelSpatialIndexListenerAsnyc::MakeShared(
          {OnlineMapProto_DataSource_QCRAFT_HDMAP,
           OnlineMapProto_DataSource_NAVINFO_HDMAP});
  mapping::BuildOption build_option = {};
  hd_semantic_map_listener_->SetLoadOption({
      .build_option = build_option,
  });
  if (FLAGS_planner_force_route_filtered_smm) {
    hd_semantic_map_listener_->EnableRouteFilter(/*enable=*/true);
  }

  planner_state_to_proto_future_.Wait();

  QLOG(INFO) << " Planner module OnInit() success.";
}

void PlannerModule::OnSubscribeChannels() {
  // NOTE: buffer size=1 means always take the lastest message.
  Subscribe(&PlannerModule::HandlePose, this, /*buffer_size=*/1);
  Subscribe(&PlannerModule::HandlePrediction, this, 20);
  Subscribe(&PlannerModule::HandlePredictionDebug, this, 20);
  Subscribe(&PlannerModule::HandleLocalizationTransform, this, 20);
  Subscribe(&PlannerModule::HandleObjects, this, 20);
  Subscribe(&PlannerModule::HandleAutonomyState, this, 20);
  Subscribe(&PlannerModule::HandleTrafficLightStates, this, 20);
  Subscribe(&PlannerModule::HandleDriverAction, this, 25);
  Subscribe(&PlannerModule::HandleRemoteAssistToCar, this, 20);
  Subscribe(&PlannerModule::HandleRoutingManagerOutputResult, this, 20);
  Subscribe(&PlannerModule::HandleChassis, this, 25);
  Subscribe(&PlannerModule::HandleSemanticMapPatch, this, 20);
  Subscribe(&PlannerModule::HandleRunEventStates, this, 20);
  Subscribe(&PlannerModule::HandlePlannerState, this, "log_planner_state_proto",
            20);
  Subscribe(&PlannerModule::HandleOraclePrediction, this,
            "oracle_objects_prediction_proto", 20);
  Subscribe(&PlannerModule::HandleOracleAvTrajectory, this,
            "oracle_trajectory_proto", 20);
  Subscribe(&PlannerModule::HandleSensorFovs, this, 20);
  Subscribe(&PlannerModule::HandleDriverCommand, this, 25);
  Subscribe(&PlannerModule::HandleOnlineSemanticMap, this, 20);
  Subscribe(&PlannerModule::HandleParkingSpotFinder, this, 20);
  Subscribe(&PlannerModule::HandleFusionFusionParkingFreespace, this, 20);
  Subscribe(&PlannerModule::HandleQRunEvents, this, 20);
  Subscribe(&PlannerModule::HandleOnlineMapProto, this, 20);
}

void PlannerModule::HandlePlannerState(
    std::shared_ptr<const PlannerStateProto> planner_state) {
  playback_planner_state_ = std::move(planner_state);
}

void PlannerModule::HandlePrediction(
    const std::shared_ptr<const ObjectsPredictionProto>& prediction) {
  FUNC_QTRACE();
  auto decompressed_prediction =
      std::make_unique<ObjectsPredictionProto>(*prediction);
  prediction::DecompressObjectsPredictionProto(decompressed_prediction.get());
  DestroyContainerAsync(std::move(onboard_input_.prediction));
  onboard_input_.prediction = std::move(decompressed_prediction);
}

void PlannerModule::HandlePredictionDebug(
    std::shared_ptr<const PredictionDebugProto> prediction_debug) {
  onboard_input_.prediction_debug = std::move(prediction_debug);
}

void PlannerModule::HandleOraclePrediction(
    const std::shared_ptr<const ObjectsPredictionProto>& oracle_prediction) {
  auto decompressed_prediction = *oracle_prediction;
  prediction::DecompressObjectsPredictionProto(&decompressed_prediction);
  onboard_input_.log_prediction =
      std::make_unique<const ObjectsPredictionProto>(
          std::move(decompressed_prediction));
}

void PlannerModule::HandleOracleAvTrajectory(
    std::shared_ptr<const TrajectoryProto> oracle_av_trajectory) {
  onboard_input_.log_av_trajectory = std::move(oracle_av_trajectory);
}

void PlannerModule::HandleSensorFovs(
    std::shared_ptr<const SensorFovsProto> sensor_fovs) {
  onboard_input_.sensor_fovs = std::move(sensor_fovs);
}

void PlannerModule::OnSetUpTimers() {}

PlannerModule::PlannerModule(LiteClientBase* client) : LiteModule(client) {}

void PlannerModule::HandlePose(std::shared_ptr<const PoseProto> pose) {
  const auto now = Clock::Now();
  const double pose_delay_seconds =
      ToUnixDoubleSeconds(now) - pose->timestamp();
  SCOPED_QTRACE_ARG2("PlannerModule::HandlePose", "now",
                     ToUnixDoubleSeconds(now), "pose_delay_ms",
                     pose_delay_seconds * 1000);

  QCOUNTER("planner_pose_delay_ms",
           static_cast<int64_t>(pose_delay_seconds * 1000));

  onboard_input_.pose = std::move(pose);

  constexpr double kReasonablePoseDelay = 0.03;  // seconds.
  if (pose_delay_seconds > kReasonablePoseDelay) {
    QLOG_EVERY_N_SEC(WARNING, 1)
        << "The pose is stale. delay: " << pose_delay_seconds << ".";
  }
  if (pose_delay_seconds > FLAGS_planner_max_pose_delay) {
    // Wait for the next pose.
    QLOG_EVERY_N_SEC(ERROR, 1)
        << "Refuse to plan: the pose is stale. delay: " << pose_delay_seconds
        << ".";
    return;
  }
  const auto duration_since_last_iteration = now - last_iteration_time_;
  const auto planner_interval = absl::Seconds(FLAGS_planner_main_loop_interval);
  if (duration_since_last_iteration < planner_interval) {
    // Not ready for the next iteration.
    QLOG_EVERY_N_SEC(WARNING, 1)
        << "The duration since last iteration is invalid. "
        << duration_since_last_iteration << ", now: " << now
        << ", last: " << last_iteration_time_ << ".";
    return;
  }
  if (absl::ToDoubleSeconds(duration_since_last_iteration) >
      FLAGS_planner_max_allowed_iteration_time) {
    QISSUEX(QIssueSeverity::QIS_ERROR, QIssueType::QIT_PERFORMANCE,
            QIssueSubType::QIST_PLANNER_PROCESS_TIMEOUT, "Planner timeout.");
  }

  MainLoop();
  const auto iteration_time = Clock::Now() - now;
  if (iteration_time > planner_interval + absl::Milliseconds(50.0)) {
    QEVENT("lidong", "planner_timeout", [iteration_time](QEvent* event) {
      event->AddField("duration", absl::ToDoubleSeconds(iteration_time));
    });
  }

  last_iteration_time_ = now;
}

void PlannerModule::HandleLocalizationTransform(
    std::shared_ptr<const LocalizationTransformProto>
        localization_transform_proto) {
  SCOPED_QTRACE("PlannerModule::HandleLocalizationTransform");
  onboard_input_.localization_transform =
      std::move(localization_transform_proto);
}

void PlannerModule::HandleObjects(std::shared_ptr<const ObjectsProto> objects) {
  FUNC_QTRACE();
  QCHECK(objects && objects->has_scope());
  switch (objects->scope()) {
    case ObjectsProto::SCOPE_REAL: {
      DestroyContainerAsync(std::move(onboard_input_.real_objects));
      onboard_input_.real_objects = std::move(objects);
      break;
    }
    case ObjectsProto::SCOPE_VIRTUAL:
      onboard_input_.virtual_objects = std::move(objects);
      break;
    case ObjectsProto::SCOPE_AV:
      onboard_input_.av_objects = std::move(objects);
      break;
  }
}

void PlannerModule::HandleAutonomyState(
    std::shared_ptr<const AutonomyStateProto> autonomy) {
  onboard_input_.autonomy_state = std::move(autonomy);
}

void PlannerModule::HandleTrafficLightStates(
    std::shared_ptr<const TrafficLightStatesProto> traffic_light_states) {
  onboard_input_.traffic_light_states = std::move(traffic_light_states);
}

void PlannerModule::HandleDriverAction(
    std::shared_ptr<const DriverAction> driver_action) {
  FUNC_QTRACE();
  onboard_input_.driver_action = std::move(driver_action);
  // TODO(lidong): Move the logic to run main loop function.

  if (const auto update_status = UpdateExternalCmdQueueFromDriverAction(
          *onboard_input_.driver_action, &ext_cmd_info_.queue);
      !update_status.ok()) {
    QLOG(WARNING) << update_status.message();
  }

  if (FLAGS_restore_from_snapshot && !IsOnboardMode() &&
      onboard_input_.driver_action->press_engage_button() &&
      playback_planner_state_ != nullptr &&
      playback_planner_state_->planner_frame_seq_num() != 0) {
    QLOG(INFO) << "Restoring planner module state with planner_state_proto of "
                  "seq num: "
               << playback_planner_state_->planner_frame_seq_num();
    onboard_input_.planner_state_proto = playback_planner_state_;
  }
}

void PlannerModule::HandleRemoteAssistToCar(
    std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car) {
  if (remote_assist_to_car->driving_action_request().has_lane_change()) {
    ext_cmd_info_.queue.pending_lane_change_requests.push(
        remote_assist_to_car->driving_action_request().lane_change());
  }
  if (remote_assist_to_car->driving_action_request()
          .has_out_of_blocked_road()) {
    ext_cmd_info_.queue.pending_out_of_blocked_road_requests.push(
        remote_assist_to_car->driving_action_request().out_of_blocked_road());
  }
  onboard_input_.remote_assist_to_car = std::move(remote_assist_to_car);
}

void PlannerModule::HandleRoutingManagerOutputResult(
    std::shared_ptr<const RouteManagerOutputProto> route_manager_output) {
  onboard_input_.route_mgr_output = std::move(route_manager_output);
}

void PlannerModule::HandleChassis(std::shared_ptr<const Chassis> chassis) {
  onboard_input_.chassis = std::move(chassis);
}

void PlannerModule::HandleSemanticMapPatch(
    std::shared_ptr<const SemanticMapModificationProto> semantic_map_mod) {
  onboard_input_.semantic_map_modification = std::move(semantic_map_mod);
}

void PlannerModule::HandleRunEventStates(
    std::shared_ptr<const QRunEventStatesProto> states) {  // NOLINT
  // Semantic map modifier.
  const auto semantic_map_modifier_or = GetQRunEventStateWithoutDuplicate(
      *states, QRunEventStatesProto::STATE_TYPE_COM_SEMANTIC_MAP_MODIFIER);
  if (semantic_map_modifier_or.ok()) {
    const auto& semantic_map_modifier_state = *semantic_map_modifier_or;
    auto modification = std::make_shared<SemanticMapModificationProto>();
    *modification->mutable_modifier() =
        semantic_map_modifier_state.run_event().semantic_map_modifier_proto();
    // Faked a lite msg header.
    *modification->mutable_header() = states->header();
    modification->mutable_header()->set_tag_number(
        LiteMsgWrapper::descriptor()
            ->FindFieldByName("semantic_map_modification_proto")
            ->number());
    onboard_input_.semantic_map_modification = std::move(modification);
  }

  // lcc cruising speed.
  const auto lcc_cruising_speed_or = GetQRunEventStateWithoutDuplicate(
      *states, QRunEventStatesProto::STATE_TYPE_QEVENT_LCC_CRUISING_SPEED);
  if (lcc_cruising_speed_or.ok()) {
    const auto& lcc_cruising_speed_state = *lcc_cruising_speed_or;
    ext_cmd_info_.status.lcc_cruising_speed_limit =
        lcc_cruising_speed_state.run_event().float_value();
  }

  // noa cruising speed.
  const auto noa_cruising_speed_or = GetQRunEventStateWithoutDuplicate(
      *states, QRunEventStatesProto::STATE_TYPE_QEVENT_NOA_CRUISING_SPEED);
  if (noa_cruising_speed_or.ok()) {
    const auto& noa_cruising_speed_state = *noa_cruising_speed_or;
    ext_cmd_info_.status.noa_cruising_speed_limit =
        noa_cruising_speed_state.run_event().float_value();
  }

  // lcc following distance level.
  const auto following_level_or = GetQRunEventStateWithoutDuplicate(
      *states,
      QRunEventStatesProto::STATE_TYPE_QEVENT_LCC_FOLLOWING_DISTANCE_LEVEL);
  if (following_level_or.ok()) {
    const auto& following_level_state = *following_level_or;
    ext_cmd_info_.status.following_distance_level =
        following_level_state.run_event().lcc_following_distance_level();
  }

  // Noa lane change need confirmation.
  const auto noa_event_state_or = GetQRunEventStateWithoutDuplicate(
      *states, QRunEventStatesProto::STATE_TYPE_NOA_SWITCH);
  if (noa_event_state_or.ok()) {
    ext_cmd_info_.status.noa_need_lane_change_confirmation =
        noa_event_state_or->run_event().noa_settings().lane_change_setting() ==
        NoaSettings::LANE_CHANGE_SETTING_NEED_CONFIRM;
    // Noa lane change style.
    if (FLAGS_planner_use_lane_change_style_from_hmi) {
      switch (noa_event_state_or->run_event()
                  .noa_settings()
                  .lane_change_style_mode()) {
        case NoaSettings::LANE_CHANGE_STYLE_MODE_NONE:
        case NoaSettings::LANE_CHANGE_STYLE_MODE_MODERATE:
          ext_cmd_info_.status.lane_change_style =
              LaneChangeStyle::LC_STYLE_NORMAL;
          break;
        case NoaSettings::LANE_CHANGE_STYLE_MODE_MILD:
          ext_cmd_info_.status.lane_change_style =
              LaneChangeStyle::LC_STYLE_CONSERVATIVE;
          break;
        case NoaSettings::LANE_CHANGE_STYLE_MODE_SPORT:
          ext_cmd_info_.status.lane_change_style =
              LaneChangeStyle::LC_STYLE_RADICAL;
          break;
      }
    }
  }

  run_event_state_seq_num_ = states->header().seq_number();
}

void PlannerModule::HandleQRunEvents(
    const std::shared_ptr<const QRunEventsProto>& q_run_events_proto) {
  for (const auto& event : q_run_events_proto->run_events()) {
    // Get auto lane change confirmation from q run event.
    if (event.key() == QRunEvent::KEY_PRODUCT_AUTO_LANE_CHANGE_DRIVER_CONFIRM) {
      ext_cmd_info_.status.alc_confirmation = event.bool_value();
    }
    const bool is_apa_active =
        onboard_input_.autonomy_state != nullptr &&
        IsApaActive(onboard_input_.autonomy_state->assist_state());
    // Get APA parking spot info when apa not active.
    if (!is_apa_active) {
      if (event.key() == QRunEvent::KEY_QCOMAND_APA_PARKING_SPOT_SELECTED) {
        ext_cmd_info_.status.apa_parking_spot_id = event.string_value();
      }
      if (event.key() == QRunEvent::KEY_QCOMAND_PARKING_SPOT_CUSTOM) {
        ext_cmd_info_.status.apa_parking_spot_id =
            event.parking_spot_info().id();
        custom_parking_spot_ = std::make_shared<ParkingSpotFinderProto>();
        *custom_parking_spot_->add_spots() = event.parking_spot_info();
        onboard_input_.parking_spot_finder = custom_parking_spot_;
      }
    }
    const bool is_parking_finished_or_exited =
        onboard_input_.autonomy_state != nullptr &&
        (onboard_input_.autonomy_state->assist_state()
                 .assist_apa_state()
                 .state() == AssistApaStateProto::APA_STATE_PARKING_FINISH ||
         onboard_input_.autonomy_state->assist_state()
                 .assist_apa_state()
                 .state() == AssistApaStateProto::APA_STATE_PARKING_SAFE_STOP);
    // Clear custom_parking_spot_ when parking finished/exited.
    if (is_parking_finished_or_exited) {
      custom_parking_spot_ = nullptr;
    }
  }
}

void PlannerModule::HandleDriverCommand(
    std::shared_ptr<const DriverCommandProto> /*driver_cmd*/) {}  // NOLINT

void PlannerModule::HandleOnlineSemanticMap(
    std::shared_ptr<const mapping::OnlineSemanticMapProto>
        online_semantic_map) {
  onboard_input_.online_semantic_map = std::move(online_semantic_map);
}
void PlannerModule::HandleParkingSpotFinder(
    std::shared_ptr<const ParkingSpotFinderProto> parking_spot_finder) {
  if (custom_parking_spot_ == nullptr) {
    onboard_input_.parking_spot_finder = std::move(parking_spot_finder);
  }
}
void PlannerModule::HandleOnlineMapProto(
    const std::shared_ptr<const OnlineMapProto>& online_map_proto) {
  if (online_map_proto != nullptr) {
    QLOG(INFO) << "Received new online map "
                  "proto, space: "
               << online_map_proto->SpaceUsedLong()
               << ", timestamp: " << online_map_proto->header().timestamp();
  }
  hd_semantic_map_listener_->UpdateOnlineMap(online_map_proto);
}

void PlannerModule::HandleFusionFusionParkingFreespace(
    std::shared_ptr<const FusionParkingFreespaceProto>
        fusion_parking_freespace) {
  onboard_input_.fusion_parking_freespace = std::move(fusion_parking_freespace);
}

void PlannerModule::UpdateOnlineHDMap(PlannerInput* input) {
  SCOPED_QTRACE("PlannerModule::UpdateOnlineHDMap");
  if (input->localization_transform == nullptr) {
    QLOG_EVERY_N_SEC(WARNING, 5) << "Empty localization transform";
    return;
  }

  const auto coordinate_converter =
      CoordinateConverter::FromLocalizationTransform(
          *input->localization_transform);
  const Vec2d pos_global = coordinate_converter.SmoothToGlobal(
      Vec2d(input->pose->pos_smooth().x(), input->pose->pos_smooth().y()));

  const bool has_prev_route_sections =
      input->planner_state_proto &&
      input->planner_state_proto->has_prev_route_sections() &&
      !input->planner_state_proto->prev_route_sections().section_id().empty();
  const auto& section_seq =
      has_prev_route_sections
          ? input->planner_state_proto->prev_route_sections()
          : (input->route_mgr_output != nullptr
                 ? input->route_mgr_output->route_sections_from_current()
                 : RouteSectionSequenceProto());

  const auto* overlap_sections =
      input->route_mgr_output != nullptr &&
              input->route_mgr_output->has_overlap_sections_precomputed() &&
              input->route_mgr_output->overlap_sections_precomputed()
          ? &input->route_mgr_output->overlap_sections()
          : nullptr;

  const auto smmsi = UpdateHdSemanticMapManagerAlongRoute(
      pos_global, section_seq, planner_state_.hd_map_state, overlap_sections,
      hd_semantic_map_listener_.get());
  if (smmsi == nullptr) {
    planner_state_.hd_map_state = std::nullopt;
    return;
  }
  const bool got_new_smmsi =
      input->semantic_map_multilevel_spatial_index == nullptr ||
      smmsi->semantic_manager()->update_id() !=
          input->semantic_map_manager->update_id();
  if (got_new_smmsi) {
    QLOG(INFO) << "Got new smmsi, update id: "
               << smmsi->semantic_manager()->update_id() << std::boolalpha
               << ", pending_load_hd_psmm: " << pending_load_hd_psmm_;
  }
  if (got_new_smmsi && !pending_load_hd_psmm_) {
    psmm_future_ = AsyncLoadPlannerSemanticMapManager(
        smmsi, coordinate_converter, thread_pool_.get());
    pending_load_hd_psmm_ = true;
    if (input->planner_semantic_map_manager == nullptr) {
      psmm_future_.Wait();
    }
  }
  bool new_psmm = false;
  if (psmm_future_.IsReady()) {
    input->planner_semantic_map_manager = psmm_future_.Get();
    new_psmm = true;
    pending_load_hd_psmm_ = false;
    input->semantic_map_multilevel_spatial_index =
        input->planner_semantic_map_manager
            ->semantic_map_multilevel_spatial_index();
    input->semantic_map_manager = input->planner_semantic_map_manager
                                      ->semantic_map_multilevel_spatial_index()
                                      ->semantic_manager();
  }

  bool new_smm_modification = false;
  if (input->semantic_map_modification != nullptr &&
      input->semantic_map_modification->has_modifier()) {
    latest_semantic_map_modification_ =
        std::move(input->semantic_map_modification);
    new_smm_modification = true;
    input->semantic_map_modification.reset();
  }

  if (new_psmm || new_smm_modification) {
    if (latest_semantic_map_modification_) {
      PlannerSemanticMapModification psmm_modifier =
          CreateSemanticMapModification(
              *input->semantic_map_manager,
              latest_semantic_map_modification_->modifier());
      input->planner_semantic_map_manager->SetSemanticMapModifier(
          std::move(psmm_modifier));
    }
  }

  // Update elements of psmm.
  input->planner_semantic_map_manager->UpdateCoordinateConverter(
      coordinate_converter);
  input->planner_semantic_map_manager->UpdateSmoothInfoOfMapElements(
      thread_pool_.get());

  if (!section_seq.section_id().empty()) {
    planner_state_.hd_map_state = ObtainHdMapState(
        *input->planner_semantic_map_manager->semantic_map_manager(),
        section_seq);
  } else {
    planner_state_.hd_map_state = std::nullopt;
  }
}

absl::Status PlannerModule::PreprocessInput(PlannerInput* input) {
  FUNC_QTRACE();

  if (input->pose == nullptr) {
    return absl::FailedPreconditionError("Empty pose proto");
  }

  UpdateOnlineHDMap(input);

  if (input->autonomy_state == nullptr) {
    return absl::NotFoundError("No autonomy status.");
  }

  if (input->traffic_light_states == nullptr) {
    input->traffic_light_states = std::make_shared<TrafficLightStatesProto>();
  }

  // teleop_state_.ClearPendingQueue();

  if (input->remote_assist_to_car != nullptr) {
    // NOTE: if remote assist proto's frequence is higher than planner module,
    // we may lose some proto.
    const auto lane_change_style = ext_cmd_info_.status.lane_change_style;
    if (const auto status = UpdateExternalCmdStatusFromRemoteAssist(
            *input->remote_assist_to_car, &ext_cmd_info_.status);
        !status.ok()) {
      input->remote_assist_to_car.reset();  // Clear if not valid.
      QLOG(WARNING) << status;
    }
    if (FLAGS_planner_use_lane_change_style_from_hmi) {
      // If lane change style is set by HMI, we should not change it from
      // remote assist.
      ext_cmd_info_.status.lane_change_style = lane_change_style;
    }
  }

  if (input->prediction_debug == nullptr) {
    input->prediction_debug = std::make_shared<PredictionDebugProto>();
  }

  return absl::OkStatus();
}

void PlannerModule::MainLoop() {
  SCOPED_QTRACE("PlannerModule::MainLoop");

  // NOTE: Make sure no changes to planner state before this line.
  auto planner_state_proto = planner_state_to_proto_future_.Get();

  // Preprocess input before Run_Main_loop
  if (const auto status = PreprocessInput(&onboard_input_); !status.ok()) {
    QLOG(ERROR) << "Failed to process input: " << status;
    return;
  }

  // Wait until the publishing the output from the previous iteration is done.
  if (publish_output_future_.IsValid()) {
    WaitForFuture(publish_output_future_);
  }

  PlannerStatus status;
  {
    if (FLAGS_use_oracle_prediction_only) {
      if (onboard_input_.log_prediction != nullptr) {
        onboard_input_.prediction = std::move(onboard_input_.log_prediction);
      }
    }
    absl::MutexLock lock(&output_mutex_);
    planner_state_proto->set_planner_frame_seq_num(
        planner_state_.planner_frame_seq_num);  // Set to current frame.

    if (onboard_input_.localization_transform != nullptr) {
      // Fill in the sequence number of this iteration to the planner state in
      // last iteration.
      FillAvLtHistories(*onboard_input_.pose,
                        *onboard_input_.localization_transform,
                        &planner_state_.history_transform);
    }
    if (FLAGS_planner_enable_objects_histories_buffer) {
      UpdateObjectHistoryBuffer(onboard_input_.real_objects,
                                onboard_input_.virtual_objects,
                                &planner_state_.object_history_map);
    }

    FillInputIterationNumToPlannerState(
        onboard_input_, run_event_state_seq_num_, planner_state_proto.get());
    DestroyContainerAsyncMarkSource(
        std::move(onboard_input_.planner_state_proto),
        "planner_module:planner_state");

    onboard_input_.planner_state_proto = planner_state_proto;

    // Clear onboard_output_ asynchronously.
    DestroyContainerAsyncMarkSource(std::move(onboard_output_),
                                    "planner_module:plan_output");
    onboard_output_ = std::make_unique<PlannerOutput>();

    status = RunMainLoop(onboard_input_, onboard_output_.get());
    if (!status.ok()) {
      switch (status.status_code()) {
        case PlannerStatusProto::NOA_MAIN_LOOP_FAILED:
          QISSUEX_WITH_ARGS(
              QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
              QIssueSubType::QIST_PLANNER_NOA_INTERNAL_FAIL,
              "Planner HDMap NOA main loop error: ", status.ToString());
          break;

        case PlannerStatusProto::ALCC_MAIN_LOOP_FAILED:
          QISSUEX_WITH_ARGS(
              QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
              QIssueSubType::QIST_PLANNER_LCC_INTERNAL_FAIL,
              "Planner ALCC main loop error: ", status.ToString());
          break;

        case PlannerStatusProto::ACC_MAIN_LOOP_FAILED:
          QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                            QIssueSubType::QIST_PLANNER_ACC_INTERNAL_FAIL,
                            "Planner ACC main loop error: ", status.ToString());
          break;

        case PlannerStatusProto::MAPLESS_NOA_MAIN_LOOP_FAILED:
          QISSUEX_WITH_ARGS(
              QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
              QIssueSubType::QIST_PLANNER_NOA_INTERNAL_FAIL,
              "Planner Mapless NOA main loop error: ", status.ToString());
          break;

        default:
          QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                            QIssueSubType::QIST_PLANNER_MAIN_LOOP_FAILED,
                            "Planner main loop error: ", status.ToString());
          break;
      }
    } else {
      // TODO(lidong): Remove this condition.
      if (IsOnboardMode() &&
          QCHECK_NOTNULL(onboard_input_.autonomy_state)->autonomy_state() ==
              AutonomyStateProto::READY_TO_AUTO_DRIVE &&
          !CheckIfDriverCanEngage(onboard_output_->trajectory()).ok()) {
        QLOG(ERROR) << "Trajectory is not ready to engage.";
      }
    }
    // NOTE(zixuan): Save to planner debug before some input fields are
    // cleared below.
    if (FLAGS_planner_debug > 1) {
      ParsePlannerInputToDebugProto(onboard_output_->mutable_planner_debug(),
                                    onboard_input_);
    }

    // last async publish planner state
    if (publish_planner_state_future_.IsValid()) {
      WaitForFuture(publish_planner_state_future_);
    }

    // Clear some of the input fields before next iteration.
    onboard_input_.BeforeNextIteration(FLAGS_planner_allow_async_in_main_thread
                                           ? thread_pool_.get()
                                           : nullptr);
    onboard_output_->mutable_planner_debug()->set_planner_frame_seq_num(
        planner_state_.planner_frame_seq_num);
    planner_state_.planner_frame_seq_num++;

    ext_cmd_info_.status.output = ExternalCommandOutput();
    ext_cmd_info_.status.alc_confirmation = std::nullopt;
    ext_cmd_info_.queue.Clear();
  }

  PublishOutputAsync(status);
}

// NOTE(lidong): Code inside this function shall depends on the provided
// `input` function.
PlannerStatus PlannerModule::RunMainLoop(const PlannerInput& input,
                                         PlannerOutput* output) {
  SCOPED_QTRACE("PlannerModule::RunMainLoop");
  ScopedMultiTimer timer("planner");

  const absl::Cleanup cleaner = [this, output] {
    SCOPED_QTRACE("cleanup");
    QCHECK(output->has_trajectory());
    if (output->planner_debug().planner_status().status() !=
        PlannerStatusProto::OK) {
      planner_state_.planner_skip_counter++;
    }
    if (publish_planner_state_future_.IsValid()) {
      publish_planner_state_future_.Wait();
    }
    planner_state_to_proto_future_ = ScheduleFuture(
        IsOnboardMode() && FLAGS_planner_allow_async_in_main_thread
            ? pub_thread_pool_.get()
            : nullptr,
        [this]() -> std::shared_ptr<PlannerStateProto> {
          SCOPED_QTRACE("PlannerStateToProto");
          auto planner_state_proto = std::make_shared<PlannerStateProto>();
          planner_state_.ToProto(planner_state_proto.get());
          return planner_state_proto;
        });
  };

  // Set the status to OK first.
  output->mutable_planner_debug()->mutable_planner_status()->set_status(
      PlannerStatusProto::OK);

  output->mutable_trajectory()->set_planner_state_seq_num(
      input.planner_state_proto->header().seq_number());

  if (IsPlannerSnapshotMode() ||
      (FLAGS_restore_from_snapshot && IsFirstSimulationFrame())) {
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        planner_state_,
        RecoverPlannerStateFromProto(input, /*recover_async_state=*/false),
        PlannerStatusProto::PLANNER_INTERNAL_FAILED);
    simulation_frame_++;
  }

  VLOG(1) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
  VLOG(1) << "Planner iteration " << planner_state_.planner_frame_seq_num;

  const CoordinateConverter coordinate_converter =
      input.localization_transform
          ? CoordinateConverter::FromLocalizationTransform(
                *input.localization_transform)
          : CoordinateConverter::FromLocale();

  if (!FLAGS_planner_load_saved_trajectory.empty()) {
    return LoadSavedTrajectory(FLAGS_planner_load_saved_trajectory,
                               coordinate_converter, *input.autonomy_state,
                               &planner_state_.previous_trajectory,
                               output->mutable_trajectory());
  }

  const auto& vehicle_geom = input.vehicle_params.vehicle_geometry_params();
  if (CheckPreEnterStandbyState(input.route_mgr_output.get(),
                                planner_state_.prev_route_sections,
                                input.pose.get())) {
    QLOG_IF_NOT_OK(WARNING, Publish(ext_cmd_info_.status.ToProto()));

    const std::string standby_reason = "AV has not received route result.";
    if (constexpr double kEnterStandbySpeedThreshold = 0.1;  // m/s.
        input.pose->speed() > kEnterStandbySpeedThreshold) {
      std::string message = absl::StrCat(standby_reason,
                                         " But AV is still moving, could not "
                                         "engage nor enter standby state.");
      QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
              QIssueSubType::QIST_PLANNER_NO_ROUTE_WHEN_DRIVING, message);
      auto* planner_status =
          output->mutable_planner_debug()->mutable_planner_status();
      planner_status->set_status(PlannerStatusProto::ROUTE_MSG_UNAVAILABLE);
      planner_status->set_message(message);
      return PlannerStatus(PlannerStatusProto::ROUTE_MSG_UNAVAILABLE, message);
    }
    QLOG(WARNING) << standby_reason << " Stay in standby mode.";

    auto* trajectory = output->mutable_trajectory();
    trajectory->set_gear(Chassis::GEAR_DRIVE);
    trajectory->set_low_speed_freespace(false);
    trajectory->set_enable_stationary_steering(false);
    return EnterStandbyState(*input.pose, vehicle_geom, standby_reason,
                             trajectory, output->mutable_planner_debug());
  }

  RouteManagerOutput route_mgr_output;
  if (input.route_mgr_output != nullptr) {
    route_mgr_output.FromProto(*input.route_mgr_output);
  }

  if (auto input_status = CheckInput(input); !input_status.ok()) {
    auto* planner_status =
        output->mutable_planner_debug()->mutable_planner_status();
    planner_status->set_status(PlannerStatusProto::INPUT_INCORRECT);
    planner_status->set_message(input_status.ToString());
    QLOG(ERROR) << "Check input failed: " << input_status;
    return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                         input_status.message());
  }

  // Collect objects.
  VLOG(2) << "Exporting objects from objects view.";
  const auto objects_proto =
      GetAllObjects(input.real_objects, input.virtual_objects);
  // Update av history.
  input.av_context->Update(*input.pose,
                           coordinate_converter.localization_transform(),
                           input.vehicle_params.vehicle_geometry_params());

  planner_state_.planner_skip_counter = 0;
  absl::Time current_time = Clock::Now();
  // Publish planner state immediately before running main loop.
  // This is done asynchronously to unblock the main thread.
  // Smelly code with copy Arg.
  publish_planner_state_future_ = ScheduleFuture(
      FLAGS_planner_allow_async_in_main_thread ? pub_thread_pool_.get()
                                               : nullptr,
      [this, current_time](PlannerStateProto* proto) {
        SCOPED_QTRACE("publish_planner_state");
        proto->set_current_time(absl::ToUnixMicros(current_time));
        QLOG_IF_NOT_OK(WARNING, Publish(*proto));
      },
      const_cast<PlannerStateProto*>(input.planner_state_proto.get()));
  const double time_interval =
      absl::ToDoubleSeconds(current_time - planner_state_.current_time);
  planner_state_.current_time = current_time;

  auto task_dispatcher_status = RunPlanTaskDispatcher(
      coordinate_converter, input, route_mgr_output,
      std::as_const(objects_proto).get(), current_time, time_interval,
      &ext_cmd_info_, &planner_state_, output, thread_pool_.get());

  *output->mutable_external_command_status() = ext_cmd_info_.status.ToProto();
  planner_state_.previous_autonomy_state = *input.autonomy_state;

  if (!task_dispatcher_status.ok()) {
    QLOG(ERROR) << "Task dispatch failed: "
                << task_dispatcher_status.ToString();
    return task_dispatcher_status;
  }

  // When av starting up, Check whether gear position is ready. If not, output
  // stationary trajectory.
  QCHECK(!planner_state_.plan_task_queue.empty());
  if (CheckPostEnterStandbyState(planner_state_.plan_task_queue.front(),
                                 *input.pose, *input.chassis,
                                 output->trajectory())) {
    const std::string standby_reason = absl::StrCat(
        "Gear not match: chassis gear is: ",
        Chassis::GearPosition_Name(input.chassis->gear_location()),
        ", trajectory gear is: ",
        Chassis::GearPosition_Name(output->trajectory().gear()), ".");
    QLOG(WARNING) << standby_reason << " Stay in standby mode.";
    return EnterStandbyState(*input.pose, vehicle_geom, standby_reason,
                             output->mutable_trajectory(),
                             output->mutable_planner_debug());
  }

  return OkPlannerStatus();
}

void PlannerModule::PublishOutput(PlannerOutput* mutable_output,
                                  const PlannerStatus& traj_status) {
  SCOPED_QTRACE("PlannerModule::PublishOutput");
  // NOTE(lidong): Make the publish runs in parallel if too slow.
  ScopedMultiTimer timer("planner_publisher");

#define PUBLISH_MSG(msg_type, msg)                                    \
  if (mutable_output->has_##msg()) {                                  \
    std::unique_ptr<msg_type> tmp_msg = std::make_unique<msg_type>(); \
    mutable_output->mutable_##msg()->Swap(tmp_msg.get());             \
    QLOG_IF_NOT_OK(WARNING, Publish(std::move(tmp_msg)));             \
  }
  // Only publish trajectory when the trajectory status is OK.
  if (traj_status.ok()) {
    PUBLISH_MSG(TrajectoryProto, trajectory);
  }
  PUBLISH_MSG(PlannerDebugProto, planner_debug);
  PUBLISH_MSG(HmiContentProto, hmi_content);
  if (FLAGS_planner_publish_chart_data) {
    PUBLISH_MSG(vis::vantage::ChartsDataProto, charts_data);
  }
  PUBLISH_MSG(PlannerExternalCommandStatusProto, external_command_status);
  // We do not publish planner states now on purpose. This is because we want
  // to record the message sequence number of external inputs of the next
  // planner iteration in planner state. Therefore, `PlannerState` will be
  // published at the beginning of next iteration.
#undef PUBLISHMSG

  if (!IsOnboardMode()) {
    vis::vantage::GetCanvasClient()->FlushAll();
  }
}

void PlannerModule::PublishOutputAsync(const PlannerStatus& traj_status) {
  publish_output_future_ =
      ScheduleFuture(IsOnboardMode() && FLAGS_planner_allow_async_in_main_thread
                         ? pub_thread_pool_.get()
                         : nullptr,
                     [this, traj_status] {
                       absl::MutexLock lock(&output_mutex_);
                       PublishOutput(onboard_output_.get(), traj_status);
                     });
}

// TODO(lidong): Make this function only depends on PlannerInput.
absl::Status PlannerModule::CheckInput(const PlannerInput& input) {
  SCOPED_QTRACE("PlannerModule::CheckInput");

  const double now = ToUnixDoubleSeconds(Clock::Now());

  if (input.pose == nullptr) {
    return absl::FailedPreconditionError(
        "pose is not available. Refuse to plan.");
  }

  if (input.chassis == nullptr) {
    return absl::FailedPreconditionError(
        "chassis is not available. Refuse to plan.");
  }

  if (input.chassis->has_steering_percentage() &&
      std::isnan(input.chassis->steering_percentage())) {
    return absl::FailedPreconditionError(
        "chassis.steering_percentage is nan. Refuse to plan.");
  }

  if (input.traffic_light_states == nullptr) {
    return absl::FailedPreconditionError(
        "Traffic light state is not available. Refuse to plan.");
  }

  if (FLAGS_planner_consider_objects && input.prediction == nullptr) {
    return absl::FailedPreconditionError("No prediction.");
  }

  // Pose quality checks.
  constexpr double kPoseLateralVelocityLimit = 1.0;  // m/s.
  if (std::abs(input.pose->vel_body().y()) > kPoseLateralVelocityLimit) {
    QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
            QIssueSubType::QIST_POSITION_POSE_VELOCITY_UNACCEPTABLE,
            "Check pose state: UNACCEPTABLE(lateral velocity)");
    return absl::FailedPreconditionError(
        "pose lateral velocity is above limit. Considering pose "
        "unreliable. Refusing to plan.");
  }

  if (const double time_diff = now - input.pose->timestamp();
      time_diff > FLAGS_planner_max_allowed_iteration_time) {
    QISSUEX(QIssueSeverity::QIS_ERROR, QIssueType::QIT_PERFORMANCE,
            QIssueSubType::QIST_PLANNER_PROTO_POSE_TIMEOUT,
            "Check pose state: OUTDATE(stale pose)");
    // In most cases stale poses are a result of planning iterations
    // being slow, which makes PlannerModule busy running the
    // PlannerModule::MainLoop() timer rather than processing pose
    // subscription callbacks. Aborting planning in such cases will
    // quickly drain the MainLoop timer backlog and allow pose
    // subscription to come through.
    return absl::FailedPreconditionError(
        absl::StrFormat("latest pose is too old: %.3f clock now: %.3f, time "
                        "diff is: %.3f. Very likely this is caused by planner "
                        "iteration timeout.",
                        input.pose->timestamp(), now, time_diff));
  }
  if (input.autonomy_state == nullptr) {
    return absl::FailedPreconditionError("autonomy_state is not available.");
  }
  if (input.real_objects == nullptr && input.virtual_objects == nullptr) {
    return absl::FailedPreconditionError(
        "perception objects message is not available.");
  }
  const auto& objects = input.real_objects != nullptr ? input.real_objects
                                                      : input.virtual_objects;
  const double obj_header_secs = objects->header().timestamp() * 1e-6;
  if (const double time_diff = now - obj_header_secs;
      time_diff > FLAGS_planner_max_perception_delay) {
    const std::string msg = absl::StrFormat(
        "Latest perception is too old: %.3f clock now: %.3f, time diff is: "
        "%.3f, more than the allowed delay %f, which is defined by "
        "FLAGS_planner_max_perception_delay",
        obj_header_secs, now, time_diff, FLAGS_planner_max_perception_delay);
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_PERFORMANCE,
                      QIssueSubType::QIST_PERCEPTION_LITE_MSG_DELAYED,
                      "Latest perception is too old", msg);
    return absl::FailedPreconditionError(msg);
  }

  if (FLAGS_planner_dumping_ml_data_in_simulation &&
      input.log_av_trajectory == nullptr) {
    return absl::FailedPreconditionError(
        "log_av_trajectory message is not available when in simulation for "
        "dumping ml data.");
  }

  if (FLAGS_planner_dumping_ml_data_in_simulation &&
      input.log_prediction == nullptr) {
    return absl::FailedPreconditionError(
        "log_prediction message is not available when in simulation for "
        "dumping ml data.");
  }

  return absl::OkStatus();
}

PlannerModule::~PlannerModule() {
  // TODO(lidong): Move to function callback such as LiteModule::NotifyStop().
  // Wait for future members to finish.
  if (publish_planner_state_future_.IsValid()) {
    publish_planner_state_future_.Get();
  }
  if (planner_state_to_proto_future_.IsValid()) {
    planner_state_to_proto_future_.Get();
  }
  if (publish_output_future_.IsValid()) {
    publish_output_future_.Get();
  }
  if (planner_state_.async_planner_state.future_multi_task_est_status
          .IsValid()) {
    planner_state_.async_planner_state.future_multi_task_est_status.Get();
  }
  if (psmm_future_.IsValid()) {
    std::ignore = psmm_future_.Get();
  }
}

std::string PlannerModule::DebugString() const {
  std::stringstream ss;
  ss << "input:" << onboard_input_.DebugString() << "\n";
  ss << "planner_snapshot_mode_:" << planner_snapshot_mode_;
  return ss.str();
}

}  // namespace qcraft::planner
