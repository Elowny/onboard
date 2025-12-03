#include "onboard/planner/router/routing_module.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <functional>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "glog/logging.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/async/future.h"
#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/hmi/states/run_event_states_util.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_listener.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/global_pose.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/interface/generate_route.h"
#include "onboard/planner/router/noa/noa_main_loop.h"
#include "onboard/planner/router/preprocess/future_pos_estimation.h"
#include "onboard/planner/router/proto/route_external_command.pb.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/road_conditions_process.h"
#include "onboard/planner/router/route_error.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/planner/router/sim/snapshot_sim_state.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/planner/util/vehicle_util.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/proto_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {

namespace {

static constexpr double kMaxDistanceMeters = 2.5;  // about 2.5 meters.
constexpr int64_t kMaxToleranceErrorInterval = 6 * 1000 * 1000;  // us.

bool ValidateLiteHeader(const LiteHeader& header,
                        const absl::Duration& duration) {
  if (header.timestamp() == 0) return true;  // backward data.atibility
  const auto& delay = Clock::Now() - absl::FromUnixMicros(header.timestamp());
  if (delay > duration) {
    QLOG(INFO) << "timestamp " << header.timestamp() << ","
               << absl::ToUnixMicros(Clock::Now()) << ", delay "
               << absl::ToInt64Microseconds(delay);
    return false;
  }
  return true;
}

bool ValidateRemoteAssistMessage(const RemoteAssistToCarProto& ra_to_car) {
  return ValidateLiteHeader(ra_to_car.header(),
                            /*FLAGS_teleop_expire_seconds*/ absl::Seconds(3));
}

void FillRouteResult(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const RouteManagerState& route_manager_state,
    RoutingResultProto* routing_result_proto) {
  routing_result_proto->clear_path_points_from_current();
  if (route_manager_state.route_from_current.has_lane_path()) {
    absl::StatusOr<std::vector<Vec2d>> points_or = SimplifyPathPoints(
        semantic_map_manager,
        route_manager_state.route_from_current.lane_path(), kMaxDistanceMeters);
    if (points_or.ok()) {
      for (const Vec2d& point : *points_or) {
        point.ToProto(routing_result_proto->add_path_points_from_current());
      }
    } else {
      QLOG(ERROR) << "Cannot generate simplify route path points.";
    }
  }
}

absl::Status CheckRoutePrecondition(const route::RoutingInput& input) {
  if (input.pose == nullptr) {
    return absl::FailedPreconditionError(
        "Check precondition failed. Routing module is not ready. "
        "Pose is null.");
  }
  if (input.localization_transform == nullptr) {
    return absl::FailedPreconditionError(
        "Check precondition failed. Routing module is not ready. "
        "localization_transform is null.");
  }
  if (input.autonomy_state == nullptr) {
    return absl::FailedPreconditionError(
        "Check precondition failed. Routing module is not ready. "
        "autonomy_state is null.");
  }

  return absl::OkStatus();
}

constexpr bool IsWaitTooLongToSendOk(int64_t init_microsec,
                                     int64_t now_microsec,
                                     double max_wait_sec) {
  return init_microsec > 0 &&
         (now_microsec - init_microsec) > SecondsToMicroSeconds(max_wait_sec);
}

absl::StatusOr<GlobalPos2d> GetGlobalPos2d(
    const LocalizationTransformProto* localization_transform_proto,
    const PoseProto* pose_proto, const GnssRawReadingProto* gnss_proto) {
  Vec2d global2d;
  double heading;
  if (localization_transform_proto != nullptr &&
      localization_transform_proto->loc_status() !=
          LocalizationTransformProto::kInvalid &&
      pose_proto != nullptr) {
    const auto cc = CoordinateConverter::FromLocalizationTransform(
        *localization_transform_proto);
    global2d = cc.SmoothToGlobal(
        {pose_proto->pos_smooth().x(), pose_proto->pos_smooth().y()});
    heading = cc.SmoothYawToGlobal(pose_proto->yaw());
  } else {
    QLOG_EVERY_N_SEC(WARNING, 3) << "Use the raw gnss localization transform "
                                    "instead, an abnormal state.";
    if (gnss_proto == nullptr) {
      QLOG_EVERY_N_SEC(WARNING, 3) << "The gnss is not ready. Waiting ...";
      return absl::FailedPreconditionError("gnss is not ready.");
    }
    global2d = {gnss_proto->longitude(), gnss_proto->latitude()};
    heading = gnss_proto->yaw();
  }
  return GlobalPos2d{
      .pos = global2d,
      .heading = heading,
  };
}

}  // namespace

RoutingModule::RoutingModule(LiteClientBase* client)
    : LiteModule(client), route_reset_action_(this) {
  route_state_ = RoutingStateProto::UNINITIALIZED;
}

void RoutingModule::HandlePlannerStateProto(
    std::shared_ptr<const PlannerStateProto> planner_state_proto) {
  if (input_.pose == nullptr) {
    return;
  }
  auto planner_state_ptr = std::move(planner_state_proto);
  if (planner_state_ptr->has_previous_trajectory() &&
      planner_state_ptr->previous_trajectory().trajectory_point_size() > 10) {
    prev_trajectory_ = planner_state_ptr->previous_trajectory();
  }
}

void RoutingModule::HandleOnlineSemanticMap(
    std::shared_ptr<const mapping::OnlineSemanticMapProto>
        online_semantic_map) {
  if (FLAGS_route_build_online_map_smm) {
    input_.online_sm_proto = std::move(online_semantic_map);
  }
}

void RoutingModule::HandleGnssRawReadingProto(
    std::shared_ptr<const GnssRawReadingProto> gnss) {
  input_.gnss = std::move(gnss);
}

void RoutingModule::HandleMppProto(
    std::shared_ptr<const MppSectionsProto> mpp) {
  input_.mpp = std::move(mpp);
}

void RoutingModule::HandleReroutingRequest(
    std::shared_ptr<const ReroutingRequestProto> rerouting_request) {
  input_.rerouting_request = std::move(rerouting_request);
  const auto& request = *(input_.rerouting_request);
  const std::string& request_id =
      request.has_routing_request() && request.routing_request().has_id()
          ? request.routing_request().id()
          : "";
  QLOG(INFO) << absl::StrCat("A new rerouting request arrived. state:",
                             route_state_, ",id: ", request_id);
  HmiContentProto hmi_content_proto;
  hmi_content_proto.mutable_route_content()->set_routing_request_id(request_id);
  QLOG_IF_NOT_OK(ERROR, Publish(hmi_content_proto));
  MainLoop();
}

void RoutingModule::HandleRunEvents(
    std::shared_ptr<const QRunEventsProto> run_events) {  // NOLINT
  for (const auto& run_event : run_events->run_events()) {
    if (run_event.has_key() &&
        run_event.key() == QRunEvent::KEY_QEVENT_SET_SD_ROUTE) {
      auto sd_route =
          std::make_shared<const SDRouteProto>(run_event.sd_route());
      HandleSdRouteProto(std::move(sd_route));
      break;
    }
  }
}

// NOLINTBEGIN(*-performance-unnecessary-value-param)
void RoutingModule::HandleRunEventStates(
    std::shared_ptr<const QRunEventStatesProto> run_event_states) {
  auto states = std::move(run_event_states);
  const auto status = GetQRunEventStateWithDuplicate(
      *states, QRunEventStatesProto::STATE_TYPE_COM_DRIVE_MISSION);
  if (!status.ok()) {
    return;
  }
  const auto& event_state = status.value();
  if (!event_state.run_event().has_routing_request_proto()) {
    return;
  }
  const auto& event_routing_request =
      event_state.run_event().routing_request_proto();
  if (last_rerouting_request_.has_routing_request() &&
      ProtoEquals(last_rerouting_request_.routing_request(),
                  event_routing_request) &&
      last_rerouting_request_.header().timestamp() <=
          event_state.run_event_state_header().send_timestamp()) {
    return;
  }
  async_retry_.Reset();
  last_rerouting_request_.mutable_header()->set_timestamp(
      event_state.run_event_state_header().send_timestamp());
  *last_rerouting_request_.mutable_routing_request() = event_routing_request;
  input_.rerouting_request =
      std::make_shared<const ReroutingRequestProto>(last_rerouting_request_);
  const auto& request = *(input_.rerouting_request);
  const std::string& request_id =
      request.has_routing_request() && request.routing_request().has_id()
          ? request.routing_request().id()
          : "";
  QLOG(INFO) << absl::StrCat("A new rerouting request arrived. state:",
                             route_state_, ", id: ", request_id)
             << ", ts:" << last_rerouting_request_.header().timestamp();
  HmiContentProto hmi_content_proto;
  hmi_content_proto.mutable_route_content()->set_routing_request_id(request_id);
  QLOG_IF_NOT_OK(ERROR, Publish(hmi_content_proto));
}

void RoutingModule::HandleSdMatchResultProto(
    std::shared_ptr<const SdRouteMatchResultProto>
        sd_route_match_result_proto) {
  input_.sd_route_match_result_proto = std::move(sd_route_match_result_proto);
}

void RoutingModule::HandleRoutingResultProto(
    std::shared_ptr<const RoutingResultProto> routing_result_proto) {
  input_.routing_result = std::move(routing_result_proto);
}

void RoutingModule::HandleRoutingStateProto(
    std::shared_ptr<const RoutingStateProto> routing_state_proto) {
  input_.routing_state = std::move(routing_state_proto);
}

void RoutingModule::HandleRouteServiceRequestProto(
    std::shared_ptr<const RouteServiceRequestProto>
        route_service_request_proto) {  // NOLINT
  if (route_service_request_proto != nullptr) {
    PathRoutingResultProto result_proto;
    *result_proto.mutable_route_service_request_proto() =
        *route_service_request_proto;
    auto* routing_result = result_proto.mutable_routing_result();
    if (smm_ == nullptr || map_index_ == nullptr) {
      QLOG(ERROR)
          << "Precondition is not satisfied, can not respond to route service "
             "routing request.";
      routing_result->set_success(false);
      QLOG_IF_NOT_OK(ERROR, Publish(result_proto));
      return;
    }
    if (!route_service_request_proto->has_routing_request()) {
      QLOG(ERROR) << "No routing request in route service "
                     "routing request.";
      routing_result->set_success(false);
      QLOG_IF_NOT_OK(ERROR, Publish(result_proto));
      return;
    }

    const bool is_bus = route_service_request_proto->has_vehicle_type() &&
                                route_service_request_proto->vehicle_type() ==
                                    RouteVehicleType::bus
                            ? true
                            : false;

    route::RouteRestrictDistrict route_restrict;
    const auto& routing_request =
        route_service_request_proto->routing_request();
    if (!routing_request.avoid_lanes().empty()) {
      route_restrict.avoid_lanes.reserve(routing_request.avoid_lanes().size());
      for (const auto& lane_id : routing_request.avoid_lanes()) {
        route_restrict.avoid_lanes.insert(mapping::ElementId(lane_id));
      }
    }

    if (routing_request.has_forbidden()) {
      const auto restrict_sections = route::ParseRestrictProtoToSectionId(
          *smm_, routing_request.forbidden());
      route_restrict.restrict_sections.insert(restrict_sections.begin(),
                                              restrict_sections.end());
    }
    const auto path_result_or = SearchRouteResultsBetweenDestinations(
        *smm_, map_index_.get(), routing_request.destinations(), route_restrict,
        is_bus);

    if (!path_result_or.ok()) {
      QLOG(ERROR) << "Failed to respond to path routing request";
      routing_result->set_success(false);
      QLOG_IF_NOT_OK(ERROR, Publish(result_proto));
      return;
    }
    routing_result->set_success(true);
    path_result_or->first.ToProto(
        routing_result->mutable_route_section_sequence());
    path_result_or->second.ToProto(routing_result->mutable_lane_path());
    const double distance =
        CalcRouteSectionsLength(*smm_, path_result_or->first);
    routing_result->set_distance(distance);
    QLOG_IF_NOT_OK(ERROR, Publish(result_proto));
  }
}

void RoutingModule::HandlePlannerExternalCommandStatus(
    std::shared_ptr<const PlannerExternalCommandStatusProto>
        planner_external_command_status_proto) {  // NOLINT
  if (planner_external_command_status_proto->has_planner_to_router_command()) {
    input_.external_router_command =
        planner_external_command_status_proto->planner_to_router_command();
    switch (input_.external_router_command) {
      case ExternalRouterCommand::NONE:
        break;
      case ExternalRouterCommand::INPLACE_REROUTE:
        QLOG_EVERY_N_SEC(INFO, 3) << "Received Inplace reroute command.";
        break;
      case ExternalRouterCommand::ANOTHER_ROUTE:
        QLOG_EVERY_N_SEC(INFO, 3) << "Received choose another route command.";
        break;
      case ExternalRouterCommand::SWITCH_ROUTE:
        QLOG_EVERY_N_SEC(INFO, 3)
            << "Received switch alternative route command.";
        break;
    }
  }
}

void RoutingModule::HandleSdRouteProto(
    std::shared_ptr<const SDRouteProto> sd_route_proto) {  // NOLINT
  QLOG_EVERY_N_SEC(INFO, 3)
      << "HandleSdRouteProto " << sd_route_proto->DebugString();
  if (input_.sd_route_proto == nullptr ||
      input_.sd_route_proto->route_id() != sd_route_proto->route_id()) {
    input_.sd_route_proto = std::move(sd_route_proto);
  }
}

void RoutingModule::OnInit() {
  DisableAutoOk();
  if (smm_ == nullptr && !FLAGS_route_noa_mode &&
      !FLAGS_route_build_online_map_smm) {
    QLOG(INFO) << "Load local v2 semantic_map_manager";
    auto loader = mapping::v2::SemanticMapLoader::MakeShared();
    auto map_fut = loader->PreloadWholeMap();
    smm_ = map_fut.Get();
    map_index_ = std::make_unique<route::MapIndex>();
    map_index_->InitIndex(*smm_);

  } else {
    QLOG(INFO) << "semantic_map_manager is specified at external.";
  }
  if (FLAGS_route_noa_mode) {
    semantic_map_listener_ =
        mapping::v2::SemanticMapSpatialIndexListenerAsnyc::MakeShared(
            {OnlineMapProto_DataSource_NAVINFO_HDMAP});
  }
  RunParamsProtoV2 run_params;
  param_manager().GetRunParams(&run_params);
  file_util::TextFileToProto(FLAGS_route_default_params_file,
                             &route_param_proto_);
  QLOG(INFO) << "Route params loaded. file_path: "
             << FLAGS_route_default_params_file
             << ", content: " << route_param_proto_.ShortDebugString();
  route_state_ = RoutingStateProto::START;
  is_bus_ = IsBus(run_params.vehicle_params().vehicle_params().model());

  async_retry_.Reset();

  QLOG(INFO) << "OnInit() success, Load vehicle params, is_bus:" << is_bus_;
  init_time_microsec_ = absl::ToUnixMicros(Clock::Now());
}

void RoutingModule::OnSubscribeChannels() {
  Subscribe(&RoutingModule::HandlePose, this, 50);
  Subscribe(&RoutingModule::HandleLocalizationTransform, this, 20);
  Subscribe(&RoutingModule::HandleRemoteAssistToCar, this, 20);
  Subscribe(&RoutingModule::HandleRecordedRoute, this, 20);
  Subscribe(&RoutingModule::HandleAutonomyState, this, 20);
  Subscribe(&RoutingModule::HandleDriverAction, this, 25);
  Subscribe(&RoutingModule::HandleRunEventStates, this, 20);
  Subscribe(&RoutingModule::HandlePlannerExternalCommandStatus, this, 20);
  Subscribe(&RoutingModule::HandlePlannerStateProto, this, 20);
  Subscribe(&RoutingModule::HandleOnlineSemanticMap, this, 20);
  Subscribe(&RoutingModule::HandleGnssRawReadingProto, this, 20);
  Subscribe(&RoutingModule::HandleRoutingResultProto, this,
            "log_routing_result_proto", 20);
  Subscribe(&RoutingModule::HandleRoutingStateProto, this,
            "log_routing_state_proto", 20);
  Subscribe(&RoutingModule::HandleRouteServiceRequestProto, this, 20);
  Subscribe(&RoutingModule::HandleMppProto, this, 20);
  Subscribe(&RoutingModule::HandleRunEvents, this, 20);
  Subscribe(&RoutingModule::HandleSdMatchResultProto, this, 20);
  if (FLAGS_route_noa_mode) {
    Subscribe(
        &mapping::v2::SemanticMapSpatialIndexListenerAsnyc::UpdateOnlineMap,
        semantic_map_listener_.get(), 20);
    Subscribe(
        &mapping::v2::SemanticMapSpatialIndexListenerAsnyc::UpdateMppSections,
        semantic_map_listener_.get(), 20);
  }
}

void RoutingModule::OnSetUpTimers() {
  AddTimerOrDie("route_main_loop", std::bind(&RoutingModule::MainLoop, this),
                absl::Milliseconds(FLAGS_route_mainloop_period),
                /*one_shot=*/false);
}

void RoutingModule::ProcessRouteManagerOutputError(
    const absl::Status& route_manager_output_status, int64_t now_microsec) {
  // Check error, it only has error details when handling the routing
  // request failed (the ego track failed excluded).
  const route::RouteErrorCode& error_code = route_mgr_->GetLastError();
  QLOG(WARNING) << "No route manager output result, "
                << route_manager_output_status << ", now: " << now_microsec
                << ", last_publish_time: " << last_publish_microsecs_
                << ", route_error: " << error_code.ToString();

  if (now_microsec - last_publish_microsecs_ >= kMaxToleranceErrorInterval) {
    // BANDAID(zuowei): This is a hack for simulation, keep route navi
    // info for a while since it's still effective.
    last_route_mgr_output_proto_.set_route_status(
        RouteManagerOutputProto::INVALID);
    QLOG_IF_NOT_OK(WARNING, Publish(last_route_mgr_output_proto_));
  }

  if (input_.rerouting_request != nullptr &&
      error_code.code() != route::RouteErrorCode::StatusCode::kOk) {
    std::string error_model;
    switch (route_manager_output_status.code()) {
      case absl::StatusCode::kInternal:
        error_model = "route";
        async_retry_.should_retry = true;
        break;
      case absl::StatusCode::kInvalidArgument:
        error_model = "onboard infra";
        break;
      case absl::StatusCode::kNotFound:
        error_model = "map/localization";
      default:
        error_model = "unknown";
        async_retry_.should_retry = true;
        break;
    }
    if (async_retry_.CanRetry()) {
      const Vec2d pos = {input_.pose->pos_smooth().x(),
                         input_.pose->pos_smooth().y()};
      auto cc = CoordinateConverter::FromLocalizationTransform(
          *input_.localization_transform);
      const Vec2d global = cc.SmoothToGlobal(pos);
      QEVENT("xiang", "route_switch_failed", [&](::qcraft::QEvent* qevent) {
        qevent->AddField("x", global.x());
        qevent->AddField("y", global.y());
        qevent->AddField("retry_times", async_retry_.retry_times);
        qevent->AddField("error_mode", error_model);
      });
    } else {
      if (!FLAGS_route_ignore_kickout) {
        QISSUEX_WITH_ARGS(
            QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
            QIssueSubType::QIST_PLANNER_ROUTING_UPDATE_FAILED,
            absl::StrCat("No route for a new request, model:", error_model),
            error_code.ToString());
      }
    }
  }
}

void RoutingModule::CheckFreqAndPublishRoutingResult(bool is_reroute,
                                                     int64_t now_microsec) {
  if ((now_microsec - last_publish_microsecs_) >= SecondsToMicroSeconds(1) ||
      is_reroute) {
    if (route_mgr_->mutable_routing_result_proto()->success()) {
      FillRouteResult(*smm_, route_mgr_->route_manager_state(),
                      route_mgr_->mutable_routing_result_proto());
    }
    QLOG_IF_NOT_OK(WARNING, Publish(route_mgr_->GetRoutingResultProto()));
    last_publish_microsecs_ = now_microsec;
  } else {
    VLOG(2) << "Ignore publish route manager output result"
            << ", now: " << now_microsec
            << ", last_publish_time: " << last_publish_microsecs_;
  }
}
void RoutingModule::RecoverSnapshotSim() {
  if (FLAGS_route_snapshot_sim_mode) {
    if (input_.routing_result == nullptr || input_.routing_state == nullptr) {
      QLOG(INFO) << "Should set routing result and state before in route "
                    "snapshot sim mode";
      return;
    }
    if (!route_mgr_->has_route() && !route_mgr_->has_offroad_route() &&
        input_.routing_result != nullptr) {
      route::RouteSnapshotSimInput snapshot_sim_input = {
          .routing_result_proto = input_.routing_result.get(),
          .rms_debug = &input_.routing_state->route_mgr_state(),
      };
      auto route_manager_state_or = route::RecoverPartialSnapshotState(
          *smm_.get(), map_index_.get(), snapshot_sim_input);
      if (route_manager_state_or.ok()) {
        VLOG(3) << "Inject route manager state";
        route_mgr_->InjectState(std::move(route_manager_state_or).value());
      } else {
        QLOG(ERROR) << "RecoverPartialSnapshotState failed, "
                    << route_manager_state_or.status();
      }
    }
  }
}

// NOLINTNEXTLINE
void RoutingModule::MainLoop() {
  SCOPED_QTRACE("RoutingModule::MainLoop");
  if (FLAGS_route_noa_mode) {
    if (input_.remote_assist_to_car != nullptr) {
      ProcessTeleopProto(*input_.remote_assist_to_car);
    }
    LiteModule::Ok();
    const auto s = NoaMainLoop();
    if (!s.ok()) {
      QLOG_EVERY_N_SEC(ERROR, 3) << s.ToString();
    }
    return;
  }

  RoutingStateProto routing_state_proto;
  bool route_precondition = true;

  const auto precondition_status = CheckRoutePrecondition(input_);
  if (!precondition_status.ok()) {
    route_state_ = RoutingStateProto::UNAVAILABLE;
    routing_state_proto.set_state(route_state_);
    QLOG_EVERY_N_SEC(ERROR, 3)
        << " Check prediction error:" << precondition_status.ToString();
    QLOG_IF_NOT_OK(WARNING, Publish(routing_state_proto));
    route_precondition = false;
  }
  if (!route_precondition) {
    if (input_.rerouting_request != nullptr) {
      // So we got a routing request but Precondition not satisfied.
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
          QIssueSubType::QIST_PLANNER_ROUTING_REQUEST_FAILED,
          "Receive a new request, but the preconditions not satisfied.",
          precondition_status.ToString());
    }
    return;
  }

  // Wait too long to send ok
  if (!route_precondition &&
      IsWaitTooLongToSendOk(init_time_microsec_,
                            absl::ToUnixMicros(Clock::Now()),
                            FLAGS_route_wait_ok_secs)) {
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_WARNING, QIssueType::QIT_BUSINESS,
                      QIssueSubType::QIST_PLANNER_ROUTING_REQUEST_FAILED,
                      "Not a route issue. Check the precondition, "
                      "pose, localization, autonomy state first, "
                      "Wait too long time(s) to send ok:",
                      std::to_string(FLAGS_route_wait_ok_secs));
  }
  if (FLAGS_route_build_online_map_smm) {
    if (input_.online_sm_proto == nullptr) {
      QLOG(ERROR) << "Empty online map proto, waiting for input...";
    } else {
      // TODO(xiang) online smm need a new design. 20230420
      auto lt = input_.online_sm_proto->localization_transform();
      lt.set_level_id(0);
    }
  }

  if (smm_ == nullptr) {
    QLOG_EVERY_N_SEC(ERROR, 5)
        << "semantic_map_manager is null, should load map before at L4 mode.";
    return;
  }

  LiteModule::Ok();

  const int64_t now_microsec = absl::ToUnixMicros(Clock::Now());
  if (module_ok_microsec_ == 0) {
    module_ok_microsec_ = now_microsec;
  }

  [[maybe_unused]] absl::Cleanup clean_fn = [&] {
    CleanBeforeNextIteration();
    if (async_retry_.should_retry &&
        last_rerouting_request_.has_routing_request()) {
      async_retry_.Retry(MicroSecondsToSeconds(now_microsec), [this]() {
        // Copy a new routing request proto for each iteration.
        // Will retry routing request next iteration
        input_.rerouting_request =
            std::make_shared<const ReroutingRequestProto>(
                last_rerouting_request_);
        QLOG(INFO) << "Will retry routing request later. "
                   << async_retry_.DebugString();
      });
    }
  };

  if (route_mgr_ == nullptr) {
    route_mgr_ = std::make_unique<RouteManager>(smm_.get(), map_index_.get(),
                                                &route_param_proto_);
  }

  if (input_.rerouting_request != nullptr) {
    absl::Status request_status =
        route_mgr_->AddManualReroutingRequest(*input_.rerouting_request);
    if (!request_status.ok()) {
      QLOG(ERROR) << "Cannot handle the routing request."
                  << input_.rerouting_request->ShortDebugString();
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
          QIssueSubType::QIST_SEMANTIC_MAP_ROUTEING_REQUEST_INVALID,
          "It is a map issue, not a route issue. "
          "Cannot handle the routing request.",
          request_status.ToString());
    }
  }

  input_.injected_teleop_micro_secs = now_microsec;
  RecoverSnapshotSim();
  if (!route_mgr_->has_route() && !route_mgr_->has_offroad_route()) {
    route_mgr_->InitRequest(input_.recorded_route, input_.routing_result,
                            input_.injected_teleop_micro_secs);
  }
  if (input_.remote_assist_to_car != nullptr) {
    ProcessTeleopProto(*input_.remote_assist_to_car);
    if (input_.rerouting_request != nullptr) {
      const absl::Status add_status =
          route_mgr_->AddManualReroutingRequest(*input_.rerouting_request);
      QLOG_IF_NOT_OK(ERROR, add_status)
          << " Add teleop failed." << add_status.ToString();
    }
  }

  MaybeInjectTeleopProto(route_mgr_.get(), input_.injected_teleop_micro_secs);
  route_mgr_->ClearLastError();

  bool is_reset_request = last_rerouting_request_.has_routing_request() &&
                          last_rerouting_request_.routing_request().reset();
  if (is_reset_request) {
    route_mgr_->RoutingResetRequest();
    ProcessRoutingResetRequest(route_mgr_->route_manager_state());
  } else {
    // Infer pos from trajectory
    constexpr auto kMaxStaleSecs = 1.0;
    const auto infer_pos_or = route::InferPosFromTrajectory(
        prev_trajectory_,
        MicroSecondsToSeconds(now_microsec) +
            route_param_proto_.look_ahead_start_secs(),
        route_param_proto_.look_ahead_start_secs() + kMaxStaleSecs);
    if (infer_pos_or.ok()) {
      input_.pred_trajectory_point = *infer_pos_or;
    } else {
      input_.pred_trajectory_point = std::nullopt;
    }
    RouteManagerInput route_manager_input(this->input_);
    auto route_mgr_output_or = route_mgr_->Update(is_bus_, route_manager_input);
    if (route_mgr_output_or.ok()) {
      // Publish  RouteManagerOutputProto
      route_reset_action_.Clear();
      last_route_mgr_output_proto_.Clear();
      route_mgr_output_or->ToProto(&last_route_mgr_output_proto_);
      QLOG_IF_NOT_OK(WARNING, Publish(last_route_mgr_output_proto_));
      // Publish state
      route_state_ = RoutingStateProto::RUNNING;
      routing_state_proto.set_state(route_state_);
      *routing_state_proto.mutable_route_mgr_state() =
          route_mgr_->route_manager_state_debug_proto();
      if (route_mgr_->route_manager_state()
              .route_from_current.has_lane_path()) {
        *routing_state_proto.mutable_route_proto_from_current() =
            route_mgr_->route_manager_state().route_from_current;
      }
      QLOG_IF_NOT_OK(WARNING, Publish(routing_state_proto));
      // Publish HMI
      HmiContentProto hmi_content_proto;
      *hmi_content_proto.mutable_route_content() =
          route_mgr_->route_manager_state().route_content_proto;
      QLOG_IF_NOT_OK(ERROR, Publish(hmi_content_proto));
      CheckFreqAndPublishRoutingResult(route_mgr_output_or->rerouted,
                                       now_microsec);
      async_retry_.Reset();
    } else {
      ProcessRouteManagerOutputError(route_mgr_output_or.status(),
                                     now_microsec);
    }
  }
}

// Only process Teleop related with route
void RoutingModule::ProcessTeleopProto(
    const RemoteAssistToCarProto& teleop_proto) {
  SCOPED_QTRACE("ProcessTeleopProto");
  if (!ValidateRemoteAssistMessage(teleop_proto)) {
    QLOG(ERROR) << "remote assist message validation failed";
  }
  if (teleop_proto.has_driving_action_request() &&
      !teleop_proto.driving_action_request().has_lane_change() &&
      teleop_proto.driving_action_request().has_rerouting() &&
      teleop_proto.driving_action_request()
          .rerouting()
          .routing_request()
          .reset()) {
    // Reset route request
    const auto ts = teleop_proto.header().timestamp();
    input_.rerouting_request = std::make_shared<const ReroutingRequestProto>(
        teleop_proto.driving_action_request().rerouting());
    QLOG(INFO) << "Receive teleop reset command, ts:" << ts;
  } else if (teleop_proto.has_driving_action_request() &&
             !teleop_proto.driving_action_request().has_lane_change() &&
             teleop_proto.driving_action_request().has_rerouting()) {
    const auto ts = teleop_proto.header().timestamp();
    QLOG(INFO) << "Receive teleop route request command, ts:" << ts;
    input_.rerouting_request = std::make_shared<const ReroutingRequestProto>(
        teleop_proto.driving_action_request().rerouting());
  }
}
void RoutingModule::MaybeInjectTeleopProto(RouteManager* route_manager,
                                           int64_t injected_teleop_micro_secs) {
  SCOPED_QTRACE("MaybeInjectTeleopProto");
  InjectedTeleopProto injected_teleop_proto;
  bool injected = file_util::StringToProto(FLAGS_route_inject_teleop_proto,
                                           &injected_teleop_proto);
  if (!injected) {
    QLOG(ERROR) << "Invalid teleop proto:" << FLAGS_route_inject_teleop_proto;
    return;
  }
  for (int i = 0; i < injected_teleop_proto.frames_size(); ++i) {
    if (injected_teleop_proto.frames(i) - injected_teleop_micro_secs <
        50 * 1000) {
      QLOG(INFO) << "Inject Teleop proto at frame(microsecs): "
                 << injected_teleop_micro_secs << ": "
                 << injected_teleop_proto.ShortDebugString();
      ProcessTeleopProto(injected_teleop_proto.contents(i));
      const absl::Status add_status =
          route_manager->AddManualReroutingRequest(*input_.rerouting_request);
      QLOG_IF_NOT_OK(ERROR, add_status)
          << " Add teleop failed." << add_status.ToString();
    }
  }
}

void RoutingModule::ProcessRoutingResetRequest(const RouteManagerState& rms) {
  route_reset_action_.HandleAction(rms, &last_route_mgr_output_proto_);
}

void RoutingModule::CleanBeforeNextIteration() {
  input_.remote_assist_to_car.reset();
  input_.rerouting_request.reset();
  input_.recorded_route.reset();
  prev_trajectory_.Clear();
  input_.external_router_command = ExternalRouterCommand::NONE;
  input_.res_button_pressed = false;
  input_.sd_route_proto.reset();
  input_.sd_route_match_result_proto.reset();
}

// NOLINTNEXTLINE
absl::Status RoutingModule::NoaMainLoop() {
  FUNC_QTRACE();
  // (1) Send to third party routing request (Hacker mode) or use sd route.
  // (2) Update route
  // (3) publish protos
  ASSIGN_OR_RETURN(const auto pos2d,
                   GetGlobalPos2d(input_.localization_transform.get(),
                                  input_.pose.get(), input_.gnss.get()));
  VLOG(3) << "global pos :" << pos2d.DebugString()
          << ", log_routing_result:" << (input_.routing_result != nullptr);
  Vec2d global2d = pos2d.pos;
  std::optional<double> heading = pos2d.heading;
  if (external_route_manager_ == nullptr) {
    external_route_manager_ = std::make_unique<route::ExternalRouteManager>();
  }
  bool is_reset_request = last_rerouting_request_.has_routing_request() &&
                          last_rerouting_request_.routing_request().reset();
  if (is_reset_request) {
    QLOG(INFO) << "Noa Reset routing request.";
    ProcessRoutingResetRequest(external_route_manager_->route_manager_state());
    return absl::OkStatus();
  }

  // Publish msg at last.
  [[maybe_unused]] absl::Cleanup publish_msg_fn = [this] {
    const auto& route_manager_state =
        external_route_manager_->route_manager_state();
    if (external_route_manager_->has_route()) {
      QLOG_IF_NOT_OK(WARNING, Publish(external_route_manager_->route()));
    }
    auto hmi_content_proto = std::make_unique<HmiContentProto>();
    *hmi_content_proto->mutable_route_content() =
        route_manager_state.route_content_proto;
    QLOG_IF_NOT_OK(WARNING, Publish(std::move(hmi_content_proto)));
    auto routing_state_proto = std::make_unique<RoutingStateProto>();
    routing_state_proto->set_state(route_state_);
    *routing_state_proto->mutable_route_mgr_state() =
        route_manager_state.rms_debug;
    *routing_state_proto->mutable_route_proto_from_current() =
        route_manager_state.route_from_current;
    QLOG_IF_NOT_OK(WARNING, Publish(std::move(routing_state_proto)));
    CleanBeforeNextIteration();
  };
  if (semantic_map_listener_ == nullptr) {
    return absl::FailedPreconditionError("Start failed as Noa mode.");
  }

  RouteManagerOutputProto rm_output_proto;
  auto incremental_noa_output = route::noa::IncrementalNoaOutput{
      .external_route_manager = external_route_manager_.get(),
      .rms = external_route_manager_->mutable_route_manager_state(),
      .route_mgr_output_proto = &rm_output_proto,
  };
  RestrictProto restrict_proto;
  if (FLAGS_route_navinfo_restrict != "" &&
      !file_util::TextFileToProto(FLAGS_route_navinfo_restrict,
                                  &restrict_proto)) {
    QLOG_EVERY_N_SEC(ERROR, 3)
        << "Cannot parse proto file: " << FLAGS_route_navinfo_restrict;
  }
  const auto noa_status = route::noa::NoaIncrementalUpdateLoop(
      input_, semantic_map_listener_, route_param_proto_,
      route::noa::NoaStartupConfig{
          .route_snapshot_sim_mode = FLAGS_route_snapshot_sim_mode,
          .route_trunc_mpp_by_nav = FLAGS_route_trunc_mpp_by_nav,
          .route_use_sdroute = FLAGS_route_use_sdroute,
          .route_sd_max_project_distance = FLAGS_route_sd_max_project_distance,
      },
      global2d, heading, std::move(restrict_proto), &incremental_noa_output);
  QLOG_IF_NOT_OK(WARNING, Publish(rm_output_proto));
  last_route_mgr_output_proto_ = std::move(rm_output_proto);
  if (!noa_status.route_nav_status.ok() && input_.mpp != nullptr) {
    QLOG_EVERY_N_SEC(ERROR, 3)
        << "Compute route nav failed. " << noa_status.route_nav_status;
  }
  if (!noa_status.sd_route_status.ok()) {
    QLOG_EVERY_N_SEC(WARNING, 3)
        << "Update sd route failed, check routing result first."
        << noa_status.sd_route_status;
  }
  route_state_ = RoutingStateProto::RUNNING;
  return absl::OkStatus();
}

}  // namespace planner
}  // namespace qcraft
