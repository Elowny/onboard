#include "onboard/prediction/prediction_module.h"

#include <stdint.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>

#include "absl/status/statusor.h"

#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/clock.h"
#include "onboard/global/counter.h"
#include "onboard/global/run_context.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/online_map_gflags.h"
#include "onboard/maps/semantic_map_io.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/container/prediction_runner.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace prediction {
PredictionModule::PredictionModule(LiteClientBase* client)
    : LiteModule(client) {
  if (FLAGS_prediction_thread_pool_size > 0) {
    thread_pool_ =
        std::make_unique<ThreadPool>(FLAGS_prediction_thread_pool_size);
  }
}

void PredictionModule::OnInit() {
  RunParamsProtoV2 run_params;
  param_manager().GetRunParams(&run_params);
  prediction_conflict_resolver_params_.LoadParams();
  vehicle_params_ = run_params.vehicle_params();

  prediction_input_ = std::make_unique<PredictionInput>(
      kTTLSteps, kHistoryLen, kShortTermDt, kTTLSteps, kLongTermHistoryLen,
      kLongTermDt);

  hd_semantic_map_multilevel_spatial_index_ = nullptr;
  vision_semantic_map_multilevel_spatial_index_ = nullptr;
  planner_semantic_map_manager_ = nullptr;

  prediction_input_->veh_geom_params =
      &vehicle_params_.vehicle_geometry_params();
  prediction_input_->traffic_light_states =
      std::make_shared<TrafficLightStatesProto>();
  prediction_input_->conflict_resolver_params =
      &prediction_conflict_resolver_params_;

  hd_semantic_map_listener_ =
      mapping::v2::SemanticMapMultilevelSpatialIndexListenerAsnyc::MakeShared(
          {OnlineMapProto_DataSource_QCRAFT_HDMAP,
           OnlineMapProto_DataSource_NAVINFO_HDMAP});
  mapping::BuildOption build_option;
  hd_semantic_map_listener_->SetLoadOption({
      .build_option = build_option,
  });
  if (FLAGS_publish_navinfo_hdmap) {
    hd_semantic_map_listener_->EnableRouteFilter(/*enable=*/true);
  }

  prediction_model_pool_ = std::make_unique<ModelPool>(
      param_manager(), param_finder(), GetNetResourceConfigMap());

  QLOG(INFO) << "Prediction module init finished";
}

void PredictionModule::OnSubscribeChannels() {
  // The same as planner module, always take the latest message.
  Subscribe(&PredictionModule::HandlePose, this, /*buffer_size=*/1);
  Subscribe(&PredictionModule::HandleLocalizationTransform, this, 20);
  Subscribe(&PredictionModule::HandleObjects, this, 20);
  Subscribe(&PredictionModule::HandleTrafficLightStates, this, 20);
  Subscribe(&PredictionModule::HandlePlannerState, this, 20);
  Subscribe(&PredictionModule::HandleOnlineSemanticMap, this, 20);
  Subscribe(&mapping::v2::SemanticMapMultilevelSpatialIndexListenerAsnyc::
                UpdateOnlineMap,
            hd_semantic_map_listener_.get(), 20);
  Subscribe(&PredictionModule::HandleRoutingManagerOutputResult, this, 20);
  Subscribe(&PredictionModule::HandleAutonomyState, this, 20);
}

void PredictionModule::HandleLocalizationTransform(
    std::shared_ptr<const LocalizationTransformProto>
        localization_transform_proto) {
  SCOPED_QTRACE("PredictionModule::HandleLocalizationTransform");
  prediction_input_->localization_transform =
      std::move(localization_transform_proto);
}

void PredictionModule::HandleTrafficLightStates(
    std::shared_ptr<const TrafficLightStatesProto> tl_states) {
  prediction_input_->traffic_light_states = std::move(tl_states);
}

void PredictionModule::HandleOnlineSemanticMap(
    std::shared_ptr<const mapping::OnlineSemanticMapProto>
        online_semantic_map) {
  online_semantic_map_ = std::move(online_semantic_map);
}

void PredictionModule::HandleObjects(
    std::shared_ptr<const ObjectsProto> objects) {
  FUNC_QTRACE();
  QCHECK(objects && objects->has_scope());
  if (prediction_input_->localization_transform == nullptr) {
    return;
  }
  // Prediction V2 operation
  absl::MutexLock lock(&unprocessed_objects_or_mutex_);
  switch (objects->scope()) {
    case ObjectsProto::SCOPE_REAL: {
      unprocessed_real_objects_or_.emplace(
          objects, prediction_input_->localization_transform);
      prediction_input_->real_objects = std::move(objects);
      break;
    }
    case ObjectsProto::SCOPE_VIRTUAL:
      unprocessed_virtual_objects_or_.emplace(
          objects, prediction_input_->localization_transform);
      prediction_input_->virtual_objects = std::move(objects);
      break;
    case ObjectsProto::SCOPE_AV:
      break;
  }
}

void PredictionModule::HandlePose(std::shared_ptr<const PoseProto> pose) {
  const auto now = Clock::Now();
  const double pose_delay_ms =
      (ToUnixDoubleSeconds(now) - pose->timestamp()) * 1e3;
  SCOPED_QTRACE_ARG2("PredictionModule::HandlePose", "now",
                     ToUnixDoubleSeconds(now), "pose_delay_ms", pose_delay_ms);
  QCOUNTER("prediction_pose_delay_ms", static_cast<int64_t>(pose_delay_ms));

  prediction_input_->pose = std::move(pose);

  constexpr absl::Duration kPredictionMainLoopInterval =
      absl::Milliseconds(100);
  if (pose_delay_ms > FLAGS_prediction_max_allowed_pose_delay_ms) {
    // Set last_iteration_time_ to a past that is not too far away from now to
    // avoid prediction module being kickout when pose delay is large.
    last_iteration_time_ =
        std::max(now - kPredictionMainLoopInterval - absl::Milliseconds(1),
                 last_iteration_time_);
    QLOG_EVERY_N_SEC(WARNING, 1)
        << "The pose is stale. Delay: " << pose_delay_ms << " ms.";
    // Wait for the next pose in queue.
    return;
  }
  // Note(runlin): last_iteration_time_ is epoch time when not set.
  const auto duration_since_last_iteration = now - last_iteration_time_;
  if (duration_since_last_iteration < kPredictionMainLoopInterval) {
    // Not ready for the next iteration.
    return;
  }
  if (prediction_input_->localization_transform == nullptr) {
    return;
  }
  auto status = PredictionMainLoop();
  if (!status.ok()) {
    QLOG(ERROR) << "Prediction main loop failed: " << status;
  }

  const auto iteration_time = Clock::Now() - now;
  if (iteration_time > kPredictionMainLoopInterval + absl::Milliseconds(50.0)) {
    QEVENT("lidong", "prediction_timeout", [iteration_time](QEvent* event) {
      event->AddField("duration_seconds",
                      absl::ToDoubleSeconds(iteration_time));
    });
  }
  if (absl::ToDoubleSeconds(iteration_time) >
      FLAGS_prediction_max_allowed_iteration_time) {
    QLOG(ERROR) << "Prediction process timeout";
    QISSUEX(QIssueSeverity::QIS_ERROR, QIssueType::QIT_PERFORMANCE,
            QIssueSubType::QIST_PREDICTION_PROCESS_TIMEOUT,
            "Prediction timeout.");
  }
  last_iteration_time_ = now;
}

void PredictionModule::HandlePlannerState(
    const std::shared_ptr<const planner::PlannerStateProto>& planner_state) {
  if (planner_state->has_prev_route_sections()) {
    section_seq_ = planner_state->prev_route_sections();
  } else {
    section_seq_ = RouteSectionSequenceProto();
  }
  if (planner_state->has_hd_map_state()) {
    planner::PlannerState::HdMapState hd_map_state;
    hd_map_state.load_distance = planner_state->hd_map_state().load_distance();
    hd_map_state.has_destination =
        planner_state->hd_map_state().has_destination();
    hd_map_state_ = hd_map_state;
  } else {
    hd_map_state_ = std::nullopt;
  }
}

void PredictionModule::HandleRoutingManagerOutputResult(
    std::shared_ptr<const planner::RouteManagerOutputProto>
        route_manager_output) {
  if (route_manager_output != nullptr) {
    prediction_input_->route_manager_output = std::move(route_manager_output);
  }
}

void PredictionModule::HandleAutonomyState(
    std::shared_ptr<const AutonomyStateProto> autonomy) {
  if (!FLAGS_prediction_use_autonomy_state) {
    return;
  }
  prediction_input_->autonomy_state = std::move(autonomy);
}

void PredictionModule::UpdateOnlineHDMap() {
  SCOPED_QTRACE("PredictionModule::UpdateOnlineHDMap");
  const auto coordinate_converter =
      CoordinateConverter::FromLocalizationTransform(
          *prediction_input_->localization_transform);
  const Vec2d pos_global = coordinate_converter.SmoothToGlobal(
      Vec2d{prediction_input_->pose->pos_smooth().x(),
            prediction_input_->pose->pos_smooth().y()});

  const auto* overlap_sections =
      prediction_input_->route_manager_output != nullptr &&
              prediction_input_->route_manager_output
                  ->has_overlap_sections_precomputed() &&
              prediction_input_->route_manager_output
                  ->overlap_sections_precomputed()
          ? &prediction_input_->route_manager_output->overlap_sections()
          : nullptr;

  const auto& section_seq =
      !section_seq_.section_id().empty()
          ? section_seq_
          : (prediction_input_->route_manager_output != nullptr
                 ? prediction_input_->route_manager_output
                       ->route_sections_from_current()
                 : RouteSectionSequenceProto());
  const auto smmsi = planner::UpdateHdSemanticMapManagerAlongRoute(
      pos_global, section_seq, hd_map_state_, overlap_sections,
      hd_semantic_map_listener_.get());
  if (smmsi == nullptr) {
    return;
  }

  const bool got_new_smmsi =
      hd_semantic_map_multilevel_spatial_index_ == nullptr ||
      smmsi->semantic_manager()->update_id() !=
          hd_semantic_map_multilevel_spatial_index_->semantic_manager()
              ->update_id();

  if (got_new_smmsi && !pending_load_hd_psmm_) {
    psmm_future_ = planner::AsyncLoadPlannerSemanticMapManager(
        smmsi, coordinate_converter, thread_pool_.get());
    pending_load_hd_psmm_ = true;
    if (planner_semantic_map_manager_ == nullptr) {
      psmm_future_.Wait();
    }
  }

  if (psmm_future_.IsReady()) {
    planner_semantic_map_manager_ = psmm_future_.Get();
    pending_load_hd_psmm_ = false;
    hd_semantic_map_multilevel_spatial_index_ =
        planner_semantic_map_manager_->semantic_map_multilevel_spatial_index();
  }
  // Update elements of psmm.
  planner_semantic_map_manager_->UpdateCoordinateConverter(
      coordinate_converter);
  planner_semantic_map_manager_->UpdateSmoothInfoOfMapElements(
      thread_pool_.get());
  prediction_input_->semantic_map_manager = planner_semantic_map_manager_.get();
}

absl::Status PredictionModule::UpdateOnlineSemanticMap() {
  SCOPED_QTRACE("PredictionModule::UpdateOnlineSemanticMap");
  if (FLAGS_prediction_enable_debug_no_map) {
    const mapping::OnlineSemanticMapProto proto;
    ASSIGN_OR_RETURN(planner_semantic_map_manager_,
                     planner::BuildOnlineMapPsmm(proto));
    prediction_input_->semantic_map_manager =
        planner_semantic_map_manager_.get();
    return absl::OkStatus();
  }

  if (prediction_input_->localization_transform == nullptr) {
    return absl::FailedPreconditionError(
        "Failed to load semantic map as localization transform is not "
        "received.");
  }
  if (prediction_input_->pose == nullptr) {
    return absl::FailedPreconditionError(
        "Failed to load semantic map as pose is not received.");
  }
  UpdateOnlineHDMap();
  if (IsRunModeL4() ||
      (prediction_input_->autonomy_state != nullptr &&
       MustReceiveHDMapForPrediction(*prediction_input_->autonomy_state))) {
    if (prediction_input_->semantic_map_manager == nullptr) {
      return absl::FailedPreconditionError("Should use HD map but it is null.");
    }
  } else {
    if (online_semantic_map_ == nullptr) {
      QLOG(WARNING) << "Should use perception map firstly but it is null, "
                       "try to use HD map.";
      if (prediction_input_->semantic_map_manager == nullptr) {
        return absl::FailedPreconditionError(
            "Both perception map and HD map are null.");
      }
      return absl::OkStatus();
    }
    ASSIGN_OR_RETURN(
        planner_semantic_map_manager_,
        planner::BuildOnlineMapPsmm(*online_semantic_map_, thread_pool_.get()));
    vision_semantic_map_multilevel_spatial_index_ =
        planner_semantic_map_manager_->semantic_map_multilevel_spatial_index();
    prediction_input_->semantic_map_manager =
        planner_semantic_map_manager_.get();
  }
  return absl::OkStatus();
}

void PredictionModule::UpdateReceivedObjects() {
  absl::MutexLock lock(&unprocessed_objects_or_mutex_);
  SCOPED_QTRACE("PredictionMainLoop::UpdateObjects");
  if (unprocessed_real_objects_or_.has_value()) {
    const auto& objects_proto = *unprocessed_real_objects_or_.value().first;
    const auto& transform_proto = *unprocessed_real_objects_or_.value().second;
    prediction_input_->objects_history->Update(objects_proto, transform_proto,
                                               thread_pool_.get());
    prediction_input_->long_term_objects_history->Update(
        objects_proto, transform_proto, thread_pool_.get());
    unprocessed_real_objects_or_.reset();
  }
  if (unprocessed_virtual_objects_or_.has_value()) {
    const auto& objects_proto = *unprocessed_virtual_objects_or_.value().first;
    const auto& transform_proto =
        *unprocessed_virtual_objects_or_.value().second;
    prediction_input_->objects_history->Update(objects_proto, transform_proto,
                                               thread_pool_.get());
    prediction_input_->long_term_objects_history->Update(
        objects_proto, transform_proto, thread_pool_.get());
    unprocessed_virtual_objects_or_.reset();
  }
}

absl::Status PredictionModule::PredictionMainLoop() {
  FUNC_QTRACE();

  RETURN_IF_ERROR(UpdateOnlineSemanticMap());

  prediction_input_->prediction_init_time = Clock::Now();
  prediction_input_->av_context->Update(
      *prediction_input_->pose, *prediction_input_->localization_transform,
      *prediction_input_->veh_geom_params);
  // Update received objects.
  UpdateReceivedObjects();

  prediction_debug_.Clear();
  auto prediction_or =
      ComputePrediction(*prediction_input_, prediction_model_pool_.get(),
                        thread_pool_.get(), &prediction_debug_);

  auto prediction = std::make_unique<ObjectsPredictionProto>();
  if (prediction_or.ok()) {
    prediction = std::move(prediction_or).value();
    prediction->set_post_process_in_planner(
        FLAGS_prediction_run_post_process_in_planner);
  } else {
    QLOG(ERROR) << "Compute prediction failed: " << prediction_or.status();
  }

  QLOG_IF_NOT_OK(WARNING, Publish(std::move(prediction)))
      << "Publish prediction failed.";
  QLOG_IF_NOT_OK(WARNING, Publish(prediction_debug_))
      << "Publish prediction debug failed";
  return absl::OkStatus();
}

PredictionModule::~PredictionModule() {}

}  // namespace prediction
}  // namespace qcraft
