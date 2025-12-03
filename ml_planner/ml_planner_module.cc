#include "onboard/ml_planner/ml_planner_module.h"

#include <ostream>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "glog/logging.h"

#include "onboard/async/thread_pool.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/run_context.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/maps/online_map_gflags.h"
#include "onboard/maps/semantic_map_io.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/ml_planner/ml_planner_defs.h"
#include "onboard/ml_planner/ml_planner_flags.h"
#include "onboard/ml_planner/ml_planner_status.h"
#include "onboard/ml_planner/ml_task_dispatcher.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/prediction_message_compressor.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::mlplanner {

namespace {
// TODO(jingjiao): Move to ml planner utils.
bool MustReceiveHDMapForMLPlanner(const AutonomyStateProto& autonomy_state) {
  if (!autonomy_state.has_assist_state()) {
    return true;
  }
  if (!autonomy_state.assist_state().has_assist_drive_system_state()) {
    return true;
  }
  switch (autonomy_state.assist_state().assist_drive_system_state()) {
    case AssistStateProto::ASSIST_OFF:
    case AssistStateProto::ASSIST_NOT_READY:
    case AssistStateProto::ASSIST_ACC_READY:
    case AssistStateProto::ASSIST_LCC_READY:
    case AssistStateProto::ASSIST_NOA_READY:
    case AssistStateProto::ASSIST_LCC_ACTIVE:
    case AssistStateProto::ASSIST_ACC_ACTIVE:
    case AssistStateProto::ASSIST_APA_ACTIVE:
      return false;
    case AssistStateProto::ASSIST_NOA_ACTIVE: {
      return true;
    }
  }
}

}  // namespace
MlPlannerModule::MlPlannerModule(LiteClientBase* client) : LiteModule(client) {}

MlPlannerModule::~MlPlannerModule() {}

void MlPlannerModule::OnInit() {
  ml_planner_input_ = std::make_unique<MLPlannerInput>();
  ml_planner_debug_ = std::make_unique<MlPlannerDebugProto>();

  RunParamsProtoV2 run_params;
  param_manager().GetRunParams(&run_params);
  ml_planner_input_->vehicle_params = run_params.vehicle_params();
  ml_planner_input_->prediction_conflict_resolver_params.LoadParams();

  if (FLAGS_ml_planner_thread_pool_size > 0) {
    ml_planner_input_->thread_pool =
        std::make_unique<ThreadPool>(FLAGS_ml_planner_thread_pool_size);
  }

  ml_planner_input_->model_pool =
      std::make_unique<planner::ModelPool>(param_manager(), param_finder());

  ml_planner_input_->av_context = std::make_unique<prediction::AvContext>(
      kAvHistoryBufferSize, kAvHistoryBufferLen);
  ml_planner_input_->traffic_light_states =
      std::make_shared<TrafficLightStatesProto>();

  hd_semantic_map_multilevel_spatial_index_ = nullptr;
  vision_semantic_map_multilevel_spatial_index_ = nullptr;

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

  QLOG(INFO) << "ML planner module init finished";
}

void MlPlannerModule::OnSubscribeChannels() {
  Subscribe(&MlPlannerModule::HandlePose, this, 50);
  Subscribe(&MlPlannerModule::HandlePrediction, this, 20);
  Subscribe(&MlPlannerModule::HandleLocalizationTransform, this, 20);
  Subscribe(&MlPlannerModule::HandlePlannerState, this, 20);
  Subscribe(&MlPlannerModule::HandleOnlineSemanticMap, this, 20);
  Subscribe(&mapping::v2::SemanticMapMultilevelSpatialIndexListenerAsnyc::
                UpdateOnlineMap,
            hd_semantic_map_listener_.get(), 20);
  Subscribe(&MlPlannerModule::HandleAutonomyState, this, 20);
}

void MlPlannerModule::HandlePose(std::shared_ptr<const PoseProto> pose) {
  SCOPED_QTRACE("MlPlannerModule::HandlePose");
  ml_planner_input_->pose = std::move(pose);

  if (ml_planner_input_->localization_transform == nullptr) {
    return;
  }

  const auto status = MLPlannerMainLoop();
  if (!status.ok()) {
    QLOG(ERROR) << "ML planner main loop failed: " << status;
  }
}

void MlPlannerModule::HandlePrediction(
    const std::shared_ptr<const ObjectsPredictionProto>& prediction) {
  SCOPED_QTRACE("MlPlannerModule::HandlePrediction");
  auto decompressed_prediction =
      std::make_unique<ObjectsPredictionProto>(*prediction);
  prediction::DecompressObjectsPredictionProto(decompressed_prediction.get());
  ml_planner_input_->prediction = std::move(decompressed_prediction);
}

void MlPlannerModule::HandleLocalizationTransform(
    std::shared_ptr<const LocalizationTransformProto>
        localization_transform_proto) {
  SCOPED_QTRACE("MlPlannerModule::HandleLocalizationTransform");
  ml_planner_input_->localization_transform =
      std::move(localization_transform_proto);
}

void MlPlannerModule::HandleTrafficLightStates(
    std::shared_ptr<const TrafficLightStatesProto> tl_states) {
  SCOPED_QTRACE("MlPlannerModule::HandleTrafficLightStates");
  ml_planner_input_->traffic_light_states = std::move(tl_states);
}

void MlPlannerModule::HandleObjects(
    std::shared_ptr<const ObjectsProto> objects) {
  SCOPED_QTRACE("MlPlannerModule::HandleObjects");
  QCHECK(objects && objects->has_scope());
  switch (objects->scope()) {
    case ObjectsProto::SCOPE_REAL: {
      ml_planner_input_->real_objects = std::move(objects);
      break;
    }
    case ObjectsProto::SCOPE_VIRTUAL:
      ml_planner_input_->virtual_objects = std::move(objects);
      break;
    case ObjectsProto::SCOPE_AV:
      ml_planner_input_->av_objects = std::move(objects);
      break;
  }
}

void MlPlannerModule::HandleOnlineSemanticMap(
    std::shared_ptr<const mapping::OnlineSemanticMapProto>
        online_semantic_map) {
  SCOPED_QTRACE("MlPlannerModule::HandleOnlineSemanticMap");
  online_semantic_map_ = std::move(online_semantic_map);
}

void MlPlannerModule::HandlePlannerState(
    const std::shared_ptr<const planner::PlannerStateProto>& planner_state) {
  SCOPED_QTRACE("MlPlannerModule::HandlePlannerState");
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

void MlPlannerModule::HandleAutonomyState(
    std::shared_ptr<const AutonomyStateProto> autonomy) {
  SCOPED_QTRACE("MlPlannerModule::HandleAutonomyState");
  if (!FLAGS_ml_planner_use_autonomy_state) {
    return;
  }
  ml_planner_input_->autonomy_state = std::move(autonomy);
}

void MlPlannerModule::UpdateOnlineHDMap() {
  SCOPED_QTRACE("MlPlannerModule::UpdateOnlineHDMap");
  const auto coordinate_converter =
      CoordinateConverter::FromLocalizationTransform(
          *ml_planner_input_->localization_transform);
  const Vec2d pos_global = coordinate_converter.SmoothToGlobal(
      Vec2d{ml_planner_input_->pose->pos_smooth().x(),
            ml_planner_input_->pose->pos_smooth().y()});

  // TODO(zuowei): Maybe inject route manager output for planner ml.
  const auto smmsi = planner::UpdateHdSemanticMapManagerAlongRoute(
      pos_global, section_seq_, hd_map_state_,
      /*::google::protobuf::RepeatedPtrField<OverlappingSection>=*/nullptr,
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
        smmsi, coordinate_converter, ml_planner_input_->thread_pool.get());
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
      ml_planner_input_->thread_pool.get());
  ml_planner_input_->planner_semantic_map_manager =
      planner_semantic_map_manager_;
}

absl::Status MlPlannerModule::UpdateOnlineSemanticMap() {
  SCOPED_QTRACE("MlPlannerModule::UpdateOnlineSemanticMap");
  if (FLAGS_ml_planner_enable_debug_no_map) {
    const mapping::OnlineSemanticMapProto proto;
    ASSIGN_OR_RETURN(planner_semantic_map_manager_,
                     planner::BuildOnlineMapPsmm(proto));
    ml_planner_input_->planner_semantic_map_manager =
        planner_semantic_map_manager_;
    return absl::OkStatus();
  }
  if (FLAGS_smm_force_built_by_online_map) {
    if (online_semantic_map_ == nullptr) {
      return absl::FailedPreconditionError("Empty online map.");
    }
    ASSIGN_OR_RETURN(
        planner_semantic_map_manager_,
        planner::BuildOnlineMapPsmm(*online_semantic_map_,
                                    ml_planner_input_->thread_pool.get()));
    vision_semantic_map_multilevel_spatial_index_ =
        planner_semantic_map_manager_->semantic_map_multilevel_spatial_index();

    ml_planner_input_->planner_semantic_map_manager =
        planner_semantic_map_manager_;
    return absl::OkStatus();
  }

  if (ml_planner_input_->localization_transform == nullptr) {
    return absl::FailedPreconditionError(
        "Failed to load semantic map as localization transform is not "
        "received.");
  }
  if (ml_planner_input_->pose == nullptr) {
    return absl::FailedPreconditionError(
        "Failed to load semantic map as pose is not received.");
  }
  UpdateOnlineHDMap();
  if (IsRunModeL4() ||
      (ml_planner_input_->autonomy_state != nullptr &&
       MustReceiveHDMapForMLPlanner(*ml_planner_input_->autonomy_state))) {
    if (ml_planner_input_->planner_semantic_map_manager == nullptr) {
      return absl::FailedPreconditionError("Should use HD map but it is null.");
    }
  } else {
    if (online_semantic_map_ == nullptr) {
      QLOG(WARNING) << "Should use perception map firstly but it is null, "
                       "try to use HD map.";
      if (ml_planner_input_->planner_semantic_map_manager == nullptr) {
        return absl::FailedPreconditionError(
            "Both perception map and HD map are null.");
      }
      return absl::OkStatus();
    }
    ASSIGN_OR_RETURN(
        planner_semantic_map_manager_,
        planner::BuildOnlineMapPsmm(*online_semantic_map_,
                                    ml_planner_input_->thread_pool.get()));
    vision_semantic_map_multilevel_spatial_index_ =
        planner_semantic_map_manager_->semantic_map_multilevel_spatial_index();
    ml_planner_input_->planner_semantic_map_manager =
        planner_semantic_map_manager_;
  }
  return absl::OkStatus();
}

absl::Status MlPlannerModule::MLPlannerMainLoop() {
  SCOPED_QTRACE("MlPlannerModule::MLPlannerMainLoop");

  RETURN_IF_ERROR(UpdateOnlineSemanticMap());

  // ml_planner_input.plan_init_time = Clock::Now();

  // Collect objects.
  VLOG(2) << "Exporting objects from objects view.";
  const auto objects_proto = planner::GetAllObjects(
      ml_planner_input_->real_objects, ml_planner_input_->virtual_objects);
  // Update av history.
  ml_planner_input_->av_context->Update(
      *ml_planner_input_->pose, *ml_planner_input_->localization_transform,
      ml_planner_input_->vehicle_params.vehicle_geometry_params());

  ml_planner_debug_->Clear();

  if (ml_planner_input_->prediction == nullptr) {
    return absl::FailedPreconditionError("No prediction.");
  }

  auto ml_planner_output = std::make_unique<MlPlannerOutputProto>();

  const auto status = RunMLTaskDispatcher(
      *ml_planner_input_, std::as_const(objects_proto).get(),
      ml_planner_output.get(), ml_planner_debug_.get());

  if (!status.ok()) {
    QLOG(ERROR) << "Compute ml planner failed: " << status.ToString();
  }

  QLOG_IF_NOT_OK(WARNING, Publish(std::move(ml_planner_output)))
      << "Publish ml planner failed.";
  QLOG_IF_NOT_OK(WARNING, Publish(*ml_planner_debug_))
      << "Publish prediction debug failed";
  return absl::OkStatus();
}

}  // namespace qcraft::mlplanner
