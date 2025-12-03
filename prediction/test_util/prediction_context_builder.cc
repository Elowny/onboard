#include "onboard/prediction/test_util/prediction_context_builder.h"

#include <memory>  // for make_shared, __shared_ptr_access, shared_ptr
#include <string>  // for to_string, string
#include <utility>

#include "absl/time/time.h"  // for Time

#include "onboard/global/clock.h"                                  // for Clock
#include "onboard/math/geometry/proto/affine_transformation.pb.h"  // for Vec3dProto
#include "onboard/math/geometry/util.h"  // for Vec3dToProto
#include "onboard/math/vec.h"            // for Vec3d
#include "onboard/planner/test_util/perception_object_builder.h"  // for PerceptionObjectBuilder
#include "onboard/prediction/container/av_context.h"       // for AvContext
#include "onboard/prediction/container/objects_history.h"  // for ObjectsHistory
#include "onboard/prediction/container/prediction_context.h"  // for PredictionContext
#include "onboard/proto/localization.pb.h"  // for LocalizationTransformProto
#include "onboard/proto/perception.pb.h"  // for ObjectsProto, ObjectProto, TrafficLightStatesProto
#include "onboard/proto/positioning.pb.h"  // for PoseProto
#include "onboard/proto/vehicle.pb.h"      // for VehicleGeometryParamsProto

namespace qcraft {
namespace prediction {
namespace {
constexpr int kTrackerHistoryLen = 11;
}
PredictionInput BuildPredictionInput(
    const planner::PlannerSemanticMapManager& psmm,
    const VehicleGeometryParamsProto& veh_params,
    const ConflictResolverParams& conf_params) {
  VehicleGeometryParamsProto params;
  params.set_width(1.0);
  auto av_pose = std::make_shared<PoseProto>();
  Vec3dProto av_pos;
  Vec3dToProto(Vec3d(0.0, 0.0, 0.0), &av_pos);
  *av_pose->mutable_pos_smooth() = av_pos;
  auto loc_transform = std::make_shared<LocalizationTransformProto>();
  absl::Time time = Clock::Now();
  PredictionInput prediction_input(/*history_size=*/100, /*history_len=*/1,
                                   /*min_dt=*/0.01,
                                   /*long_term_history_size=*/100,
                                   /*long_term_history_len=*/30,
                                   /*long_term_min_dt=*/0.5);
  prediction_input.prediction_init_time = time;
  auto objects_proto = std::make_shared<ObjectsProto>();
  prediction_input.pose = av_pose;
  prediction_input.traffic_light_states =
      std::make_shared<TrafficLightStatesProto>(TrafficLightStatesProto());
  prediction_input.localization_transform = std::move(loc_transform);

  prediction_input.semantic_map_manager = &psmm;
  prediction_input.veh_geom_params = &veh_params;
  prediction_input.conflict_resolver_params = &conf_params;

  return prediction_input;
}
PredictionContext BuildOneObjectPredictionContext(
    absl::string_view id, PredictionInput* prediction_input) {
  PredictionContext context(*prediction_input);
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_trajectory(kTrackerHistoryLen)
                 .Build();
  auto objects_proto = std::make_shared<ObjectsProto>();
  *objects_proto->add_objects() = std::move(obj);
  prediction_input->av_context->Update(
      *prediction_input->pose, *prediction_input->localization_transform,
      *prediction_input->veh_geom_params);
  prediction_input->objects_history->Update(
      *objects_proto, *prediction_input->localization_transform,
      /*thread_pool=*/nullptr);
  prediction_input->long_term_objects_history->Update(
      *objects_proto, *prediction_input->localization_transform,
      /*thread_pool=*/nullptr);
  prediction_input->real_objects = std::move(objects_proto);
  return context;
}

PredictionContext BuildMultiObjectsPredictionContext(
    const int obj_num, PredictionInput* prediction_input) {
  PredictionContext context(*prediction_input);
  auto objects_proto = std::make_shared<ObjectsProto>();
  for (int i = 0; i < obj_num; ++i) {
    auto obj = planner::PerceptionObjectBuilder()
                   .set_id(std::to_string(i))
                   .set_trajectory(kTrackerHistoryLen)
                   .Build();
    *objects_proto->add_objects() = std::move(obj);
  }
  prediction_input->av_context->Update(
      *prediction_input->pose, *prediction_input->localization_transform,
      *prediction_input->veh_geom_params);
  prediction_input->objects_history->Update(
      *objects_proto, *prediction_input->localization_transform,
      /*thread_pool=*/nullptr);
  prediction_input->long_term_objects_history->Update(
      *objects_proto, *prediction_input->localization_transform,
      /*thread_pool=*/nullptr);
  prediction_input->real_objects = std::move(objects_proto);

  return context;
}

}  // namespace prediction
}  // namespace qcraft
