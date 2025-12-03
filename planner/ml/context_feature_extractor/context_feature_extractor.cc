#include "onboard/planner/ml/context_feature_extractor/context_feature_extractor.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include <algorithm>  // for max
#include <memory>     // for allocator, allocator_traits<>::val...
#include <numeric>    // for accumulate
#include <utility>    // for move
#include <vector>     // for vector

#include "onboard/global/trace.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/box2d.h"  // for Box2d
#include "onboard/math/util.h"
#include "onboard/planner/ml/context_feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/planner/ml/planner_ml_defs.h"
#include "onboard/planner/object/object_vector.h"         // for ObjectVector
#include "onboard/planner/object/planner_object.h"        // for PlannerObject
#include "onboard/prediction/container/object_history.h"  // for ObjectHistory
#include "onboard/prediction/container/prediction_object.h"  // for GetStationaryState
#include "onboard/prediction/container/traffic_light_manager.h"  // for TrafficLightManager
#include "onboard/prediction/feature_extractor/act_net_feature.h"  // for ActNetObjectFeature, ActNetPolylin...
#include "onboard/prediction/feature_extractor/act_net_feature_extractor.h"  // for ExtractMapFeatureFromPolylines
#include "onboard/prediction/feature_extractor/map_sampler.h"  // for MapSampler, MapSampler::SampleType
#include "onboard/prediction/object_prediction.h"  // for ObjectLongTermBehavior, ObjectPred...
#include "onboard/prediction/prediction_defs.h"  // for ObjectMotionHistory, kAvObjectId
#include "onboard/prediction/prediction_util.h"  // for ObjectLongTermBehavior, ObjectPred...

namespace qcraft {
namespace planner {
namespace ml {

double GetCurrentTimeStamp(const prediction::AvContext& av_context,
                           const PlannerObjectManager& object_manager) {
  SCOPED_QTRACE("ContextFeature::GetCurrentTimeStamp");
  const double av_ts = av_context.GetAvObjectHistory().timestamp();
  const auto& objs = object_manager.planner_objects();
  return (std::accumulate(objs.begin(), objs.end(), 0.0,
                          [](const double sum, const auto& obj) {
                            return sum + obj.object_proto().timestamp();
                          }) +
          av_ts) /
         (objs.size() + 1);
}

double GetCurrentTimeStamp(
    const prediction::AvContext& av_context,
    const std::shared_ptr<const ObjectsProto>& real_objects,
    const std::shared_ptr<const ObjectsProto>& virtual_objects) {
  SCOPED_QTRACE("ContextFeature::GetCurrentTimeStamp");
  const double av_ts = av_context.GetAvObjectHistory().timestamp();
  const auto& real_objects_ts =
      real_objects != nullptr
          ? std::accumulate(real_objects->objects().begin(),
                            real_objects->objects().end(), 0.0,
                            [](const double sum, const auto& obj) {
                              return sum + obj.timestamp();
                            })
          : 0.0;
  const auto& real_objects_size =
      real_objects != nullptr ? real_objects->objects().size() : 0;
  const auto& virtual_objects_ts =
      virtual_objects != nullptr
          ? std::accumulate(virtual_objects->objects().begin(),
                            virtual_objects->objects().end(), 0.0,
                            [](const double sum, const auto& obj) {
                              return sum + obj.timestamp();
                            })
          : 0.0;
  const auto& virtual_objects_size =
      virtual_objects != nullptr ? virtual_objects->objects().size() : 0;
  return (real_objects_ts + virtual_objects_ts + av_ts) /
         (real_objects_size + virtual_objects_size + 1);
}

InferredFeature ExtractInferredFeature(
    const PlannerObjectManager& object_manager) {
  SCOPED_QTRACE("ContextFeature::ExtractInferredFeature");
  InferredFeature inferred_feature;
  constexpr float kIsLongTermObservation = 0.1;  // s.
  for (const auto& planner_obj : object_manager.planner_objects()) {
    ObjectInferredFeature feat;
    feat.object_id = planner_obj.id();
    feat.is_static = prediction::GetStationaryState(planner_obj.object_proto());
    const auto& lt = planner_obj.prediction().long_term_behavior();
    if (lt.obs_duration < kIsLongTermObservation) {
      feat.has_longterm_observation = true;
      feat.average_speed = lt.avg_speed;
      feat.observation_duration = lt.obs_duration;
    } else {
      feat.has_longterm_observation = false;
    }
    inferred_feature.inferred_object_feature.push_back(feat);
  }
  return inferred_feature;
}

ContextFeature ExtractContextFeature(
    const ContextFeatureExtractionInput& input) {
  SCOPED_QTRACE("ContextFeature::ExtractContextFeature");
  // Get Current Timestamp which decides the ref pose and heading
  const double current_ts = GetCurrentTimeStamp(
      *input.av_context, input.real_objects, input.virtual_objects);
  auto act_net_feature = ExtractActNetContextFeature(current_ts, input);
  const auto ref_position = act_net_feature.ref_position;
  const auto rot_rad = act_net_feature.rot_rad;

  auto inferred_feature = ExtractInferredFeature(*input.object_manager);
  return {.current_ts = current_ts,
          .ref_position = ref_position,
          .rot_rad = rot_rad,
          .act_net_feature = std::move(act_net_feature),
          .inferred_feature = std::move(inferred_feature)};
}

prediction::ActNetFeature ExtractActNetContextFeature(
    const double current_ts, const ContextFeatureExtractionInput& input) {
  SCOPED_QTRACE("ContextFeature::ExtractModelContextFeature");
  // Get config.
  const auto& config = kActNetContextConfig;

  // Construct traffic light manager.
  prediction::TrafficLightManager traffic_light_manager;
  traffic_light_manager.UpdateTlStateMap(*input.psmm,
                                         *input.traffic_light_states);

  // Construct object sampler.
  const ObjectHistorySampler obj_sampler(
      input.real_objects, input.virtual_objects,
      input.av_context->GetAvObjectHistory(), current_ts,
      config.history_time_step, config.history_step_num,
      /*enable_smoothing=*/true);

  // Construct map sampler.
  prediction::MapSampler map_sampler(
      *input.psmm, traffic_light_manager.GetOriginalTlStateMap(),
      kMaxMapSampleLen, kMapSegmentNum,
      prediction::MapSampler::SampleType::ADAPTIVE);

  // Get ref pose.
  const auto& agent_history =
      obj_sampler.GetResampledMotionHistoryByObjectId(prediction::kAvObjectId);
  const double heading = agent_history.states.back().heading;
  auto ref_position = agent_history.states.back().pos;
  const double rot_rad = -heading;
  const double normal_rot_rad = NormalizeAngle(rot_rad);
  const auto agent_coord_transform = prediction::AgentCoordTransform{
      .orig = ref_position,
      .rot_rad = rot_rad,
      .rot_sin = fast_math::SinNormalized(normal_rot_rad),
      .rot_cos = fast_math::CosNormalized(normal_rot_rad)};

  // Get agent feature.
  auto agent_feat = prediction::GetActNetObjectAbsoluteFeature(
      agent_history, agent_coord_transform, config.coord_num,
      config.history_time_step, config.history_step_num);

  // Get objects feature.
  const Box2d region_box = prediction::GetRegionBox(
      ref_position, heading, config.scan_box_front, config.scan_box_back,
      config.scan_box_half_width);
  const auto objects_in_box2d = obj_sampler.GetResampledMotionHistoryInBox2d(
      prediction::kAvObjectId, region_box, config.max_objects_num);
  std::vector<prediction::ActNetObjectFeature> context_obj_features;
  context_obj_features.reserve(objects_in_box2d.size());
  for (const auto* obj_hist : objects_in_box2d) {
    auto abs_feat = prediction::GetActNetObjectAbsoluteFeature(
        *obj_hist, agent_coord_transform, config.coord_num,
        config.history_time_step, config.history_step_num);

    auto rel_feat = prediction::GetActNetObjectRelFeature(
        agent_feat, abs_feat, config.coord_num, config.history_step_num);
    context_obj_features.push_back(prediction::ActNetObjectFeature{
        .id = obj_hist->id,
        .abs_feat = std::move(abs_feat),
        .rel_feat = std::move(rel_feat),
    });
  }
  std::vector<float> context_obj_masks(config.max_objects_num, 0.0);
  for (int i = 0; i < context_obj_features.size(); ++i) {
    context_obj_masks[i] = 1.0;
  }

  // Get map feature.
  const auto lcs = map_sampler.GetNearestLaneCenterPolylinesInBox2d(
      region_box, config.max_lc_num);
  auto lc_feats = prediction::ExtractMapFeatureFromPolylines(
      lcs, region_box, agent_coord_transform, config.max_lane_seg_num,
      config.max_lc_num);
  std::vector<float> lc_masks(config.max_lc_num, 0.0);
  for (int i = 0; i < lc_feats.size(); ++i) {
    lc_masks[i] = 1.0;
  }
  const auto lbs = map_sampler.GetNearestSolidLaneBoundaryPolylinesInBox2d(
      region_box, config.max_lb_num);
  auto lb_feats = prediction::ExtractMapFeatureFromPolylines(
      lbs, region_box, agent_coord_transform, config.max_lane_seg_num,
      config.max_lb_num);
  std::vector<float> lb_masks(config.max_lb_num, 0.0);
  for (int i = 0; i < lb_feats.size(); ++i) {
    lb_masks[i] = 1.0;
  }
  const auto cws = map_sampler.GetNearestCrosswalkPolylinesInBox2d(
      region_box, config.max_crosswalk_num);
  auto cw_feats = prediction::ExtractMapFeatureFromPolylines(
      cws, region_box, agent_coord_transform, config.max_lane_seg_num,
      config.max_crosswalk_num);
  std::vector<float> cw_masks(config.max_crosswalk_num, 0.0);
  for (int i = 0; i < cw_feats.size(); ++i) {
    cw_masks[i] = 1.0;
  }

  auto actnet_feature = prediction::ActNetFeature{
      .agent_id = prediction::kAvObjectId,
      .ref_position = std::move(ref_position),
      .rot_rad = static_cast<float>(rot_rad),
      .agent_feature = std::move(agent_feat),
      .context_obj_features = std::move(context_obj_features),
      .context_obj_masks = std::move(context_obj_masks),
      .lc_features = std::move(lc_feats),
      .solid_lb_features = std::move(lb_feats),
      .cw_features = std::move(cw_feats),
      .lc_masks = std::move(lc_masks),
      .solid_lb_masks = std::move(lb_masks),
      .cw_masks = std::move(cw_masks),
  };

  return actnet_feature;
}

}  // namespace ml
}  // namespace planner
}  // namespace qcraft
