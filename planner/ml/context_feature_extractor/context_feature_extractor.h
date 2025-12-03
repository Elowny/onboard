#ifndef ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_CONTEXT_FEATURE_EXTRACTOR_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_CONTEXT_FEATURE_EXTRACTOR_H_  // NOLINT

#include <memory>

#include "onboard/planner/ml/context_feature_extractor/context_feature.h"  // for ContextFeature
#include "onboard/planner/object/planner_object_manager.h"  // for PlannerObjectManager
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/container/av_context.h"  // for AvContext
#include "onboard/prediction/feature_extractor/act_net_feature.h"  // for ActNetFeature
#include "onboard/proto/perception.pb.h"  // for TrafficLightStatesProto

namespace qcraft {
namespace planner {
namespace ml {

struct ContextFeatureExtractionInput {
  const prediction::AvContext* av_context = nullptr;
  const TrafficLightStatesProto* traffic_light_states = nullptr;
  const PlannerObjectManager* object_manager = nullptr;
  std::shared_ptr<const ObjectsProto> real_objects;
  std::shared_ptr<const ObjectsProto> virtual_objects;
  const planner::PlannerSemanticMapManager* psmm = nullptr;
};

double GetCurrentTimeStamp(const prediction::AvContext& av_context,
                           const PlannerObjectManager& object_manager);

double GetCurrentTimeStamp(
    const prediction::AvContext& av_context,
    const std::shared_ptr<const ObjectsProto>& real_objects,
    const std::shared_ptr<const ObjectsProto>& virtual_objects);

InferredFeature ExtractInferredFeature(
    const PlannerObjectManager& object_manager);

prediction::ActNetFeature ExtractActNetContextFeature(
    const double current_ts, const ContextFeatureExtractionInput& input);

ContextFeature ExtractContextFeature(
    const ContextFeatureExtractionInput& input);

}  // namespace ml
}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_CONTEXT_FEATURE_EXTRACTOR_H_
