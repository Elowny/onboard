#ifndef ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_CONTEXT_FEATURE_PROTO_CONVERTER_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_CONTEXT_FEATURE_PROTO_CONVERTER_H_  // NOLINT

#include "onboard/planner/ml/context_feature_extractor/context_feature.h"
#include "onboard/planner/ml/context_feature_extractor/proto/context_feature.pb.h"

namespace qcraft {
namespace planner {
namespace ml {

ContextFeatureProto ContextFeatureToProto(const ContextFeature& feat);

}  // namespace ml
}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_CONTEXT_FEATURE_PROTO_CONVERTER_H_
