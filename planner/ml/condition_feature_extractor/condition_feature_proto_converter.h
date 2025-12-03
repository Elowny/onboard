#ifndef ONBOARD_PLANNER_ML_CONDITION_FEATURE_EXTRACTOR_CONDITION_FEATURE_PROTO_CONVERTER_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CONDITION_FEATURE_EXTRACTOR_CONDITION_FEATURE_PROTO_CONVERTER_H_  // NOLINT

#include "onboard/planner/ml/condition_feature_extractor/condition_feature.h"
#include "onboard/planner/ml/condition_feature_extractor/proto/condition_feature.pb.h"

namespace qcraft {
namespace planner {
namespace ml {

TrajectoryFeatureProto TrajectoryFeatureToProto(const TrajectoryFeature& feat);

LineSegmentsFeatureProto LineSegmentsFeatureToProto(
    const LineSegmentsFeature& feat);

PathBoundaryFeatureProto PathBoundaryFeatureToProto(
    const PathBoundaryFeature& feat);

}  // namespace ml
}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CONDITION_FEATURE_EXTRACTOR_CONDITION_FEATURE_PROTO_CONVERTER_H_
