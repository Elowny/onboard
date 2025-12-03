#ifndef ONBOARD_PREDICTION_TEST_UTIL_PREDICTION_CONTEXT_BUILDER_H_
#define ONBOARD_PREDICTION_TEST_UTIL_PREDICTION_CONTEXT_BUILDER_H_

#include "absl/strings/string_view.h"  // for string_view

#include "onboard/planner/planner_semantic_map_manager.h"  // for SemanticMapManager
#include "onboard/prediction/container/prediction_context.h"  // for PredictionContext
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"  // for ConflictResolverParams
#include "onboard/proto/vehicle.pb.h"  // for VehicleGeometryParamsProto

namespace qcraft {
namespace prediction {
PredictionInput BuildPredictionInput(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const VehicleGeometryParamsProto& veh_params,
    const ConflictResolverParams& conf_params);

PredictionContext BuildOneObjectPredictionContext(
    absl::string_view id, PredictionInput* prediction_input);
PredictionContext BuildMultiObjectsPredictionContext(
    const int obj_num, PredictionInput* prediction_input);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_TEST_UTIL_PREDICTION_CONTEXT_BUILDER_H_
