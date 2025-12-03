#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_INPUT_ASSEMBLE_H_
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_INPUT_ASSEMBLE_H_

#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/ml/condition_feature_extractor/condition_feature.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature.h"

namespace qcraft::planner::ml {

captain_net::CaptainNetFeature AssembleInputFeature(
    const ContextFeature& context_feature,
    const MultiLanePathFeature& condition_feature);

}  // namespace qcraft::planner::ml
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_INPUT_ASSEMBLE_H_
