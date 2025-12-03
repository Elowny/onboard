#ifndef ONBOARD_ML_PLANNER_MULTI_CONDITIONS_ML_CRUISE_PLANNER_INPUT_H_
#define ONBOARD_ML_PLANNER_MULTI_CONDITIONS_ML_CRUISE_PLANNER_INPUT_H_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "onboard/ml_planner/ml_planner_input.h"
// #include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature.h"
// #include "onboard/planner/ml/model_pool.h"
// #include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft {
namespace mlplanner {

struct MultiConditionsMLCruisePlannerInput {
  const MLPlannerInput& ml_planner_input;

  std::shared_ptr<const planner::ml::ContextFeature> context_feature = nullptr;

  // const planner::PlanStartPointInfo* start_point_info = nullptr;

  std::vector<mapping::LanePath>* lane_paths;

  // std::shared_ptr<planner::PlannerSemanticMapManager>
  // planner_semantic_map_manager = nullptr;

  // const VehicleParamApi* vehicle_params = nullptr;

  // const planner::ModelPool* model_pool = nullptr;
};

}  // namespace mlplanner
}  // namespace qcraft

#endif  // ONBOARD_ML_PLANNER_MULTI_CONDITIONS_ML_CRUISE_PLANNER_INPUT_H_
