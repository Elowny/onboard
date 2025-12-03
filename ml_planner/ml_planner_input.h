#ifndef ONBOARD_ML_PLANNER_ML_PLANNER_INPUT_H_
#define ONBOARD_ML_PLANNER_ML_PLANNER_INPUT_H_

#include <memory>
#include <string>

#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace mlplanner {

// The input data of ml planner module. The struct should be read-only after
// construction.
struct MLPlannerInput {
  std::shared_ptr<planner::PlannerSemanticMapManager>
      planner_semantic_map_manager;

  VehicleParamApi vehicle_params;

  std::shared_ptr<const PoseProto> pose;

  // Optional in onboard as planner computes prediction inside planner module.
  // Required in offboard case as planner may not able to reproduce the same
  // prediction result.
  std::unique_ptr<const ObjectsPredictionProto> prediction;

  // Load params for prediction post process.
  prediction::ConflictResolverParams prediction_conflict_resolver_params;

  // Three ObjectsProto:scope different objects
  std::shared_ptr<const ObjectsProto> real_objects;
  std::shared_ptr<const ObjectsProto> virtual_objects;
  std::shared_ptr<const ObjectsProto> av_objects;

  std::shared_ptr<const LocalizationTransformProto> localization_transform;
  std::shared_ptr<const AutonomyStateProto> autonomy_state;
  std::shared_ptr<const TrafficLightStatesProto> traffic_light_states;

  // ML Planner Models
  std::unique_ptr<planner::ModelPool> model_pool;

  // Planner main thread pool.
  std::unique_ptr<ThreadPool> thread_pool;

  // cross-iteration
  // TODO(Jinyun): Define another avcontext class with less unnecessary
  // operations.
  std::unique_ptr<prediction::AvContext> av_context;
};

}  // namespace mlplanner
}  // namespace qcraft

#endif  // ONBOARD_ML_PLANNER_ML_PLANNER_INPUT_H_
