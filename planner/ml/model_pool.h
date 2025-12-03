#ifndef ONBOARD_PLANNER_MODEL_MODEL_POOL_H_
#define ONBOARD_PLANNER_MODEL_MODEL_POOL_H_

#include <memory>

#include "onboard/params/param_finder.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/ml/captain_net/inference/captain_net_inference.h"
#include "onboard/planner/ml/captain_net/inference/captain_net_inferencer.h"
#include "onboard/planner/ml/captain_net/inference/captain_net_j5_inferencer.h"
#include "onboard/planner/ml/selector_models/selector_scoring_net_inference.h"
#include "onboard/prediction/inferencer/act_net_speed_inferencer.h"

namespace qcraft {
namespace planner {
class ModelPool {
 public:
  ModelPool(const ParamManager& param_manager, const ParamFinder& param_finder);

  const SelectorScoringNetInference& GetSelectorScoringNetInference() const {
    return *selector_scoring_net_inference_.get();
  }

  const ml::captain_net::CaptainNetInference& GetCaptainNetInference() const {
    return *captain_net_inference_.get();
  }

  const ml::captain_net::CaptainNetInferencer& GetCaptainNetInferencer() const {
    return *captain_net_inferencer_.get();
  }

  const ml::captain_net::CaptainNetJ5Inferencer& GetCaptainNetJ5Inferencer()
      const {
    return *captain_net_j5_inferencer_.get();
  }

  const actnetspeed::ActNetSpeedInferencer* GetActNetSpeedInferencer() const {
    return act_net_speed_inferencer_.get();
  }

 private:
  std::unique_ptr<SelectorScoringNetInference> selector_scoring_net_inference_;
  std::unique_ptr<ml::captain_net::CaptainNetInference> captain_net_inference_;
  std::unique_ptr<ml::captain_net::CaptainNetInferencer>
      captain_net_inferencer_;
  std::unique_ptr<ml::captain_net::CaptainNetJ5Inferencer>
      captain_net_j5_inferencer_;
  std::unique_ptr<actnetspeed::ActNetSpeedInferencer> act_net_speed_inferencer_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_MODEL_MODEL_POOL_H_
