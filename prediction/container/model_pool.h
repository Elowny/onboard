#ifndef ONBOARD_PREDICTION_CONTAINER_MODEL_POOL_H_
#define ONBOARD_PREDICTION_CONTAINER_MODEL_POOL_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/params/param_manager.h"
#include "onboard/prediction/inferencer/act_net_inferencer.h"
#include "onboard/prediction/inferencer/cutin_sl_net_inferencer.h"
#include "onboard/prediction/net/horizon/act_net_j5_inferencer.h"
#include "onboard/prediction/net/horizon/act_net_local_j5_inferencer.h"
#include "onboard/prediction/net/horizon/cutin_sl_net_j5_inferencer.h"
#include "onboard/prediction/net/horizon/lane_selection_net_j5_inferencer.h"

namespace qcraft {
namespace prediction {
class ModelPool {
 public:
  explicit ModelPool(
      const ParamManager& param_manager, const ParamFinder& param_finder,
      const std::unordered_map<
          std::string,
          LaunchRunConfig::ModuleResourceConfig::NetResourceConfig>&
          net_resource_configs);

  const actnet::ActNetInferencer* GetActNetInferencer() const {
    return act_net_inferencer_.get();
  }

  const cutin_sl_net::CutinSLNetInferencer* GetCutinSLNetInferencer() const {
    return cutin_sl_net_inferencer_.get();
  }
  const cutin_sl_net_j5::CutinNetJ5Inferencer* GetCutinSLNetJ5Inferencer()
      const {
    return cutin_sl_net_j5_inferencer_.get();
  }
  const actnetj5::ActNetJ5Inferencer* GetActNetJ5Inferencer() const {
    return act_net_j5_inferencer_.get();
  }
  const lane_selection_net::LaneSelectionNetJ5Inferencer*
  GetLaneSelectionNetJ5Inferencer() const {
    return lane_selection_net_j5_inferencer_.get();
  }

  const actnetlocalj5::ActNetLocalJ5Inferencer* GetActNetLocalJ5Inferencer()
      const {
    return act_net_local_j5_inferencer_.get();
  }

 private:
  std::unique_ptr<actnet::ActNetInferencer> act_net_inferencer_;
  std::unique_ptr<cutin_sl_net::CutinSLNetInferencer> cutin_sl_net_inferencer_;
  std::unique_ptr<cutin_sl_net_j5::CutinNetJ5Inferencer>
      cutin_sl_net_j5_inferencer_;
  std::unique_ptr<actnetj5::ActNetJ5Inferencer> act_net_j5_inferencer_;
  std::unique_ptr<lane_selection_net::LaneSelectionNetJ5Inferencer>
      lane_selection_net_j5_inferencer_;
  std::unique_ptr<actnetlocalj5::ActNetLocalJ5Inferencer>
      act_net_local_j5_inferencer_;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_MODEL_POOL_H_
