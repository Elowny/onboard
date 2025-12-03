#include "onboard/prediction/container/model_pool.h"

#include <string>
#include <unordered_map>

#include "absl/status/statusor.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/run_context.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr char kActNetParamKey[] = "act_net_param";

constexpr char kCutinSLNetParamKey[] = "cutin_sl_net_param";
constexpr char kCutinSLNetJ5ParamKey[] = "cutin_sl_net_j5_param";
constexpr char kActNetJ5ParamKey[] = "act_net_j5_param";
constexpr char kLaneSelectionNetJ5ParamKey[] = "lane_selection_net_j5_param";
constexpr char kActNetLocalJ5ParamKey[] = "act_net_local_j5_param";

absl::StatusOr<NetParam> GetNetParamWithResourceConf(
    const ParamFinder& param_finder,
    const std::unordered_map<
        std::string, LaunchRunConfig::ModuleResourceConfig::NetResourceConfig>&
        net_resource_configs,
    const std::string& key) {
  NetParam net_param;
  RETURN_IF_ERROR(param_finder.GetParam(key, &net_param));
  net_param.set_device_id(GetNetDeviceIdFromNetResourceConfigWithDefault(
      GetNetResourceConfigFromMap(net_resource_configs, key),
      net_param.device_id()));
  return net_param;
}

}  // namespace

ModelPool::ModelPool(
    const ParamManager& /*param_manager*/,  // NOLINT
    const ParamFinder& param_finder,
    const std::unordered_map<
        std::string, LaunchRunConfig::ModuleResourceConfig::NetResourceConfig>&
        net_resource_configs) {
#if defined(__J5__)
  bool is_j5 = true;
#else
  bool is_j5 = false;
#endif
  if (!IsRunModeL4() || is_j5) {
    if (FLAGS_prediction_enable_act_net_j5) {
      ASSIGN_OR_DIE(NetParam act_net_j5_param,
                    GetNetParamWithResourceConf(
                        param_finder, net_resource_configs, kActNetJ5ParamKey));
      act_net_j5_inferencer_ =
          std::make_unique<actnetj5::ActNetJ5Inferencer>(act_net_j5_param);
    }
    if (FLAGS_prediction_replace_act_net_j5_with_local) {
      ASSIGN_OR_DIE(
          NetParam act_net_local_j5_param,
          GetNetParamWithResourceConf(param_finder, net_resource_configs,
                                      kActNetLocalJ5ParamKey));
      act_net_local_j5_inferencer_ =
          std::make_unique<actnetlocalj5::ActNetLocalJ5Inferencer>(
              act_net_local_j5_param);
    }
    if (FLAGS_prediction_enable_auxiliary_cutin_sl_net_j5) {
      ASSIGN_OR_DIE(
          NetParam cutin_sl_net_j5_param,
          GetNetParamWithResourceConf(param_finder, net_resource_configs,
                                      kCutinSLNetJ5ParamKey));
      cutin_sl_net_j5_inferencer_ =
          std::make_unique<cutin_sl_net_j5::CutinNetJ5Inferencer>(
              cutin_sl_net_j5_param);
    }
    if (FLAGS_prediction_enable_lane_selection_net_j5) {
      ASSIGN_OR_DIE(
          NetParam lane_selection_net_j5_param,
          GetNetParamWithResourceConf(param_finder, net_resource_configs,
                                      kLaneSelectionNetJ5ParamKey));
      lane_selection_net_j5_inferencer_ =
          std::make_unique<lane_selection_net::LaneSelectionNetJ5Inferencer>(
              lane_selection_net_j5_param);
    }
    return;
  } else {
    if (FLAGS_prediction_enable_act_net) {
      ASSIGN_OR_DIE(NetParam act_net_param,
                    GetNetParamWithResourceConf(
                        param_finder, net_resource_configs, kActNetParamKey));
      act_net_inferencer_ =
          std::make_unique<actnet::ActNetInferencer>(act_net_param);
    }

    if (FLAGS_prediction_enable_auxiliary_cutin_sl_net) {
      ASSIGN_OR_DIE(
          NetParam cutin_sl_net_param,
          GetNetParamWithResourceConf(param_finder, net_resource_configs,
                                      kCutinSLNetParamKey));
      cutin_sl_net_inferencer_ =
          std::make_unique<cutin_sl_net::CutinSLNetInferencer>(
              cutin_sl_net_param);
    }
    return;
  }
}

}  // namespace prediction
}  // namespace qcraft
