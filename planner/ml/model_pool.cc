#include "onboard/planner/ml/model_pool.h"

#include "glog/logging.h"

#include "onboard/global/run_context.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/utils/errors.h"

namespace qcraft {
namespace planner {

namespace {
constexpr char kSelectorScoringNetParamKey[] = "selector_scoring_net_param";
constexpr char kCaptainNetParamKey[] = "captain_net_param";
constexpr char kCaptainNetOTParamKey[] = "captain_net_onnx_trt_param";
constexpr char kCaptainNetJ5ParamKey[] = "captain_net_j5_param";
constexpr char kActNetSpeedParamKey[] = "act_net_speed_param";
}  // namespace

ModelPool::ModelPool(const ParamManager& param_manager,
                     const ParamFinder& param_finder) {
  RunParamsProtoV2 run_params;
  param_manager.GetRunParams(&run_params);
  if (FLAGS_planner_enable_captain_net_j5) {
    NetParam captain_net_j5_param;
    CHECK_OK(
        param_finder.GetParam(kCaptainNetJ5ParamKey, &captain_net_j5_param));
    captain_net_j5_inferencer_ =
        std::make_unique<ml::captain_net::CaptainNetJ5Inferencer>(
            captain_net_j5_param);
  } else {
    if (IsRunModeL4() && FLAGS_planner_enable_selector_scoring_net) {
      NetParam selector_scoring_net_param;
      CHECK_OK(param_finder.GetParam(kSelectorScoringNetParamKey,
                                     &selector_scoring_net_param));
      selector_scoring_net_inference_ =
          std::make_unique<SelectorScoringNetInference>(
              run_params, selector_scoring_net_param);
    }

    if (IsRunModeL4() && FLAGS_planner_enable_captain_net) {
      NetParam captain_net_param;
      CHECK_OK(param_finder.GetParam(kCaptainNetParamKey, &captain_net_param));
      captain_net_inference_ =
          std::make_unique<ml::captain_net::CaptainNetInference>(
              run_params, captain_net_param);
    }

    if (IsRunModeL4() && FLAGS_planner_enable_captain_net_onnx_trt) {
      NetParam captain_net_ot_param;
      CHECK_OK(
          param_finder.GetParam(kCaptainNetOTParamKey, &captain_net_ot_param));
      captain_net_inferencer_ =
          std::make_unique<ml::captain_net::CaptainNetInferencer>(
              captain_net_ot_param);
    }

    if (IsRunModeL4() && FLAGS_planner_enable_act_net_speed) {
      NetParam act_net_speed_param;
      CHECK_OK(
          param_finder.GetParam(kActNetSpeedParamKey, &act_net_speed_param));
      act_net_speed_inferencer_ =
          std::make_unique<actnetspeed::ActNetSpeedInferencer>(
              act_net_speed_param);
    }
  }
}

}  // namespace planner
}  // namespace qcraft
