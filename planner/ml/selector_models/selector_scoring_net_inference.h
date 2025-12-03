#ifndef ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_INFERENCE_H_  // NOLINT
#define ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_INFERENCE_H_  // NOLINT

#include <memory>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_manager.h"     // IWYU pragma: keep
#include "onboard/params/utils/param_util.h"  // IWYU pragma: keep
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/ml/selector_models/selector_scoring_net.h"

namespace qcraft {

class SelectorScoringNetInference {
 public:
  SelectorScoringNetInference(const RunParamsProtoV2& run_param,
                              const NetParam& net_param);

  absl::StatusOr<std::vector<float>> EvaluateScores(
      const selector_scoring_net::SelectorScoringNetFeature& input_features)
      const;

 private:
  int device_id_ = 0;
  bool use_gpu_;
  const NetParam net_param_;
  std::unique_ptr<selector_scoring_net::SelectorScoringNet>
      selector_scoring_net_;
};

}  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_INFERENCE_H_
