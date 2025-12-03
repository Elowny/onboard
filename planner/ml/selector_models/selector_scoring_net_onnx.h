#ifndef ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_ONNX_H_  // NOLINT
#define ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_ONNX_H_  // NOLINT

#include <memory>
#include <string>
#include <vector>

#include "onboard/nets/trt/onnxruntime_net.h"
#include "onboard/planner/ml/selector_models/selector_scoring_net.h"

namespace qcraft {
namespace selector_scoring_net {

class SelectorScoringNetCPU : public OnnxNet, public SelectorScoringNet {
 public:
  static std::unique_ptr<SelectorScoringNet> Create(
      const std::string& model_path,
      const std::vector<std::vector<int>>& input_dims,
      const std::vector<std::vector<int>>& output_dims);

  bool GetOutputs(const SelectorScoringNetFeature& input_features,
                  std::vector<std::vector<float>>* scores) override;

  void UnitTest() override;

 private:
  SelectorScoringNetCPU(const std::string& model_path,
                        const std::vector<std::vector<int>>& input_dims,
                        const std::vector<std::vector<int>>& output_dims)
      : OnnxNet(model_path, input_dims, output_dims) {}
};

}  // namespace selector_scoring_net
}  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_ONNX_H_
