#include "onboard/planner/ml/selector_models/selector_scoring_net_onnx.h"

#include <string>

namespace qcraft {
namespace selector_scoring_net {

// Create
std::unique_ptr<SelectorScoringNet> SelectorScoringNetCPU::Create(
    const std::string& model_path,
    const std::vector<std::vector<int>>& input_dims,
    const std::vector<std::vector<int>>& output_dims) {
  return std::unique_ptr<SelectorScoringNet>(
      new SelectorScoringNetCPU(model_path, input_dims, output_dims));
}

void SelectorScoringNetCPU::UnitTest() {}

bool SelectorScoringNetCPU::GetOutputs(
    const SelectorScoringNetFeature& /*input_features*/,
    std::vector<std::vector<float>>* /*scores*/) {
  return false;
}

}  // namespace selector_scoring_net
}  // namespace qcraft
