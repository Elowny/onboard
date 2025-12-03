#ifndef ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_TORCH_H_  // NOLINT
#define ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_TORCH_H_  // NOLINT

#include <memory>
#include <string>
#include <vector>

#include "torch/script.h"  // IWYU pragma: keep
#include "torch/torch.h"   // IWYU pragma: keep

#include "onboard/planner/ml/selector_models/selector_scoring_net.h"

namespace qcraft {
namespace selector_scoring_net {

class SelectorScoringNetTorch : public SelectorScoringNet {
 public:
  explicit SelectorScoringNetTorch(const std::string& model_path,
                                   const bool use_gpu, const int gpu_id,
                                   const int warmup_times);
  /**
   * Load a new network instance
   * @param model_path File path to the Torch model
   */
  static std::unique_ptr<SelectorScoringNet> Create(
      const std::string& model_path, const bool use_gpu, const int gpu_id,
      const int warmup_times);
  bool GetOutputs(const SelectorScoringNetFeature& input_features,
                  std::vector<std::vector<float>>* scores) override;

  void UnitTest() override;

 private:
  /**
   * @brief Load model file and warmup model.
   */
  void InitModel(const std::string& model_path, const int warmup_times);

  void DummyInputInference(const int times);

 private:
  torch::jit::script::Module selector_scoring_net_;
  torch::Device device_;
};

}  // namespace selector_scoring_net
}  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_NETS_TRT_SELECTOR_SCORING_NET_TORCH_H_
