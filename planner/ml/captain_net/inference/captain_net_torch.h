#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_CAPTAIN_NET_TORCH_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_CAPTAIN_NET_TORCH_H_  // NOLINT

#include <memory>  // for unique_ptr
#include <string>  // for string
#include <vector>  // for vector

#include <c10/core/Device.h>  // for Device // NOLINT

#include "torch/csrc/jit/api/module.h"  // for Module
#include "torch/script.h"               // IWYU pragma: keep
#include "torch/torch.h"                // IWYU pragma: keep

#include "onboard/planner/ml/captain_net/captain_net.h"  // for CaptainNet, CaptainNetFeature

namespace qcraft::planner::ml::captain_net {

class CaptainNetTorch : public CaptainNet {
 public:
  explicit CaptainNetTorch(const std::string& model_path, const bool use_gpu,
                           const int gpu_id, const int warmup_times);
  /**
   * Load a new network instance
   * @param model_path File path to the Torch model
   */
  static std::unique_ptr<CaptainNet> Create(const std::string& model_path,
                                            const bool use_gpu,
                                            const int gpu_id,
                                            const int warmup_times);
  bool GetOutputs(
      const CaptainNetFeature& input_features,
      std::vector<std::vector<std::vector<float>>>* prob_out,
      std::vector<std::vector<std::vector<float>>>* traj_out) override;

  void UnitTest() override;

 private:
  /**
   * @brief Load model file and warmup model.
   */
  void InitModel(const std::string& model_path, const int warmup_times);

  void DummyInputInference(const int times);

 private:
  torch::jit::script::Module captain_net_;
  torch::Device device_;
};

}  // namespace qcraft::planner::ml::captain_net

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_CAPTAIN_NET_TORCH_H_
