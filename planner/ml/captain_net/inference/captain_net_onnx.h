#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_CAPTAIN_NET_ONNX_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_CAPTAIN_NET_ONNX_H_  // NOLINT

#include <memory>
#include <string>
#include <vector>

#include "onboard/nets/trt/onnxruntime_net.h"
#include "onboard/planner/ml/captain_net/captain_net.h"

namespace qcraft::planner::ml::captain_net {

class CaptainNetCPU : public OnnxNet, public CaptainNet {
 public:
  static std::unique_ptr<CaptainNet> Create(
      const std::string& model_path,
      const std::vector<std::vector<int>>& input_dims,
      const std::vector<std::vector<int>>& output_dims);

  bool GetOutputs(
      const CaptainNetFeature& input_features,
      std::vector<std::vector<std::vector<float>>>* prob_out,
      std::vector<std::vector<std::vector<float>>>* traj_out) override;

  void UnitTest() override;

 private:
  CaptainNetCPU(const std::string& model_path,
                const std::vector<std::vector<int>>& input_dims,
                const std::vector<std::vector<int>>& output_dims)
      : OnnxNet(model_path, input_dims, output_dims) {}
};

}  // namespace qcraft::planner::ml::captain_net

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_CAPTAIN_NET_ONNX_H_
