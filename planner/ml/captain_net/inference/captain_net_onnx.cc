#include "onboard/planner/ml/captain_net/inference/captain_net_onnx.h"

#include <string>

namespace qcraft::planner::ml::captain_net {

// Create
std::unique_ptr<CaptainNet> CaptainNetCPU::Create(
    const std::string& model_path,
    const std::vector<std::vector<int>>& input_dims,
    const std::vector<std::vector<int>>& output_dims) {
  return std::unique_ptr<CaptainNet>(
      new CaptainNetCPU(model_path, input_dims, output_dims));
}

void CaptainNetCPU::UnitTest() {}

bool CaptainNetCPU::GetOutputs(
    const CaptainNetFeature& /*input_features*/,
    std::vector<std::vector<std::vector<float>>>* /*prob_out*/,
    std::vector<std::vector<std::vector<float>>>* /*traj_out*/) {
  return false;
}

}  // namespace qcraft::planner::ml::captain_net
