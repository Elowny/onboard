#ifndef ONBOARD_NETS_TRT_CAPTAIN_NET_ONNX_H_
#define ONBOARD_NETS_TRT_CAPTAIN_NET_ONNX_H_

#include <memory>  // for allocator, unique_ptr
#include <string>  // for string
#include <vector>  // for vector

#include "onboard/nets/proto/net_param.pb.h"             // for NetParam
#include "onboard/planner/ml/captain_net/captain_net.h"  // for CaptainNetFeature
#include "onboard/planner/ml/planner_ml_qnn.h"           // for PlannerMLQNN

namespace qcraft::planner::ml::captain_net {

class CaptainNetInferencer {
 public:
  explicit CaptainNetInferencer(const NetParam& net_param);

  bool PlanningTrajectory(
      const CaptainNetFeature& input_features,
      std::vector<std::vector<std::vector<float>>>* prob_out,
      std::vector<std::vector<std::vector<float>>>* traj_out) const;

 private:
  // Warm up to load net parameters into GPU memory.
  void WarmUp() const;

 private:
  const NetParam net_param_;
  std::unique_ptr<PlannerMLQNN> captain_net_;
  std::string model_context_ = "";
};

}  // namespace qcraft::planner::ml::captain_net

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_NETS_TRT_CAPTAIN_NET_ONNX_H_
