#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_J5_INFERENCER_H_
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_J5_INFERENCER_H_

#include <memory>  // for allocator, unique_ptr
#include <string>  // for string
#include <vector>  // for vector

#include "onboard/nets/proto/net_param.pb.h"             // for NetParam
#include "onboard/planner/ml/captain_net/captain_net.h"  // for CaptainNetFeature
#include "onboard/planner/ml/captain_net_j5/planner_ml_j5_qnn.h"  // for PLANNERMLJ5QNN

namespace qcraft::planner::ml::captain_net {

class CaptainNetJ5Inferencer {
 public:
  explicit CaptainNetJ5Inferencer(const NetParam& net_param);

  bool PlanningTrajectory(
      const CaptainNetFeature& input_features,
      std::vector<std::vector<std::vector<float>>>* prob_out,
      std::vector<std::vector<std::vector<float>>>* traj_out) const;

  // Get batch inputs from sampler.
  int GetBatchInputs(const NetParam& net_param,
                     const CaptainNetFeature& input_features) const;  // NOLINT
  // Extract outputs.
  bool ExtractOutputs(
      int current_batch_size,
      std::vector<std::vector<std::vector<float>>>* prob_out,
      std::vector<std::vector<std::vector<float>>>* traj_out) const;

 private:
  // Warm up to load net parameters into GPU memory.
  void WarmUp() const;

 private:
  const NetParam net_param_;
  std::unique_ptr<PLANNERMLJ5QNN> captain_net_j5_;
  std::string model_context_ = "";
};

}  // namespace qcraft::planner::ml::captain_net

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_J5_INFERENCER_H_