#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_CAPTAIN_NET_INFERENCE_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_INFERENCE_CAPTAIN_NET_INFERENCE_H_  // NOLINT

#include <memory>  // for unique_ptr
#include <vector>  // for vector

#include "onboard/nets/proto/net_param.pb.h"   // for NetParam
#include "onboard/params/vehicle_param_api.h"  // for RunParamsProtoV2
#include "onboard/planner/ml/captain_net/captain_net.h"  // for CaptainNet, CaptainNetFeature

namespace qcraft::planner::ml::captain_net {

class CaptainNetInference {
 public:
  CaptainNetInference(const RunParamsProtoV2& run_param,
                      const NetParam& net_param);

  bool PlanningTrajectory(
      const CaptainNetFeature& input_features,
      std::vector<std::vector<std::vector<float>>>* prob_out,
      std::vector<std::vector<std::vector<float>>>* traj_out) const;

 private:
  int device_id_ = 0;
  bool use_gpu_;
  const NetParam net_param_;
  std::unique_ptr<CaptainNet> captain_net_;
};

}  // namespace qcraft::planner::ml::captain_net

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_TCAPTAIN_NET_INFERENCE_CAPTAIN_NET_INFERENCE_H_
