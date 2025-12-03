#ifndef ONBOARD_PREDICTION_INFERENCER_LANE_SELECTION_NET_INFERENCER_H_
#define ONBOARD_PREDICTION_INFERENCER_LANE_SELECTION_NET_INFERENCER_H_

#include <memory>  // for unique_ptr
#include <vector>

#include "onboard/nets/proto/net_param.pb.h"  // for NetParam
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/net/horizon/prediction_j5_qnn.h"  // for PredictionJ5QNN
#include "onboard/prediction/prediction_defs.h"  // for LaneSelectionObjectsOut

namespace qcraft {
namespace prediction {
namespace lane_selection_net {

struct AgentDpsIndexInfo {
  ObjectIDType agent_id;
  int dp_idx;
  bool is_valid = true;
};

class LaneSelectionNetJ5Inferencer {
 public:
  explicit LaneSelectionNetJ5Inferencer(const NetParam& net_param);

  LaneSelectionInferencerOutputMap PredictForObjects(
      const prediction::ObjectHistorySampler& obj_sampler,
      const AgentDrivePassagesMap& agent_dps_map) const;

  std::vector<AgentDpsIndexInfo> GenBatchInputs(
      const AgentDrivePassagesMap& agent_dps_map,
      const ObjectHistorySampler& obj_sampler) const;
  // Extract outputs.
  LaneSelectionInferencerOutputMap ExtractOutputs(
      const std::vector<AgentDpsIndexInfo>& batch_idx_info) const;

 private:
  NetParam net_param_;
  std::unique_ptr<prediction::PredictionJ5QNN> lane_selection_net_j5_;
};

}  // namespace lane_selection_net
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_INFERENCER_LANE_SELECTION_NET_INFERENCER_H_
