#ifndef ONBOARD_NETS_ACT_NET_J5_INFERENCER_H_
#define ONBOARD_NETS_ACT_NET_J5_INFERENCER_H_

#include <memory>  // for allocator, unique_ptr
#include <string>  // for string
#include <vector>  // for vector

#include "absl/types/span.h"  // for Span

#include "onboard/async/thread_pool.h"                    // for ThreadPool
#include "onboard/math/vec.h"                             // for Vec2d
#include "onboard/nets/proto/net_param.pb.h"              // for NetParam
#include "onboard/prediction/container/object_history.h"  // for ObjectHistory
#include "onboard/prediction/feature_extractor/map_sampler.h"  // for MapSampler
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/net/horizon/prediction_j5_qnn.h"  // for PredictionJ5QNN
#include "onboard/prediction/prediction_defs.h"  // for AgentCentricObjectsOut
namespace qcraft {
namespace prediction {
namespace actnetj5 {
struct AgentRef {
  std::string agent_id;
  double rot_rad;
  Vec2d ref_position;
};

class ActNetJ5Inferencer {
 public:
  explicit ActNetJ5Inferencer(const NetParam& net_param);

  AgentCentricObjectsOut PredictForObjects(
      absl::Span<const ObjectHistory* const> objects_history,
      const ObjectHistorySampler& obj_sampler, MapSampler* map_sampler_ptr,
      ThreadPool* thread_pool) const;

  // Get batch inputs from sampler.
  int GenBatchInputs(absl::Span<const ObjectHistory* const> objects_history,
                     const ObjectHistorySampler& obj_sampler,
                     MapSampler* map_sampler_ptr,
                     std::vector<AgentRef>& agent_origins) const;  // NOLINT
  // Extract outputs.
  AgentCentricObjectsOut ExtractOutputs(
      int current_batch_size, const std::vector<AgentRef>& agent_origins,
      ThreadPool* thread_pool) const;

  // Get J5 QNN to check internal state accuracy.
  PredictionJ5QNN* GetJ5QNN() const { return act_net_j5_.get(); }

 private:
  const NetParam net_param_;
  std::unique_ptr<PredictionJ5QNN> act_net_j5_;
};

}  // namespace actnetj5
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_NETS_ACT_NET_J5_INFERENCER_H_
