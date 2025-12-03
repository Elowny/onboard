#ifndef ONBORAD_PREDICTION_NET_HORIZON_ACT_NET_LOCAL_J5_INFERENCER_H_
#define ONBORAD_PREDICTION_NET_HORIZON_ACT_NET_LOCAL_J5_INFERENCER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/math/vec.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/net/horizon/prediction_j5_qnn.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
namespace actnetlocalj5 {

struct AgentRef {
  std::string agent_id;
  double rot_rad;
  Vec2d ref_position;
};

class ActNetLocalJ5Inferencer {  // Scene centric.
 public:
  explicit ActNetLocalJ5Inferencer(const NetParam& net_param);

  AgentCentricObjectsOut PredictForObjects(
      absl::Span<const ObjectHistory* const> objects_history,
      const ObjectHistorySampler& obj_sampler, MapSampler* map_sampler_ptr,
      ThreadPool* thread_pool) const;

  int GetBatchInputs(absl::Span<const ObjectHistory* const> objects_history,
                     const ObjectHistorySampler& obj_sampler,
                     MapSampler* map_sampler_ptr,
                     std::vector<AgentRef>& agent_origins) const;  // NOLINT

  AgentCentricObjectsOut ExtractOutputs(
      int current_batch_size, const std::vector<AgentRef>& agent_origins,
      ThreadPool* thread_pool) const;

 private:
  const NetParam net_param_;
  std::unique_ptr<PredictionJ5QNN> act_net_local_j5_;
};

}  // namespace actnetlocalj5
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBORAD_PREDICTION_NET_HORIZON_ACT_NET_LOCAL_J5_INFERENCER_H_
