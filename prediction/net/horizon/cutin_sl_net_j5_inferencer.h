#ifndef ONBOARD_NETS_CUTIN_SL_NET_J5_INFERENCER_H_
#define ONBOARD_NETS_CUTIN_SL_NET_J5_INFERENCER_H_

#include <memory>  // for allocator, unique_ptr
#include <vector>

#include "onboard/nets/proto/net_param.pb.h"  // for NetParam
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/net/horizon/prediction_j5_qnn.h"  // for PredictionJ5QNN
#include "onboard/prediction/prediction_defs.h"  // for AgentCentricObjectsOut
namespace qcraft {
namespace prediction {
namespace cutin_sl_net_j5 {

struct AgentValidAndL {
  bool valid;
  double l;
};

class CutinNetJ5Inferencer {
 public:
  explicit CutinNetJ5Inferencer(const NetParam& net_param);

  CutinSLObjectsOut PredictForObjects(
      const std::vector<ObjectIDType>& objs_to_predict_ids,
      const ObjectHistorySampler& obj_sampler, MapSampler* const map_sampler,
      const planner::DrivePassage& drive_passage) const;

  std::vector<AgentValidAndL> GetBatchInputs(
      const std::vector<ObjectIDType>& objs_to_predict_ids,
      const ObjectHistorySampler& obj_sampler, MapSampler* const map_sampler,
      const planner::DrivePassage& drive_passage) const;

 private:
  NetParam net_param_;
  std::unique_ptr<PredictionJ5QNN> cutin_sl_net_j5_;
};

}  // namespace cutin_sl_net_j5
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_NETS_CUTIN_SL_NET_J5_INFERENCER_H_
