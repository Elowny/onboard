#ifndef ONBOARD_PREDICTION_POST_PROCESS_POST_PROCESS_DEFS_H_
#define ONBOARD_PREDICTION_POST_PROCESS_POST_PROCESS_DEFS_H_

#include <vector>

#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace prediction {
struct ObjectPredictionPostProcess {
  const ObjectProto* object_proto = nullptr;
  std::vector<PredictedTrajectory*> ptrs_mutable_trajs;
};
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_POST_PROCESS_POST_PROCESS_DEFS_H_
