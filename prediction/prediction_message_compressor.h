#ifndef ONBOARD_PREDICTION_PREDICTION_MESSAGE_COMPRESSOR_H_
#define ONBOARD_PREDICTION_PREDICTION_MESSAGE_COMPRESSOR_H_

#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
PredictedTrajectoryPointProto LerpPredictedTrajectoryPointProto(
    const PredictedTrajectoryPointProto& a,
    const PredictedTrajectoryPointProto& b, double alpha);

class PredictedTrajectoryPointProtoLerper {
 public:
  PredictedTrajectoryPointProto operator()(
      const PredictedTrajectoryPointProto& a,
      const PredictedTrajectoryPointProto& b, double alpha) const {
    return LerpPredictedTrajectoryPointProto(a, b, alpha);
  }
};

void CompressPredictedTrajectoryProtoByDownSampling(
    PredictedTrajectoryProto* traj);
void DecompressPredictedTrajectoryProto(PredictedTrajectoryProto* traj);
void DecompressObjectsPredictionProto(ObjectsPredictionProto* proto);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTION_MESSAGE_COMPRESSOR_H_
