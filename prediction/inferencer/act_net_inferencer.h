#ifndef ONBOARD_PREDICTION_INFERENCER_ACT_NET_INFERENCER_H_
#define ONBOARD_PREDICTION_INFERENCER_ACT_NET_INFERENCER_H_

#include <memory>
#include <string>

#include "absl/types/span.h"  // for Span

#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/inferencer/prediction_qnn.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace actnet {
inline constexpr int kOutCoords = 5;
inline constexpr int kTrajectoryNum = 3;
inline constexpr int kObjectTypeDim = 16;
inline constexpr int kLaneTypeDim = 16;
inline constexpr int kLaneLightDim = 4;

inline constexpr int kSegDim = 4;
inline constexpr int kLaneSegsNum = 5;
inline constexpr int kCoords = prediction::kActNetConfig.coord_num;
inline constexpr int kMaxOtherObjsNum =
    prediction::kActNetConfig.max_other_objects_num;
inline constexpr int kMaxLaneCenterNum = prediction::kActNetConfig.max_lc_num;
inline constexpr int kMaxLaneBoundaryNum =
    prediction::kActNetConfig.max_solid_lb_num;
inline constexpr int kMaxCrossWalkNum =
    prediction::kActNetConfig.max_crosswalk_num;
inline constexpr int kHistoryNum = 10;  // to confirm
inline constexpr int kFutureNum = prediction::kActNetConfig.future_num;
inline constexpr int kStopInfoDim = 3;

class ActNetInferencer {
 public:
  explicit ActNetInferencer(const NetParam& net_param);

  prediction::AgentCentricObjectsOut PredictForObjects(
      absl::Span<const prediction::ActNetFeature> input_features) const;

 private:
  const NetParam net_param_;
  std::unique_ptr<prediction::PredictionQNN> act_net_;
  std::string model_context_ = "";
};

}  // namespace actnet
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_INFERENCER_ACT_NET_INFERENCER_H_
