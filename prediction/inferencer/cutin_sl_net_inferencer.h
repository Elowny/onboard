#ifndef ONBOARD_PREDICTION_INFERENCER_CUTIN_SL_NET_INFERENCER_H_
#define ONBOARD_PREDICTION_INFERENCER_CUTIN_SL_NET_INFERENCER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/prediction/feature_extractor/cutin_sl_feature.h"
#include "onboard/prediction/inferencer/prediction_qnn.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace cutin_sl_net {
inline constexpr int kObjectTypeDim = 16;
inline constexpr int kLaneTypeDim = 16;
inline constexpr int kLaneLightDim = 4;
inline constexpr int kShapetDim = 4;

inline constexpr int kSegDim = 4;
inline constexpr int kLaneSegsNum = 5;
inline constexpr int kCoords = prediction::kCutinSLConfig.coord_num;
inline constexpr int kMaxOtherObjsNum = 8;
inline constexpr int kMaxLaneCenterNum = prediction::kCutinSLConfig.max_lc_num;
inline constexpr int kMaxLaneBoundaryNum =
    prediction::kCutinSLConfig.max_lb_num;
inline constexpr int kMaxCrossWalkNum =
    prediction::kCutinSLConfig.max_crosswalk_num;
inline constexpr int kHistoryNum = 10;

inline constexpr double kLengthScale =
    1.0 / prediction::kCutinSLConfig.length_scale;
inline constexpr double kWidthScale =
    1.0 / prediction::kCutinSLConfig.width_scale;

class CutinSLNetInferencer {
 public:
  explicit CutinSLNetInferencer(const NetParam& net_param);

  prediction::CutinSLObjectsOut PredictForObjects(
      absl::Span<const prediction::CutinSLFeature> input_features) const;
  std::vector<std::vector<float>> GetBatchInputs(
      const NetParam& net_param,
      absl::Span<const prediction::CutinSLFeature> input_features) const;

 private:
  const NetParam net_param_;
  std::unique_ptr<prediction::PredictionQNN> cutin_sl_net_;
  std::string model_context_ = "";
};

}  // namespace cutin_sl_net
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_INFERENCER_CUTIN_SL_NET_INFERENCER_H_
