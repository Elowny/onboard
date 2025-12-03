#include <algorithm>
#include <vector>

#include "benchmark/benchmark.h"
#include "glog/logging.h"

#include "onboard/math/vec.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/inferencer/act_net_inferencer.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/utils/errors.h"

namespace qcraft {
namespace actnet {

static void RunActNetInferenceBM(benchmark::State& state,  // NOLINT
                                 const bool use_gpu) {
  NetParam act_net_param;
  CHECK_OK(GetProtoParamById("Q0001", "act_net_param", &act_net_param));
  if (!use_gpu) {
    act_net_param.set_device_type(NetParam_DeviceType_CPU);
  }
  ActNetInferencer act_net_inferencer(act_net_param);
  const int max_bs = act_net_param.max_batch_size();
  const int history_len = prediction::kFeatureV2HistoryStepNum;
  const auto abs_feat = prediction::ActNetObjectAbsoluteFeature{
      .pos = std::vector<float>(history_len * kCoords),
      .pos_diff = std::vector<float>(history_len * kCoords),
      .speed = std::vector<float>(history_len * kCoords),
      .yaw = std::vector<float>(history_len * kCoords),
      .shape = std::vector<float>(history_len * kCoords * 4),
      .ts = std::vector<float>(history_len),
      .mask = std::vector<float>(history_len),
      .type = 2.0,
      .lw = std::vector<float>(2),
  };
  const auto rel_feat = prediction::ActNetObjectRelFeature{
      .rel_pos = std::vector<float>(history_len * kCoords),
      .rel_dist = std::vector<float>(history_len),
      .rel_yaw = std::vector<float>(history_len * kCoords),
      .yaw_diff = std::vector<float>(history_len * kCoords),
      .rel_speed = std::vector<float>(history_len * kCoords),
      .rel_shape = std::vector<float>(history_len * kCoords * 4),
      .rel_mask = std::vector<float>(history_len),
  };
  const auto object_feature = prediction::ActNetObjectFeature{
      .id = "", .abs_feat = abs_feat, .rel_feat = rel_feat};
  const auto lane_feature = prediction::ActNetPolylineFeature{
      .seg = std::vector<float>(kLaneSegsNum * kSegDim),
      .unit_seg_vec = std::vector<float>(kLaneSegsNum * kCoords),
      .seg_len = std::vector<float>(kLaneSegsNum),
      .unit_tangent = std::vector<float>(kLaneSegsNum * kCoords),
      .dist = std::vector<float>(kLaneSegsNum),
      .unit_dist_vec = std::vector<float>(kLaneSegsNum * kCoords),
      .nearest_to_end_dist = std::vector<float>(kLaneSegsNum),
      .yaw_diff = std::vector<float>(kLaneSegsNum * kCoords),
      .type = std::vector<float>(kLaneSegsNum),
      .light = std::vector<float>(kLaneSegsNum, kLaneLightDim),
      .mask = std::vector<float>(kLaneSegsNum),
      .speed_limit_kph = std::vector<float>(kLaneSegsNum),
  };
  const auto input_feature = prediction::ActNetFeature{
      .agent_id = "001",
      .ref_position = Vec2d(),
      .rot_rad = 60.0,
      .stop_time_info = std::vector<float>(kStopInfoDim),
      .agent_feature = abs_feat,
      .context_obj_features = std::vector<prediction::ActNetObjectFeature>(
          kMaxOtherObjsNum, object_feature),
      .context_obj_masks = std::vector<float>(kMaxOtherObjsNum),
      .lc_features = std::vector<prediction::ActNetPolylineFeature>(
          kMaxLaneCenterNum, lane_feature),
      .solid_lb_features = std::vector<prediction::ActNetPolylineFeature>(
          kMaxLaneBoundaryNum, lane_feature),
      .cw_features = std::vector<prediction::ActNetPolylineFeature>(
          kMaxCrossWalkNum, lane_feature),
      .lc_masks = std::vector<float>(kMaxLaneCenterNum),
      .solid_lb_masks = std::vector<float>(kMaxLaneBoundaryNum),
      .cw_masks = std::vector<float>(kMaxCrossWalkNum),
  };
  std::vector<prediction::ActNetFeature> input_features;
  input_features.reserve(max_bs);

  for (int i = 0; i < max_bs; ++i) {
    input_features.push_back(input_feature);
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        act_net_inferencer.PredictForObjects(input_features));
  }
}

static void BM_ActNetInferenceGPU(benchmark::State& state) {  // NOLINT
  RunActNetInferenceBM(state, /*use_gpu=*/true);
}
static void BM_ActNetInferenceCPU(benchmark::State& state) {  // NOLINT
  RunActNetInferenceBM(state, /*use_gpu=*/false);
}

BENCHMARK(BM_ActNetInferenceGPU);
BENCHMARK(BM_ActNetInferenceCPU);
}  // namespace actnet
}  // namespace qcraft

BENCHMARK_MAIN();
