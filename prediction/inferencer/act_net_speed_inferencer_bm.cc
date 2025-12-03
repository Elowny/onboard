#include <vector>

#include "benchmark/benchmark.h"
#include "glog/logging.h"

#include "onboard/math/vec.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/feature_extractor/act_net_speed_feature.h"
#include "onboard/prediction/inferencer/act_net_speed_inferencer.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/utils/errors.h"

namespace qcraft {
namespace actnetspeed {
static void RunActNetSpeedInferenceBM(benchmark::State& state,  // NOLINT
                                      const bool use_gpu) {
  NetParam act_net_speed_param;
  CHECK_OK(
      GetProtoParamById("Q0001", "act_net_speed_param", &act_net_speed_param));

  if (!use_gpu) {
    act_net_speed_param.set_device_type(NetParam_DeviceType_CPU);
  }
  ActNetSpeedInferencer act_net_speed_inferencer(act_net_speed_param);
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
  const auto path_feature = prediction::PathFeature{
      .path_xy = std::vector<float>(kPathPointNum * kCoords),
      .path_s = std::vector<float>(kPathPointNum),
      .path_s_offset = std::vector<float>(kPathPointNum),
      .path_heading = std::vector<float>(kPathPointNum),
      .path_xy_preint =
          std::vector<float>(kPathPointPreIntersectionNum * kCoords),
      .path_s_preint = std::vector<float>(kPathPointPreIntersectionNum),
      .path_s_offset_preint = std::vector<float>(kPathPointPreIntersectionNum),
      .path_heading_preint =
          std::vector<float>(kPathPointPreIntersectionNum * kCoords),
      .point_valid = std::vector<float>(kPathPointNum),
  };

  const auto input_feat = prediction::ActNetSpeedFeature{
      .agent_id = "001",
      .ref_position = Vec2d(),
      .rot_rad = 0.0,
      .agent_feature = abs_feat,
      .context_obj_features = std::vector<prediction::ActNetObjectFeature>(
          kMaxOtherObjsNum, object_feature),
      .context_obj_masks = std::vector<float>(kMaxOtherObjsNum),
      .agent_path = path_feature,
      .av_path = path_feature,
      .path_valid_mask = {1.0, 1.0},
  };

  std::vector<prediction::ActNetSpeedFeature> input_features(kMaxBatch,
                                                             input_feat);
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        act_net_speed_inferencer.PredictForObjects(input_features));
  }
}

static void BM_ActNetSpeedInferenceGPU(benchmark::State& state) {  // NOLINT
  RunActNetSpeedInferenceBM(state, /*use_gpu=*/true);
}
static void BM_ActNetSpeedInferenceCPU(benchmark::State& state) {  // NOLINT
  RunActNetSpeedInferenceBM(state, /*use_gpu=*/false);
}

BENCHMARK(BM_ActNetSpeedInferenceGPU);
BENCHMARK(BM_ActNetSpeedInferenceCPU);

}  // namespace actnetspeed
}  // namespace qcraft

BENCHMARK_MAIN();
