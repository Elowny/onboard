#include "onboard/prediction/inferencer/cutin_sl_net_inferencer.h"

#include "gtest/gtest.h"

#include "onboard/params/param_finder.h"
#include "onboard/prediction/feature_extractor/cutin_sl_feature.h"
#include "onboard/utils/errors.h"

namespace qcraft {
namespace prediction {
namespace {

std::vector<CutinSLFeature> FakeBatchCutinSLNetFeatures(int batch_size) {
  const int history_len = kFeatureV2HistoryStepNum;
  const int coords_num = cutin_sl_net::kCoords;

  const auto abs_feat = CutinSLObjectAbsoluteFeature{
      .sl_pos = std::vector<float>(history_len * coords_num),
      .sl_speed = std::vector<float>(history_len * coords_num),
      .yaw_diff_dp = std::vector<float>(history_len * coords_num),
      .length_width = std::vector<float>(2),
      .sl_shape = std::vector<float>(history_len * coords_num * 4),
      .yaw = std::vector<float>(history_len * coords_num),
      .mask = std::vector<float>(history_len),
      .type = 2.0,
      .pos = std::vector<float>(history_len * coords_num),
      .agent_center_dist_to_av_left_lane_boundary =
          std::vector<float>(history_len),
      .agent_center_dist_to_av_right_lane_boundary =
          std::vector<float>(history_len),
  };

  const auto object_feature = CutinSLObjectFeatures{
      .sl_pos = std::vector<float>(history_len * coords_num),
      .sl_speed = std::vector<float>(history_len * coords_num),
      .yaw_diff_dp = std::vector<float>(history_len * coords_num),
      .length_width = std::vector<float>(2),
      .sl_shape = std::vector<float>(history_len * coords_num * 4),
      .type = 2.0,

      .rel_sl_pos_agent = std::vector<float>(history_len * coords_num),
      .rel_sl_speed_agent = std::vector<float>(history_len * coords_num),
      .yaw_diff_agent = std::vector<float>(history_len * coords_num),
      .rel_dist_agent = std::vector<float>(history_len),
  };

  const int lane_segs_num = cutin_sl_net::kLaneSegsNum;
  const auto lane_feature = prediction::CutinSLPolylineFeatures{
      .sl_seg = std::vector<float>(lane_segs_num * cutin_sl_net::kSegDim),
      .seg_len = std::vector<float>(lane_segs_num),
      .dist = std::vector<float>(lane_segs_num),
      .nearest_to_end_dist = std::vector<float>(lane_segs_num),
      .yaw_diff = std::vector<float>(lane_segs_num * coords_num),
      .type = std::vector<float>(lane_segs_num),
      .light = std::vector<float>(lane_segs_num, cutin_sl_net::kLaneLightDim),
      .mask = std::vector<float>(lane_segs_num),
      .seg = std::vector<float>(lane_segs_num * cutin_sl_net::kSegDim),
  };

  const auto input_feature = CutinSLFeature{
      .agent_id = "001",
      .agent_sl_feature = abs_feat,
      .objs_sl_features = std::vector<CutinSLObjectFeatures>(
          cutin_sl_net::kMaxOtherObjsNum, object_feature),
      .lc_sl_features = std::vector<prediction::CutinSLPolylineFeatures>(
          cutin_sl_net::kMaxLaneCenterNum, lane_feature),
      .lb_sl_features = std::vector<prediction::CutinSLPolylineFeatures>(
          cutin_sl_net::kMaxLaneBoundaryNum, lane_feature),
      .cw_sl_features = std::vector<prediction::CutinSLPolylineFeatures>(
          cutin_sl_net::kMaxCrossWalkNum, lane_feature),
      .lc_masks = std::vector<float>(cutin_sl_net::kMaxLaneCenterNum),
      .lb_masks = std::vector<float>(cutin_sl_net::kMaxLaneBoundaryNum),
      .cw_masks = std::vector<float>(cutin_sl_net::kMaxCrossWalkNum),
  };
  std::vector<CutinSLFeature> input_features;
  input_features.reserve(batch_size);
  for (int i = 0; i < batch_size; ++i) {
    input_features.push_back(input_feature);
  }
  return input_features;
}

TEST(CutinSLNetInferencerTest, PredictForObjectsOnCPU) {
  NetParam cutin_sl_net_param;
  CHECK_OK(
      GetProtoParamById("Q0001", "cutin_sl_net_param", &cutin_sl_net_param));
#ifdef Q_CPU_ONLY
  cutin_sl_net_param.set_device_type(NetParam_DeviceType_CPU);
#endif
  cutin_sl_net::CutinSLNetInferencer cutin_sl_net_inferencer(
      cutin_sl_net_param);
  const auto input_features = FakeBatchCutinSLNetFeatures(1);
  const auto inference_results =
      cutin_sl_net_inferencer.PredictForObjects(input_features);
  EXPECT_EQ(inference_results.size(), 1);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
