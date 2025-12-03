#include "onboard/prediction/inferencer/act_net_speed_inferencer.h"

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/params/param_finder.h"
#include "onboard/utils/errors.h"

namespace qcraft {
namespace actnetspeed {
namespace {

std::vector<prediction::ActNetSpeedFeature> FakeBatchActNetSpeedFeatures(
    int batch_size) {
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

  std::vector<prediction::ActNetSpeedFeature> input_features(batch_size,
                                                             input_feat);
  return input_features;
}

TEST(ActNetSpeedInferencerTest, PredictForObjectsOnCPU) {
  NetParam act_net_speed_param;
  CHECK_OK(
      GetProtoParamById("Q0001", "act_net_speed_param", &act_net_speed_param));
  act_net_speed_param.set_device_type(NetParam_DeviceType_CPU);
  ActNetSpeedInferencer act_net_speed_inferencer(act_net_speed_param);
  const auto input_features = FakeBatchActNetSpeedFeatures(1);
  const auto inference_results =
      act_net_speed_inferencer.PredictForObjects(input_features);
  EXPECT_EQ(inference_results.size(), input_features.size());
  LOG(INFO) << inference_results.at("001").at(0).RelationDebugString();
  LOG(INFO) << inference_results.at("001").at(0).AgentSpeedDebugString();
}

}  // namespace
}  // namespace actnetspeed
}  // namespace qcraft
