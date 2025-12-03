#include "onboard/prediction/inferencer/act_net_inferencer.h"

#include <algorithm>
#include <vector>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/params/param_finder.h"
#include "onboard/utils/errors.h"

namespace qcraft {
namespace prediction {
namespace {
std::vector<ActNetFeature> FakeBatchActNetFeatures(int batch_size) {
  const int history_len = kFeatureV2HistoryStepNum;
  const int coords_num = actnet::kCoords;
  const auto abs_feat = ActNetObjectAbsoluteFeature{
      .pos = std::vector<float>(history_len * coords_num),
      .pos_diff = std::vector<float>(history_len * coords_num),
      .speed = std::vector<float>(history_len * coords_num),
      .yaw = std::vector<float>(history_len * coords_num),
      .shape = std::vector<float>(history_len * coords_num * 4),
      .ts = std::vector<float>(history_len),
      .mask = std::vector<float>(history_len),
      .type = 2.0,
      .lw = std::vector<float>(2),
  };
  const auto rel_feat = ActNetObjectRelFeature{
      .rel_pos = std::vector<float>(history_len * coords_num),
      .rel_dist = std::vector<float>(history_len),
      .rel_yaw = std::vector<float>(history_len * coords_num),
      .yaw_diff = std::vector<float>(history_len * coords_num),
      .rel_speed = std::vector<float>(history_len * coords_num),
      .rel_shape = std::vector<float>(history_len * coords_num * 4),
      .rel_mask = std::vector<float>(history_len),
  };
  const auto object_feature =
      ActNetObjectFeature{.id = "", .abs_feat = abs_feat, .rel_feat = rel_feat};
  const int lane_segs_num = actnet::kLaneSegsNum;
  const auto lane_feature = prediction::ActNetPolylineFeature{
      .seg = std::vector<float>(lane_segs_num * actnet::kSegDim),
      .unit_seg_vec = std::vector<float>(lane_segs_num * coords_num),
      .seg_len = std::vector<float>(lane_segs_num),
      .unit_tangent = std::vector<float>(lane_segs_num * coords_num),
      .dist = std::vector<float>(lane_segs_num),
      .unit_dist_vec = std::vector<float>(lane_segs_num * coords_num),
      .nearest_to_end_dist = std::vector<float>(lane_segs_num),
      .yaw_diff = std::vector<float>(lane_segs_num * coords_num),
      .type = std::vector<float>(lane_segs_num),
      .light = std::vector<float>(lane_segs_num, actnet::kLaneLightDim),
      .mask = std::vector<float>(lane_segs_num),
      .speed_limit_kph = std::vector<float>(lane_segs_num),
  };
  const auto input_feature = ActNetFeature{
      .agent_id = "001",
      .ref_position = Vec2d(),
      .rot_rad = 60.0,
      .stop_time_info = std::vector<float>(actnet::kStopInfoDim),
      .agent_feature = abs_feat,
      .context_obj_features = std::vector<ActNetObjectFeature>(
          actnet::kMaxOtherObjsNum, object_feature),
      .context_obj_masks = std::vector<float>(actnet::kMaxOtherObjsNum),
      .lc_features = std::vector<prediction::ActNetPolylineFeature>(
          actnet::kMaxLaneCenterNum, lane_feature),
      .solid_lb_features = std::vector<prediction::ActNetPolylineFeature>(
          actnet::kMaxLaneBoundaryNum, lane_feature),
      .cw_features = std::vector<prediction::ActNetPolylineFeature>(
          actnet::kMaxCrossWalkNum, lane_feature),
      .lc_masks = std::vector<float>(actnet::kMaxLaneCenterNum),
      .solid_lb_masks = std::vector<float>(actnet::kMaxLaneBoundaryNum),
      .cw_masks = std::vector<float>(actnet::kMaxCrossWalkNum),
  };
  std::vector<ActNetFeature> input_features;
  input_features.reserve(batch_size);
  for (int i = 0; i < batch_size; ++i) {
    input_features.push_back(input_feature);
  }
  return input_features;
}

TEST(ActNetInferencerTest, PredictForObjectsOnCPU) {
  NetParam act_net_param;
  CHECK_OK(GetProtoParamById("Q0001", "act_net_param", &act_net_param));
  act_net_param.set_device_type(NetParam_DeviceType_CPU);
  actnet::ActNetInferencer act_net_inferencer(act_net_param);
  const auto input_features = FakeBatchActNetFeatures(1);
  const auto inference_results =
      act_net_inferencer.PredictForObjects(input_features);
  EXPECT_EQ(inference_results.size(), 1);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
