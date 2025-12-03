#include "onboard/planner/ml/captain_net/inference/captain_net_j5_inferencer.h"

#include <algorithm>      // for max, min
#include <array>          // for array
#include <map>            // for map
#include <string>         // for string
#include <unordered_map>  // for unordered_map

#include "onboard/global/trace.h"             // for SCOPED_QTRACE, ScopedTrace
#include "onboard/nets/proto/net_param.pb.h"  // for NetParam
#include "onboard/planner/ml/captain_net_j5/horizon_tensor_wrapper.h"  // for HorizonTensorWrapper
#include "onboard/planner/ml/captain_net_j5/planner_ml_j5_qnn.h"  // for PLANNERMLJ5QNN
namespace qcraft::planner::ml::captain_net {

using CaptainNetJ5Out = std::map<std::string, std::vector<float>>;
using NLLTrajPoint = std::array<double, 5>;  // x, y, s1, s2, c.

CaptainNetJ5Inferencer::CaptainNetJ5Inferencer(const NetParam& net_param)
    : net_param_(net_param) {
  captain_net_j5_ = std::make_unique<PLANNERMLJ5QNN>(net_param_);
}

int CaptainNetJ5Inferencer::GetBatchInputs(
    const NetParam& /*net_param*/,
    const CaptainNetFeature& input_features) const {
  captain_net_j5_->ResetInputs();
  const std::unordered_map<std::string, std::shared_ptr<HorizonTensorWrapper>>&
      batch_map = captain_net_j5_->GetInputMap();

  auto& ego_feat = *batch_map.at("ego_feat");
  auto& actor_feat = *batch_map.at("actor_feat");
  auto& lc_feat = *batch_map.at("lc_feat");
  auto& tl_feat = *batch_map.at("tl_feat");

  // ego_feat  (batch_size,6,10,1)
  const auto& agent_feat = input_features.agent_feature;

  const int state_num = agent_feat.agent_pos_diff.size() / 2;
  const int start_idx = kHistoryNum - state_num;
  const int start = std::max(0, start_idx);
  for (int hist_idx = start; hist_idx < kHistoryNum; ++hist_idx) {
    // when use obj_his_sampler, need add #if (defined(__ARM_NEON) &&
    // defined(__J5__)) 351,352 act_net_j5_inferencer.cc
    const int ego_feat_index_x = 2 + hist_idx * 2;
    const int ego_feat_index_y = 3 + hist_idx * 2;
    ego_feat.index_put_qat(0, 0, hist_idx, 0,
                           agent_feat.agent_pos_diff.at(ego_feat_index_x));
    ego_feat.index_put_qat(0, 1, hist_idx, 0,
                           agent_feat.agent_pos_diff.at(ego_feat_index_y));
    ego_feat.index_put_qat(
        0, 2, hist_idx, 0,
        agent_feat.agent_speed.at(ego_feat_index_x) * kInvSpeedScale);
    ego_feat.index_put_qat(
        0, 3, hist_idx, 0,
        agent_feat.agent_speed.at(ego_feat_index_y) * kInvSpeedScale);
    ego_feat.index_put_qat(0, 4, hist_idx, 0,
                           agent_feat.agent_heading.at(ego_feat_index_x));
    ego_feat.index_put_qat(0, 5, hist_idx, 0,
                           agent_feat.agent_heading.at(ego_feat_index_y));
  }

  // actor_feat  (batch_size,6,10,32)
  const auto& actors_feature = input_features.actors_feature;
  const int actors_num =
      actors_feature.rel_pos.size() / ((kHistoryNum + 1) * 2);
  for (int actor_idx = 0; actor_idx < actors_num; ++actor_idx) {
    for (int hist_idx = start; hist_idx < kHistoryNum; ++hist_idx) {
      const int actor_feat_index_x =
          2 + hist_idx * 2 + actor_idx * ((kHistoryNum + 1) * 2);
      const int actor_feat_index_y =
          3 + hist_idx * 2 + actor_idx * ((kHistoryNum + 1) * 2);
      actor_feat.index_put_qat(
          0, 0, hist_idx, actor_idx + 1,
          actors_feature.rel_pos.at(actor_feat_index_x) * kInvCoordScale);
      actor_feat.index_put_qat(
          0, 1, hist_idx, actor_idx + 1,
          actors_feature.rel_pos.at(actor_feat_index_y) * kInvCoordScale);
      actor_feat.index_put_qat(
          0, 2, hist_idx, actor_idx + 1,
          actors_feature.rel_speed.at(actor_feat_index_x) * kInvSpeedScale);
      actor_feat.index_put_qat(
          0, 3, hist_idx, actor_idx + 1,
          actors_feature.rel_speed.at(actor_feat_index_y) * kInvSpeedScale);
      actor_feat.index_put_qat(0, 4, hist_idx, actor_idx + 1,
                               actors_feature.rel_yaw.at(actor_feat_index_x));
      actor_feat.index_put_qat(0, 5, hist_idx, actor_idx + 1,
                               actors_feature.rel_yaw.at(actor_feat_index_y));
    }
  }

  // lc_feat (batch_size,18,5,160)
  const auto& lc_feature = input_features.lane_center_feature;
  for (int lc_idx = 0; lc_idx < kMaxLaneCenterNum; ++lc_idx) {
    for (int seg_num = 0; seg_num < kLaneSegsNum; ++seg_num) {
      const int seg_index_x =
          0 + seg_num * kSegDim + lc_idx * (kSegDim * kLaneSegsNum);
      const int seg_index_y =
          1 + seg_num * kSegDim + lc_idx * (kSegDim * kLaneSegsNum);
      const int seg_index_z =
          2 + seg_num * kSegDim + lc_idx * (kSegDim * kLaneSegsNum);
      const int seg_index_w =
          3 + seg_num * kSegDim + lc_idx * (kSegDim * kLaneSegsNum);

      const int unit_seg_vec_index_x =
          0 + seg_num * 2 + lc_idx * (2 * kLaneSegsNum);
      const int unit_seg_vec_index_y =
          1 + seg_num * 2 + lc_idx * (2 * kLaneSegsNum);

      const int dist_index_x = 0 + seg_num + lc_idx * kLaneSegsNum;
      if (lc_idx < lc_feature.seg.size() / (kLaneSegsNum * 4)) {
        lc_feat.index_put_qat(0, 0, seg_num, lc_idx,
                              lc_feature.seg.at(seg_index_x) * kInvCoordScale);
        lc_feat.index_put_qat(0, 1, seg_num, lc_idx,
                              lc_feature.seg.at(seg_index_y) * kInvCoordScale);
        lc_feat.index_put_qat(0, 2, seg_num, lc_idx,
                              lc_feature.seg.at(seg_index_z) * kInvCoordScale);
        lc_feat.index_put_qat(0, 3, seg_num, lc_idx,
                              lc_feature.seg.at(seg_index_w) * kInvCoordScale);
      }
      if (lc_idx < lc_feature.unit_seg_vec.size() / (kLaneSegsNum * 2)) {
        lc_feat.index_put_qat(0, 4, seg_num, lc_idx,
                              lc_feature.unit_seg_vec.at(unit_seg_vec_index_x));
        lc_feat.index_put_qat(0, 5, seg_num, lc_idx,
                              lc_feature.unit_seg_vec.at(unit_seg_vec_index_y));
      }
      if (lc_idx < lc_feature.unit_tangent.size() / (kLaneSegsNum * 2)) {
        lc_feat.index_put_qat(0, 6, seg_num, lc_idx,
                              lc_feature.unit_tangent.at(unit_seg_vec_index_x));
        lc_feat.index_put_qat(0, 7, seg_num, lc_idx,
                              lc_feature.unit_tangent.at(unit_seg_vec_index_y));
      }
      if (lc_idx < lc_feature.dist.size() / kLaneSegsNum) {
        lc_feat.index_put_qat(
            0, 8, seg_num, lc_idx,
            lc_feature.dist.at(dist_index_x) * kInvCoordScale);
      }
      if (lc_idx < lc_feature.unit_dist_vec.size() / (kLaneSegsNum * 2)) {
        lc_feat.index_put_qat(
            0, 9, seg_num, lc_idx,
            lc_feature.unit_dist_vec.at(unit_seg_vec_index_x));
        lc_feat.index_put_qat(
            0, 10, seg_num, lc_idx,
            lc_feature.unit_dist_vec.at(unit_seg_vec_index_y));
      }
      if (lc_idx < lc_feature.nearest_to_end_dist.size() / kLaneSegsNum) {
        lc_feat.index_put_qat(0, 11, seg_num, lc_idx,
                              lc_feature.nearest_to_end_dist.at(dist_index_x));
      }
      if (lc_idx < lc_feature.yaw_diff.size() / (kLaneSegsNum * 2)) {
        lc_feat.index_put_qat(0, 12, seg_num, lc_idx,
                              lc_feature.yaw_diff.at(unit_seg_vec_index_x));
        lc_feat.index_put_qat(0, 13, seg_num, lc_idx,
                              lc_feature.yaw_diff.at(unit_seg_vec_index_y));
      }
      if (lc_idx < lc_feature.light.size() / (kLaneSegsNum * 4)) {
        lc_feat.index_put_qat(0, 14, seg_num, lc_idx,
                              lc_feature.light.at(seg_index_x));
        lc_feat.index_put_qat(0, 15, seg_num, lc_idx,
                              lc_feature.light.at(seg_index_y));
        lc_feat.index_put_qat(0, 16, seg_num, lc_idx,
                              lc_feature.light.at(seg_index_z));
        lc_feat.index_put_qat(0, 17, seg_num, lc_idx,
                              lc_feature.light.at(seg_index_w));
      }
    }
  }

  // tl_feat (batch_size,50,1,3)
  const auto& tl_feature = input_features.target_lane_feature;
  for (int i = 0; i < tl_feature.target_lanes.size(); i++) {
    const auto& single_target_lane = tl_feature.target_lanes[i];
    const auto& current_tl_seg_num = static_cast<int>(
        single_target_lane.size() / (kMaxTargetLanePointNum * kCoords));
    for (int j = 0; j < std::min(kMaxTargetLaneSegNum, current_tl_seg_num);
         j++) {
      const auto& single_target_lane_seg =
          single_target_lane.begin() + j * kMaxTargetLanePointNum * kCoords;
      for (int k = 0; k < kMaxTargetLanePointNum / 4; k++) {
        const auto& tl_feat_point_x = single_target_lane_seg + k * 4 * kCoords;
        const auto& tl_feat_point_y =
            single_target_lane_seg + k * 4 * kCoords + 1;
        tl_feat.index_put_qat(0, 0 + 2 * k + 2 * kMaxTargetLaneSegNum * j, 0, i,
                              *tl_feat_point_x * kInvCoordScale);
        tl_feat.index_put_qat(0, 1 + 2 * k + 2 * kMaxTargetLaneSegNum * j, 0, i,
                              *tl_feat_point_y * kInvCoordScale);
      }
    }
  }
  return 1;
}

bool CaptainNetJ5Inferencer::ExtractOutputs(
    int /*current_batch_size*/,
    std::vector<std::vector<std::vector<float>>>* prob_out,
    std::vector<std::vector<std::vector<float>>>* traj_out) const {
  const auto& batch_out_map = captain_net_j5_->GetOutputMap();
  //   const int traj_size = kFutureNum * kOutCoords;
  CaptainNetJ5Out objs_out;
  const auto& batch_prob_out = batch_out_map.at("prob_out");
  const auto& batch_traj_out = batch_out_map.at("traj_out");

  prob_out->resize(kMaxTargetLanesNum);
  traj_out->resize(kMaxTargetLanesNum);
  for (int j = 0; j < kMaxTargetLanesNum; ++j) {
    (*prob_out)[j].resize(kModes);
    (*traj_out)[j].resize(kModes);
    (*prob_out)[j][0].resize(kModes);
    (*traj_out)[j][0].resize(kFutureNum * kOutputPointDim);
    for (int k = 0; k < kFutureNum; ++k) {
      (*traj_out)[j][0][0 + k * kOutputPointDim] =
          (batch_traj_out->index_get_deqat(0, k * kOutputPointDim, 0, j));
      (*traj_out)[j][0][1 + k * kOutputPointDim] =
          (batch_traj_out->index_get_deqat(0, k * kOutputPointDim + 1, 0, j));
      (*traj_out)[j][0][2 + k * kOutputPointDim] =
          (batch_traj_out->index_get_deqat(0, k * kOutputPointDim + 2, 0, j));
      (*traj_out)[j][0][3 + k * kOutputPointDim] =
          (batch_traj_out->index_get_deqat(0, k * kOutputPointDim + 3, 0, j));
      (*traj_out)[j][0][4 + k * kOutputPointDim] =
          (batch_traj_out->index_get_deqat(0, k * kOutputPointDim + 4, 0, j));
    }
    (*prob_out)[j][0][0] = (batch_prob_out->index_get_deqat(0, 0, 0, j));
  }
  return true;
}

bool CaptainNetJ5Inferencer::PlanningTrajectory(
    const CaptainNetFeature& input_features,
    std::vector<std::vector<std::vector<float>>>* prob_out,
    std::vector<std::vector<std::vector<float>>>* traj_out) const {
  SCOPED_QTRACE("CaptainNetJ5Inferencer::PlanningTrajectory");
  std::vector<std::vector<float>> batch_inputs;
  CaptainNetJ5Out net_j5_out;
  int current_batch_size = 0;

  // 1. Feature to Batch inputs.
  {
    SCOPED_QTRACE("CaptainNetJ5Inferencer::GetBatchInputs");
    current_batch_size = GetBatchInputs(net_param_, input_features);
  }
  //   int current_batch_size = 1;
  // 2. Run CaptainNetj5 forward.
  {
    SCOPED_QTRACE("CaptainNetJ5Inferencer::ProcessInEngine");
    captain_net_j5_->Run(current_batch_size);
  }

  // 3.
  {
    SCOPED_QTRACE("CaptainNetJ5Inferencer::Postprocess");
    ExtractOutputs(current_batch_size, prob_out, traj_out);
  }
  return true;
}

}  // namespace qcraft::planner::ml::captain_net
