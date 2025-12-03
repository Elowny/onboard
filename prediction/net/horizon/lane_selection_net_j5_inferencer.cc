#include "onboard/prediction/net/horizon/lane_selection_net_j5_inferencer.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/feature_extractor/lane_selection_feature.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/net/horizon/horizon_tensor_wrapper.h"
#include "onboard/prediction/net/horizon/lane_selection_net.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace prediction {
namespace lane_selection_net {
namespace {}  // namespace

LaneSelectionNetJ5Inferencer::LaneSelectionNetJ5Inferencer(
    const NetParam& net_param)
    : net_param_(net_param) {
  lane_selection_net_j5_ = std::make_unique<PredictionJ5QNN>(net_param_);
}

// GetInput from sampler.
std::vector<AgentDpsIndexInfo>
LaneSelectionNetJ5Inferencer::GenBatchInputs(  // NOLINT
    const AgentDrivePassagesMap& agent_dps_map,
    const ObjectHistorySampler& obj_sampler) const {
  SCOPED_QTRACE_ARG1("GenBatchInputs", "objs_num:", agent_dps_map.size());
  // Get input tensors to fill
  lane_selection_net_j5_->ResetInputs();
  const std::unordered_map<std::string, std::shared_ptr<HorizonTensorWrapper>>&
      batch_map = lane_selection_net_j5_->GetInputMap();
  auto& agent_info = *batch_map.at("agent_info");
  auto& agent_attr = *batch_map.at("agent_attr");
  auto& agent_ctrs = *batch_map.at("agent_ctrs");

  // Record the idx to flatten the lane paths
  std::vector<AgentDpsIndexInfo> batch_idx_info;
  batch_idx_info.reserve(kMaxPredLanePathsNum);
  for (auto iter = agent_dps_map.begin(); iter != agent_dps_map.end(); ++iter) {
    for (int j = 0; j < iter->second.size(); j++) {
      batch_idx_info.push_back(AgentDpsIndexInfo{
          .agent_id = iter->first,
          .dp_idx = j,
      });
    }
  }

  // main loop for flattened lane paths
  for (int batch_idx = 0; batch_idx < batch_idx_info.size(); ++batch_idx) {
    const auto& agent_id = batch_idx_info[batch_idx].agent_id;
    const auto& agent_with_dps = agent_dps_map.at(agent_id);
    const auto& drive_passage =
        *agent_with_dps[batch_idx_info[batch_idx].dp_idx];

    const auto& agent_history =
        obj_sampler.GetResampledMotionHistoryById(agent_id);
    const auto& agent_states = agent_history.states;
    const int agent_state_num = agent_states.size();
    const int agent_hist_num = std::min(agent_state_num, kHistoryNum);

    // agent
    {
      Vec2d start_sl_pos, end_sl_pos;
      // Init rescaled features
      for (int i = 0; i < kHistoryNum; ++i) {
        agent_info.index_put_qat(
            batch_idx, 0, i, 0,
            (kAgentSBackDist * kInvAgentSfullScale - 0.5f) * 2.0f);
        agent_info.index_put_qat(batch_idx, 6, i, 0, -0.5f * 2.0f);
      }
      agent_attr.index_put_qat(batch_idx, 16, 0, 0, -0.5f * 2.0f);
      agent_attr.index_put_qat(batch_idx, 17, 0, 0, -0.5f * 2.0f);

      // agent_info (batch, 7, 10 , 1):
      for (int j = 0; j < agent_hist_num; ++j) {
        const auto& state = agent_states[agent_state_num - agent_hist_num + j];
        const int hist_idx = kHistoryNum - agent_hist_num + j;
        const auto& pos = state.pos;

        // Extract sl pos
        const auto sl_pos_or =
            drive_passage.QueryUnboundedFrenetCoordinateAt(pos);
        if (!sl_pos_or.ok() ||
            std::abs(sl_pos_or->s) > lane_selection_net::kSLCorMax ||
            std::abs(sl_pos_or->l) > lane_selection_net::kSLCorMax) {
          batch_idx_info[batch_idx].is_valid = false;
          VLOG(2) << "agent " << agent_id << "hist number " << j
                  << " : lane selection feature sl pos invalid!";
          break;
        }

        if (j == 0) {
          start_sl_pos = Vec2d(sl_pos_or->s, sl_pos_or->l);
        }
        if (j == agent_hist_num - 1) {
          end_sl_pos = Vec2d(sl_pos_or->s, sl_pos_or->l);
        }

        // Scale and fill agent s pos.
        const float agent_scaled_s =
            ((sl_pos_or->s + kAgentSBackDist) * kInvAgentSfullScale - 0.5f) *
            2.0f;
        agent_info.index_put_qat(batch_idx, 0, hist_idx, 0,
                                 std::clamp(agent_scaled_s, -1.0f, 1.0f));

        // Scale and fill agent l pos.
        const float agent_scaled_l = sl_pos_or->l * kInvAgentLScale;
        agent_info.index_put_qat(batch_idx, 1, hist_idx, 0,
                                 std::clamp(agent_scaled_l, -1.0f, 1.0f));

        // Extract yaw diff to drive passage of current lane path
        const double yaw = state.heading;
        const auto dp_angle_or =
            drive_passage.QueryTangentAngleAtS(sl_pos_or->s);
        if (!dp_angle_or.ok()) {
          VLOG(2) << "agent " << agent_id << "hist number " << j
                  << " : Angle query invalid!";
          batch_idx_info[batch_idx].is_valid = false;
          break;
        }
        const Vec2d yaw_diff_dp = Vec2d::UnitFromAngle(*dp_angle_or - yaw);
        agent_info.index_put_qat(batch_idx, 2, hist_idx, 0, yaw_diff_dp.x());
        agent_info.index_put_qat(batch_idx, 3, hist_idx, 0, yaw_diff_dp.y());

        // Extract agent_dist_to_left_lane_boundary and
        // agent_dist_to_right_lane_boundary
        const auto& agent_center_dist_to_lr_lane_boundary =
            QueryDistanceToLeftAndRightAvLaneBoundary(
                drive_passage, sl_pos_or->s, sl_pos_or->l);

        // Scale and fill agent dist to left boundary
        const float agent_scaled_dist_to_l_boundary =
            agent_center_dist_to_lr_lane_boundary.first *
            kInvAgentDistToLeftLaneBoundaryScale;
        agent_info.index_put_qat(
            batch_idx, 4, hist_idx, 0,
            std::clamp(agent_scaled_dist_to_l_boundary, -1.0f, 1.0f));

        // Scale and fill agent dist to right boundary
        const float agent_scaled_dist_to_r_boundary =
            agent_center_dist_to_lr_lane_boundary.second *
            kInvAgentDistToRightLaneBoundaryScale;
        agent_info.index_put_qat(
            batch_idx, 5, hist_idx, 0,
            std::clamp(agent_scaled_dist_to_r_boundary, -1.0f, 1.0f));

        // Scale and fill lane width.
        const float agent_scaled_lane_width =
            (std::abs(agent_center_dist_to_lr_lane_boundary.first -
                      agent_center_dist_to_lr_lane_boundary.second) *
                 kInvAgentLaneWidthScale -
             0.5f) *
            2.0f;
        agent_info.index_put_qat(
            batch_idx, 6, hist_idx, 0,
            std::clamp(agent_scaled_lane_width, -1.0f, 1.0f));
      }

      if (!batch_idx_info[batch_idx].is_valid) continue;

      // agent_attr (batch, 19, 1 , 1):
      // agent_type onehot
      const int type = static_cast<int>(
          ObjectTypeToLaneSelectionNetType(agent_history.type));
      agent_attr.index_put_qat(batch_idx, type, 0, 0, 1.0f);

      // agent_lw
      const float agent_cur_length = agent_states.back().bbox.length();
      const float agent_cur_width = agent_states.back().bbox.width();
      agent_attr.index_put_qat(
          batch_idx, 16, 0, 0,
          (std::clamp(agent_cur_length * kInvObjectLengthScale, 0.0f, 1.0f) -
           0.5f) *
              2.0f);
      agent_attr.index_put_qat(
          batch_idx, 17, 0, 0,
          (std::clamp(agent_cur_width * kInvObjectWidthScale, 0.0f, 1.0f) -
           0.5f) *
              2.0f);
      // agent_l_leap
      const auto diff_sl_pos = end_sl_pos - start_sl_pos;
      const float scaled_l_leap = diff_sl_pos.y() * kInvAgentLLeapScale;
      agent_attr.index_put_qat(batch_idx, 18, 0, 0,
                               std::clamp(scaled_l_leap, -1.0f, 1.0f));

      // agent_ctrs (batch, 2, 1, 1)
      const float scaled_last_s =
          ((end_sl_pos.x() + kAgentSBackDist) * kInvAgentSfullScale - 0.5f) *
          2.0f;
      agent_ctrs.index_put_qat(batch_idx, 0, 0, 0,
                               std::clamp(scaled_last_s, -1.0f, 1.0f));

      const float scaled_last_l = end_sl_pos.y() * kInvAgentLScale;
      agent_ctrs.index_put_qat(batch_idx, 1, 0, 0,
                               std::clamp(scaled_last_l, -1.0f, 1.0f));
    }  // agent
  }
  return batch_idx_info;
}

LaneSelectionInferencerOutputMap
LaneSelectionNetJ5Inferencer::PredictForObjects(
    const prediction::ObjectHistorySampler& obj_sampler,
    const AgentDrivePassagesMap& agent_dps_map) const {
  SCOPED_QTRACE("LaneSelectionNetJ5Inferencer::Prediction for Objects");
  // 1. Fill features in the input tensors.
  int current_batch_size = 0;
  std::vector<AgentDpsIndexInfo> batch_idx_info;
  {
    SCOPED_QTRACE(
        "LaneSelectionNetJ5Inferencer::GenBatchInputs from obj sampler");
    batch_idx_info = GenBatchInputs(agent_dps_map, obj_sampler);
    current_batch_size = batch_idx_info.size();
    if (current_batch_size == 0) {
      return LaneSelectionInferencerOutputMap();
    }
  }

  // 2. Run LaneSelectionNet forward.
  {
    SCOPED_QTRACE("LaneSelectionNetJ5inferencer::ProcessInEngine");
    lane_selection_net_j5_->Run(current_batch_size);
  }

  // 3. Set objects
  {
    SCOPED_QTRACE("LaneSelectionNetJ5inferencer::AssemblePathProb");
    return ExtractOutputs(batch_idx_info);
  }
}

LaneSelectionInferencerOutputMap LaneSelectionNetJ5Inferencer::ExtractOutputs(
    const std::vector<AgentDpsIndexInfo>& batch_idx_info) const {
  LaneSelectionInferencerOutputMap result;
  const auto& batch_out_map = lane_selection_net_j5_->GetOutputMap();
  const auto& batch_path_probs = batch_out_map.at("lane_path_prob");
  for (int i = 0; i < batch_idx_info.size(); ++i) {
    const auto agent_id = batch_idx_info[i].agent_id;
    const auto lane_path_prob =
        Sigmoid(batch_path_probs->index_get_deqat(i, 0, 0, 0));
    result[agent_id].dp_scores.push_back(lane_path_prob);

    if (batch_idx_info[i].is_valid) {
      result[agent_id].is_valid.push_back(true);
    } else {
      result[agent_id].is_valid.push_back(false);
    }
  }

  return result;
}

}  // namespace lane_selection_net
}  // namespace prediction
}  // namespace qcraft
