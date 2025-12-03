#include "onboard/planner/ml/captain_net/inference/captain_net_inferencer.h"

#include <algorithm>      // for copy, max, min
#include <functional>     // for multiplies
#include <map>            // for map
#include <numeric>        // for accumulate
#include <string>         // for string, basic_string
#include <unordered_map>  // for unordered_map
#include <utility>        // for move

#include "google/protobuf/repeated_ptr_field.h"  // for RepeatedPtrField, RepeatedPtrIterator

#include "onboard/global/trace.h"             // for SCOPED_QTRACE, ScopedTrace
#include "onboard/lite/check.h"               // for QCHECK_GT
#include "onboard/lite/logging.h"             // for BufferedLoggerWrapper
#include "onboard/nets/proto/net_param.pb.h"  // for NetParam, TensorInfo, RepeatedField
#include "onboard/planner/ml/common/feature_extraction_utils.h"  // for OneHotObjectType
#include "onboard/planner/ml/planner_ml_qnn.h"  // for PlannerMLQNN
namespace qcraft::planner::ml::captain_net {

using CaptainNetOut = std::map<std::string, std::vector<float>>;

CaptainNetInferencer::CaptainNetInferencer(const NetParam& net_param)
    : net_param_(net_param) {
  const std::string& onnx_model_path = net_param.onnx_model_path();
  const int max_batch_size = net_param_.max_batch_size();
  QCHECK_GT(max_batch_size, 0);
  captain_net_ = std::unique_ptr<planner::ml::PlannerMLQNN>(
      new planner::ml::PlannerMLQNN(onnx_model_path, net_param_));
}

void FillInMapElementFeature(
    LanesFeature features, int /*batch_idx*/, int /*max_element_num*/,
    std::vector<float>* seg, std::vector<float>* unit_seg_vec,
    std::vector<float>* light, std::vector<float>* dist,
    std::vector<float>* nearest_to_end_dist, std::vector<float>* yaw_diff,
    std::vector<float>* unit_dist_vec, std::vector<float>* unit_tangent,
    std::vector<float>* valid_mask) {
  // seg
  std::copy(features.seg.begin(), features.seg.end(), seg->begin());
  // unit_seg_vec
  std::copy(features.unit_seg_vec.begin(), features.unit_seg_vec.end(),
            unit_seg_vec->begin());
  // light
  std::copy(features.light.begin(), features.light.end(), light->begin());
  // dist
  std::copy(features.dist.begin(), features.dist.end(), dist->begin());
  // nearest_to_end_dist
  std::copy(features.nearest_to_end_dist.begin(),
            features.nearest_to_end_dist.end(), nearest_to_end_dist->begin());
  // yaw_diff
  std::copy(features.yaw_diff.begin(), features.yaw_diff.end(),
            yaw_diff->begin());
  // unit_dist_vec
  std::copy(features.unit_dist_vec.begin(), features.unit_dist_vec.end(),
            unit_dist_vec->begin());
  // unit_tangent
  std::copy(features.unit_tangent.begin(), features.unit_tangent.end(),
            unit_tangent->begin());
  // lc valid mask
  std::copy(features.valid_mask.begin(), features.valid_mask.end(),
            valid_mask->begin());
}

std::vector<std::vector<float>> GetBatchInputs(
    const NetParam& net_param, const CaptainNetFeature& input_features) {
  std::unordered_map<std::string, std::vector<float>> batch_map;

  std::vector<std::string> input_names;
  input_names.reserve(net_param.input_tensor_list_size());
  for (const auto& input_tensor : net_param.input_tensor_list()) {
    const int data_size =
        std::accumulate(input_tensor.shape().begin() + 1,
                        input_tensor.shape().end(), 1, std::multiplies<int>());
    batch_map[input_tensor.name()] = std::vector<float>(data_size, 0.0);
    input_names.push_back(input_tensor.name());
  }

  // Agent
  auto& batch_agent_pos_diff = batch_map["agent_pos_diff"];
  auto& batch_agent_speed = batch_map["agent_speed"];
  auto& batch_agent_heading = batch_map["agent_heading"];

  const auto& agent_feat = input_features.agent_feature;

  // agent_pos_diff

  std::copy(agent_feat.agent_pos_diff.begin(), agent_feat.agent_pos_diff.end(),
            batch_agent_pos_diff.begin());
  // agent_speed
  std::copy(agent_feat.agent_speed.begin(), agent_feat.agent_speed.end(),
            batch_agent_speed.begin());
  // agent_heading
  std::copy(agent_feat.agent_heading.begin(), agent_feat.agent_heading.end(),
            batch_agent_heading.begin());

  // Object
  auto& batch_obj_type = batch_map["obj_type"];
  auto& batch_rel_pos = batch_map["rel_pos"];
  auto& batch_rel_yaw = batch_map["rel_yaw"];
  auto& batch_rel_speed = batch_map["rel_speed"];
  auto& batch_actor_valid_mask = batch_map["actor_valid_mask"];

  const auto& actors_feature = input_features.actors_feature;

  // actor_type
  const auto& actor_one_hot =
      planner::OneHotObjectsType(actors_feature.obj_type, kActorTypeDim);
  std::copy(actor_one_hot.begin(), actor_one_hot.end(), batch_obj_type.begin());

  std::copy(actors_feature.rel_pos.begin(), actors_feature.rel_pos.end(),
            batch_rel_pos.begin());
  // obj_rel_yaw
  std::copy(actors_feature.rel_yaw.begin(), actors_feature.rel_yaw.end(),
            batch_rel_yaw.begin());
  // obj_rel_speed
  std::copy(actors_feature.rel_speed.begin(), actors_feature.rel_speed.end(),
            batch_rel_speed.begin());
  // }
  // obj_valid_mask
  std::copy(actors_feature.valid_mask.begin(), actors_feature.valid_mask.end(),
            batch_actor_valid_mask.begin());

  // Map elements.

  // lanecenter feature
  FillInMapElementFeature(
      input_features.lane_center_feature, 0, kMaxLaneCenterNum,
      &batch_map["lc_seg"], &batch_map["lc_unit_seg_vec"],
      &batch_map["lc_light"], &batch_map["lc_dist"],
      &batch_map["lc_nearest_to_end_dist"], &batch_map["lc_yaw_diff"],
      &batch_map["lc_unit_dist_vec"], &batch_map["lc_unit_tangent"],
      &batch_map["lc_valid_mask"]);

  // Target Lanes elements
  auto& batch_tl_flattened_mask = batch_map["tl_flattened_mask"];
  auto& batch_target_lane_data = batch_map["target_lane_data"];

  const auto& target_lane_feature = input_features.target_lane_feature;
  // target_lane_data
  for (int i = 0; i < target_lane_feature.target_lanes.size(); i++) {
    const auto& single_target_lane = target_lane_feature.target_lanes[i];

    const auto& current_tl_seg_num = static_cast<int>(
        single_target_lane.size() / (kMaxTargetLanePointNum * kCoords));
    for (int j = 0; j < std::min(kMaxTargetLaneSegNum, current_tl_seg_num);
         j++) {
      const auto& single_target_lane_seg =
          single_target_lane.begin() + j * kMaxTargetLanePointNum * kCoords;
      for (int k = 0; k < kMaxTargetLanePointNum / 4; k++) {
        std::copy(single_target_lane_seg + k * 4 * kCoords,
                  single_target_lane_seg + k * 4 * kCoords + kCoords,
                  batch_target_lane_data.begin() +
                      i * kMaxTargetLaneSegNum * (kMaxTargetLanePointNum / 4) *
                          kCoords +
                      j * (kMaxTargetLanePointNum / 4) * kCoords + k * kCoords);
      }
      batch_tl_flattened_mask[i * kMaxTargetLaneSegNum + j + 1] = 1.0;
    }
    batch_tl_flattened_mask[0] = 1.0;
  }

  // Move batch data to a vector of vector.
  std::vector<std::vector<float>> batch;
  batch.resize(input_names.size());
  for (int i = 0; i < input_names.size(); ++i) {
    batch[i] = std::move(batch_map[input_names[i]]);
  }
  return batch;
}

bool CaptainNetInferencer::PlanningTrajectory(
    const CaptainNetFeature& input_features,
    std::vector<std::vector<std::vector<float>>>* prob_out,
    std::vector<std::vector<std::vector<float>>>* traj_out) const {
  SCOPED_QTRACE("CaptainNetInferencer::PlanningTrajectory");
  std::vector<std::vector<float>> batch_inputs;
  CaptainNetOut net_out;

  // 1. Feature to Batch inputs.
  {
    SCOPED_QTRACE("CaptainNetInferencer::GetBatchInputs");
    batch_inputs = GetBatchInputs(net_param_, input_features);
  }
  int current_batch_size = 1;
  // 2. Run CaptainNet forward.
  {
    SCOPED_QTRACE("CaptainNetInferencer::ProcessInEngine");
    net_out = captain_net_->Run(batch_inputs, current_batch_size);
  }

  // 3. Get outputs
  {
    SCOPED_QTRACE("CaptainNetInferencer::Postprocess");

    const auto& out_prob = net_out["prob_out"];
    const auto& out_traj = net_out["traj_out"];

    const int traj_out_size = kPredictSec / kTimeStep * kOutputPointDim;
    const int lane_num = kMaxTargetLanesNum;
    prob_out->resize(lane_num);
    traj_out->resize(lane_num);
    for (int i = 0; i < lane_num; ++i) {
      (*prob_out)[i].resize(kModes);
      (*traj_out)[i].resize(kModes);
      for (int j = 0; j < kModes; ++j) {
        (*prob_out)[i][j].assign(out_prob.begin() + i * kModes + j,
                                 out_prob.begin() + i * kModes + j + 1);
        (*traj_out)[i][j].assign(
            out_traj.begin() + i * kModes * traj_out_size + j * traj_out_size,
            out_traj.begin() + i * kModes * traj_out_size +
                (j + 1) * traj_out_size);
      }
    }
  }

  return true;
}

}  // namespace qcraft::planner::ml::captain_net
