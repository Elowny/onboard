#include "onboard/prediction/inferencer/act_net_speed_inferencer.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <ext/alloc_traits.h>

#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/prediction/feature_extractor/act_net_speed_feature.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace actnetspeed {

namespace {
const double kLengthScaleFactor = 1 / 10.0;
const double kWidthScaleFactor = 1 / 3.0;

std::vector<std::vector<float>> GetBatchInputs(
    const NetParam& net_param,
    absl::Span<const prediction::ActNetSpeedFeature> input_features) {
  std::unordered_map<std::string, std::vector<float>> map_name_batch;
  const int num_objs = input_features.size();

  std::vector<std::string> input_names;
  input_names.reserve(net_param.input_tensor_list_size());
  for (const auto& input_tensor : net_param.input_tensor_list()) {
    const int data_size = std::accumulate(input_tensor.shape().begin() + 1,
                                          input_tensor.shape().end(), num_objs,
                                          std::multiplies<int>());
    map_name_batch[input_tensor.name()] = std::vector<float>(data_size, 0.0);
    input_names.push_back(input_tensor.name());
  }

  auto& batch_agent_pos_diff = map_name_batch["agent_pos_diff"];
  auto& batch_agent_speed = map_name_batch["agent_speed"];
  auto& batch_agent_heading = map_name_batch["agent_heading"];
  auto& batch_agent_type = map_name_batch["agent_type"];
  auto& batch_agent_lw = map_name_batch["agent_lw"];
  auto& batch_agent_pos = map_name_batch["agent_pos"];

  auto& batch_obj_pos_diff = map_name_batch["obj_pos_diff"];
  auto& batch_obj_speed = map_name_batch["obj_speed"];
  auto& batch_obj_heading = map_name_batch["obj_heading"];
  auto& batch_obj_type = map_name_batch["obj_type"];
  auto& batch_obj_valid_mask = map_name_batch["actor_valid_mask"];
  auto& batch_obj_rel_speed = map_name_batch["rel_speed"];
  auto& batch_obj_rel_dist = map_name_batch["rel_dist"];
  auto& batch_obj_rel_yaw = map_name_batch["rel_yaw"];
  auto& batch_obj_lw = map_name_batch["obj_lw"];
  auto& batch_obj_pos = map_name_batch["obj_pos"];

  auto& batch_path_xy = map_name_batch["path_xy"];
  auto& batch_path_heading = map_name_batch["path_heading"];
  auto& batch_path_s_offset = map_name_batch["path_offset"];
  auto& batch_path_valid_mask = map_name_batch["path_valid_mask"];

  // Skip history.
  const int skipped_hist = prediction::kFeatureV2HistoryStepNum - kHistoryNum;

  // TODO(changqing): Skip history and do scale in C++ once retrain.
  for (int i = 0; i < num_objs; ++i) {
    const auto& feat = input_features[i];
    // Agent features.
    // agent_pos_diff
    std::copy(feat.agent_feature.pos_diff.begin() + skipped_hist * kCoords,
              feat.agent_feature.pos_diff.end(),
              batch_agent_pos_diff.begin() + i * kHistoryNum * kCoords);
    // agent_speed
    std::copy(feat.agent_feature.speed.begin() + skipped_hist * kCoords,
              feat.agent_feature.speed.end(),
              batch_agent_speed.begin() + i * kHistoryNum * kCoords);
    // agent_heading
    std::copy(feat.agent_feature.yaw.begin() + skipped_hist * kCoords,
              feat.agent_feature.yaw.end(),
              batch_agent_heading.begin() + i * kHistoryNum * kCoords);
    // agent_type
    const auto& one_hot =
        prediction::OneHotObjectType(feat.agent_feature.type, kObjectTypeDim);
    std::copy(one_hot.begin(), one_hot.end(),
              batch_agent_type.begin() + i * kObjectTypeDim);
    // agent_lw
    std::copy(feat.agent_feature.lw.begin(), feat.agent_feature.lw.end(),
              batch_agent_lw.begin() + i * 2);
    // agent_pos
    std::copy(feat.agent_feature.pos.begin() + skipped_hist * kCoords,
              feat.agent_feature.pos.end(),
              batch_agent_pos.begin() + i * kHistoryNum * kCoords);
    // Pre intersection path.
    // agent_path_xy
    std::copy(feat.agent_path.path_xy_preint.begin(),
              feat.agent_path.path_xy_preint.end(),
              batch_path_xy.begin() +
                  i * kPathPointPreIntersectionNum * kCoords * kPathNum);
    // av_path_xy
    std::copy(feat.av_path.path_xy_preint.begin(),
              feat.av_path.path_xy_preint.end(),
              batch_path_xy.begin() +
                  i * kPathPointPreIntersectionNum * kCoords * kPathNum +
                  kPathPointPreIntersectionNum * kCoords);
    // agent_path_s_offset
    std::copy(feat.agent_path.path_s_offset_preint.begin(),
              feat.agent_path.path_s_offset_preint.end(),
              batch_path_s_offset.begin() +
                  i * kPathPointPreIntersectionNum * kPathNum);
    // av_path_s_offset
    std::copy(feat.av_path.path_s_offset_preint.begin(),
              feat.av_path.path_s_offset_preint.end(),
              batch_path_s_offset.begin() +
                  i * kPathPointPreIntersectionNum * kPathNum +
                  kPathPointPreIntersectionNum);
    // agent_path_heading
    std::copy(feat.agent_path.path_heading_preint.begin(),
              feat.agent_path.path_heading_preint.end(),
              batch_path_heading.begin() +
                  i * kPathPointPreIntersectionNum * kCoords * kPathNum);
    // av_path_heading
    std::copy(feat.av_path.path_heading_preint.begin(),
              feat.av_path.path_heading_preint.end(),
              batch_path_heading.begin() +
                  i * kPathPointPreIntersectionNum * kCoords * kPathNum +
                  kPathPointPreIntersectionNum * kCoords);
    // path valid mask
    std::copy(feat.path_valid_mask.begin(), feat.path_valid_mask.end(),
              batch_path_valid_mask.begin() + i * 2);

    // Scaling for agent lw.
    for (int k = 0; k < batch_agent_lw.size() / 2; ++k) {
      batch_agent_lw[2 * k] =
          std::clamp(batch_agent_lw[2 * k] * kLengthScaleFactor, 0.0, 1.0);
      batch_agent_lw[2 * k + 1] =
          std::clamp(batch_agent_lw[2 * k + 1] * kWidthScaleFactor, 0.0, 1.0);
    }

    // Object(av) features.
    {  // Hack for other obj num difference between act net def and act net
       // speed def.
      for (int j = 0; j < feat.context_obj_features.size(); j++) {
        auto& abs_feat = feat.context_obj_features[j].abs_feat;
        auto& rel_feat = feat.context_obj_features[j].rel_feat;
        const auto& obj_query_idx =
            i * kMaxOtherObjsNum + j;  // j th obj in i th agent in the batch.
        // obj_pos_diff
        std::copy(
            abs_feat.pos_diff.begin() + skipped_hist * kCoords,
            abs_feat.pos_diff.end(),
            batch_obj_pos_diff.begin() + obj_query_idx * kHistoryNum * kCoords);
        // obj_speed
        std::copy(
            abs_feat.speed.begin() + skipped_hist * kCoords,
            abs_feat.speed.end(),
            batch_obj_speed.begin() + obj_query_idx * kHistoryNum * kCoords);
        // obj_heading
        std::copy(
            abs_feat.yaw.begin() + skipped_hist * kCoords, abs_feat.yaw.end(),
            batch_obj_heading.begin() + obj_query_idx * kHistoryNum * kCoords);
        // obj_type
        const auto& one_hot =
            prediction::OneHotObjectType(abs_feat.type, kObjectTypeDim);
        std::copy(one_hot.begin(), one_hot.end(),
                  batch_obj_type.begin() + obj_query_idx * kObjectTypeDim);
        // obj_rel_speed
        std::copy(rel_feat.rel_speed.begin() + skipped_hist * kCoords,
                  rel_feat.rel_speed.end(),
                  batch_obj_rel_speed.begin() +
                      obj_query_idx * kHistoryNum * kCoords);
        // obj_rel_yaw
        std::copy(
            rel_feat.rel_yaw.begin() + skipped_hist * kCoords,
            rel_feat.rel_yaw.end(),
            batch_obj_rel_yaw.begin() + obj_query_idx * kHistoryNum * kCoords);
        // obj_rel_dist
        std::copy(rel_feat.rel_dist.begin() + skipped_hist,
                  rel_feat.rel_dist.end(),
                  batch_obj_rel_dist.begin() + obj_query_idx * kHistoryNum);
        // obj_lw
        std::copy(abs_feat.lw.begin(), abs_feat.lw.end(),
                  batch_obj_lw.begin() + obj_query_idx * 2);
        // obj_pos
        std::copy(
            abs_feat.pos.begin() + skipped_hist * kCoords, abs_feat.pos.end(),
            batch_obj_pos.begin() + obj_query_idx * kHistoryNum * kCoords);
      }
      // obj_valid_mask
      std::copy(feat.context_obj_masks.begin(), feat.context_obj_masks.end(),
                batch_obj_valid_mask.begin() + i * kMaxOtherObjsNum);
    }

    // scale
    for (int k = 0; k < batch_obj_lw.size() / 2; ++k) {
      batch_obj_lw[2 * k] =
          std::clamp(batch_obj_lw[2 * k] * kLengthScaleFactor, 0.0, 1.0);
      batch_obj_lw[2 * k + 1] =
          std::clamp(batch_obj_lw[2 * k + 1] * kWidthScaleFactor, 0.0, 1.0);
    }
  }
  // From map to nested vectors.
  std::vector<std::vector<float>> batch;
  batch.resize(input_names.size());
  for (int i = 0; i < input_names.size(); ++i) {
    batch[i] = std::move(map_name_batch[input_names[i]]);
  }
  return batch;
}
}  // namespace

ActNetSpeedInferencer::ActNetSpeedInferencer(const NetParam& net_param)
    : net_param_(net_param) {
  const std::string& trt_model_path = net_param.trt_model_path();
  const int max_batch_size = net_param_.max_batch_size();
  QCHECK_GT(max_batch_size, 0);
  act_net_speed_ =
      std::make_unique<prediction::PredictionQNN>(trt_model_path, net_param_);
}

ObjectsSpeedTrajPairs ActNetSpeedInferencer::PredictForObjects(
    absl::Span<const prediction::ActNetSpeedFeature> input_features) const {
  SCOPED_QTRACE("ActNetSpeedInferencer::PredictForObjects");
  std::map<std::string, std::vector<float>> net_out;
  std::vector<std::vector<float>> batch_inputs;

  ScopedMultiTimer timer("ActNetSpeedInferencer::PredictForObjects");

  ObjectsSpeedTrajPairs object_speed_pairs;  // result map.
  const int current_batch_size = input_features.size();
  if (current_batch_size == 0) {
    return object_speed_pairs;
  }

  // 1. Feature to inputs.
  {
    SCOPED_QTRACE("ActNetSpeedInferencer::GetBatchInputs");
    batch_inputs = GetBatchInputs(net_param_, input_features);
    timer.Mark("ActNetSpeedInferencer::GetBatchInputs");
  }

  // 2. Run forward.
  {
    SCOPED_QTRACE("ActNetSpeedInferencer::ProcessInEngine");
    net_out = act_net_speed_->Run(batch_inputs, current_batch_size);
    timer.Mark("ActNetSpeedInferencer::Inference");
  }

  // 3. Assemble.
  // Relation out: batch_size * 3.
  // Agent speed out: batch_size * (2 * future_num).
  // Av speed out: batch_size * (2 * future_num).
  {
    SCOPED_QTRACE("ActNetSpeedInferencer::AssembleSpeedTrajPairs");
    const auto& relation_out = net_out["relation_probs"];
    const int relation_num = relation_out.size() / input_features.size();
    for (int i = 0; i < input_features.size(); ++i) {
      // Push back to result vector for current object_id.
      const auto& agent_id = input_features[i].agent_id;
      auto* ptr_speed_pairs = FindOrNull(object_speed_pairs, agent_id);
      if (ptr_speed_pairs == nullptr) {
        SpeedTrajPairs speed_pairs;
        object_speed_pairs[agent_id] = std::move(speed_pairs);
        ptr_speed_pairs =
            FindOrNull(object_speed_pairs,
                       agent_id);  // Point to the new vector in the map.
      }
      auto& speed_pairs = *ptr_speed_pairs;  // Vector.
      SpeedTrajPair speed_pair;
      speed_pair.traj_index = input_features[i].traj_index;
      // Assign relation.
      prediction::AVObjectRelation relation_probs;
      const int query_idx = relation_num * i;
      relation_probs.no_relation = relation_out[query_idx];
      relation_probs.yield = relation_out[query_idx + 1];
      relation_probs.pass = relation_out[query_idx + 2];
      speed_pair.relation_probs = relation_probs;
      VLOG(2) << absl::StrFormat("object %s av relation: %s", agent_id,
                                 relation_probs.DebugString());
      // Assign agent speed and av speeds. Not activated.
      // SpeedTraj agent_speed;
      // SpeedTraj av_speed;
      // agent_speed.yield_speed.reserve(future_num);
      // agent_speed.pass_speed.reserve(future_num);
      // av_speed.yield_speed.reserve(future_num);
      // av_speed.pass_speed.reserve(future_num);
      // const auto& agent_speed_out = agent_speeds[i];
      // const auto& av_speed_out = av_speeds[i];
      // for (int k = 0; k < future_num; ++k) {
      //   agent_speed.yield_speed.push_back(agent_speed_out[k]);
      //   agent_speed.pass_speed.push_back(agent_speed_out[k + future_num]);
      //   av_speed.yield_speed.push_back(av_speed_out[k]);
      //   av_speed.pass_speed.push_back(av_speed_out[k + future_num]);
      // }
      // speed_pair.av_speeds = std::move(av_speed);
      // speed_pair.agent_speeds = std::move(agent_speed);
      // Push to the object result vector.
      speed_pairs.push_back(std::move(speed_pair));
    }
    timer.Mark("ActNetSpeedInferencer::Assemble");
  }
  return object_speed_pairs;
}

}  // namespace actnetspeed
}  // namespace qcraft
