#include "onboard/prediction/inferencer/cutin_sl_net_inferencer.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/math/util.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"

namespace qcraft {
namespace cutin_sl_net {
namespace {
using CutinSLNetOut = std::map<std::string, std::vector<float>>;

void FillInMapElementFeature(
    absl::Span<const prediction::CutinSLPolylineFeatures> features,
    int batch_idx, int max_element_num, std::vector<float>* sl_seg,
    std::vector<float>* seg_len, std::vector<float>* light,
    std::vector<float>* type, std::vector<float>* dist,
    std::vector<float>* nearest_to_end_dist, std::vector<float>* yaw_diff) {
  for (int j = 0; j < features.size(); ++j) {
    const auto& feat = features[j];
    // sl seg
    std::copy(feat.sl_seg.begin(), feat.sl_seg.end(),
              sl_seg->begin() +
                  (batch_idx * max_element_num + j) * kLaneSegsNum * kSegDim);
    // seg len
    std::copy(
        feat.seg_len.begin(), feat.seg_len.end(),
        seg_len->begin() + (batch_idx * max_element_num + j) * kLaneSegsNum);
    // light
    std::copy(feat.light.begin(), feat.light.end(),
              light->begin() + (batch_idx * max_element_num + j) *
                                   kLaneSegsNum * kLaneLightDim);
    // type
    for (int ele_idx = 0; ele_idx < kLaneSegsNum; ++ele_idx) {
      const auto& one_hot =
          prediction::OneHotObjectType(feat.type[ele_idx], kLaneTypeDim);
      std::copy(
          one_hot.begin(), one_hot.end(),
          type->begin() +
              ((batch_idx * max_element_num + j) * kLaneSegsNum + ele_idx) *
                  kLaneTypeDim);
    }
    // dist
    std::copy(feat.dist.begin(), feat.dist.end(),
              dist->begin() + (batch_idx * max_element_num + j) * kLaneSegsNum);
    // nearest_to_end_dist
    std::copy(feat.nearest_to_end_dist.begin(), feat.nearest_to_end_dist.end(),
              nearest_to_end_dist->begin() +
                  (batch_idx * max_element_num + j) * kLaneSegsNum);
    // yaw_diff
    std::copy(feat.yaw_diff.begin(), feat.yaw_diff.end(),
              yaw_diff->begin() +
                  (batch_idx * max_element_num + j) * kLaneSegsNum * kCoords);
  }
}
}  // namespace

std::vector<std::vector<float>> CutinSLNetInferencer::GetBatchInputs(
    const NetParam& net_param,
    absl::Span<const prediction::CutinSLFeature> input_features) const {
  std::unordered_map<std::string, std::vector<float>> batch_map;
  const int num_objs = input_features.size();
  std::vector<std::string> input_names;
  input_names.reserve(net_param.input_tensor_list_size());
  for (const auto& input_tensor : net_param.input_tensor_list()) {
    const int data_size = std::accumulate(input_tensor.shape().begin() + 1,
                                          input_tensor.shape().end(), num_objs,
                                          std::multiplies<int>());
    batch_map[input_tensor.name()] = std::vector<float>(data_size, 0.0);
    input_names.push_back(input_tensor.name());
  }
  const int skipped_hist = prediction::kFeatureV2HistoryStepNum - kHistoryNum;
  // Agent
  auto& batch_agent_sl_pos = batch_map["agent_sl_pos"];
  auto& batch_agent_sl_speed = batch_map["agent_sl_speed"];
  auto& batch_agent_yaw_diff_dp = batch_map["agent_yaw_diff_dp"];
  auto& batch_agent_dist_to_llb =
      batch_map["agent_center_dist_to_av_left_lane_boundary"];
  auto& batch_agent_dist_to_rlb =
      batch_map["agent_center_dist_to_av_right_lane_boundary"];
  auto& batch_agent_length_width = batch_map["agent_length_width"];
  auto& batch_agent_sl_shape = batch_map["agent_sl_shape"];
  auto& batch_agent_type = batch_map["agent_type"];

  for (int i = 0; i < num_objs; ++i) {
    const auto& feat = input_features[i];
    // agent_sl_pos
    std::copy(feat.agent_sl_feature.sl_pos.begin() + skipped_hist * kCoords,
              feat.agent_sl_feature.sl_pos.end(),
              batch_agent_sl_pos.begin() + i * kHistoryNum * kCoords);
    // agent_sl_speed
    std::copy(feat.agent_sl_feature.sl_speed.begin() + skipped_hist * kCoords,
              feat.agent_sl_feature.sl_speed.end(),
              batch_agent_sl_speed.begin() + i * kHistoryNum * kCoords);
    // agent_yaw_diff_dp
    std::copy(
        feat.agent_sl_feature.yaw_diff_dp.begin() + skipped_hist * kCoords,
        feat.agent_sl_feature.yaw_diff_dp.end(),
        batch_agent_yaw_diff_dp.begin() + i * kHistoryNum * kCoords);
    // agent_dist_to_llb
    std::copy(
        feat.agent_sl_feature.agent_center_dist_to_av_left_lane_boundary
                .begin() +
            skipped_hist,
        feat.agent_sl_feature.agent_center_dist_to_av_left_lane_boundary.end(),
        batch_agent_dist_to_llb.begin() + i * kHistoryNum);
    // agent_dist_to_rlb
    std::copy(
        feat.agent_sl_feature.agent_center_dist_to_av_right_lane_boundary
                .begin() +
            skipped_hist,
        feat.agent_sl_feature.agent_center_dist_to_av_right_lane_boundary.end(),
        batch_agent_dist_to_rlb.begin() + i * kHistoryNum);
    // agent_length_width
    std::copy(feat.agent_sl_feature.length_width.begin(),
              feat.agent_sl_feature.length_width.end(),
              batch_agent_length_width.begin() + i * kCoords);
    // agent_sl_shape
    std::copy(
        feat.agent_sl_feature.sl_shape.begin() +
            skipped_hist * kShapetDim * kCoords,
        feat.agent_sl_feature.sl_shape.end(),
        batch_agent_sl_shape.begin() + i * kHistoryNum * kShapetDim * kCoords);
    // agent_type
    const auto& one_hot = prediction::OneHotObjectType(
        feat.agent_sl_feature.type, kObjectTypeDim);
    std::copy(one_hot.begin(), one_hot.end(),
              batch_agent_type.begin() + i * kObjectTypeDim);
  }
  // scale
  for (int k = 0; k < batch_agent_length_width.size() / 2; ++k) {
    batch_agent_length_width[2 * k] =
        std::clamp(batch_agent_length_width[2 * k] * kLengthScale, 0.0, 1.0);
    batch_agent_length_width[2 * k + 1] =
        std::clamp(batch_agent_length_width[2 * k + 1] * kWidthScale, 0.0, 1.0);
  }
  // Object
  auto& batch_obj_types = batch_map["obj_types"];
  auto& batch_obj_sl_pos = batch_map["obj_sl_pos"];
  auto& batch_obj_sl_speed = batch_map["obj_sl_speed"];
  auto& batch_obj_yaw_diff_dp = batch_map["obj_yaw_diff_dp"];
  auto& batch_obj_length_width = batch_map["obj_length_width"];
  auto& batch_obj_sl_shape = batch_map["obj_sl_shape"];
  auto& batch_rel_sl_pos_agent = batch_map["rel_sl_pos_agent"];
  auto& batch_yaw_diff_agent = batch_map["yaw_diff_agent"];
  auto& batch_rel_dist_agent = batch_map["rel_dist_agent"];
  auto& batch_rel_sl_speed_agent = batch_map["rel_sl_speed_agent"];
  // auto& batch_actor_valid_mask = batch_map["actor_valid_mask"];

  for (int i = 0; i < num_objs; ++i) {
    const auto& feat = input_features[i];
    // objects feature
    for (int j = 0; j < feat.objs_sl_features.size(); ++j) {
      auto& obj_feat = feat.objs_sl_features[j];
      // batch_obj_types
      const auto& one_hot =
          prediction::OneHotObjectType(obj_feat.type, kObjectTypeDim);
      std::copy(one_hot.begin(), one_hot.end(),
                batch_obj_types.begin() +
                    (i * kMaxOtherObjsNum + j) * kObjectTypeDim);
      // batch_obj_sl_pos
      std::copy(obj_feat.sl_pos.begin() + skipped_hist * kCoords,
                obj_feat.sl_pos.end(),
                batch_obj_sl_pos.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
      // batch_obj_sl_speed
      std::copy(obj_feat.sl_speed.begin() + skipped_hist * kCoords,
                obj_feat.sl_speed.end(),
                batch_obj_sl_speed.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
      // batch_obj_yaw_diff_dp
      std::copy(obj_feat.yaw_diff_dp.begin() + skipped_hist * kCoords,
                obj_feat.yaw_diff_dp.end(),
                batch_obj_yaw_diff_dp.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
      // batch_obj_length_width
      std::copy(obj_feat.length_width.begin(), obj_feat.length_width.end(),
                batch_obj_length_width.begin() +
                    (i * kMaxOtherObjsNum + j) * kCoords);
      // batch_obj_sl_shape
      std::copy(obj_feat.sl_shape.begin() + skipped_hist * kShapetDim * kCoords,
                obj_feat.sl_shape.end(),
                batch_obj_sl_shape.begin() + (i * kMaxOtherObjsNum + j) *
                                                 kHistoryNum * kShapetDim *
                                                 kCoords);
      // batch_rel_sl_pos_agent
      std::copy(obj_feat.rel_sl_pos_agent.begin() + skipped_hist * kCoords,
                obj_feat.rel_sl_pos_agent.end(),
                batch_rel_sl_pos_agent.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
      // batch_yaw_diff_agent
      std::copy(obj_feat.yaw_diff_agent.begin() + skipped_hist * kCoords,
                obj_feat.yaw_diff_agent.end(),
                batch_yaw_diff_agent.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
      // batch_rel_dist_agent
      std::copy(obj_feat.rel_dist_agent.begin() + skipped_hist,
                obj_feat.rel_dist_agent.end(),
                batch_rel_dist_agent.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum);
      // batch_rel_sl_speed_agent
      std::copy(obj_feat.rel_sl_speed_agent.begin() + skipped_hist * kCoords,
                obj_feat.rel_sl_speed_agent.end(),
                batch_rel_sl_speed_agent.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
    }
  }
  // scale
  for (int k = 0; k < batch_obj_length_width.size() * 0.5; ++k) {
    batch_obj_length_width[2 * k] =
        std::clamp(batch_obj_length_width[2 * k] * kLengthScale, 0.0, 1.0);
    batch_obj_length_width[2 * k + 1] =
        std::clamp(batch_obj_length_width[2 * k + 1] * kWidthScale, 0.0, 1.0);
  }

  // Map elements.
  for (int i = 0; i < num_objs; ++i) {
    const auto& feat = input_features[i];
    // lanecenter feature
    FillInMapElementFeature(
        feat.lc_sl_features, i, kMaxLaneCenterNum, &batch_map["lc_sl_seg"],
        &batch_map["lc_seg_len"], &batch_map["lc_light"], &batch_map["lc_type"],
        &batch_map["lc_dist"], &batch_map["lc_nearest_to_end_dist"],
        &batch_map["lc_yaw_diff"]);
    // lc valid mask
    std::copy(feat.lc_masks.begin(), feat.lc_masks.end(),
              batch_map["lc_valid_mask"].begin() + i * kMaxLaneCenterNum);

    // laneboundary feature
    FillInMapElementFeature(
        feat.lb_sl_features, i, kMaxLaneBoundaryNum, &batch_map["lb_sl_seg"],
        &batch_map["lb_seg_len"], &batch_map["lb_light"], &batch_map["lb_type"],
        &batch_map["lb_dist"], &batch_map["lb_nearest_to_end_dist"],
        &batch_map["lb_yaw_diff"]);
    // lb valid mask
    std::copy(feat.lb_masks.begin(), feat.lb_masks.end(),
              batch_map["lb_valid_mask"].begin() + i * kMaxLaneBoundaryNum);

    // crosswalk feature
    FillInMapElementFeature(
        feat.cw_sl_features, i, kMaxCrossWalkNum, &batch_map["cw_sl_seg"],
        &batch_map["cw_seg_len"], &batch_map["cw_light"], &batch_map["cw_type"],
        &batch_map["cw_dist"], &batch_map["cw_nearest_to_end_dist"],
        &batch_map["cw_yaw_diff"]);
    // cw valid mask
    std::copy(feat.cw_masks.begin(), feat.cw_masks.end(),
              batch_map["cw_valid_mask"].begin() + i * kMaxCrossWalkNum);
  }

  std::vector<std::vector<float>> batch;
  batch.resize(input_names.size());
  for (int i = 0; i < input_names.size(); ++i) {
    batch[i] = std::move(batch_map[input_names[i]]);
  }
  return batch;
}

CutinSLNetInferencer::CutinSLNetInferencer(const NetParam& net_param)
    : net_param_(net_param) {
#ifdef Q_CPU_ONLY
  const std::string& model_path = net_param.onnx_model_path();
  model_context_ = net_param.onnx_model_path();
#else
  std::string model_path;
  if (net_param.device_type() == NetParam_DeviceType_CPU) {
    model_path = net_param.onnx_model_path();
    model_context_ = net_param.onnx_model_path();
  } else {
    model_path = net_param.trt_model_path();
    model_context_ = net_param.trt_model_path();
  }
#endif
  // Fold directories into file for cloud storage.
  std::replace(model_context_.begin(), model_context_.end(), '/', '.');
  cutin_sl_net_ =
      std::make_unique<prediction::PredictionQNN>(model_path, net_param_);
}

// Run CutinNet model, and get the result.
prediction::CutinSLObjectsOut CutinSLNetInferencer::PredictForObjects(
    absl::Span<const prediction::CutinSLFeature> input_features) const {
  SCOPED_QTRACE("CutinSLNetinferencer::Prediction for Objects");
  std::vector<std::vector<float>> batch_inputs;
  CutinSLNetOut net_out;
  prediction::CutinSLObjectsOut objs_out;
  const int current_batch_size = input_features.size();

  // 1. Feature to Batch inputs.
  {
    SCOPED_QTRACE("CutinSLNetInferencer::GetBatchInputs");
    batch_inputs = GetBatchInputs(net_param_, input_features);
  }
  // 2. Run CutinSLNet forward.
  net_out = cutin_sl_net_->Run(batch_inputs, current_batch_size);

  // 3. Set objects
  {
    std::vector<std::pair<double, double>> channel_bucket = {
        std::make_pair(std::numeric_limits<double>::lowest(), -3),
        std::make_pair(-3, -1), std::make_pair(-1, 1), std::make_pair(1, 3),
        std::make_pair(1, std::numeric_limits<double>::max())};

    SCOPED_QTRACE("CutinSLNetInferencer::AssembleOutput");
    for (int i = 0; i < current_batch_size; ++i) {
      // Assemble channel probabilities
      const int batch_channel_size =
          net_out["channel_prob_out"].size() / current_batch_size;
      const std::vector<float> channel_probs(
          net_out["channel_prob_out"].begin() + i * batch_channel_size,
          net_out["channel_prob_out"].begin() + (i + 1) * batch_channel_size);
      auto channel_probs_softmax = SoftMax(channel_probs);
      const auto max_position_it = max_element(channel_probs_softmax.begin(),
                                               channel_probs_softmax.end());
      const auto cur_pos_l = input_features[i].agent_sl_feature.sl_pos.back();

      int cur_channel = 0;
      for (int j = 0; j < channel_bucket.size(); ++j) {
        if (cur_pos_l > channel_bucket[j].first &&
            cur_pos_l < channel_bucket[j].second) {
          cur_channel = j;
          break;
        }
      }
      int predicted_channel = max_position_it - channel_probs_softmax.begin();
      objs_out[input_features[i].agent_id] = prediction::CutinSLObjectOut({
          .channle_probs = std::move(channel_probs_softmax),
          .predicted_channel = predicted_channel,
          .cur_channel = cur_channel,
      });
    }
  }
  return objs_out;
}

}  // namespace cutin_sl_net
}  // namespace qcraft
