#include "onboard/prediction/inferencer/act_net_inferencer.h"

#include <stdint.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <map>
#include <numeric>
#include <ostream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "boost/functional/hash.hpp"
#include "boost/serialization/level_enum.hpp"
#include "boost/serialization/map.hpp"  // IWYU pragma: keep
#include "boost/serialization/tracking_enum.hpp"
#include "boost/serialization/vector.hpp"  // IWYU pragma: keep
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/simulation/sim_cache.h"

namespace qcraft {
namespace actnet {
namespace {
using ActNetOut = std::map<std::string, std::vector<float>>;
using prediction::AgentCentricObjectProbTraj;
using prediction::AgentCentricObjectProbTrajs;
using prediction::AgentCentricObjectsOut;
using prediction::AgentCentricObjectsProbTrajs;
using prediction::NLLTrajPoint;
using prediction::ObjectProbTrajs;
using prediction::ObjectsProbTrajs;
using prediction::ProbTrajPair;
void FillInMapElementFeature(
    absl::Span<const prediction::ActNetPolylineFeature> features, int batch_idx,
    int max_element_num, std::vector<float>* seg, std::vector<float>* seg_len,
    std::vector<float>* light, std::vector<float>* speed_limits,
    std::vector<float>* type, std::vector<float>* dist,
    std::vector<float>* nearest_to_end_dist, std::vector<float>* unit_dist_vec,
    std::vector<float>* unit_tangent) {
  for (int j = 0; j < features.size(); ++j) {
    const auto& feat = features[j];
    // seg
    std::copy(feat.seg.begin(), feat.seg.end(),
              seg->begin() +
                  (batch_idx * max_element_num + j) * kLaneSegsNum * kSegDim);
    // seg len
    std::copy(
        feat.seg_len.begin(), feat.seg_len.end(),
        seg_len->begin() + (batch_idx * max_element_num + j) * kLaneSegsNum);
    // light
    std::copy(feat.light.begin(), feat.light.end(),
              light->begin() + (batch_idx * max_element_num + j) *
                                   kLaneSegsNum * kLaneLightDim);
    // speed limits
    std::copy(feat.speed_limit_kph.begin(), feat.speed_limit_kph.end(),
              speed_limits->begin() +
                  (batch_idx * max_element_num + j) * kLaneSegsNum);
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

    // unit_dist_vec
    std::copy(feat.unit_dist_vec.begin(), feat.unit_dist_vec.end(),
              unit_dist_vec->begin() +
                  (batch_idx * max_element_num + j) * kLaneSegsNum * kCoords);
    // unit_tangent
    std::copy(feat.unit_tangent.begin(), feat.unit_tangent.end(),
              unit_tangent->begin() +
                  (batch_idx * max_element_num + j) * kLaneSegsNum * kCoords);
  }
}

std::vector<std::vector<float>> GetBatchInputs(
    const NetParam& net_param,
    absl::Span<const prediction::ActNetFeature> input_features) {
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
  auto& batch_agent_type = batch_map["agent_type"];
  auto& batch_agent_pos_diff = batch_map["agent_pos_diff"];
  auto& batch_agent_speed = batch_map["agent_speed"];
  auto& batch_agent_heading = batch_map["agent_heading"];
  auto& batch_agent_lw = batch_map["agent_lw"];
  auto& batch_agent_pos = batch_map["agent_pos"];
  auto& batch_agent_stop_info = batch_map["agent_stop_time_info"];
  for (int i = 0; i < num_objs; ++i) {
    const auto& feat = input_features[i];
    // agent_type
    const auto& one_hot =
        prediction::OneHotObjectType(feat.agent_feature.type, kObjectTypeDim);
    std::copy(one_hot.begin(), one_hot.end(),
              batch_agent_type.begin() + i * kObjectTypeDim);
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
    // batch_agent_lw
    std::copy(feat.agent_feature.lw.begin(), feat.agent_feature.lw.end(),
              batch_agent_lw.begin() + i * kCoords);
    // batch_agent_pos
    std::copy(feat.agent_feature.pos.begin() + skipped_hist * kCoords,
              feat.agent_feature.pos.end(),
              batch_agent_pos.begin() + i * kHistoryNum * kCoords);
    // batch_agent_stop_time_info
    std::copy(feat.stop_time_info.begin(), feat.stop_time_info.end(),
              batch_agent_stop_info.begin() + i * kStopInfoDim);
  }
  // scale
  for (int k = 0; k < batch_agent_lw.size() / 2; ++k) {
    batch_agent_lw[2 * k] = std::clamp(batch_agent_lw[2 * k] / 10.0, 0.0, 1.0);
    batch_agent_lw[2 * k + 1] =
        std::clamp(batch_agent_lw[2 * k + 1] / 3.0, 0.0, 1.0);
  }
  // Object
  auto& batch_obj_type = batch_map["obj_type"];
  auto& batch_obj_pos_diff = batch_map["obj_pos_diff"];
  auto& batch_obj_speed = batch_map["obj_speed"];
  auto& batch_obj_heading = batch_map["obj_heading"];
  auto& batch_obj_pos = batch_map["obj_pos"];
  auto& batch_obj_lw = batch_map["obj_lw"];
  auto& batch_rel_dist = batch_map["rel_dist"];
  auto& batch_rel_yaw = batch_map["rel_yaw"];
  auto& batch_rel_speed = batch_map["rel_speed"];
  auto& batch_actor_valid_mask = batch_map["actor_valid_mask"];
  for (int i = 0; i < num_objs; ++i) {
    const auto& feat = input_features[i];
    // objects feature, caution model may be less than feature
    for (int j = 0; j < feat.context_obj_features.size(); ++j) {
      auto& abs_feat = feat.context_obj_features[j].abs_feat;
      auto& rel_feat = feat.context_obj_features[j].rel_feat;
      // obj_type
      const auto& one_hot =
          prediction::OneHotObjectType(abs_feat.type, kObjectTypeDim);
      std::copy(
          one_hot.begin(), one_hot.end(),
          batch_obj_type.begin() + (i * kMaxOtherObjsNum + j) * kObjectTypeDim);
      // obj_pos_diff
      std::copy(abs_feat.pos_diff.begin() + skipped_hist * kCoords,
                abs_feat.pos_diff.end(),
                batch_obj_pos_diff.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
      // obj_speed
      std::copy(abs_feat.speed.begin() + skipped_hist * kCoords,
                abs_feat.speed.end(),
                batch_obj_speed.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
      // obj_heading
      std::copy(abs_feat.yaw.begin() + skipped_hist * kCoords,
                abs_feat.yaw.end(),
                batch_obj_heading.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
      // batch_obj_pos
      std::copy(abs_feat.pos.begin() + skipped_hist * kCoords,
                abs_feat.pos.end(),
                batch_obj_pos.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
      // batch_obj_lw
      std::copy(abs_feat.lw.begin(), abs_feat.lw.end(),
                batch_obj_lw.begin() + (i * kMaxOtherObjsNum + j) * kCoords);
      // obj_rel_dist
      std::copy(
          rel_feat.rel_dist.begin() + skipped_hist, rel_feat.rel_dist.end(),
          batch_rel_dist.begin() + (i * kMaxOtherObjsNum + j) * kHistoryNum);
      // obj_rel_yaw
      std::copy(rel_feat.rel_yaw.begin() + skipped_hist * kCoords,
                rel_feat.rel_yaw.end(),
                batch_rel_yaw.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
      // obj_rel_speed
      std::copy(rel_feat.rel_speed.begin() + skipped_hist * kCoords,
                rel_feat.rel_speed.end(),
                batch_rel_speed.begin() +
                    (i * kMaxOtherObjsNum + j) * kHistoryNum * kCoords);
    }
    // obj_valid_mask
    std::copy(feat.context_obj_masks.begin(), feat.context_obj_masks.end(),
              batch_actor_valid_mask.begin() + i * kMaxOtherObjsNum);
  }
  // scale
  for (int k = 0; k < batch_obj_lw.size() / 2; ++k) {
    batch_obj_lw[2 * k] = std::clamp(batch_obj_lw[2 * k] / 10.0, 0.0, 1.0);
    batch_obj_lw[2 * k + 1] =
        std::clamp(batch_obj_lw[2 * k + 1] / 3.0, 0.0, 1.0);
  }

  // Map elements.
  for (int i = 0; i < num_objs; ++i) {
    const auto& feat = input_features[i];
    // lanecenter feature
    FillInMapElementFeature(
        feat.lc_features, i, kMaxLaneCenterNum, &batch_map["lc_seg"],
        &batch_map["lc_seg_len"], &batch_map["lc_light"],
        &batch_map["lc_speed_limits"], &batch_map["lc_type"],
        &batch_map["lc_dist"], &batch_map["lc_nearest_to_end_dist"],
        &batch_map["lc_unit_dist_vec"], &batch_map["lc_unit_tangent"]);
    // lc valid mask
    std::copy(feat.lc_masks.begin(), feat.lc_masks.end(),
              batch_map["lc_valid_mask"].begin() + i * kMaxLaneCenterNum);

    // laneboundary feature
    FillInMapElementFeature(
        feat.solid_lb_features, i, kMaxLaneBoundaryNum, &batch_map["lb_seg"],
        &batch_map["lb_seg_len"], &batch_map["lb_light"],
        &batch_map["lb_speed_limits"], &batch_map["lb_type"],
        &batch_map["lb_dist"], &batch_map["lb_nearest_to_end_dist"],
        &batch_map["lb_unit_dist_vec"], &batch_map["lb_unit_tangent"]);
    // lb valid mask
    std::copy(feat.solid_lb_masks.begin(), feat.solid_lb_masks.end(),
              batch_map["lb_valid_mask"].begin() + i * kMaxLaneBoundaryNum);
    // crosswalk feature
    FillInMapElementFeature(
        feat.cw_features, i, kMaxCrossWalkNum, &batch_map["cw_seg"],
        &batch_map["cw_seg_len"], &batch_map["cw_light"],
        &batch_map["cw_speed_limits"], &batch_map["cw_type"],
        &batch_map["cw_dist"], &batch_map["cw_nearest_to_end_dist"],
        &batch_map["cw_unit_dist_vec"], &batch_map["cw_unit_tangent"]);
    // cw valid mask
    std::copy(feat.cw_masks.begin(), feat.cw_masks.end(),
              batch_map["cw_valid_mask"].begin() + i * kMaxCrossWalkNum);
  }

  // Move batch data to a vector of vector.
  std::vector<std::vector<float>> batch;
  batch.resize(input_names.size());
  for (int i = 0; i < input_names.size(); ++i) {
    batch[i] = std::move(batch_map[input_names[i]]);
  }
  return batch;
}
}  // namespace

ActNetInferencer::ActNetInferencer(const NetParam& net_param)
    : net_param_(net_param) {
  const std::string& model_path = net_param.onnx_model_path();
  // Note(runlin): When update sim cache IO must update key.
  model_context_ = net_param.onnx_model_path();
  // Fold directories into file for cloud storage.
  std::replace(model_context_.begin(), model_context_.end(), '/', '.');
  act_net_ =
      std::make_unique<prediction::PredictionQNN>(model_path, net_param_);
}

// Run ActNet model, and get the result.
AgentCentricObjectsOut ActNetInferencer::PredictForObjects(
    absl::Span<const prediction::ActNetFeature> input_features) const {
  SCOPED_QTRACE("ActNetinferencer::Prediction for Objects");
  std::vector<std::vector<float>> batch_inputs;
  ActNetOut net_out;
  AgentCentricObjectsOut objs_out;
  const int current_batch_size = input_features.size();
  if (current_batch_size == 0) {
    return objs_out;
  }
  // 1. Feature to Batch inputs.
  {
    SCOPED_QTRACE("ActNetinferencer::GetBatchInputs");
    batch_inputs = GetBatchInputs(net_param_, input_features);
  }
  // Currently only has a speed-up effect for CPU inference.
  if (SimCacheActive()) {
    QLOG_EVERY_N_SEC(INFO, 10) << "ActNet inferencer sim_cache is active";
    std::size_t hash_seed = batch_inputs.size();
    for (const auto& input : batch_inputs) {
      boost::hash_combine(hash_seed,
                          boost::hash_range(input.begin(), input.end()));
    }
    std::string cache_key = absl::StrFormat("%u", hash_seed);
    auto sim_cache =
        GlobalSimCache::GetSimCache({"ActNetCache"}, model_context_);
    if (!sim_cache->GetResult(cache_key, &net_out)) {
      QLOG(WARNING) << "Not hit cache_key: " << cache_key;
      int64_t penalty_start_time = absl::ToUnixMillis(absl::Now());
      net_out = act_net_->Run(batch_inputs, current_batch_size);
      sim_cache->SetResult(cache_key, net_out, penalty_start_time);
    }
  } else {
    // 2. Run ActNet forward.
    SCOPED_QTRACE("ActNetinferencer::ProcessInEngine");
    net_out = act_net_->Run(batch_inputs, current_batch_size);
  }

  // 3. Set objects
  const int traj_points_size = kFutureNum;
  {
    SCOPED_QTRACE("ActNetInferencer::AssembleProbTrajs");
    for (int i = 0; i < current_batch_size; ++i) {
      AgentCentricObjectProbTrajs object_prob_trajs;
      object_prob_trajs.resize(kTrajectoryNum);
      const int batch_traj_size =
          net_out["traj_points"].size() / current_batch_size;
      const std::vector<float> trajs(
          net_out["traj_points"].begin() + i * batch_traj_size,
          net_out["traj_points"].begin() + (i + 1) * batch_traj_size);
      std::array<double, 3> relation_probs;
      for (int rel_idx = 0; rel_idx < 3; ++rel_idx) {
        relation_probs[rel_idx] = net_out["relation_probs"][i * 3 + rel_idx];
      }
      for (int j = 0; j < kTrajectoryNum; ++j) {
        std::vector<NLLTrajPoint> traj_points;
        traj_points.reserve(traj_points_size);
        traj_points.reserve(traj_points_size);
        for (int k = 0; k < traj_points_size; ++k) {
          // Query multi dims in one dim array.
          int query_idx = j * traj_points_size * kOutCoords + k * kOutCoords;
          // (x, y) coord convert.
          Vec2d pt(trajs[query_idx], trajs[query_idx + 1]);
          pt = pt.Rotate(-input_features[i].rot_rad) +
               input_features[i].ref_position;
          traj_points.push_back(
              NLLTrajPoint{pt.x(), pt.y(), trajs[query_idx + 2],
                           trajs[query_idx + 3], trajs[query_idx + 4]});
        }

        // Assign probabilities.
        const float mode_prob = net_out["traj_probs"][i * kTrajectoryNum + j];

        object_prob_trajs[j] = AgentCentricObjectProbTraj({
            .mode_prob = mode_prob,
            .relation_probs = relation_probs,
            .traj_points = std::move(traj_points),
            // Angle between ac coord and smooth coord.
            .rot_rad = -input_features[i].rot_rad,
        });
      }
      // Sort prob_trajs by probability descending.
      std::sort(object_prob_trajs.begin(), object_prob_trajs.end(),
                [](const auto& a, const auto& b) {
                  return a.mode_prob > b.mode_prob;
                });

      objs_out[input_features[i].agent_id] = prediction::AgentCentricObjectOut({
          .prob_trajs = std::move(object_prob_trajs),
          .startup_prob = net_out["startup_prob"][i],
      });
    }
  }
  return objs_out;
}
}  // namespace actnet
}  // namespace qcraft
