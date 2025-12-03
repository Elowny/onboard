#include "onboard/planner/ml/captain_net/inference/captain_net_torch.h"

#include <stdint.h>  // for int64_t

#include <numeric>  // for accumulate
#include <string>   // for string
#include <utility>  // for move

#include <ATen/Functions.h>          // NOLINT
#include <ATen/TensorIndexing.h>     // NOLINT
#include <ATen/TensorOperators.h>    // NOLINT
#include <ATen/core/Dict.h>          // NOLINT
#include <ATen/core/Dict_inl.h>      // NOLINT
#include <ATen/core/TensorBody.h>    // NOLINT
#include <ATen/core/ivalue.h>        // NOLINT
#include <ATen/core/ivalue_inl.h>    // NOLINT
#include <c10/core/DeviceType.h>     // NOLINT
#include <c10/core/ScalarType.h>     // NOLINT
#include <c10/core/TensorOptions.h>  // NOLINT
#include <c10/util/intrusive_ptr.h>  // NOLINT

#include "torch/csrc/autograd/generated/variable_factories.h"  // for from_blob, arange, zeros, ones
#include "torch/csrc/jit/runtime/interpreter.h"    // for IValue
#include "torch/csrc/jit/serialization/import.h"   // for load
#include "torch/csrc/jit/serialization/pickler.h"  // for IValue
#include "torch/nn/functional/padding.h"           // for pad
#include "torch/nn/options/padding.h"              // for PadFuncOptions
#include "torch/optim/optimizer.h"                 // for Tensor
#include "torch/types.h"                           // for kFloat32

#include "onboard/global/trace.h"  // for SCOPED_QTRACE, ScopedTrace
#include "onboard/utils/lfs_access/lfs_access.h"  // for LFSAccess

namespace qcraft::planner::ml::captain_net {

CaptainNetTorch::CaptainNetTorch(const std::string& model_path,
                                 const bool use_gpu, const int gpu_id,
                                 const int warmup_times)
    : device_(use_gpu ? torch::Device(torch::kCUDA, gpu_id)
                      : torch::Device(torch::kCPU)) {
  InitModel(model_path, warmup_times);
}

std::unique_ptr<CaptainNet> CaptainNetTorch::Create(
    const std::string& model_path, const bool use_gpu, const int gpu_id,
    const int warmup_times) {
  return std::unique_ptr<CaptainNet>(
      new CaptainNetTorch(model_path, use_gpu, gpu_id, warmup_times));
}

void CaptainNetTorch::InitModel(const std::string& model_path,
                                const int warmup_times) {
  captain_net_ =
      torch::jit::load(qcraft::LFSAccess::GetLFSFilePath(model_path));
  captain_net_.to(device_);
  DummyInputInference(warmup_times);
}

void CaptainNetTorch::DummyInputInference(const int times) {
  std::vector<std::vector<std::vector<float>>> prob_out;
  std::vector<std::vector<std::vector<float>>> traj_out;
  std::vector<float> target_lane_tmp(kMaxTargetLaneSegNum *
                                     kMaxTargetLanePointNum * kCoords);
  const auto input_features = CaptainNetFeature{
      .agent_feature =
          AgentFeature{
              .agent_type = 1,
              .agent_lw = std::vector<float>(kCoords),
              .agent_pos = std::vector<float>(kHistoryNum * kCoords),
              .agent_pos_diff = std::vector<float>(kHistoryNum * kCoords),
              .agent_speed = std::vector<float>(kHistoryNum * kCoords),
              .agent_heading = std::vector<float>(kHistoryNum * kCoords),
          },
      .actors_feature =
          ActorsFeature{
              .obj_type = std::vector<int64_t>(kMaxObjectsNum * kActorTypeDim),
              .obj_lw = std::vector<float>(kMaxObjectsNum * kCoords),
              .obj_pos =
                  std::vector<float>(kMaxObjectsNum * kHistoryNum * kCoords),
              .obj_pos_diff =
                  std::vector<float>(kMaxObjectsNum * kHistoryNum * kCoords),
              .obj_speed =
                  std::vector<float>(kMaxObjectsNum * kHistoryNum * kCoords),
              .obj_heading =
                  std::vector<float>(kMaxObjectsNum * kHistoryNum * kCoords),
              .rel_dist = std::vector<float>(kMaxObjectsNum * kHistoryNum),
              .rel_pos =
                  std::vector<float>(kMaxObjectsNum * kHistoryNum * kCoords),
              .rel_yaw =
                  std::vector<float>(kMaxObjectsNum * kHistoryNum * kCoords),
              .rel_speed =
                  std::vector<float>(kMaxObjectsNum * kHistoryNum * kCoords),
              .valid_mask = std::vector<int>(kMaxObjectsNum + 1, 1),
          },
      .lane_center_feature =
          LanesFeature{
              .seg = std::vector<float>(kMaxLaneCenterNum * kLanePointsNum *
                                        kCoords * 2),
              .unit_seg_vec = std::vector<float>(kMaxLaneCenterNum *
                                                 kLanePointsNum * kCoords),
              .seg_len = std::vector<float>(kMaxLaneCenterNum * kLanePointsNum),
              .unit_tangent = std::vector<float>(kMaxLaneCenterNum *
                                                 kLanePointsNum * kCoords),
              .dist = std::vector<float>(kMaxLaneCenterNum * kLanePointsNum),
              .unit_dist_vec = std::vector<float>(kMaxLaneCenterNum *
                                                  kLanePointsNum * kCoords),
              .nearest_to_end_dist =
                  std::vector<float>(kMaxLaneCenterNum * kLanePointsNum),
              .speed_limits =
                  std::vector<float>(kMaxLaneCenterNum * kLanePointsNum),
              .yaw_diff = std::vector<float>(kMaxLaneCenterNum *
                                             kLanePointsNum * kCoords),
              .light = std::vector<float>(kMaxLaneCenterNum * kLanePointsNum *
                                          kMaxLightNum),
              .type = std::vector<int64_t>(kMaxLaneCenterNum * kLanePointsNum *
                                           kLaneTypeDim),
              .valid_mask = std::vector<int>(kMaxLaneCenterNum, 1),
          },
      .lane_boundary_feature =
          LanesFeature{
              .seg = std::vector<float>(kMaxLaneBoundaryNum * kLanePointsNum *
                                        kCoords * 2),
              .seg_len =
                  std::vector<float>(kMaxLaneBoundaryNum * kLanePointsNum),
              .unit_tangent = std::vector<float>(kMaxLaneBoundaryNum *
                                                 kLanePointsNum * kCoords),
              .dist = std::vector<float>(kMaxLaneBoundaryNum * kLanePointsNum),
              .unit_dist_vec = std::vector<float>(kMaxLaneBoundaryNum *
                                                  kLanePointsNum * kCoords),
              .nearest_to_end_dist =
                  std::vector<float>(kMaxLaneBoundaryNum * kLanePointsNum),
              .speed_limits =
                  std::vector<float>(kMaxLaneBoundaryNum * kLanePointsNum),
              .light = std::vector<float>(kMaxLaneBoundaryNum * kLanePointsNum *
                                          kMaxLightNum),
              .type = std::vector<int64_t>(kMaxLaneBoundaryNum *
                                           kLanePointsNum * kLaneTypeDim),
              .valid_mask = std::vector<int>(kMaxLaneBoundaryNum, 1),
          },
      .crosswalk_feature =
          LanesFeature{
              .seg = std::vector<float>(kMaxCrosswalkNum * kLanePointsNum *
                                        kCoords * 2),
              .seg_len = std::vector<float>(kMaxCrosswalkNum * kLanePointsNum),
              .unit_tangent = std::vector<float>(kMaxCrosswalkNum *
                                                 kLanePointsNum * kCoords),
              .dist = std::vector<float>(kMaxCrosswalkNum * kLanePointsNum),
              .unit_dist_vec = std::vector<float>(kMaxCrosswalkNum *
                                                  kLanePointsNum * kCoords),
              .nearest_to_end_dist =
                  std::vector<float>(kMaxCrosswalkNum * kLanePointsNum),
              .speed_limits =
                  std::vector<float>(kMaxCrosswalkNum * kLanePointsNum),
              .light = std::vector<float>(kMaxCrosswalkNum * kLanePointsNum *
                                          kMaxLightNum),
              .type = std::vector<int64_t>(kMaxCrosswalkNum * kLanePointsNum *
                                           kLaneTypeDim),
              .valid_mask = std::vector<int>(kMaxCrosswalkNum, 1),
          },
      .target_lane_feature =
          TargetLanesFeature{
              .target_lanes = std::vector<std::vector<float>>(
                  kMaxTargetLanesNum, target_lane_tmp),
              .valid_mask = std::vector<int>(kMaxTargetLanesNum, 1),
          },
  };
  for (int i = 0; i < times; ++i) {
    GetOutputs(input_features, &prob_out, &traj_out);
  }
}

void CaptainNetTorch::UnitTest() {}

// TODO(jingqiao): fix clang issue function exceeds recommended size/complexity
// thresholds [readability-function-size,-warnings-as-errors] NOLINTNEXTLINE
bool CaptainNetTorch::GetOutputs(
    const CaptainNetFeature& input_features,
    std::vector<std::vector<std::vector<float>>>* prob_out,
    std::vector<std::vector<std::vector<float>>>* traj_out) {
  // Process input
  torch::Tensor agent_type_tensor, agent_lw_tensor, agent_pos_tensor,
      agent_pos_diff_tensor, agent_speed_tensor, agent_heading_tensor,
      obj_type_tensor, obj_lw_tensor, obj_pos_tensor, obj_pos_diff_tensor,
      obj_speed_tensor, obj_heading_tensor, rel_dist_tensor, rel_yaw_tensor,
      rel_speed_tensor, actor_valid_mask_tensor, lane_center_seg_tensor,
      lane_center_seg_len_tensor, lane_center_unit_tangent_tensor,
      lane_center_dist_tensor, lane_center_unit_dist_vec_tensor,
      lane_center_nearest_to_end_dist_tensor, lane_center_speed_limits_tensor,
      lane_center_light_tensor, lane_center_type_tensor,
      lane_center_valid_mask_tensor, lane_boundary_seg_tensor,
      lane_boundary_seg_len_tensor, lane_boundary_unit_tangent_tensor,
      lane_boundary_dist_tensor, lane_boundary_unit_dist_vec_tensor,
      lane_boundary_nearest_to_end_dist_tensor,
      lane_boundary_speed_limits_tensor, lane_boundary_light_tensor,
      lane_boundary_type_tensor, lane_boundary_valid_mask_tensor,
      crosswalk_seg_tensor, crosswalk_seg_len_tensor,
      crosswalk_unit_tangent_tensor, crosswalk_dist_tensor,
      crosswalk_unit_dist_vec_tensor, crosswalk_nearest_to_end_dist_tensor,
      crosswalk_speed_limits_tensor, crosswalk_light_tensor,
      crosswalk_type_tensor, crosswalk_valid_mask_tensor;
  // Build input features for torch

  std::vector<torch::jit::IValue> torch_inputs;
  {
    SCOPED_QTRACE("CaptainNetTorch::Preprocess");
    // Note: here use const_cast hack to allow create tensor from blob.
    auto options = torch::TensorOptions().dtype(torch::kFloat32);

    const auto& agent = input_features.agent_feature;

    agent_type_tensor = torch::zeros({kActorTypeDim}, options);
    agent_type_tensor[agent.agent_type] = 1.0;
    agent_type_tensor = agent_type_tensor.unsqueeze_(0);

    agent_lw_tensor =
        torch::from_blob(const_cast<float*>(agent.agent_lw.data()), {kCoords},
                         options)
            .unsqueeze_(0);

    agent_pos_tensor =
        torch::from_blob(const_cast<float*>(agent.agent_pos.data()),
                         {kHistoryNum, kCoords}, options)
            .unsqueeze_(0);

    agent_pos_diff_tensor =
        torch::from_blob(const_cast<float*>(agent.agent_pos_diff.data()),
                         {kHistoryNum, kCoords}, options)
            .unsqueeze_(0);

    agent_speed_tensor =
        torch::from_blob(const_cast<float*>(agent.agent_speed.data()),
                         {kHistoryNum, kCoords}, options)
            .unsqueeze_(0);

    agent_heading_tensor =
        torch::from_blob(const_cast<float*>(agent.agent_heading.data()),
                         {kHistoryNum, kCoords}, options)
            .unsqueeze_(0);

    c10::Dict<std::string, torch::Tensor> agent_input;
    agent_input.insert("agent_type", agent_type_tensor.to(device_));
    agent_input.insert("agent_lw", agent_lw_tensor.to(device_));
    agent_input.insert("agent_pos", agent_pos_tensor.to(device_));
    agent_input.insert("agent_pos_diff", agent_pos_diff_tensor.to(device_));
    agent_input.insert("agent_speed", agent_speed_tensor.to(device_));
    agent_input.insert("agent_heading", agent_heading_tensor.to(device_));
    torch_inputs.push_back(agent_input);

    const auto& actors = input_features.actors_feature;
    const int valid_object_num =
        std::accumulate(actors.valid_mask.begin(), actors.valid_mask.end(), 0);
    const int object_pad_num = kMaxObjectsNum + 1 - valid_object_num;

    torch::Tensor original_obj_type_tensor =
        torch::from_blob(const_cast<int64_t*>(actors.obj_type.data()),
                         {valid_object_num},
                         torch::TensorOptions().dtype(torch::kLong))
            .slice(0, 1, valid_object_num);
    obj_type_tensor = torch::zeros({kMaxObjectsNum, kActorTypeDim}, options);
    obj_type_tensor
        .index_put_({torch::arange(0, original_obj_type_tensor.size(0)),
                     original_obj_type_tensor},
                    1.0)
        .unsqueeze_(0);

    obj_lw_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(actors.obj_lw.data()),
                             {valid_object_num, kCoords}, options)
                .slice(0, 1, valid_object_num),
            torch::nn::functional::PadFuncOptions({0, 0, 0, object_pad_num}))
            .unsqueeze_(0);

    obj_pos_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(actors.obj_pos.data()),
                             {valid_object_num, kHistoryNum, kCoords}, options)
                .slice(0, 1, valid_object_num),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, object_pad_num}))
            .unsqueeze_(0);

    obj_pos_diff_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(actors.obj_pos_diff.data()),
                             {valid_object_num, kHistoryNum, kCoords}, options)
                .slice(0, 1, valid_object_num),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, object_pad_num}))
            .unsqueeze_(0);

    obj_speed_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(actors.obj_speed.data()),
                             {valid_object_num, kHistoryNum, kCoords}, options)
                .slice(0, 1, valid_object_num),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, object_pad_num}))
            .unsqueeze_(0);

    obj_heading_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(actors.obj_heading.data()),
                             {valid_object_num, kHistoryNum, kCoords}, options)
                .slice(0, 1, valid_object_num),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, object_pad_num}))
            .unsqueeze_(0);

    rel_dist_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(actors.rel_dist.data()),
                             {valid_object_num, kHistoryNum}, options)
                .slice(0, 1, valid_object_num),
            torch::nn::functional::PadFuncOptions({0, 0, 0, object_pad_num}))
            .unsqueeze_(0);
    torch::Tensor rel_pos_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(actors.rel_pos.data()),
                             {valid_object_num, kHistoryNum, kCoords}, options)
                .slice(0, 1, valid_object_num),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, object_pad_num}))
            .unsqueeze_(0);
    rel_yaw_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(actors.rel_yaw.data()),
                             {valid_object_num, kHistoryNum, kCoords}, options)
                .slice(0, 1, valid_object_num),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, object_pad_num}))
            .unsqueeze_(0);
    rel_speed_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(actors.rel_speed.data()),
                             {valid_object_num, kHistoryNum, kCoords}, options)
                .slice(0, 1, valid_object_num),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, object_pad_num}))
            .unsqueeze_(0);
    actor_valid_mask_tensor =
        torch::from_blob(const_cast<int*>(actors.valid_mask.data()),
                         {kMaxObjectsNum + 1},
                         torch::TensorOptions().dtype(torch::kInt))
            .slice(0, 1, kMaxObjectsNum + 1)
            .unsqueeze_(0);

    c10::Dict<std::string, torch::Tensor> actor_input;
    actor_input.insert("obj_type", obj_type_tensor.to(device_));
    actor_input.insert("obj_lw", obj_lw_tensor.to(device_));
    actor_input.insert("obj_pos", obj_pos_tensor.to(device_));
    actor_input.insert("obj_pos_diff", obj_pos_diff_tensor.to(device_));
    actor_input.insert("obj_speed", obj_speed_tensor.to(device_));
    actor_input.insert("obj_heading", obj_heading_tensor.to(device_));
    actor_input.insert("rel_dist", rel_dist_tensor.to(device_));
    actor_input.insert("rel_pos", rel_pos_tensor.to(device_));
    actor_input.insert("rel_yaw", rel_yaw_tensor.to(device_));
    actor_input.insert("rel_speed", rel_speed_tensor.to(device_));
    actor_input.insert("actor_valid_mask", actor_valid_mask_tensor.to(device_));
    torch_inputs.push_back(actor_input);

    const auto& lane_center = input_features.lane_center_feature;
    const int valid_lane_center_num = std::accumulate(
        lane_center.valid_mask.begin(), lane_center.valid_mask.end(), 0);
    const int lane_center_pad_num = kMaxLaneCenterNum - valid_lane_center_num;

    lane_center_seg_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_center.seg.data()),
                {valid_lane_center_num, kLanePointsNum, kCoords * 2}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, lane_center_pad_num}))
            .unsqueeze_(0);

    torch::Tensor lane_center_unit_seg_vec_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_center.unit_seg_vec.data()),
                {valid_lane_center_num, kLanePointsNum, kCoords}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, lane_center_pad_num}))
            .unsqueeze_(0);

    lane_center_seg_len_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(lane_center.seg_len.data()),
                             {valid_lane_center_num, kLanePointsNum}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, lane_center_pad_num}))
            .unsqueeze_(0);

    lane_center_unit_tangent_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_center.unit_tangent.data()),
                {valid_lane_center_num, kLanePointsNum, kCoords}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, lane_center_pad_num}))
            .unsqueeze_(0);

    lane_center_dist_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(lane_center.dist.data()),
                             {valid_lane_center_num, kLanePointsNum}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, lane_center_pad_num}))
            .unsqueeze_(0);

    lane_center_unit_dist_vec_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_center.unit_dist_vec.data()),
                {valid_lane_center_num, kLanePointsNum, kCoords}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, lane_center_pad_num}))
            .unsqueeze_(0);

    lane_center_nearest_to_end_dist_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_center.nearest_to_end_dist.data()),
                {valid_lane_center_num, kLanePointsNum}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, lane_center_pad_num}))
            .unsqueeze_(0);

    lane_center_speed_limits_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_center.speed_limits.data()),
                {valid_lane_center_num, kLanePointsNum}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, lane_center_pad_num}))
            .unsqueeze_(0);

    torch::Tensor lane_center_yaw_diff_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(lane_center.yaw_diff.data()),
                             {valid_lane_center_num, kLanePointsNum, kCoords},
                             options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, lane_center_pad_num}))
            .unsqueeze_(0);

    lane_center_light_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_center.light.data()),
                {valid_lane_center_num, kLanePointsNum, kMaxLightNum}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, lane_center_pad_num}))
            .unsqueeze_(0);

    torch::Tensor original_lane_center_type_tensor =
        torch::from_blob(const_cast<int64_t*>(lane_center.type.data()),
                         {valid_lane_center_num, kLanePointsNum},
                         torch::TensorOptions().dtype(torch::kLong));

    lane_center_type_tensor = torch::zeros(
        {kMaxLaneCenterNum, kLanePointsNum, kLaneTypeDim}, options);
    lane_center_type_tensor
        .index_put_(
            {torch::arange(0, valid_lane_center_num)
                 .repeat_interleave(kLanePointsNum),
             torch::arange(0, kLanePointsNum).repeat(valid_lane_center_num),
             original_lane_center_type_tensor.flatten()},
            1.0)
        .unsqueeze_(0);

    lane_center_valid_mask_tensor =
        torch::from_blob(const_cast<int*>(lane_center.valid_mask.data()),
                         {kMaxLaneCenterNum},
                         torch::TensorOptions().dtype(torch::kInt))
            .unsqueeze_(0);

    c10::Dict<std::string, torch::Tensor> lane_center_input;
    lane_center_input.insert("lc_seg", lane_center_seg_tensor.to(device_));
    lane_center_input.insert("lc_unit_seg_vec",
                             lane_center_unit_seg_vec_tensor.to(device_));
    lane_center_input.insert("lc_seg_len",
                             lane_center_seg_len_tensor.to(device_));
    lane_center_input.insert("lc_unit_tangent",
                             lane_center_unit_tangent_tensor.to(device_));
    lane_center_input.insert("lc_dist", lane_center_dist_tensor.to(device_));
    lane_center_input.insert("lc_unit_dist_vec",
                             lane_center_unit_dist_vec_tensor.to(device_));
    lane_center_input.insert(
        "lc_nearest_to_end_dist",
        lane_center_nearest_to_end_dist_tensor.to(device_));
    lane_center_input.insert("lc_speed_limits",
                             lane_center_speed_limits_tensor.to(device_));
    lane_center_input.insert("lc_yaw_diff",
                             lane_center_yaw_diff_tensor.to(device_));
    lane_center_input.insert("lc_light", lane_center_light_tensor.to(device_));
    lane_center_input.insert("lc_type", lane_center_type_tensor.to(device_));
    lane_center_input.insert("lc_valid_mask",
                             lane_center_valid_mask_tensor.to(device_));
    torch_inputs.push_back(lane_center_input);

    const auto& lane_boundary = input_features.lane_boundary_feature;
    const int valid_lane_boundary_num = std::accumulate(
        lane_boundary.valid_mask.begin(), lane_boundary.valid_mask.end(), 0);
    const int lane_boundary_pad_num =
        kMaxLaneBoundaryNum - valid_lane_boundary_num;

    lane_boundary_seg_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_boundary.seg.data()),
                {valid_lane_boundary_num, kLanePointsNum, kCoords * 2},
                options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, lane_boundary_pad_num}))
            .unsqueeze_(0);
    lane_boundary_seg_len_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(lane_boundary.seg_len.data()),
                             {valid_lane_boundary_num, kLanePointsNum},
                             options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, lane_boundary_pad_num}))
            .unsqueeze_(0);

    lane_boundary_unit_tangent_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_boundary.unit_tangent.data()),
                {valid_lane_boundary_num, kLanePointsNum, kCoords}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, lane_boundary_pad_num}))
            .unsqueeze_(0);

    lane_boundary_dist_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(lane_boundary.dist.data()),
                             {valid_lane_boundary_num, kLanePointsNum},
                             options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, lane_boundary_pad_num}))
            .unsqueeze_(0);

    lane_boundary_unit_dist_vec_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_boundary.unit_dist_vec.data()),
                {valid_lane_boundary_num, kLanePointsNum, kCoords}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, lane_boundary_pad_num}))
            .unsqueeze_(0);

    lane_boundary_nearest_to_end_dist_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_boundary.nearest_to_end_dist.data()),
                {valid_lane_boundary_num, kLanePointsNum}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, lane_boundary_pad_num}))
            .unsqueeze_(0);

    lane_boundary_speed_limits_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_boundary.speed_limits.data()),
                {valid_lane_boundary_num, kLanePointsNum}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, lane_boundary_pad_num}))
            .unsqueeze_(0);
    lane_boundary_light_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(lane_boundary.light.data()),
                {valid_lane_boundary_num, kLanePointsNum, kMaxLightNum},
                options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, lane_boundary_pad_num}))
            .unsqueeze_(0);

    torch::Tensor original_lane_boundary_type_tensor =
        torch::from_blob(const_cast<int64_t*>(lane_boundary.type.data()),
                         {valid_lane_boundary_num, kLanePointsNum},
                         torch::TensorOptions().dtype(torch::kLong));

    lane_boundary_type_tensor = torch::zeros(
        {kMaxLaneBoundaryNum, kLanePointsNum, kLaneTypeDim}, options);
    lane_boundary_type_tensor
        .index_put_(
            {torch::arange(0, valid_lane_boundary_num)
                 .repeat_interleave(kLanePointsNum),
             torch::arange(0, kLanePointsNum).repeat(valid_lane_boundary_num),
             original_lane_boundary_type_tensor.flatten()},
            1.0)
        .unsqueeze_(0);

    lane_boundary_valid_mask_tensor =
        torch::from_blob(const_cast<int*>(lane_boundary.valid_mask.data()),
                         {kMaxLaneBoundaryNum},
                         torch::TensorOptions().dtype(torch::kInt))
            .unsqueeze_(0);

    c10::Dict<std::string, torch::Tensor> lane_boundary_input;
    lane_boundary_input.insert("lb_seg", lane_boundary_seg_tensor.to(device_));
    lane_boundary_input.insert("lb_seg_len",
                               lane_boundary_seg_len_tensor.to(device_));
    lane_boundary_input.insert("lb_unit_tangent",
                               lane_boundary_unit_tangent_tensor.to(device_));
    lane_boundary_input.insert("lb_dist",
                               lane_boundary_dist_tensor.to(device_));
    lane_boundary_input.insert("lb_unit_dist_vec",
                               lane_boundary_unit_dist_vec_tensor.to(device_));
    lane_boundary_input.insert(
        "lb_nearest_to_end_dist",
        lane_boundary_nearest_to_end_dist_tensor.to(device_));
    lane_boundary_input.insert("lb_speed_limits",
                               lane_boundary_speed_limits_tensor.to(device_));
    lane_boundary_input.insert("lb_light",
                               lane_boundary_light_tensor.to(device_));
    lane_boundary_input.insert("lb_type",
                               lane_boundary_type_tensor.to(device_));
    lane_boundary_input.insert("lb_valid_mask",
                               lane_boundary_valid_mask_tensor.to(device_));
    // torch_inputs.push_back(lane_boundary_input);

    const auto& crosswalk = input_features.crosswalk_feature;
    const int valid_crosswalk_num = std::accumulate(
        crosswalk.valid_mask.begin(), crosswalk.valid_mask.end(), 0);
    const int crosswalk_pad_num = kMaxCrosswalkNum - valid_crosswalk_num;

    crosswalk_seg_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(crosswalk.seg.data()),
                             {valid_crosswalk_num, kLanePointsNum, kCoords * 2},
                             options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, crosswalk_pad_num}))
            .unsqueeze_(0);
    crosswalk_seg_len_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(crosswalk.seg_len.data()),
                             {valid_crosswalk_num, kLanePointsNum}, options),
            torch::nn::functional::PadFuncOptions({0, 0, 0, crosswalk_pad_num}))
            .unsqueeze_(0);

    crosswalk_unit_tangent_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(crosswalk.unit_tangent.data()),
                             {valid_crosswalk_num, kLanePointsNum, kCoords},
                             options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, crosswalk_pad_num}))
            .unsqueeze_(0);

    crosswalk_dist_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(crosswalk.dist.data()),
                             {valid_crosswalk_num, kLanePointsNum}, options),
            torch::nn::functional::PadFuncOptions({0, 0, 0, crosswalk_pad_num}))
            .unsqueeze_(0);

    crosswalk_unit_dist_vec_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(crosswalk.unit_dist_vec.data()),
                             {valid_crosswalk_num, kLanePointsNum, kCoords},
                             options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, crosswalk_pad_num}))
            .unsqueeze_(0);

    crosswalk_nearest_to_end_dist_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(crosswalk.nearest_to_end_dist.data()),
                {valid_crosswalk_num, kLanePointsNum}, options),
            torch::nn::functional::PadFuncOptions({0, 0, 0, crosswalk_pad_num}))
            .unsqueeze_(0);

    crosswalk_speed_limits_tensor =
        torch::nn::functional::pad(
            torch::from_blob(const_cast<float*>(crosswalk.speed_limits.data()),
                             {valid_crosswalk_num, kLanePointsNum}, options),
            torch::nn::functional::PadFuncOptions({0, 0, 0, crosswalk_pad_num}))
            .unsqueeze_(0);
    crosswalk_light_tensor =
        torch::nn::functional::pad(
            torch::from_blob(
                const_cast<float*>(crosswalk.light.data()),
                {valid_crosswalk_num, kLanePointsNum, kMaxLightNum}, options),
            torch::nn::functional::PadFuncOptions(
                {0, 0, 0, 0, 0, crosswalk_pad_num}))
            .unsqueeze_(0);

    torch::Tensor original_crosswalk_type_tensor =
        torch::from_blob(const_cast<int64_t*>(crosswalk.type.data()),
                         {valid_crosswalk_num, kLanePointsNum},
                         torch::TensorOptions().dtype(torch::kLong));

    crosswalk_type_tensor =
        torch::zeros({kMaxCrosswalkNum, kLanePointsNum, kLaneTypeDim}, options);
    crosswalk_type_tensor
        .index_put_(
            {torch::arange(0, valid_crosswalk_num)
                 .repeat_interleave(kLanePointsNum),
             torch::arange(0, kLanePointsNum).repeat(valid_crosswalk_num),
             original_crosswalk_type_tensor.flatten()},
            1.0)
        .unsqueeze_(0);

    crosswalk_valid_mask_tensor =
        torch::from_blob(const_cast<int*>(crosswalk.valid_mask.data()),
                         {kMaxCrosswalkNum},
                         torch::TensorOptions().dtype(torch::kInt))
            .unsqueeze_(0);

    c10::Dict<std::string, torch::Tensor> crosswalk_input;
    crosswalk_input.insert("cw_seg", crosswalk_seg_tensor.to(device_));
    crosswalk_input.insert("cw_seg_len", crosswalk_seg_len_tensor.to(device_));
    crosswalk_input.insert("cw_unit_tangent",
                           crosswalk_unit_tangent_tensor.to(device_));
    crosswalk_input.insert("cw_dist", crosswalk_dist_tensor.to(device_));
    crosswalk_input.insert("cw_unit_dist_vec",
                           crosswalk_unit_dist_vec_tensor.to(device_));
    crosswalk_input.insert("cw_nearest_to_end_dist",
                           crosswalk_nearest_to_end_dist_tensor.to(device_));
    crosswalk_input.insert("cw_speed_limits",
                           crosswalk_speed_limits_tensor.to(device_));
    crosswalk_input.insert("cw_light", crosswalk_light_tensor.to(device_));
    crosswalk_input.insert("cw_type", crosswalk_type_tensor.to(device_));
    crosswalk_input.insert("cw_valid_mask",
                           crosswalk_valid_mask_tensor.to(device_));
    // torch_inputs.push_back(crosswalk_input);

    const auto& target_lane = input_features.target_lane_feature;

    torch::Tensor target_lane_data_tensor;
    torch::Tensor flattened_mask_tensor =
        torch::zeros({kMaxTargetLanesNum, kMaxTargetLaneSegNum},
                     torch::TensorOptions().dtype(torch::kInt));
    for (int i = 0; i < kMaxTargetLanesNum; ++i) {
      if (i < target_lane.target_lanes.size()) {
        torch::Tensor single_target_lane_data_tensor =
            torch::from_blob(
                const_cast<float*>(target_lane.target_lanes[i].data()),
                {static_cast<int>(target_lane.target_lanes[i].size() /
                                  (kMaxTargetLanePointNum * kCoords)),
                 kMaxTargetLanePointNum, kCoords},
                options)
                .slice(1, 0, kMaxTargetLanePointNum, 4);
        const int target_lane_seg_pad_num =
            kMaxTargetLaneSegNum - single_target_lane_data_tensor.size(0);
        flattened_mask_tensor.index_put_(
            {i,
             torch::indexing::Slice({torch::indexing::None,
                                     single_target_lane_data_tensor.size(0)})},
            1);
        if (target_lane_seg_pad_num > 0) {
          single_target_lane_data_tensor =
              torch::nn::functional::pad(
                  single_target_lane_data_tensor,
                  torch::nn::functional::PadFuncOptions(
                      {0, 0, 0, 0, 0, target_lane_seg_pad_num}))
                  .unsqueeze_(0);
        } else {
          single_target_lane_data_tensor =
              single_target_lane_data_tensor.slice(0, 0, kMaxTargetLaneSegNum)
                  .unsqueeze_(0);
        }
        if (i == 0) {
          target_lane_data_tensor = single_target_lane_data_tensor;
        } else {
          target_lane_data_tensor = torch::cat(
              {target_lane_data_tensor, single_target_lane_data_tensor}, 0);
        }
      } else {
        target_lane_data_tensor = torch::nn::functional::pad(
            target_lane_data_tensor,
            torch::nn::functional::PadFuncOptions({0, 0, 0, 0, 0, 0, 0, 1}));
      }
    }
    flattened_mask_tensor =
        torch::cat({torch::ones({1}, torch::TensorOptions().dtype(torch::kInt)),
                    flattened_mask_tensor.flatten()},
                   0)
            .unsqueeze_(0);
    target_lane_data_tensor = target_lane_data_tensor.unsqueeze_(0);
    torch::Tensor target_lane_valid_mask_tensor =
        torch::from_blob(const_cast<int*>(target_lane.valid_mask.data()),
                         {kMaxTargetLanesNum},
                         torch::TensorOptions().dtype(torch::kInt))
            .unsqueeze_(0);

    c10::Dict<std::string, torch::Tensor> target_lane_input;
    target_lane_input.insert("target_lane_data",
                             target_lane_data_tensor.to(device_));
    target_lane_input.insert("tl_valid_mask",
                             target_lane_valid_mask_tensor.to(device_));
    target_lane_input.insert("tl_flattened_mask",
                             flattened_mask_tensor.to(device_));
    torch_inputs.push_back(target_lane_input);
  }

  // Inference
  c10::IValue model_output;
  {
    SCOPED_QTRACE("CaptainNetTorch::Infer");
    model_output = captain_net_.forward(torch_inputs);
  }

  // Get outputs
  {
    SCOPED_QTRACE("CaptainNetTorch::Postprocess");
    const auto& elements = model_output.toTuple()->elements();
    const auto& cls_tensor = elements[0].toTensor().to(torch::kCPU);
    const auto& reg_tensor = elements[1].toTensor().to(torch::kCPU);

    const int traj_out_size =
        static_cast<int>(kPredictSec / kTimeStep) * kOutputPointDim;
    const int lane_num = kMaxTargetLanesNum;
    prob_out->resize(lane_num);
    traj_out->resize(lane_num);
    for (int i = 0; i < lane_num; ++i) {
      (*prob_out)[i].resize(kModes);
      (*traj_out)[i].resize(kModes);
      for (int j = 0; j < kModes; ++j) {
        (*prob_out)[i][j].assign(
            cls_tensor.data_ptr<float>() + i * kModes + j,
            cls_tensor.data_ptr<float>() + i * kModes + j + 1);
        (*traj_out)[i][j].assign(
            reg_tensor.data_ptr<float>() + i * kModes * traj_out_size +
                j * traj_out_size,
            reg_tensor.data_ptr<float>() + i * kModes * traj_out_size +
                (j + 1) * traj_out_size);
      }
    }
  }
  return true;
}  // NOLINT

}  // namespace qcraft::planner::ml::captain_net
