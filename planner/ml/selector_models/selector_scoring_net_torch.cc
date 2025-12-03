#include "onboard/planner/ml/selector_models/selector_scoring_net_torch.h"

#include <string>

#include "onboard/global/trace.h"
#include "onboard/utils/lfs_access/lfs_access.h"

namespace qcraft {
namespace selector_scoring_net {

SelectorScoringNetTorch::SelectorScoringNetTorch(const std::string& model_path,
                                                 const bool use_gpu,
                                                 const int gpu_id,
                                                 const int warmup_times)
    : device_(use_gpu ? torch::Device(torch::kCUDA, gpu_id)
                      : torch::Device(torch::kCPU)) {
  InitModel(model_path, warmup_times);
}

std::unique_ptr<SelectorScoringNet> SelectorScoringNetTorch::Create(
    const std::string& model_path, const bool use_gpu, const int gpu_id,
    const int warmup_times) {
  return std::unique_ptr<SelectorScoringNet>(
      new SelectorScoringNetTorch(model_path, use_gpu, gpu_id, warmup_times));
}

void SelectorScoringNetTorch::InitModel(const std::string& model_path,
                                        const int warmup_times) {
  selector_scoring_net_ =
      torch::jit::load(qcraft::LFSAccess::GetLFSFilePath(model_path));
  selector_scoring_net_.to(device_);
  DummyInputInference(warmup_times);
}

void SelectorScoringNetTorch::DummyInputInference(const int times) {
  const auto input_features = SelectorScoringNetFeature{
      .actors_feature =
          ActorsFeature{
              .trajs =
                  std::vector<float>(kMaxObjectsNum * kHistoryNum * kCoords),
              .speeds =
                  std::vector<float>(kMaxObjectsNum * kHistoryNum * kCoords),
              .headings =
                  std::vector<float>(kMaxObjectsNum * kHistoryNum * kCoords),
              .types = std::vector<float>(kMaxObjectsNum, 0),
              .cur_poses = std::vector<float>(kMaxObjectsNum * kCoords),
              .mask = std::vector<int>(kMaxObjectsNum, 1),
          },
      .lanes_feature =
          LanesFeature{
              .lane_centers = std::vector<float>(
                  kMaxLanesNum * (kLanePointsNum - 1) * 2 * kCoords),
              .lane_lights = std::vector<float>(
                  kMaxLanesNum * (kLanePointsNum - 1) * kLaneLightsNum),
              .lane_types =
                  std::vector<int64_t>(kMaxLanesNum * (kLanePointsNum - 1), 1),
              .mask = std::vector<int>(kMaxLanesNum, 1),
          },
      .bounds_feature =
          LaneBoundaryFeature{
              .boundaries = std::vector<float>(
                  kMaxBoundsNum * (kLanePointsNum - 1) * 2 * kCoords),
              .boundary_types = std::vector<int64_t>(
                  kMaxBoundsNum * (kLanePointsNum - 1), 10),
              .mask = std::vector<int>(kMaxBoundsNum, 1),
          },
      .cws_feature =
          CrossWalkFeature{
              .encirclingline = std::vector<float>(
                  kMaxCWNum * (kLanePointsNum - 1) * 2 * kCoords),
              .mask = std::vector<int>(kMaxCWNum, 1),
          },
      .c_trajs_feature =
          // NOLINTNEXTLINE
      {
          ActorsFeature{
              .trajs = std::vector<float>(kFutureNum * kCoords),
              .speeds = std::vector<float>(kFutureNum),
              .headings = std::vector<float>(kFutureNum * kCoords),
              .types = std::vector<float>(1),
              .cur_poses = std::vector<float>(kCoords),
          },
          ActorsFeature{
              .trajs = std::vector<float>(kFutureNum * kCoords),
              .speeds = std::vector<float>(kFutureNum),
              .headings = std::vector<float>(kFutureNum * kCoords),
              .types = std::vector<float>(1),
              .cur_poses = std::vector<float>(kCoords),
          },
      },
      .tl_feature =
          // NOLINTNEXTLINE
      {
          TargetLanesFeature{
              .lane_centers =
                  std::vector<float>(kTargetLanePointsNum * kCoords),
              .stats = std::vector<float>(5),
          },
          TargetLanesFeature{
              .lane_centers =
                  std::vector<float>(kTargetLanePointsNum * kCoords),
              .stats = std::vector<float>(5),
          },
      }};
  for (int i = 0; i < times; ++i) {
    std::vector<std::vector<float>> scores;
    GetOutputs(input_features, &scores);
  }
}

void SelectorScoringNetTorch::UnitTest() {}

bool SelectorScoringNetTorch::GetOutputs(
    const SelectorScoringNetFeature& input_features,
    std::vector<std::vector<float>>* scores) {
  std::vector<torch::jit::IValue> torch_inputs;
  {
    SCOPED_QTRACE("SelectorScoringNetTorch::Preprocess");
    auto options = torch::TensorOptions().dtype(torch::kFloat32);
    // Build actor features
    const auto& actors = input_features.actors_feature;
    const int valid_object_num =
        std::accumulate(actors.mask.begin(), actors.mask.end(), 0);
    torch::Tensor trajs_tensor =
        torch::from_blob(const_cast<float*>(actors.trajs.data()),
                         {valid_object_num, kHistoryNum, kCoords}, options);
    torch::Tensor speeds_tensor =
        torch::from_blob(const_cast<float*>(actors.speeds.data()),
                         {valid_object_num, kHistoryNum, kCoords}, options);
    speeds_tensor = torch::sqrt(
        torch::sum(torch::pow(speeds_tensor, 2), {-1}, /*keepdim=*/false));
    torch::Tensor headings_tensor =
        torch::from_blob(const_cast<float*>(actors.headings.data()),
                         {valid_object_num, kHistoryNum, kCoords}, options);
    torch::Tensor types_tensor = torch::from_blob(
        const_cast<float*>(actors.types.data()), {valid_object_num}, options);
    torch::Tensor poses_tensor =
        torch::from_blob(const_cast<float*>(actors.cur_poses.data()),
                         {valid_object_num, kHistoryNum, kCoords}, options)
            .slice(1, kHistoryNum - 1, kHistoryNum)
            .squeeze_(1);
    c10::Dict<std::string, c10::List<torch::Tensor>> actor_input;
    actor_input.insert("trajs", c10::List({trajs_tensor.to(device_)}));
    actor_input.insert("speeds", c10::List({speeds_tensor.to(device_)}));
    actor_input.insert("headings", c10::List({headings_tensor.to(device_)}));
    actor_input.insert("types", c10::List({types_tensor.to(device_)}));
    actor_input.insert("ctrs", c10::List({poses_tensor.to(device_)}));
    torch_inputs.push_back(actor_input);

    // Build lane features
    const auto& lanes = input_features.lanes_feature;
    const int valid_lanes_num =
        std::accumulate(lanes.mask.begin(), lanes.mask.end(), 0);
    torch::Tensor lane_points_tensor = torch::from_blob(
        const_cast<float*>(lanes.lane_centers.data()),
        {valid_lanes_num, kLanePointsNum - 1, 2, kCoords}, options);
    lane_points_tensor = torch::cat(
        {
            lane_points_tensor.slice(2, 0, 1).squeeze_(2),
            lane_points_tensor.slice(1, kLanePointsNum - 2, kLanePointsNum - 1)
                .slice(2, 1, 2)
                .squeeze_(2),
        },
        1);
    torch::Tensor lane_lights_tensor =
        torch::from_blob(const_cast<float*>(lanes.lane_lights.data()),
                         {valid_lanes_num, kLanePointsNum - 1, kLaneLightsNum},
                         options)
            .slice(1, 0, 1)
            .squeeze_(1);
    torch::Tensor lane_types_tensor =
        torch::from_blob(const_cast<int64_t*>(lanes.lane_types.data()),
                         {valid_lanes_num, kLanePointsNum - 1},
                         torch::TensorOptions().dtype(torch::kLong))
            .slice(1, 0, 1)
            .squeeze_(1);
    torch::Tensor one_hot_lane_types =
        torch::zeros({lane_types_tensor.size(0), kLaneTypesNum}, options);
    one_hot_lane_types.index_put_(
        {torch::arange(0, lane_types_tensor.size(0)), lane_types_tensor}, 1.0);
    c10::Dict<std::string, c10::List<torch::Tensor>> lane_input;
    lane_input.insert("centerlines",
                      c10::List({lane_points_tensor.to(device_)}));
    lane_input.insert("lanelights",
                      c10::List({lane_lights_tensor.to(device_)}));
    lane_input.insert("lanetypes", c10::List({one_hot_lane_types.to(device_)}));
    torch_inputs.push_back(lane_input);

    // Build bound features
    const auto& bounds = input_features.bounds_feature;
    const int valid_bounds_num =
        std::accumulate(bounds.mask.begin(), bounds.mask.end(), 0);
    torch::Tensor lane_bound_tensor = torch::from_blob(
        const_cast<float*>(bounds.boundaries.data()),
        {valid_bounds_num, kLanePointsNum - 1, 2, kCoords}, options);
    lane_bound_tensor = torch::cat(
        {
            lane_bound_tensor.slice(2, 0, 1).squeeze_(2),
            lane_bound_tensor.slice(1, kLanePointsNum - 2, kLanePointsNum - 1)
                .slice(2, 1, 2)
                .squeeze_(2),
        },
        1);
    torch::Tensor lane_bound_types_tensor =
        torch::from_blob(const_cast<int64_t*>(bounds.boundary_types.data()),
                         {valid_bounds_num, kLanePointsNum - 1},
                         torch::TensorOptions().dtype(torch::kLong))
            .slice(1, 0, 1)
            .squeeze_(1);
    torch::Tensor one_hot_lane_bonud_types = torch::zeros(
        {lane_bound_types_tensor.size(0), kBoundTypesNum}, options);
    lane_bound_types_tensor =
        torch::where(lane_bound_types_tensor == 0, 9, lane_bound_types_tensor);
    one_hot_lane_bonud_types.index_put_(
        {torch::arange(0, lane_bound_types_tensor.size(0)),
         lane_bound_types_tensor - 9},
        1.0);
    c10::Dict<std::string, c10::List<torch::Tensor>> bound_input;
    bound_input.insert("boundaries",
                       c10::List({lane_bound_tensor.to(device_)}));
    bound_input.insert("boundarytypes",
                       c10::List({one_hot_lane_bonud_types.to(device_)}));
    torch_inputs.push_back(bound_input);

    // Build cw features
    const auto& cws = input_features.cws_feature;
    const int valid_cw_num =
        std::accumulate(cws.mask.begin(), cws.mask.end(), 0);
    torch::Tensor cw_tensor = torch::from_blob(
        const_cast<float*>(cws.encirclingline.data()),
        {valid_cw_num, kLanePointsNum - 1, 2, kCoords}, options);
    cw_tensor = torch::cat(
        {
            cw_tensor.slice(2, 0, 1).squeeze_(2),
            cw_tensor.slice(1, kLanePointsNum - 2, kLanePointsNum - 1)
                .slice(2, 1, 2)
                .squeeze_(2),
        },
        1);
    c10::Dict<std::string, c10::List<torch::Tensor>> cw_input;
    cw_input.insert("encirclingline", c10::List({cw_tensor.to(device_)}));
    torch_inputs.push_back(cw_input);

    // Build candidate trajectory features
    const auto& c_trajs_feature = input_features.c_trajs_feature;
    torch::Tensor c_trajs_tensor = torch::zeros(
        {static_cast<uint32_t>(c_trajs_feature.size()), kFutureNum, kCoords},
        options);
    torch::Tensor c_speeds_tensor = torch::zeros(
        {static_cast<uint32_t>(c_trajs_feature.size()), kFutureNum}, options);
    torch::Tensor c_headings_tensor = torch::zeros(
        {static_cast<uint32_t>(c_trajs_feature.size()), kFutureNum, kCoords},
        options);
    torch::Tensor c_types_tensor =
        torch::zeros({static_cast<uint32_t>(c_trajs_feature.size())}, options);
    torch::Tensor c_poses_tensor = torch::zeros(
        {static_cast<uint32_t>(c_trajs_feature.size()), kCoords}, options);

    for (int i = 0; i < c_trajs_feature.size(); ++i) {
      auto feature = c_trajs_feature[i];
      c_trajs_tensor[i] =
          torch::from_blob(const_cast<float*>(feature.trajs.data()),
                           {1, kFutureNum, kCoords}, options)[0];
      c_speeds_tensor[i] =
          torch::from_blob(const_cast<float*>(feature.speeds.data()),
                           {1, kFutureNum}, options)[0];
      c_headings_tensor[i] =
          torch::from_blob(const_cast<float*>(feature.headings.data()),
                           {1, kFutureNum, kCoords}, options)[0];
      c_types_tensor[i] = torch::ones({1}, options)[0];
      c_poses_tensor[i] =
          torch::from_blob(const_cast<float*>(feature.cur_poses.data()),
                           {1, kCoords}, options)[0];
    }

    c10::Dict<std::string, c10::List<torch::Tensor>> c_trajs_input;
    c_trajs_input.insert("trajs", c10::List({c_trajs_tensor.to(device_)}));
    c_trajs_input.insert("speeds", c10::List({c_speeds_tensor.to(device_)}));
    c_trajs_input.insert("headings",
                         c10::List({c_headings_tensor.to(device_)}));
    c_trajs_input.insert("types", c10::List({c_types_tensor.to(device_)}));
    c_trajs_input.insert("ctrs", c10::List({c_poses_tensor.to(device_)}));
    torch_inputs.push_back(c_trajs_input);

    // Build target lane path features
    const auto& tl_feature = input_features.tl_feature;
    std::vector<torch::Tensor> tl_feature_vec;
    std::vector<torch::Tensor> tl_stat_vec;
    for (int i = 0; i < tl_feature.size(); ++i) {
      torch::Tensor tl = torch::nn::functional::pad(
          torch::from_blob(
              const_cast<float*>(tl_feature[i].lane_centers.data()),
              {1, static_cast<int>(tl_feature[i].lane_centers.size() / kCoords),
               kCoords},
              options),
          torch::nn::functional::PadFuncOptions(
              {0, 0, 0,
               kTargetLanePointsNum -
                   static_cast<int>(tl_feature[i].lane_centers.size() /
                                    kCoords),
               0, 0}));
      tl_feature_vec.push_back(tl);
      torch::Tensor stat = torch::from_blob(
          const_cast<float*>(tl_feature[i].stats.data()), {1, 5}, options);
      tl_stat_vec.push_back(stat);
    }
    torch::Tensor tl_feature_tensor = torch::cat(tl_feature_vec, 0);
    torch::Tensor tl_stat_tensor = torch::cat(tl_stat_vec, 0);
    c10::Dict<std::string, c10::List<torch::Tensor>> tl_input;
    tl_input.insert("target_ref_lines",
                    c10::List({tl_feature_tensor.to(device_)}));
    tl_input.insert("stats", c10::List({tl_stat_tensor.to(device_)}));
    torch_inputs.push_back(tl_input);
  }

  // Inference
  c10::IValue model_output;
  {
    SCOPED_QTRACE("SelectorScoringNetTorch::Infer");
    model_output = selector_scoring_net_.forward(torch_inputs);
  }

  // Get outputs
  {
    SCOPED_QTRACE("SelectorScoringNetTorch::Postprocess");
    const auto& cls_tensor = model_output.toTensor().to(torch::kCPU);
    const auto& c_trajs_feature = input_features.c_trajs_feature;
    const int cls_size = c_trajs_feature.size();
    auto torch_output = cls_tensor.accessor<float, 2>();
    scores->push_back({});
    for (int i = 0; i < cls_size; ++i) {
      (*scores)[0].push_back(torch_output[i][0]);
    }
  }

  return true;
}

}  // namespace selector_scoring_net
}  // namespace qcraft
