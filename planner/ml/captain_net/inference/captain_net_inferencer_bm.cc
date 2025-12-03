#include <vector>  // for vector, allocator

#include "benchmark/benchmark.h"  // for State, DoNotOptimize, BENCHMARK, Bench...
#include "glog/logging.h"  // for COMPACT_GOOGLE_LOG_FATAL, LogMessageFatal

#include "onboard/nets/proto/net_param.pb.h"  // for NetParam, NetParam_DeviceType
#include "onboard/params/param_finder.h"      // for GetProtoParamById
#include "onboard/planner/ml/captain_net/captain_net.h"  // for kCoords, kMaxLaneCenterNum, kLanePoint...
#include "onboard/planner/ml/captain_net/inference/captain_net_inferencer.h"  // for CaptainNetInferencer
#include "onboard/utils/errors.h"  // for CHECK_OK

namespace qcraft::planner::ml::captain_net {

static void RunCaptainNetInferenceBM(benchmark::State& state,  // NOLINT
                                     const bool use_gpu) {
  NetParam captain_net_param;
  CHECK_OK(GetProtoParamById("Q0001", "captain_net_onnx_trt_param",
                             &captain_net_param));
  if (!use_gpu) {
    captain_net_param.set_device_type(NetParam_DeviceType_CPU);
  }
  CaptainNetInferencer captain_net_inferencer(captain_net_param);

  const int history_len = kHistoryNum;

  const auto agent_feature = AgentFeature{
      .agent_type = 1,
      .agent_pos_diff = std::vector<float>(history_len * kCoords),
      .agent_speed = std::vector<float>(history_len * kCoords),
      .agent_heading = std::vector<float>(history_len * kCoords),
  };

  const auto actors_feature = ActorsFeature{
      .rel_pos = std::vector<float>(kMaxObjectsNum * history_len * kCoords),
      .rel_yaw = std::vector<float>(kMaxObjectsNum * history_len * kCoords),
      .rel_speed = std::vector<float>(kMaxObjectsNum * history_len * kCoords),
      .valid_mask = std::vector<int>(kMaxObjectsNum),
  };

  const auto lc_feature = LanesFeature{
      .seg = std::vector<float>(kMaxLaneCenterNum * kLanePointsNum * 4),
      .unit_seg_vec =
          std::vector<float>(kMaxLaneCenterNum * kLanePointsNum * kCoords),
      .unit_tangent =
          std::vector<float>(kMaxLaneCenterNum * kLanePointsNum * kCoords),
      .dist = std::vector<float>(kMaxLaneCenterNum * kLanePointsNum),
      .unit_dist_vec =
          std::vector<float>(kMaxLaneCenterNum * kLanePointsNum * kCoords),
      .nearest_to_end_dist =
          std::vector<float>(kMaxLaneCenterNum * kLanePointsNum),
      .yaw_diff =
          std::vector<float>(kMaxLaneCenterNum * kLanePointsNum * kCoords),
      .light =
          std::vector<float>(kMaxLaneCenterNum * kLanePointsNum * kMaxLightNum),
      .valid_mask = std::vector<int>(kMaxLaneCenterNum),
  };

  const auto target_lanes_feature = TargetLanesFeature{
      .target_lanes = std::vector<std::vector<float>>(
          kMaxTargetLanesNum,
          std::vector<float>(kMaxTargetLaneSegNum * kMaxTargetLanePointNum *
                             kCoords)),
      .valid_mask = std::vector<int>(kLaneTypeDim),
  };

  const auto input_feature = CaptainNetFeature{
      .agent_feature = agent_feature,
      .actors_feature = actors_feature,
      .lane_center_feature = lc_feature,
      .target_lane_feature = target_lanes_feature,
  };

  std::vector<std::vector<std::vector<float>>> prob_out;
  std::vector<std::vector<std::vector<float>>> traj_out;

  for (auto _ : state) {
    benchmark::DoNotOptimize(captain_net_inferencer.PlanningTrajectory(
        input_feature, &prob_out, &traj_out));
  }
}

static void BM_CaptainNetInferenceGPU(benchmark::State& state) {  // NOLINT
  RunCaptainNetInferenceBM(state, /*use_gpu=*/true);
}
static void BM_CaptainNetInferenceCPU(benchmark::State& state) {  // NOLINT
  RunCaptainNetInferenceBM(state, /*use_gpu=*/false);
}

BENCHMARK(BM_CaptainNetInferenceGPU);
BENCHMARK(BM_CaptainNetInferenceCPU);
}  // namespace qcraft::planner::ml::captain_net

BENCHMARK_MAIN();
