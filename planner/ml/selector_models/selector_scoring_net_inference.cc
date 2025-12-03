#include "onboard/planner/ml/selector_models/selector_scoring_net_inference.h"

#include "absl/status/status.h"

#include "onboard/global/car_common.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/ml/selector_models/selector_scoring_net_onnx.h"
#include "onboard/proto/ads.pb.h"
#if defined(__x86_64__) && !defined(Q_CPU_ONLY)
#include "onboard/planner/ml/selector_models/selector_scoring_net_torch.h"
#endif

namespace qcraft {

SelectorScoringNetInference::SelectorScoringNetInference(
    const RunParamsProtoV2& run_param, const NetParam& net_param)
    : net_param_(net_param) {
// Note: use cpu_only macro for CI scenario test and segment test.
#if Q_CPU_ONLY == 1
  use_gpu_ = false;
#else
  use_gpu_ = net_param_.use_gpu();
#endif

  if (IsOnboardMode() && net_param_.gpu_id() != GPU_UNKNOWN) {
    device_id_ =
        param_util::GetDeviceIdFromGpuId(run_param, net_param_.gpu_id());
  } else if (!use_gpu_) {
    device_id_ = -1;
  } else {
    // In simulation, we could assume that we always want to use gpu 0 if we
    // want to use GPU.
    device_id_ = 0;
  }
  const int max_batch_size = net_param.max_batch_size();
  QCHECK_GT(max_batch_size, 0);

#if defined(__x86_64__)
#if defined(Q_CPU_ONLY)
  // Use onnx inference for cpu mode to make it easier for migration.
  selector_scoring_net_ = selector_scoring_net::SelectorScoringNetCPU::Create(
      net_param.model_path(), selector_scoring_net::kInitInputDims,
      selector_scoring_net::kOutputDims);
#else   // defined(Q_CPU_ONLY)
  if (!use_gpu_) {
    selector_scoring_net_ = selector_scoring_net::SelectorScoringNetCPU::Create(
        net_param.model_path(), selector_scoring_net::kInitInputDims,
        selector_scoring_net::kOutputDims);
  } else {
    // The times are adjusted according to road and simulation test.
    constexpr int kWarmUpTimes = 2;
    selector_scoring_net_ =
        selector_scoring_net::SelectorScoringNetTorch::Create(
            net_param.libtorch_gpu_path(), use_gpu_, device_id_, kWarmUpTimes);
  }
#endif  // defined(Q_CPU_ONLY)

#else   // defined(__x86_64__)
  selector_scoring_net_ = selector_scoring_net::SelectorScoringNetCPU::Create(
      net_param.model_path(), selector_scoring_net::kInitInputDims,
      selector_scoring_net::kOutputDims);
#endif  // defined(__x86_64__)
}

absl::StatusOr<std::vector<float>> SelectorScoringNetInference::EvaluateScores(
    const selector_scoring_net::SelectorScoringNetFeature& input_features)
    const {
  if (selector_scoring_net_.get() == nullptr) {
    return absl::InternalError(
        "Inferencer for SelectorScoringNetInference::EvaluateScores is not "
        "initialized.");
  }
  std::vector<std::vector<float>> scores;
  if (!selector_scoring_net_->GetOutputs(input_features, &scores)) {
    return absl::InternalError(
        "SelectorScoringNetInference::EvaluateScores failed.");
  }
  return scores[0];
}

}  // namespace qcraft
