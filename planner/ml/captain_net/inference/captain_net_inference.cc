#include "onboard/planner/ml/captain_net/inference/captain_net_inference.h"

#include <ostream>  // for operator<<

#include "onboard/global/car_common.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/params/utils/param_util.h"
#include "onboard/planner/ml/captain_net/inference/captain_net_onnx.h"
#if defined(__x86_64__) && !defined(Q_CPU_ONLY)
#include "onboard/planner/ml/captain_net/inference/captain_net_torch.h"
#endif
#include "onboard/proto/ads.pb.h"  // for GpuId

namespace qcraft::planner::ml::captain_net {

CaptainNetInference::CaptainNetInference(const RunParamsProtoV2& run_param,
                                         const NetParam& net_param)
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
  captain_net_ = CaptainNetCPU::Create(net_param.model_path(), kInitInputDims,
                                       kOutputDims);
#else   // defined(Q_CPU_ONLY)
  // NOTE(Jiaming): for x86_64 in gpu mode, we can choose freely to use cpu net
  // or torch. Note that although torch can work in cpu mode, as libtorch_cpu.so
  // depends on  libcudart (which is not available on pure-cpu systems, thus
  // causing extra deployment maitainence), we EXPLICITLY disabled torch in CPU
  // mode here.
  if (!use_gpu_) {
    captain_net_ = CaptainNetCPU::Create(net_param.model_path(), kInitInputDims,
                                         kOutputDims);
  } else {
    // The times are adjusted according to road and simulation test.
    constexpr int kWarmUpTimes = 2;
    captain_net_ = CaptainNetTorch::Create(net_param.libtorch_gpu_path(),
                                           use_gpu_, device_id_, kWarmUpTimes);
  }
#endif  // defined(Q_CPU_ONLY)
#else   // defined(__x86_64__)
  // NOTE(Jiaming): libtorch was not provided for non-x86_64 platforms
  captain_net_ = CaptainNetCPU::Create(net_param.model_path(), kInitInputDims,
                                       kOutputDims);
#endif  // defined(__x86_64__)
}

bool CaptainNetInference::PlanningTrajectory(
    const CaptainNetFeature& input_features,
    std::vector<std::vector<std::vector<float>>>* prob_out,
    std::vector<std::vector<std::vector<float>>>* traj_out) const {
  if (captain_net_.get() == nullptr) {
    QLOG(ERROR) << "Inferencer for CaptainNetInference::PlanningTrajectory is "
                   "not initialized.";
    return false;
  }
  if (!captain_net_->GetOutputs(input_features, prob_out, traj_out)) {
    QLOG(ERROR) << "CaptainNetInference::PlanningTrajectory failed.";
    return false;
  }
  return true;
}

}  // namespace qcraft::planner::ml::captain_net
