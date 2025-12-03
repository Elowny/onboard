#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/prediction/net/horizon/horizon_tensor_wrapper.h"
#include "onboard/prediction/net/horizon/prediction_j5_qnn.h"
#include "onboard/prediction/test_util/net_input_generator.h"

namespace qcraft {
namespace actnetj5 {
namespace {
constexpr float kMaxDiff = 0.01f;
}  // namespace

// NOTE(yinbao): Use "--test_timeout=600" if test failed with log "Test timed
// out".
TEST(ActNetJ5SyncTest, SyncTest) {
  NetParam actnetj5_param;
  QCHECK_OK(GetProtoParamById("Q2206", "act_net_j5_param", &actnetj5_param));

  std::vector<std::vector<float>> batch_inputs =
      prediction::GenUniformDistributedInputByNetParam(actnetj5_param);
  const int max_batch_size = actnetj5_param.max_batch_size();
  QCHECK_GT(max_batch_size, 0);

  // Gpu model.
  actnetj5_param.set_device_type(NetParam_DeviceType_GPU);
  actnetj5_param.set_backend_type(NetParam_BackendType_TENSORRT);
  std::unique_ptr<prediction::PredictionJ5QNN> gpu_net(
      new prediction::PredictionJ5QNN(actnetj5_param));
  if (!gpu_net) {
    QLOG(ERROR) << "Failed to initialize ActNetJ5QNN GPU.";
    return;
  }
  for (int i = 0; i < batch_inputs.size(); ++i) {
    const auto& name = actnetj5_param.input_tensor_list()[i].name();
    float* data_ptr = gpu_net->GetInputByName(name)->GetDataPtr<float>();
    std::memcpy(data_ptr, batch_inputs[i].data(),
                batch_inputs[i].size() * sizeof(float));
  }
  auto run_res_gpu = gpu_net->Run(max_batch_size);
  run_res_gpu = gpu_net->Run(max_batch_size);

  // Cpu model.
  actnetj5_param.set_device_type(NetParam_DeviceType_CPU);
  actnetj5_param.set_backend_type(NetParam_BackendType_ONNX);
  std::unique_ptr<prediction::PredictionJ5QNN> cpu_net(
      new prediction::PredictionJ5QNN(actnetj5_param));
  if (!cpu_net) {
    QLOG(ERROR) << "Failed to initialize ActNetJ5QNN CPU.";
    return;
  }
  for (int i = 0; i < batch_inputs.size(); ++i) {
    const auto& name = actnetj5_param.input_tensor_list()[i].name();
    float* data_ptr = cpu_net->GetInputByName(name)->GetDataPtr<float>();
    std::memcpy(data_ptr, batch_inputs[i].data(),
                batch_inputs[i].size() * sizeof(float));
  }
  auto run_res_cpu = cpu_net->Run(max_batch_size);
  run_res_cpu = cpu_net->Run(max_batch_size);

  // compare
  for (const auto& [name, out_gpu] : run_res_gpu) {
    float max_diff = 0.0;
    CHECK_EQ(run_res_cpu[name]->size(), out_gpu->size());
    float* res_cpu = run_res_cpu[name]->GetDataPtr<float>();
    float* res_gpu = out_gpu->GetDataPtr<float>();
    for (int j = 0; j < out_gpu->size(); ++j) {
      max_diff = std::max(std::abs(res_cpu[j] - res_gpu[j]), max_diff);
    }
    EXPECT_LT(max_diff, kMaxDiff) << "name:" << name;
  }
}

}  // namespace actnetj5
}  // namespace qcraft
