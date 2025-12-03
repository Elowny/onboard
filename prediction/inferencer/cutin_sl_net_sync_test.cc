#include <random>

#include "gtest/gtest.h"

#include "onboard/lite/logging.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/prediction/inferencer/prediction_qnn.h"

namespace qcraft {
namespace cutin_sl_net {
namespace {

constexpr float kMaxDiff = 0.01;
std::vector<std::vector<float>> GenerateRandomBatchInput(
    const NetParam& net_param) {
  std::vector<std::vector<float>> batch_inputs;
  const int max_obj = net_param.max_batch_size();
  const int input_size = net_param.input_tensor_list_size();
  batch_inputs.resize(input_size);

  std::random_device rnd_device;
  std::mt19937 mersenne_engine{rnd_device()};  // Generates random integers
  std::uniform_real_distribution<float> dist{-1.0, 1.0};
  auto gen = [&dist, &mersenne_engine]() { return dist(mersenne_engine); };
  int i = 0;
  for (const auto& input_tensor : net_param.input_tensor_list()) {
    const int data_size = std::accumulate(input_tensor.shape().begin() + 1,
                                          input_tensor.shape().end(), max_obj,
                                          std::multiplies<int>());
    batch_inputs[i].resize(data_size);
    std::generate(std::begin(batch_inputs[i]), std::end(batch_inputs[i]), gen);
    i++;
  }
  return batch_inputs;
}

}  // namespace

TEST(CutinSLNetSyncTest, SyncTest) {
  NetParam cutin_sl_net_param;
  QCHECK_OK(
      GetProtoParamById("Q2206", "cutin_sl_net_param", &cutin_sl_net_param));
  const std::string trt_model_path = cutin_sl_net_param.trt_model_path();
  const int max_batch_size = cutin_sl_net_param.max_batch_size();
  const auto batch_inputs = GenerateRandomBatchInput(cutin_sl_net_param);
  QCHECK_GT(max_batch_size, 0);

  // Gpu model.
  std::unique_ptr<prediction::PredictionQNN> gpu_net(
      new prediction::PredictionQNN(trt_model_path, cutin_sl_net_param));
  if (!gpu_net) {
    QLOG(ERROR) << "Failed to initialize CutinSLNetQNN GPU.";
    return;
  }

  auto run_res_gpu = gpu_net->Run(batch_inputs, max_batch_size);

  // Cpu model.
  cutin_sl_net_param.set_device_type(NetParam_DeviceType_CPU);
  std::unique_ptr<prediction::PredictionQNN> cpu_net(
      new prediction::PredictionQNN(trt_model_path, cutin_sl_net_param));
  if (!cpu_net) {
    QLOG(ERROR) << "Failed to initialize CutinSLNetQNN CPU.";
    return;
  }

  auto run_res_cpu = cpu_net->Run(batch_inputs, max_batch_size);
  for (const auto& [name, out_gpu] : run_res_gpu) {
    float max_diff = 0.0;
    CHECK_EQ(run_res_cpu[name].size(), out_gpu.size());
    for (int j = 0; j < out_gpu.size(); ++j) {
      max_diff = std::max(run_res_cpu[name][j] - out_gpu[j], max_diff);
    }
    EXPECT_LT(max_diff, kMaxDiff) << "name:" << name;
  }
}
}  // namespace cutin_sl_net
}  // namespace qcraft
