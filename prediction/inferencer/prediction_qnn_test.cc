#include "onboard/prediction/inferencer/prediction_qnn.h"

#include "gtest/gtest.h"

#include "onboard/lite/logging.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/prediction/test_util/net_input_generator.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(PredictionQNNTest, RunActNetOnCPU) {
  NetParam actnet_param;
  QCHECK_OK(GetProtoParamById("Q2206", "act_net_param", &actnet_param));
  const auto batch_inputs = GenUniformDistributedInputByNetParam(actnet_param);
  std::string trt_model_path = actnet_param.trt_model_path();
  const int max_batch_size = actnet_param.max_batch_size();
  actnet_param.set_device_type(NetParam_DeviceType_CPU);
  std::unique_ptr<prediction::PredictionQNN> cpu_net(
      new prediction::PredictionQNN(trt_model_path, actnet_param));
  const auto run_res_cpu = cpu_net->Run(batch_inputs, max_batch_size);
  EXPECT_EQ(run_res_cpu.size(), actnet_param.output_tensor_list_size());
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
