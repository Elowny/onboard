#include "onboard/prediction/test_util/net_input_generator.h"

#include "gtest/gtest.h"

#include "onboard/lite/logging.h"
#include "onboard/params/param_finder.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(NetInputGeneratorTest, GenUniformDistributedInputByNetParam) {
  NetParam actnet_param;
  QCHECK_OK(GetProtoParamById("Q2206", "act_net_param", &actnet_param));
  const auto actnet_inputs = GenUniformDistributedInputByNetParam(actnet_param);
  EXPECT_EQ(actnet_inputs.size(), actnet_param.input_tensor_list_size());
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
