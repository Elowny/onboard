#ifndef ONBOARD_PREDICTION_TEST_UTIL_NET_INPUT_GENERATOR_H_
#define ONBOARD_PREDICTION_TEST_UTIL_NET_INPUT_GENERATOR_H_

#include <vector>

#include "onboard/nets/proto/net_param.pb.h"
namespace qcraft {
namespace prediction {

std::vector<std::vector<float>> GenUniformDistributedInputByNetParam(
    const NetParam& net_param);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_TEST_UTIL_NET_INPUT_GENERATOR_H_
