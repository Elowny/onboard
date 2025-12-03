#include "onboard/prediction/test_util/net_input_generator.h"

#include <functional>
#include <random>
namespace qcraft {
namespace prediction {

std::vector<std::vector<float>> GenUniformDistributedInputByNetParam(
    const NetParam& net_param) {
  const int max_bs = net_param.max_batch_size();
  std::vector<std::vector<float>> batch_inputs;
  const int input_size = net_param.input_tensor_list_size();
  batch_inputs.resize(input_size);

  std::random_device rnd_device;
  std::mt19937 mersenne_engine{rnd_device()};  // Generates random integers
  std::uniform_real_distribution<float> dist{-1.0, 1.0};
  auto gen = [&dist, &mersenne_engine]() { return dist(mersenne_engine); };
  int i = 0;
  for (const auto& input_tensor : net_param.input_tensor_list()) {
    const int data_size = std::accumulate(input_tensor.shape().begin() + 1,
                                          input_tensor.shape().end(), max_bs,
                                          std::multiplies<int>());
    batch_inputs[i].resize(data_size);
    std::generate(std::begin(batch_inputs[i]), std::end(batch_inputs[i]), gen);
    i++;
  }
  return batch_inputs;
}
}  // namespace prediction
}  // namespace qcraft
