#include "onboard/prediction/net/horizon/tensor_id_name.h"

#include <array>
#include <memory>

#include "magic_enum/magic_enum.hpp"

#include "gtest/gtest.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(NameToIdAndIdToName, Works) {
  for (const auto& [tensor_id, _] : magic_enum::enum_entries<J5QNNTensorId>()) {
    ASSERT_EQ(tensor_id, J5QNNTensorNameToId(J5QNNTensorIdToName(tensor_id)));
  }
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
