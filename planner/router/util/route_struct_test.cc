#include "onboard/planner/router/util/route_struct.h"

#include "gtest/gtest.h"

namespace qcraft::planner::route {
namespace {
TEST(RouteStructTest, SearchStateT) {
  SearchStateT<int64_t> search_state = {
      .id = 1,
      .connect_type = ConnectTypeFromParent::kConnect,
      .which = 0b1010,
  };
  EXPECT_FALSE(search_state.IsOpen());
  EXPECT_TRUE(search_state.IsClose());
  EXPECT_FALSE(search_state.IsForward());
  EXPECT_TRUE(search_state.IsBackward());
}
}  // namespace
}  // namespace qcraft::planner::route
