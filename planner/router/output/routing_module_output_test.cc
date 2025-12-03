
#include "onboard/planner/router/output/routing_module_output.h"

#include "gtest/gtest.h"

namespace qcraft::planner::route {
namespace {

TEST(RouingModuleOutputTest, PublishPolicyTest) {
  {
    int64_t now_micros = 2000;
    auto last_publish_micros = 1000;
    ASSERT_TRUE(
        internal::CheckTimeInterval(last_publish_micros, now_micros, 1));
  }
  {
    RoutingOutputPublisher publisher;
    publisher.last_publish_routing_result_micros_ = 123456L;
    publisher.last_publish_hmi_micros_ = 123456L;
    RoutingOutput output;
    PublishPolicy policy = {.routing_result_interval_ms = 1000,
                            .hmi_interval_ms = 1000};
    ASSERT_FALSE(publisher
                     .PublishRoutingOutput(output, policy,
                                           /*now_micros=*/123456L, nullptr)
                     .ok());
  }
}

}  // namespace
}  // namespace qcraft::planner::route
