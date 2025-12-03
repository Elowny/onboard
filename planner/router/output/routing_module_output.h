#ifndef ONBOARD_PLANNER_ROUTER_OUTPUT_ROUTING_MOUDLE_OUTPUT_H_
#define ONBOARD_PLANNER_ROUTER_OUTPUT_ROUTING_MOUDLE_OUTPUT_H_
#include "absl/status/status.h"

#include "onboard/lite/lite_module.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {
namespace internal {
bool CheckTimeInterval(int64_t last_publish_micros, int64_t now_micros,
                       int interval_ms);
}
struct RoutingOutput {
  HmiContentProto hmi_content_proto;
  RouteManagerOutputProto route_manger_output_proto;
  RoutingResultProto routing_result_proto;
};

struct PublishPolicy {
  int routing_result_interval_ms = 0;
  int hmi_interval_ms = 0;
};

class RoutingOutputPublisher {
 public:
  RoutingOutputPublisher() = default;
  absl::Status PublishRoutingOutput(const RoutingOutput& routing_output,
                                    const PublishPolicy& publish_policy,
                                    int64_t now_micros,
                                    LiteModule* lite_module);

 private:
  RoutingOutput last_output_;
  int64_t last_publish_routing_result_micros_;
  int64_t last_publish_hmi_micros_;
};
}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_OUTPUT_ROUTING_MOUDLE_OUTPUT_H_
