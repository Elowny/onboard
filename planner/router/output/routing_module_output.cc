
#include "onboard/planner/router/output/routing_module_output.h"

#include "onboard/lite/logging.h"
namespace qcraft::planner::route {
namespace internal {
bool CheckTimeInterval(int64_t last_publish_micros, int64_t now_micros,
                       int interval_ms) {
  return now_micros - last_publish_micros >= interval_ms * 1000;
}
}  // namespace internal

absl::Status RoutingOutputPublisher::PublishRoutingOutput(
    const RoutingOutput& routing_output, const PublishPolicy& publish_policy,
    int64_t now_micros, LiteModule* lite_module) {
  if (lite_module == nullptr) {
    return absl::InvalidArgumentError(
        "lite_module is nullptr, cannot publish messages.");
  }
  if (internal::CheckTimeInterval(last_publish_hmi_micros_, now_micros,
                                  publish_policy.hmi_interval_ms)) {
    QLOG_IF_NOT_OK(WARNING,
                   lite_module->Publish(routing_output.hmi_content_proto));
    last_output_.hmi_content_proto = routing_output.hmi_content_proto;
  }
  if (internal::CheckTimeInterval(last_publish_routing_result_micros_,
                                  now_micros,
                                  publish_policy.routing_result_interval_ms)) {
    QLOG_IF_NOT_OK(WARNING,
                   lite_module->Publish(routing_output.routing_result_proto));
    last_output_.routing_result_proto = routing_output.routing_result_proto;
  }
  QLOG_IF_NOT_OK(
      WARNING, lite_module->Publish(routing_output.route_manger_output_proto));
  return absl::OkStatus();
}

}  // namespace qcraft::planner::route
