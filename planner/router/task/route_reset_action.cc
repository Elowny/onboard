#include "onboard/planner/router/task/route_reset_action.h"

#include <stdint.h>

#include <ostream>

#include "absl/status/statusor.h"

#include "onboard/lite/lite_module.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/planner/router/multi_stops_request.h"

namespace qcraft::planner::route {

void RouteResetAction::ResetProtoContent(
    const RouteManagerState& rms,
    RouteManagerOutputProto* last_route_mgr_output_proto) {
  const int64_t update_id = rms.request_id;
  last_route_mgr_output_proto->set_update_id(update_id);
  last_route_mgr_output_proto->set_route_status(RouteManagerOutputProto::RESET);
  routing_result_proto_.set_success(true);
  routing_result_proto_.set_update_id(update_id);
  auto* route_content = hmi_content_proto_.mutable_route_content();
  route_content->set_routing_request_id(rms.multi_stops.request_id().data());
}

void RouteResetAction::HandleAction(
    const RouteManagerState& rms,
    RouteManagerOutputProto* last_route_mgr_output_proto) {
  routing_result_proto_.Clear();
  hmi_content_proto_.Clear();
  if (++publish_route_times_when_reset_ > max_keep_publish_times()) {
    last_route_mgr_output_proto->Clear();
    last_route_mgr_output_proto->set_route_status(
        RouteManagerOutputProto::INVALID);
    QLOG_IF_NOT_OK(WARNING,
                   lite_module_->Publish(*last_route_mgr_output_proto));
    return;
  }
  ResetProtoContent(rms, last_route_mgr_output_proto);
  if (lite_module_ != nullptr) {
    QLOG_IF_NOT_OK(WARNING, lite_module_->Publish(routing_result_proto()));
    QLOG_IF_NOT_OK(WARNING,
                   lite_module_->Publish(*last_route_mgr_output_proto));
    QLOG_IF_NOT_OK(WARNING, lite_module_->Publish(hmi_content_proto()));
    QLOG_EVERY_N_SEC(INFO, 3) << "Receive a reset request, Send reset command.";
  }
}
}  // namespace qcraft::planner::route
