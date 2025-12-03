#ifndef ONBOARD_PLANNER_ROUTER_TASK_ROUTE_RESET_ACTION_H_
#define ONBOARD_PLANNER_ROUTER_TASK_ROUTE_RESET_ACTION_H_

#include "onboard/lite/lite_module.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {

class RouteResetAction {
 public:
  explicit RouteResetAction(LiteModule* lite_module)
      : lite_module_(lite_module) {}

  void HandleAction(const RouteManagerState& rms,
                    RouteManagerOutputProto* last_route_mgr_output_proto);
  void Clear() { publish_route_times_when_reset_ = 0; }

  void set_max_keep_publish_times(int max_keep_publish_times) {
    max_keep_publish_times_ = max_keep_publish_times;
  }
  int max_keep_publish_times() const { return max_keep_publish_times_; }

  const RoutingResultProto& routing_result_proto() const {
    return routing_result_proto_;
  }

  const HmiContentProto& hmi_content_proto() const {
    return hmi_content_proto_;
  }

 private:
  void ResetProtoContent(const RouteManagerState& rms,
                         RouteManagerOutputProto* last_route_mgr_output_proto);
  LiteModule* lite_module_ = nullptr;
  int publish_route_times_when_reset_ = 0;
  int max_keep_publish_times_ = 2;

  RoutingResultProto routing_result_proto_;
  HmiContentProto hmi_content_proto_;
};

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_TASK_ROUTE_RESET_ACTION_H_
