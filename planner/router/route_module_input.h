#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_MODULE_INPUT_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_MODULE_INPUT_H_

#include <stdint.h>  // for int64_t

#include <memory>    // for shared_ptr
#include <optional>  // for optional

#include "onboard/maps/proto/online_semantic_map.pb.h"  // for OnlineSemanticMapProto
#include "onboard/math/vec.h"                           // for Vec2d
#include "onboard/planner/router/proto/route_external_command.pb.h"  // for ExternalRouterCommand
#include "onboard/proto/adasis.pb.h"          // for MppSectionsProto
#include "onboard/proto/autonomy_state.pb.h"  // for AutonomyStateProto, DriverAction
#include "onboard/proto/localization.pb.h"  // for LocalizationTransformProto
#include "onboard/proto/positioning.pb.h"  // for GnssRawReadingProto, PoseProto
#include "onboard/proto/remote_assist.pb.h"  // for RemoteAssistToCarProto
#include "onboard/proto/route.pb.h"  // for RecordedRouteProto, ReroutingRequestProto, Rout...

namespace qcraft::planner::route {
struct RoutingInput {
  std::shared_ptr<const PoseProto> pose;
  std::shared_ptr<const LocalizationTransformProto> localization_transform;
  std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car;
  std::shared_ptr<const ReroutingRequestProto> rerouting_request;
  // TODO(zuowei): Only for Sim compatibility, delete later.
  std::shared_ptr<const RecordedRouteProto> recorded_route;
  std::shared_ptr<const AutonomyStateProto> autonomy_state;
  std::shared_ptr<const DriverAction> driver_action;
  std::shared_ptr<const RoutingResultProto> routing_result;
  std::shared_ptr<const mapping::OnlineSemanticMapProto> online_sm_proto;
  std::shared_ptr<const RoutingStateProto> routing_state;
  std::shared_ptr<const GnssRawReadingProto> gnss;
  std::shared_ptr<const MppSectionsProto> mpp;
  std::shared_ptr<const SDRouteProto> sd_route_proto;
  std::shared_ptr<const SdRouteMatchResultProto> sd_route_match_result_proto;

  int64_t injected_teleop_micro_secs = 0;  // TODO(xiang): set value
  ExternalRouterCommand external_router_command = ExternalRouterCommand::NONE;
  bool res_button_pressed = false;
  std::optional<Vec2d> pred_trajectory_point;
};
}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_MODULE_INPUT_H_
