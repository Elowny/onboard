#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_OUTPUT_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_OUTPUT_H_

#include <optional>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/road_constraints.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/turn_signal.pb.h"

namespace qcraft {
namespace planner {
constexpr int kInvalidRouteUpdateId = -1;

struct RouteManagerOutput {
  void ToProto(RouteManagerOutputProto* proto) const;
  void FromProto(const RouteManagerOutputProto& proto);

  struct AlternateRouteMsg {
    RouteSections route_sections_from_current;
    RouteNaviInfo route_navi_info;
    double roundabout_distance;
  };

  std::optional<AlternateRouteMsg> alter_route_msg = std::nullopt;

  RouteSections route_sections_from_current;
  absl::flat_hash_set<mapping::ElementId> avoid_lanes;
  route::AvoidLaneSegmentMap avoid_lane_segments;

  RouteNaviInfo route_navi_info;

  RoutingRequestProto routing_request_proto;

  TurnSignal signal = TURN_SIGNAL_NONE;
  // TODO(weijun): add signal reason.
  bool rerouted = false;

  std::optional<double> recommend_max_speed_limit = std::nullopt;  // Unit: m/s.

  MultipleStopsRequestProto::StopProto destination_stop;
  int update_id = kInvalidRouteUpdateId;
  RouteManagerOutputProto::RouteStatus status = RouteManagerOutputProto::NORMAL;

  // Self-intersection info, only for navinfo map.
  std::vector<OverlappingSection> overlap_sections;
  bool overlap_sections_precomputed = false;
};

RouteManagerOutputProto CompatibilityProcessForRouteManagerOutput(
    const RouteManagerOutputProto& rm_out);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_OUTPUT_H_
