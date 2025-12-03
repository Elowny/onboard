#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_STATE_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_STATE_H_

#include <stdint.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

struct SdRouteMatchState {
  struct DistRange {
    double start;
    double end;
  };
  std::shared_ptr<const SDRouteProto> sd_route_proto;
  std::shared_ptr<const SdRouteMatchResultProto> sd_route_match_result_proto;

  std::vector<Vec2d> coords;  // Global points;
  // The same size of sd_route_proto.links
  std::vector<int> link_points_size_accumulator;
  // The same size of sd_route_proto.links, distance from start.
  std::vector<double> link_s_accumulator;
  // Add 0 at front. so the size is the same with coords
  std::vector<double> segment_dists;
  std::vector<DistRange> match_dist_ranges_from_start;
  int sd_route_length = 0;

  // Update with the av pose.
  int cur_match_link_index = 0;
  int cur_match_point_on_link_index = 0;
  int cur_coord_index = 0;
  double s = 0;            // The total travel distance from the sd route start
  Vec2d av_project_point;  // Global point.
  std::vector<DistRange> match_dist_ranges_from_cur;

  std::string ShortDebugString() const;
  void Clear();
};

struct RouteManagerState {
  bool has_route() const {
    return route.has_route_section_sequence() || route.has_lane_path();
  }
  RouteProto route;
  RouteProto route_from_current;

  struct AlterRouteState {
    RouteProto route;
    RouteProto route_from_current;
    RoutingResultProto routing_result_proto;
    double roundabout_distance;
  };

  std::optional<AlterRouteState> alter_route_state = std::nullopt;

  SdRouteMatchState sd_route_match_state;
  // Routing request to be sent to routing_module.
  std::shared_ptr<PlannerRoutingRequestProto> active_routing_request = nullptr;
  MultipleStopsRequest pending_multi_stops;
  // Request id count.
  int64_t request_id = -1;
  MultipleStopsRequest multi_stops;
  int next_destination_index = -1;
  CompositeLanePath::CompositeIndex composite_index = {0, 0};
  RoutingResultProto routing_result_proto;
  // TODO(xiang): move to state proto
  mapping::LanePoint last_lane_point = {mapping::kInvalidElementId, 0.0};
  RouteManagerStateDebugProto rms_debug;
  RouteContentProto route_content_proto;
  double vx_p90 = 0.0;

  void Clear();
};

void RouteResetRequest(RouteManagerState* state);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_STATE_H_
