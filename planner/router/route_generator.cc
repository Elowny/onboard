#include "onboard/planner/router/route_generator.h"

#include <ostream>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/off_road_route_searcher.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_core_searcher.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner::route {
namespace {
std::tuple<route::RouteCorePrecondition, Vec2d, double> PrepareForRouteCore(
    const CoordinateConverter& coordinate_converter, const PoseProto& pose,
    const route::RouteRestrictDistrict& route_restrict, bool is_bus,
    bool use_time) {
  const Vec2d car_smooth = {pose.pos_smooth().x(), pose.pos_smooth().y()};
  const auto car_global = coordinate_converter.SmoothToGlobal(car_smooth);
  const auto heading = coordinate_converter.SmoothYawToGlobal(pose.yaw());

  route::RouteCorePrecondition route_pre{
      .is_bus = is_bus,
      .use_time = use_time,
      .route_restrict_district = route_restrict};
  return std::make_tuple(route_pre, car_global, heading);
}
}  // namespace
absl::StatusOr<RoutingResultProto> InitRoute(
    const route::RoutingRequestContext& context, const CoordinateConverter& cc,
    const route::RouteRestrictDistrict& route_restrict_district,
    const PlannerRoutingRequestProto::InitRouteRequest& request, bool is_bus,
    route::RouteCore* route_core) {
  FUNC_QTRACE();
  const mapping::v2::SemanticMapManager& semantic_map_manager =
      *context.semantic_map_manager;
  const auto map_index = context.map_index;

  RoutingResultProto result_proto;
  result_proto.set_update_id(context.request_id);
  // create default destinations
  if (!request.has_routing_request()) {
    result_proto.set_success(false);
    return result_proto;
  }

  const RoutingRequestProto routing_request = request.routing_request();
  result_proto.set_routing_type(RoutingType::INIT_ROUTE);
  if (HasOnlyOneOffRoadRequest(routing_request)) {
    result_proto.set_success(true);
    *result_proto.mutable_routing_request() = routing_request;
    result_proto.set_routing_type(RoutingType::PARKING_OFFROAD);
    return result_proto;
  }

  // search for route path
  absl::Status search_status;

  const auto [route_pre, global_pose, heading] = PrepareForRouteCore(
      cc, request.pose(), route_restrict_district, is_bus, /*use_time=*/true);
  const auto sections_lanes_or = route::SearchForRouteCorePathFromGlobalPose(
      context, cc, routing_request, global_pose, route_pre, heading,
      route_core);
  if (sections_lanes_or.ok()) {
    sections_lanes_or->first.ToProto(
        result_proto.mutable_route_section_sequence());
    sections_lanes_or->second.ToProto(result_proto.mutable_lane_path());
    *result_proto.mutable_routing_request() = routing_request;
    result_proto.set_success(true);
    result_proto.set_distance(CalcRouteSectionsLength(
        semantic_map_manager, sections_lanes_or->first));
    return result_proto;
  } else {
    search_status = sections_lanes_or.status();
  }

  // Offroad search if allowed.
  if (FLAGS_router_enable_off_road_search &&
      absl::IsOutOfRange(search_status)) {
    // Pose not on road.
    ASSIGN_OR_RETURN(const auto multi_stop_request,
                     BuildMultipleStopsRequest(semantic_map_manager, map_index,
                                               routing_request));
    auto off_road_route_or = SearchForRouteFromOffRoad(
        context, cc, request.pose(), multi_stop_request,
        /*search_radius=*/20.0, route_core);
    if (off_road_route_or.ok()) {
      // TODO(weijun): Refactor repeated code.
      off_road_route_or->on_road_route_lane_path.ToProto(
          result_proto.mutable_lane_path());
      off_road_route_or->route_sections.ToProto(
          result_proto.mutable_route_section_sequence());
      *result_proto.mutable_routing_request() = routing_request;
      off_road_route_or->link_lane_point.ToProto(
          result_proto.mutable_link_lane_point());
      result_proto.set_success(true);
      return result_proto;
    } else {
      QLOG_EVERY_N_SEC(ERROR, 3)
          << "Init route failed:" << search_status.message()
          << ", Off-road routing also failed:"
          << off_road_route_or.status().message()
          << "routing request:" << routing_request.ShortDebugString();
      return absl::NotFoundError(absl::StrCat(
          "Cannot find any route. pose:", request.pose().ShortDebugString(),
          ", request", request.routing_request().ShortDebugString()));
    }
  } else {
    const Vec3d pos = Vec3dFromProto(request.pose().pos_smooth());
    const Vec3d global = cc.SmoothToGlobal(pos);

    QEVENT("xiang", "planner_routing_failure", [&](qcraft::QEvent* qevent) {
      qevent->AddField("x", global.x());
      qevent->AddField("y", global.y());
      qevent->AddField("z", global.z());
      qevent->AddField("request", request.routing_request().ShortDebugString());
    });
    QLOG_EVERY_N_SEC(ERROR, 3)
        << "Init route failed: cannot find any route path to destinations: "
        << routing_request.ShortDebugString();
    return absl::NotFoundError(absl::StrCat(
        "Cannot search a route.pose:", global.DebugString(), ", request",
        request.routing_request().ShortDebugString()));
  }
  result_proto.set_success(false);
  return result_proto;
}

absl::StatusOr<RoutingResultProto> GenerateRoutingResultProto(
    const route::RoutingRequestContext& context, const CoordinateConverter& cc,
    const route::RouteRestrictDistrict& route_restrict_district,
    const PlannerRoutingRequestProto* request, bool is_bus,
    route::RouteCore* route_core) {
  FUNC_QTRACE();
  if (request->has_init()) {
    return InitRoute(context, cc, route_restrict_district, request->init(),
                     is_bus, route_core);
  } else {
    return absl::InvalidArgumentError("Invalid routing request type.");
  }
}

}  // namespace qcraft::planner::route
