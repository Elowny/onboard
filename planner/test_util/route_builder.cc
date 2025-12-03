#include "onboard/planner/test_util/route_builder.h"

#include <string>
#include <utility>

#include "absl/hash/hash.h"
#include "glog/logging.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/lane_point.pb.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/route_core_searcher.h"
#include "onboard/planner/router/route_util.h"

namespace qcraft {
namespace planner {

absl::StatusOr<CompositeLanePath> CalcRoutingRequest(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const route::MapIndex& map_index,
    const RoutingRequestProto& routing_request, const PoseProto& pose) {
  const auto param = CreateDefaultRouteParam();
  route::RoutingRequestContext context = {
      .semantic_map_manager = &semantic_map_manager,
      .map_index = &map_index,
      .route_param_proto = &param,
  };
  route::RouteCore route_core;
  route_core.set_semantic_map_manager(&semantic_map_manager);
  route_core.set_route_param_proto(&param);
  route::RouteCorePrecondition cond = {
      .is_bus = true,
      .use_time = true,
  };
  const double heading = cc.SmoothYawToGlobal(pose.yaw());
  const Vec2d global_pose =
      cc.SmoothToGlobal({pose.pos_smooth().x(), pose.pos_smooth().y()});
  auto route_lane_path = route::SearchForRouteCorePathFromGlobalPose(
      context, cc, routing_request, global_pose, cond, heading, &route_core);
  if (!route_lane_path.ok()) {
    return CompositeLanePath();
  }
  return route_lane_path->second;
}

CompositeLanePath RoutingToNameSpot(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const PoseProto& pose,
    std::string name_spot) {
  route::MapIndex map_index;
  map_index.InitIndex(semantic_map_manager);
  return RoutingToNameSpot(semantic_map_manager, cc, map_index, pose,
                           std::move(name_spot));
}
CompositeLanePath RoutingToNameSpot(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const route::MapIndex& map_index,
    const PoseProto& pose, std::string name_spot) {
  RoutingRequestProto routing_request;
  routing_request.add_destinations()->set_named_spot(name_spot);
  auto lane_path_or = CalcRoutingRequest(semantic_map_manager, cc, map_index,
                                         routing_request, pose);
  CHECK(lane_path_or.ok());
  return *lane_path_or;
}

absl::StatusOr<CompositeLanePath> RoutingToLanePoint(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const PoseProto& pose,
    const CompositeLanePath::LanePoint& lane_point) {
  route::MapIndex map_index;
  map_index.InitIndex(semantic_map_manager);
  return RoutingToLanePoint(semantic_map_manager, cc, map_index, pose,
                            lane_point);
}

absl::StatusOr<CompositeLanePath> RoutingToLanePoint(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const route::MapIndex& map_index,
    const PoseProto& pose, const CompositeLanePath::LanePoint& lane_point) {
  RoutingRequestProto routing_request;
  auto* lane_point_proto =
      routing_request.add_destinations()->mutable_lane_point();
  lane_point_proto->set_lane_id(lane_point.lane_id().value());
  lane_point_proto->set_fraction(lane_point.fraction());
  return CalcRoutingRequest(semantic_map_manager, cc, map_index,
                            routing_request, pose);
}

}  // namespace planner
}  // namespace qcraft
