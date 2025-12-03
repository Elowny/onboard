#include "onboard/planner/router/route_core_searcher.h"

// IWYU pragma: no_include "absl/hash/hash.h" // for Hash

#include <algorithm>
#include <optional>

#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "common/proto/lane_point.pb.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/smm_proto_util.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/utils/file_util.h"

namespace qcraft::planner::route {
namespace {
TEST(SearchForRouteCorePathFromGlobalPose, NoPredRouteTest) {
  const auto& smm = LoadDojoMap();
  auto cc = CoordinateConverter::FromMap("dojo");

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);

  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  MapIndex map_index;
  map_index.InitIndex(smm);

  RoutingRequestProto routing_request;
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(129);
  dest1.mutable_lane_point()->set_fraction(0.5);
  routing_request.mutable_destinations()->Add(std::move(dest1));

  const Vec2d global_pose(-1.7025472208616248e-05, 5.4425251826731613e-07);

  RoutingRequestContext route_context{.semantic_map_manager = &smm,
                                      .map_index = &map_index,
                                      .route_param_proto = &route_params,
                                      .pred_trajectory_point = std::nullopt};

  const auto sections_lanes_or = SearchForRouteCorePathFromGlobalPose(
      route_context, cc, routing_request, global_pose, route_pre,
      /*heading=*/0.0, &route_core);
  EXPECT_OK(sections_lanes_or);
  EXPECT_EQ(sections_lanes_or->first.size(), 11);
  EXPECT_NEAR(sections_lanes_or->second.length(), 404.408, 0.1);
}

TEST(SearchForRouteCorePathFromGlobalPose, PredRouteTest) {
  const auto& smm = LoadDojoMap();
  auto cc = CoordinateConverter::FromMap("dojo");

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);

  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  MapIndex map_index;
  map_index.InitIndex(smm);

  RoutingRequestProto routing_request;
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(129);
  dest1.mutable_lane_point()->set_fraction(0.5);
  routing_request.mutable_destinations()->Add(std::move(dest1));

  const Vec2d global_pose(-1.7025472208616248e-05, 5.4425251826731613e-07);

  RoutingRequestContext route_context{
      .semantic_map_manager = &smm,
      .map_index = &map_index,
      .route_param_proto = &route_params,
      .pred_trajectory_point = std::optional<Vec2d>({-101.69, 3.447})};

  const auto sections_lanes_or = SearchForRouteCorePathFromGlobalPose(
      route_context, cc, routing_request, global_pose, route_pre,
      /*heading=*/0.0, &route_core);
  EXPECT_OK(sections_lanes_or);
  EXPECT_EQ(sections_lanes_or->first.size(), 11);
  EXPECT_NEAR(sections_lanes_or->second.length(), 404.408, 0.1);
}

TEST(SearchForRouteCorePathToRoutingRequest, RouteTest) {
  const auto& smm = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);

  RoutingRequestProto routing_request;
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(2471);
  dest1.mutable_lane_point()->set_fraction(0.5);
  routing_request.mutable_destinations()->Add(std::move(dest1));

  RoutingDestinationProto dest2;
  dest2.mutable_lane_point()->set_lane_id(129);
  dest2.mutable_lane_point()->set_fraction(0.5);
  routing_request.mutable_destinations()->Add(std::move(dest2));

  RoutingRequestContext route_context{.semantic_map_manager = &smm,
                                      .map_index = map_index,
                                      .route_param_proto = &route_params,
                                      .pred_trajectory_point = std::nullopt};

  std::vector<mapping::PointToLane> point_to_lane;
  point_to_lane.reserve(2);
  point_to_lane.push_back(mapping::PointToLane{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.5,
      .fraction = 0.5});
  point_to_lane.push_back(mapping::PointToLane{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(3)),
      .dist = 0.5,
      .fraction = 0.5});

  const auto sections_lanes_or = SearchForRouteCorePathToRoutingRequest(
      route_context, routing_request, point_to_lane,
      {
          .is_bus = true,
          .use_time = true,
      },
      &route_core);
  EXPECT_OK(sections_lanes_or);
  EXPECT_EQ(sections_lanes_or->first.size(), 7);
  EXPECT_NEAR(sections_lanes_or->second.length(), 267.79, 0.1);
}

}  // namespace
}  // namespace qcraft::planner::route
