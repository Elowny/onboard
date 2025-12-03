#include "onboard/planner/router/off_road_route_searcher.h"

#include <string>

#include "absl/strings/str_cat.h"

#include "gtest/gtest.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

TEST(FindCrossingLanePointsFromPose, FromOffRoad) {
  SetMap("dojo");
  const auto& semantic_map_manager = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();
  const auto cc = CoordinateConverter::FromMap("dojo");

  {
    // From parking spot
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(536.98);
    pose.mutable_pos_smooth()->set_y(29.33);
    pose.set_yaw(-1.63);

    const auto lane_points = FindCrossingLanePointsFromPose(
        semantic_map_manager, *map_index, cc, pose, /*radius=*/20.0);

    // std::vector<Vec2d> points;
    // points.reserve(lane_points.size());
    // for (const auto lp : lane_points) {
    //   points.emplace_back(lp.ComputePos(semantic_map_manager));
    // }

    // SendPointsToCanvas(points, "test/crossing_points", vis::Color::kRed);

    EXPECT_EQ(lane_points.size(), 2);
    EXPECT_EQ(lane_points[0].lane_id().value(), 3104);
    EXPECT_EQ(lane_points[1].lane_id().value(), 3093);
  }

  {
    // From side road
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(673.1);
    pose.mutable_pos_smooth()->set_y(-1346);
    pose.set_yaw(0.648);

    const auto lane_points = FindCrossingLanePointsFromPose(
        semantic_map_manager, *map_index, cc, pose, /*radius=*/20.0);

    // TODO(xiang) Remove render to the canvas
    // std::vector<Vec2d> points;
    // points.reserve(lane_points.size());
    // for (const auto lp : lane_points) {
    //   points.emplace_back(lp.ComputePos(semantic_map_manager));
    // }

    // SendPointsToCanvas(points, "test/crossing_points_1", vis::Color::kBlue);

    EXPECT_EQ(lane_points.size(), 1);
    EXPECT_EQ(lane_points[0].lane_id().value(), 8294);
  }
}

TEST(SearchForRouteFromOffRoad, FromOffRoad) {
  SetMap("dojo");
  const auto* semantic_map_manager = &LoadDojoMap();
  const auto& map_index = *LoadDojoIndex();
  const auto cc = CoordinateConverter::FromMap("dojo");

  const auto& psmm = CreateDojoTestPSMM();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);

  route::RoutingRequestContext context{
      .semantic_map_manager = semantic_map_manager,
      .map_index = &map_index,
      .route_param_proto = &route_params};

  route::RouteCore route_core(semantic_map_manager, &route_params);

  {
    // From parking spot
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(536.98);
    pose.mutable_pos_smooth()->set_y(29.33);
    pose.set_yaw(-1.63);

    RoutingRequestProto routing_request_proto;

    auto* stop = routing_request_proto.mutable_multi_stops()->add_stops();
    mapping::LanePoint(mapping::ElementId(3841), 0.7)
        .ToProto(stop->mutable_stop_point()->mutable_lane_point());

    ASSIGN_OR_DIE(auto request,
                  BuildMultipleStopsRequest(*semantic_map_manager, &map_index,
                                            routing_request_proto));

    ASSIGN_OR_DIE(auto route_result, SearchForRouteFromOffRoad(
                                         context, cc, pose, request,
                                         /*search_radius=*/20.0, &route_core));

    std::string channel = "test1/whole_route";
    SendRouteLanePathToCanvas(psmm, route_result.on_road_route_lane_path,
                              absl::StrCat(channel, "_on_road"));

    // auto& canvas =
    //     vantage_client_man::GetCanvas(absl::StrCat(channel, "_off_road"));
    // canvas.SetGroundZero(1);
    // const Vec2d link_point =
    //     route_result.link_lane_point.ComputePos(*semantic_map_manager);
    // canvas.DrawLine(Vec3d(pose.pos_smooth().x(), pose.pos_smooth().y(), 0.0),
    //                 Vec3d(link_point.x(), link_point.y(), 0.0),
    //                 vis::Color::kBlue);
    // vantage_client_man::FlushAll();
  }

  {
    // From parking spot
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(536.98);
    pose.mutable_pos_smooth()->set_y(29.33);
    pose.set_yaw(-1.63);

    RoutingRequestProto routing_request_proto;

    auto* stop = routing_request_proto.mutable_multi_stops()->add_stops();
    mapping::LanePoint(mapping::ElementId(3092), 0.7)
        .ToProto(stop->mutable_stop_point()->mutable_lane_point());

    ASSIGN_OR_DIE(auto request,
                  BuildMultipleStopsRequest(*semantic_map_manager, &map_index,
                                            routing_request_proto));

    ASSIGN_OR_DIE(auto route_result, SearchForRouteFromOffRoad(
                                         context, cc, pose, request,
                                         /*search_radius=*/20.0, &route_core));

    std::string channel = "test2/whole_route";
    SendRouteLanePathToCanvas(psmm, route_result.on_road_route_lane_path,
                              absl::StrCat(channel, "_on_road"));

    // auto& canvas =
    //     vantage_client_man::GetCanvas(absl::StrCat(channel, "_off_road"));
    // canvas.SetGroundZero(1);
    // const Vec2d link_point =
    //     route_result.link_lane_point.ComputePos(*semantic_map_manager);
    // canvas.DrawLine(Vec3d(pose.pos_smooth().x(), pose.pos_smooth().y(), 0.0),
    //                 Vec3d(link_point.x(), link_point.y(), 0.0),
    //                 vis::Color::kBlue);
    // vantage_client_man::FlushAll();
  }
}

}  // namespace

}  // namespace qcraft::planner
