
#include <cmath>
#include <string>
#include <vector>

#include "absl/status/statusor.h"

#include "gtest/gtest.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_test_util.h"
// #include "onboard/planner/router/plot_util.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace planner {
namespace {

PoseProto CarPose(double smooth_x = 0.0, double smooth_y = 0.0,
                  double heading = 0.0, double smooth_v_x = 0.0,
                  double smooth_v_y = 0.0) {
  PoseProto car_pose;
  car_pose.mutable_pos_smooth()->set_x(smooth_x);
  car_pose.mutable_pos_smooth()->set_y(smooth_y);
  car_pose.set_yaw(heading);
  car_pose.mutable_vel_smooth()->set_x(smooth_v_x);
  car_pose.mutable_vel_smooth()->set_y(smooth_v_y);
  return car_pose;
}

bool RouteIsExpected(const CompositeLanePath& this_route,
                     const CompositeLanePath& expected_route) {
  bool route_correct = false;
  constexpr double kFractionThreshold = 0.03;
  if (this_route.lane_paths().size() != expected_route.lane_paths().size()) {
    return false;
  }
  for (int i = 0; i < expected_route.lane_paths().size(); ++i) {
    if (expected_route.lane_path(i).size() != this_route.lane_path(i).size())
      break;
    if (std::fabs(expected_route.lane_path(i).start_fraction() -
                      this_route.lane_path(i).start_fraction() >
                  kFractionThreshold) ||
        std::fabs(expected_route.lane_path(i).end_fraction() -
                      this_route.lane_path(i).end_fraction() >
                  kFractionThreshold))
      break;
    for (int j = 0; j < expected_route.lane_path(i).size(); ++j) {
      if (expected_route.lane_path(i).lane_id(j) !=
          this_route.lane_path(i).lane_id(j))
        break;
      route_correct = true;
    }
  }

  return route_correct;
}

absl::StatusOr<CompositeLanePath> Routing(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const PoseProto& pose,
    std::string name_spot) {
  route::MapIndex map_index;
  map_index.InitIndex(semantic_map_manager);
  RoutingRequestProto routing_request;
  routing_request.add_destinations()->set_named_spot(name_spot);
  return CalcRoutingRequest(semantic_map_manager, cc, map_index,
                            routing_request, pose);
}

TEST(RouteTest, ChoseBestRoute1) {
  // Load map
  SetMap("dojo");
  const auto* smm = &LoadDojoMap();
  auto cc = CoordinateConverter::FromMap("dojo");

  // route test 1
  CompositeLanePath expected_route_path(
      {mapping::LanePath(smm,
                         {mapping::ElementId(96), mapping::ElementId(2),
                          mapping::ElementId(938), mapping::ElementId(940),
                          mapping::ElementId(2470), mapping::ElementId(51)},
                         0.143225, 1.0),
       mapping::LanePath(smm,
                         {mapping::ElementId(53), mapping::ElementId(187),
                          mapping::ElementId(114), mapping::ElementId(136),
                          mapping::ElementId(142)},
                         1.0, 0.499336)},
      CompositeLanePath::kDefaultTransition);

  CompositeLanePath expected_route_path2(mapping::LanePath(
      smm,
      {mapping::ElementId(97), mapping::ElementId(2448), mapping::ElementId(1),
       mapping::ElementId(34), mapping::ElementId(2471), mapping::ElementId(53),
       mapping::ElementId(187), mapping::ElementId(114),
       mapping::ElementId(136), mapping::ElementId(142)},
      0.143225, 0.499336));

  const auto route = Routing(*smm, cc, CarPose(-33.015, 3.423), "8a_n2");
  EXPECT_TRUE(route.ok());
  //
  // SendRouteLanePathToCanvas(semantic_map_manager, *route, "8a_n2");
  EXPECT_TRUE(RouteIsExpected(*route, expected_route_path) ||
              RouteIsExpected(*route, expected_route_path2))
      << "route is not correct";
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
