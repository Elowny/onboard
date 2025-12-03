#include "onboard/planner/router/drive_passage.h"

// IWYU pragma: no_include <ext/alloc_traits.h>

#include <algorithm>
#include <optional>
#include <ostream>

#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

// NOLINTNEXTLINE(readability-function-size)
TEST(DrivePassage, QueryTestStraight) {
  FLAGS_planner_enable_dynamic_lane_speed_limit = false;

  // Load map
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();
  auto start_time = absl::Now();

  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  start_time = absl::Now();
  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "7c_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in routing";

  start_time = absl::Now();
  ASSIGN_OR_DIE(const auto passage,
                BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                                  route_path.lane_paths().front(),
                                  route_path.lane_paths().front(),
                                  /*anchor_point=*/mapping::LanePoint(),
                                  route_sections.planning_horizon(psmm),
                                  route_sections.destination(),
                                  /*all_lanes_virtual=*/false,
                                  /*override_speed_limit_mps=*/std::nullopt),
                "Building drive passage failed!");
  // SendRouteLanePathToCanvas(psmm,route_path,"7c_n1");
  // SendDrivePassageToCanvas(passage, "test_0");
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building DrivePassage";

  ASSERT_FALSE(passage.empty()) << "Empty drive passage!";

  start_time = absl::Now();
  // straight lane point
  const Vec2d query_point(42.2, 4.0);

  ASSIGN_OR_DIE(const auto point_xy, passage.QueryPointXYAtS(42.2));
  EXPECT_LT(point_xy.DistanceTo(Vec2d(42.2, 0.06)), 1e-3);

  ASSIGN_OR_DIE(const auto speed_limit, passage.QuerySpeedLimitAt(query_point));
  EXPECT_NEAR(speed_limit, 11.111, 1e-3);

  ASSIGN_OR_DIE(const auto speed_limit1, passage.QuerySpeedLimitAtS(42.2));
  EXPECT_NEAR(speed_limit1, 11.111, 1e-3);

  ASSIGN_OR_DIE(const auto curb_offset, passage.QueryCurbOffsetAt(query_point));
  EXPECT_NEAR(curb_offset.first, -9.2907, 1e-4);
  EXPECT_NEAR(curb_offset.second, 5.69, 1e-4);

  ASSIGN_OR_DIE(const auto curb_info,
                passage.QueryCurbOffsetAndHeightAt(query_point));
  EXPECT_NEAR(curb_info.height.first, 10.0, 1e-4);
  EXPECT_NEAR(curb_info.height.second, 0.0, 1e-4);

  ASSIGN_OR_DIE(const auto curb_offset1, passage.QueryCurbOffsetAtS(42.2));
  EXPECT_NEAR(curb_offset1.first, -5.3503, 1e-4);
  EXPECT_NEAR(curb_offset1.second, 9.6304, 1e-4);

  ASSIGN_OR_DIE(const auto curb_info1,
                passage.QueryCurbOffsetAndHeightAtS(42.2));
  EXPECT_NEAR(curb_info1.height.first, 10.0, 1e-4);
  EXPECT_NEAR(curb_info1.height.second, 0.0, 1e-4);

  ASSIGN_OR_DIE(const auto curb_boundaries,
                passage.QueryCurbBoundariesAt(query_point));
  EXPECT_NEAR(curb_boundaries.first.lat_offset, -9.2907, 1e-4);
  EXPECT_EQ(curb_boundaries.first.type, StationBoundaryType::CURB);
  EXPECT_NEAR(curb_boundaries.second.lat_offset, 5.69, 1e-4);
  EXPECT_EQ(curb_boundaries.second.type, StationBoundaryType::VIRTUAL_CURB);

  ASSIGN_OR_DIE(const auto curb_boundaries1,
                passage.QueryCurbBoundariesAtS(42.2));
  EXPECT_NEAR(curb_boundaries1.first.lat_offset, -5.3503, 1e-4);
  EXPECT_EQ(curb_boundaries1.first.type, StationBoundaryType::CURB);
  EXPECT_NEAR(curb_boundaries1.second.lat_offset, 9.6304, 1e-4);
  EXPECT_EQ(curb_boundaries1.second.type, StationBoundaryType::VIRTUAL_CURB);

  ASSIGN_OR_DIE(const auto lane_boundaries,
                passage.QueryEnclosingLaneBoundariesAt(query_point));
  EXPECT_EQ(lane_boundaries.right->type, StationBoundaryType::BROKEN_WHITE);
  EXPECT_NEAR(lane_boundaries.right->lat_offset, -2.2427, 1e-4);
  EXPECT_EQ(lane_boundaries.left->type,
            StationBoundaryType::SOLID_DOUBLE_YELLOW);
  EXPECT_NEAR(lane_boundaries.left->lat_offset, 1.18, 1e-4);

  ASSIGN_OR_DIE(const auto tangent, passage.QueryTangentAt(query_point));
  EXPECT_NEAR(tangent.Angle(), 0.0, 2e-3);

  ASSIGN_OR_DIE(const auto tangent1, passage.QueryTangentAtS(42.2));
  EXPECT_NEAR(tangent1.Angle(), 0.0, 2e-3);

  ASSIGN_OR_DIE(const auto frenet_lon_offset,
                passage.QueryFrenetLonOffsetAt(query_point));
  EXPECT_EQ(frenet_lon_offset.station_index.value(), 21);
  EXPECT_NEAR(frenet_lon_offset.lon_offset, 0.204, 1e-3);

  ASSIGN_OR_DIE(const auto frenet_lat_offset,
                passage.QueryFrenetLatOffsetAt(query_point));
  EXPECT_NEAR(frenet_lat_offset, 3.94, 1e-3);

  FrenetCoordinate frenet_coord;
  ASSIGN_OR_DIE(frenet_coord,
                passage.QueryFrenetCoordinateAt(Vec2d(10.0, -2.0)));
  EXPECT_NEAR(frenet_coord.s, 10.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -2.0, 0.1);
  EXPECT_FALSE(passage.QueryFrenetCoordinateAt(Vec2d(10.0, -10.0)).ok());
  EXPECT_FALSE(passage.QueryFrenetCoordinateAt(Vec2d(-10.0, -10.0)).ok());

  ASSIGN_OR_DIE(frenet_coord, passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                                  Vec2d(10.0, -10.0)));
  EXPECT_NEAR(frenet_coord.s, 10.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -10.0, 0.1);
  EXPECT_FALSE(
      passage.QueryLaterallyUnboundedFrenetCoordinateAt(Vec2d(-10.0, -10.0))
          .ok());

  ASSIGN_OR_DIE(frenet_coord,
                passage.QueryUnboundedFrenetCoordinateAt(Vec2d(-10.0, -10.0)));
  EXPECT_NEAR(frenet_coord.s, -10.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -10.0, 0.1);

  ASSIGN_OR_DIE(const auto nearest_point,
                passage.FindNearestPointOnCenterLine(query_point));
  EXPECT_NEAR(nearest_point.x(), 42.2, 0.1);
  EXPECT_NEAR(nearest_point.y(), 0.0, 0.1);

  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in query operations";

  SendDrivePassageToCanvas(passage, "drive_passage2");
  EXPECT_EQ(true, true);
}

TEST(DrivePassage, QueryTestBox) {
  // Load map
  auto start_time = absl::Now();

  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(-3.5);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  start_time = absl::Now();
  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "7c_s3");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in routing";

  start_time = absl::Now();
  ASSIGN_OR_DIE(const auto passage,
                BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                                  route_path.lane_paths().front(),
                                  route_path.lane_paths().front(),
                                  /*anchor_point=*/mapping::LanePoint(),
                                  route_sections.planning_horizon(psmm),
                                  route_sections.destination(),
                                  /*all_lanes_virtual=*/false,
                                  /*override_speed_limit_mps=*/std::nullopt),
                "Building drive passage failed!");
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building DrivePassage";

  ASSERT_FALSE(passage.empty()) << "Empty drive passage!";

  start_time = absl::Now();

  const Box2d small_box = Box2d::CreateAABox({6.0, 0.0}, {10.0, -3.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(small_box));

  const Box2d wide_box = Box2d::CreateAABox({6.0, 10.0}, {10.0, -10.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(wide_box));

  const Box2d long_box = Box2d::CreateAABox({-5.0, 10.0}, {5.0, 0.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(long_box));

  const Box2d super_long_box = Box2d::CreateAABox({-5.0, 10.0}, {100.0, 0.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(super_long_box));

  const Box2d super_large_box =
      Box2d::CreateAABox({-10.0, 10.0}, {100.0, -60.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(super_large_box));

  const Box2d corner_box = Box2d::CreateAABox({70.0, -7.0}, {80.0, -14.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(corner_box));

  const Box2d back_box = Box2d::CreateAABox({-10.0, 3.0}, {-3.0, -3.0});
  EXPECT_FALSE(passage.QueryFrenetBoxAt(back_box).ok());

  const Box2d side_box = Box2d::CreateAABox({6.0, -7.0}, {10.0, -10.0});
  EXPECT_FALSE(passage.QueryFrenetBoxAt(side_box).ok());

  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in query operations";

  SendDrivePassageToCanvas(passage, "drive_passage_query_box");
}

TEST(DrivePassage, BatchQuerySL) {
  // Load map
  auto start_time = absl::Now();
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  start_time = absl::Now();
  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "7c_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in routing";

  start_time = absl::Now();
  ASSIGN_OR_DIE(const auto passage,
                BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                                  route_path.lane_paths().front(),
                                  route_path.lane_paths().front(),
                                  /*anchor_point=*/mapping::LanePoint(),
                                  route_sections.planning_horizon(psmm),
                                  route_sections.destination(),
                                  /*all_lanes_virtual=*/false,
                                  /*override_speed_limit_mps=*/std::nullopt),
                "Building drive passage failed!");
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building DrivePassage";

  ASSERT_FALSE(passage.empty()) << "Empty drive passage!";

  start_time = absl::Now();
  // straight lane point
  const Vec2d query_point1(42.2, 4.0);
  ASSIGN_OR_DIE(const auto lon_pt,
                passage.QueryFrenetLonOffsetAt(query_point1));
  ASSIGN_OR_DIE(const auto lat, passage.QueryFrenetLatOffsetAt(query_point1));
  auto lon_idx = lon_pt.station_index;
  const auto& station = passage.station(lon_idx);
  const auto& query_point1_1 = station.lon_point(lon_pt.lon_offset);
  const double lon_offset = station.lon_offset(query_point1_1);
  EXPECT_NEAR(lon_offset, lon_pt.lon_offset, 1e-7);
  ++lon_idx;
  ++lon_idx;
  const auto& next_station = passage.station(lon_idx);
  const Vec2d query_point2 = next_station.lat_point(lat);
  const double lat_offset = next_station.lat_offset(query_point2);
  EXPECT_NEAR(lat_offset, lat, 1e-7);
  std::vector<Vec2d> queries;
  queries.reserve(3);
  queries.push_back(query_point1);
  queries.push_back(query_point2);
  queries.push_back(query_point1_1);
  ASSIGN_OR_DIE(const auto frenet_coords,
                passage.BatchQueryFrenetCoordinates(queries));
  EXPECT_NEAR(
      passage.station(lon_pt.station_index).accumulated_s() + lon_pt.lon_offset,
      frenet_coords[0].s, 0.01);
  EXPECT_NEAR(lat, frenet_coords[0].l, 0.01);
  EXPECT_NEAR(next_station.accumulated_s(), frenet_coords[1].s, 0.01);
  EXPECT_NEAR(lat, frenet_coords[1].l, 0.01);
  EXPECT_DOUBLE_EQ(lon_pt.accum_s, frenet_coords[2].s);
}

// NOLINTNEXTLINE(readability-function-size)
TEST(DrivePassage, QueryTestCrossing) {
  // Load map
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "b7_e2_start");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);

  const auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);

  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";

  const auto& passage = drive_passage.value();
  SendDrivePassageToCanvas(passage, "test_drive_passage/query_crossing");
  // point at crossing
  const Vec2d query_point(83.0, 0.0);
  constexpr double kDistanceError = 0.05;  // m.
  ASSIGN_OR_DIE(const auto point_xy, passage.QueryPointXYAtS(82.9));
  EXPECT_LT(point_xy.DistanceTo(Vec2d(82.9, 0.03)), kDistanceError)
      << point_xy.DebugString();

  ASSIGN_OR_DIE(const auto speed_limit, passage.QuerySpeedLimitAt(query_point));
  EXPECT_NEAR(speed_limit, 11.111, 1e-3);

  ASSIGN_OR_DIE(const auto speed_limit1, passage.QuerySpeedLimitAtS(83.0));
  EXPECT_NEAR(speed_limit1, 11.111, 1e-3);

  ASSIGN_OR_DIE(const auto curb_offset, passage.QueryCurbOffsetAt(query_point));
  EXPECT_NEAR(curb_offset.first, -10.0, kDistanceError);
  EXPECT_NEAR(curb_offset.second, 10.0, kDistanceError);

  ASSIGN_OR_DIE(const auto curb_offset1, passage.QueryCurbOffsetAtS(82.9));
  EXPECT_NEAR(curb_offset1.first, -10.0, kDistanceError);
  EXPECT_NEAR(curb_offset1.second, 10.0, kDistanceError);

  ASSIGN_OR_DIE(const auto curb_boundaries,
                passage.QueryCurbBoundariesAt(query_point));
  EXPECT_NEAR(curb_boundaries.first.lat_offset, -10.0, kDistanceError);
  EXPECT_EQ(curb_boundaries.first.type, StationBoundaryType::VIRTUAL_CURB);
  EXPECT_NEAR(curb_boundaries.second.lat_offset, 10.0, kDistanceError);
  EXPECT_EQ(curb_boundaries.second.type, StationBoundaryType::VIRTUAL_CURB);

  ASSIGN_OR_DIE(const auto curb_boundaries1,
                passage.QueryCurbBoundariesAtS(82.9));
  EXPECT_NEAR(curb_boundaries1.first.lat_offset, -10.0, kDistanceError);
  EXPECT_EQ(curb_boundaries1.first.type, StationBoundaryType::VIRTUAL_CURB);
  EXPECT_NEAR(curb_boundaries1.second.lat_offset, 10.0, kDistanceError);
  EXPECT_EQ(curb_boundaries1.second.type, StationBoundaryType::VIRTUAL_CURB);

  ASSIGN_OR_DIE(const auto lane_boundaries,
                passage.QueryEnclosingLaneBoundariesAt(query_point));
  EXPECT_EQ(lane_boundaries.right->type, StationBoundaryType::VIRTUAL_CURB);
  EXPECT_EQ(lane_boundaries.left->type, StationBoundaryType::VIRTUAL_CURB);

  ASSIGN_OR_DIE(const auto tangent, passage.QueryTangentAt(query_point));
  EXPECT_NEAR(tangent.Angle(), 0.0, 1e-3);

  ASSIGN_OR_DIE(const auto frenet_lon_offset,
                passage.QueryFrenetLonOffsetAt(query_point));
  EXPECT_EQ(frenet_lon_offset.station_index.value(), 42);
  EXPECT_NEAR(frenet_lon_offset.lon_offset, -1.0, kDistanceError);

  ASSIGN_OR_DIE(const auto frenet_lat_offset,
                passage.QueryFrenetLatOffsetAt(query_point));
  EXPECT_NEAR(frenet_lat_offset, 0.0, kDistanceError);

  FrenetCoordinate frenet_coord;
  ASSIGN_OR_DIE(frenet_coord,
                passage.QueryFrenetCoordinateAt(Vec2d(85.0, -5.0)));
  EXPECT_NEAR(frenet_coord.s, 85.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -5.0, 0.1);

  ASSIGN_OR_DIE(frenet_coord, passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                                  Vec2d(85.0, -13.0)));
  EXPECT_NEAR(frenet_coord.s, 85.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -13.0, 0.1);

  ASSIGN_OR_DIE(frenet_coord,
                passage.QueryUnboundedFrenetCoordinateAt(Vec2d(130.0, -3.0)));
  EXPECT_NEAR(frenet_coord.s, 130.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -3.0, 0.1);

  ASSIGN_OR_DIE(const auto nearest_point,
                passage.FindNearestPointOnCenterLine(query_point));
  EXPECT_NEAR(nearest_point.x(), 83.0, 0.1);
  EXPECT_NEAR(nearest_point.y(), 0.0, 0.1);
}

// NOLINTNEXTLINE(readability-function-size)
TEST(DrivePassage, QuerySpacetimeTrajectory) {
  // Load map
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "b7_e2_start");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);

  const auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);

  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";

  const auto& passage = drive_passage.value();
  SendDrivePassageToCanvas(passage,
                           "test_drive_passage/query_spacetime_trajectory");

  constexpr double kDistanceError = 0.03;  // m.
  {
    const Vec2d start_pos(10.0, -3.0);
    const Vec2d velocity(10.0, 0.0);
    const auto end_pos = start_pos + velocity * 10.0;
    const double speed = velocity.Length();
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj = perception_builder.set_id("abc")
                                    .set_type(OT_VEHICLE)
                                    .set_pos(start_pos)
                                    .set_timestamp(1.0)
                                    .set_velocity(speed)
                                    .set_yaw(velocity.FastAngle())
                                    .set_length_width(4.0, 2.0)
                                    .set_box_center(start_pos)
                                    .Build();
    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(0.5)
        .set_straight_line(start_pos, end_pos,
                           /*init_v=*/speed, /*last_v=*/speed);
    const PlannerObject object = builder.Build();
    const auto& traj = object.traj(0);
    const auto states = SampleTrajectoryStates(
        traj, object.pose().pos(), object.contour(), object.bounding_box());
    std::vector<Box2d> box_vec;
    box_vec.reserve(states.size());
    for (const auto& state : states) {
      box_vec.push_back(state.box);
    }

    const auto fbox_vec_or =
        passage.BatchQueryFrenetBoxes(box_vec, /*laterally_bounded=*/true);
    EXPECT_OK(fbox_vec_or);

    const auto& fbox_vec = *fbox_vec_or;
    EXPECT_EQ(fbox_vec.size(), states.size());
    for (int i = 0; i < states.size(); ++i) {
      const auto fbox_origin_or = passage.QueryFrenetBoxAt(states[i].box);
      if (!fbox_origin_or.ok()) continue;
      EXPECT_TRUE(fbox_vec[i].has_value());
      EXPECT_NEAR(fbox_vec[i]->s_min, fbox_origin_or->s_min, kDistanceError);
      EXPECT_NEAR(fbox_vec[i]->s_max, fbox_origin_or->s_max, kDistanceError);
      EXPECT_NEAR(fbox_vec[i]->l_min, fbox_origin_or->l_min, kDistanceError);
      EXPECT_NEAR(fbox_vec[i]->l_max, fbox_origin_or->l_max, kDistanceError);
    }
  }

  {
    const Vec2d start_pos(10.0, -3.0);
    const Vec2d velocity(2.0, 0.0);
    const auto end_pos = start_pos + velocity * 10.0;
    const double speed = velocity.Length();
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj = perception_builder.set_id("abc")
                                    .set_type(OT_VEHICLE)
                                    .set_pos(start_pos)
                                    .set_timestamp(1.0)
                                    .set_velocity(speed)
                                    .set_yaw(velocity.FastAngle())
                                    .set_length_width(4.0, 2.0)
                                    .set_box_center(start_pos)
                                    .Build();
    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(0.5)
        .set_straight_line(start_pos, end_pos,
                           /*init_v=*/speed, /*last_v=*/speed);
    const PlannerObject object = builder.Build();
    const auto& traj = object.traj(0);
    const auto states = SampleTrajectoryStates(
        traj, object.pose().pos(), object.contour(), object.bounding_box());
    std::vector<Box2d> box_vec;
    box_vec.reserve(states.size());
    for (const auto& state : states) {
      box_vec.push_back(state.box);
    }

    const auto fbox_vec_or =
        passage.BatchQueryFrenetBoxes(box_vec, /*laterally_bounded=*/true);
    EXPECT_OK(fbox_vec_or);

    const auto& fbox_vec = *fbox_vec_or;
    EXPECT_EQ(fbox_vec.size(), states.size());
    for (int i = 0; i < states.size(); ++i) {
      const auto fbox_origin_or = passage.QueryFrenetBoxAt(states[i].box);
      if (!fbox_origin_or.ok()) continue;
      EXPECT_TRUE(fbox_vec[i].has_value());
      EXPECT_NEAR(fbox_vec[i]->s_min, fbox_origin_or->s_min, kDistanceError);
      EXPECT_NEAR(fbox_vec[i]->s_max, fbox_origin_or->s_max, kDistanceError);
      EXPECT_NEAR(fbox_vec[i]->l_min, fbox_origin_or->l_min, kDistanceError);
      EXPECT_NEAR(fbox_vec[i]->l_max, fbox_origin_or->l_max, kDistanceError);
    }
  }

  {
    const Vec2d start_pos(10.0, -3.0);
    const Vec2d velocity(10.0, 1.0);
    const auto end_pos = start_pos + velocity * 10.0;
    const double speed = velocity.Length();
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj = perception_builder.set_id("abc")
                                    .set_type(OT_VEHICLE)
                                    .set_pos(start_pos)
                                    .set_timestamp(1.0)
                                    .set_velocity(speed)
                                    .set_yaw(velocity.FastAngle())
                                    .set_length_width(4.0, 2.0)
                                    .set_box_center(start_pos)
                                    .Build();
    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(0.5)
        .set_straight_line(start_pos, end_pos,
                           /*init_v=*/speed, /*last_v=*/speed);
    const PlannerObject object = builder.Build();
    const auto& traj = object.traj(0);
    const auto states = SampleTrajectoryStates(
        traj, object.pose().pos(), object.contour(), object.bounding_box());
    std::vector<Box2d> box_vec;
    box_vec.reserve(states.size());
    for (const auto& state : states) {
      box_vec.push_back(state.box);
    }

    const auto fbox_vec_or =
        passage.BatchQueryFrenetBoxes(box_vec, /*laterally_bounded=*/true);
    EXPECT_OK(fbox_vec_or);

    const auto& fbox_vec = *fbox_vec_or;
    EXPECT_EQ(fbox_vec.size(), states.size());
    for (int i = 0; i < states.size(); ++i) {
      const auto fbox_origin_or = passage.QueryFrenetBoxAt(states[i].box);
      if (!fbox_origin_or.ok()) continue;
      EXPECT_TRUE(fbox_vec[i].has_value());
      EXPECT_NEAR(fbox_vec[i]->s_min, fbox_origin_or->s_min, kDistanceError);
      EXPECT_NEAR(fbox_vec[i]->s_max, fbox_origin_or->s_max, kDistanceError);
      EXPECT_NEAR(fbox_vec[i]->l_min, fbox_origin_or->l_min, kDistanceError);
      EXPECT_NEAR(fbox_vec[i]->l_max, fbox_origin_or->l_max, kDistanceError);
    }
  }
}

TEST(DrivePassage, QuerySpacetimeTrajectoryUnbounded) {
  // Load map
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(108.0);
  pose.mutable_pos_smooth()->set_y(70.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "a7_e1_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);

  const auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);

  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";

  const auto& passage = drive_passage.value();
  SendDrivePassageToCanvas(passage,
                           "test_drive_passage/query_spacetime_unbounded");

  {
    const Vec2d start_pos(112.0, 76.0);
    const Vec2d velocity(-10.0, 0.0);
    const auto end_pos = start_pos + velocity * 10.0;
    const double speed = velocity.Length();
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj = perception_builder.set_id("abc")
                                    .set_type(OT_VEHICLE)
                                    .set_pos(start_pos)
                                    .set_timestamp(1.0)
                                    .set_velocity(speed)
                                    .set_yaw(velocity.FastAngle())
                                    .set_length_width(4.0, 2.0)
                                    .set_box_center(start_pos)
                                    .Build();
    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(0.5)
        .set_straight_line(start_pos, end_pos,
                           /*init_v=*/speed, /*last_v=*/speed);
    const PlannerObject object = builder.Build();
    const auto& traj = object.traj(0);
    const auto states = SampleTrajectoryStates(
        traj, object.pose().pos(), object.contour(), object.bounding_box());
    std::vector<Box2d> box_vec;
    box_vec.reserve(states.size());
    for (const auto& state : states) {
      box_vec.push_back(state.box);
    }

    const auto fbox_vec_or =
        passage.BatchQueryFrenetBoxes(box_vec, /*laterally_bounded=*/false);
    EXPECT_OK(fbox_vec_or);

    const auto& fbox_vec = *fbox_vec_or;
    EXPECT_FALSE(fbox_vec.empty());
    EXPECT_EQ(fbox_vec.size(), states.size());
    EXPECT_TRUE(fbox_vec.front().has_value());
    EXPECT_GT(fbox_vec.front()->l_min, 4.0);
  }
}

}  // namespace
}  // namespace qcraft::planner
