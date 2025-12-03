#include "onboard/planner/decision/solid_line_within_boundary.h"

#include <algorithm>
#include <optional>
#include <string>

#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft::planner {

namespace {

void DrawAvoidLineToCanvas(
    absl::Span<const ConstraintProto::AvoidLineProto> avoid_lines,
    const std::string& channel) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto& avoid_line : avoid_lines) {
    std::vector<Vec2d> xy_points_vec;
    xy_points_vec.reserve(avoid_line.xy_points_size());
    for (const auto& point : avoid_line.xy_points()) {
      xy_points_vec.emplace_back(point);
    }
    for (int i = 0; i + 1 < xy_points_vec.size(); ++i) {
      canvas.DrawLine(Vec3d(xy_points_vec[i], 0.1),
                      Vec3d(xy_points_vec[i + 1], 0.1), vis::Color::kRed, 2);
    }
    canvas.DrawLine(Vec3d(xy_points_vec.back(), 0.1),
                    Vec3d(xy_points_vec.front(), 0.1), vis::Color::kRed, 2);
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

TEST(BuildPathBoundaryFromPose, SolidLineTest) {
  const TestRouteResult route_result =
      CreateAStraightForwardRouteWithSolidInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();
  const auto dp_or =
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                        route_result.route_lane_path.lane_paths().front(),
                        route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(psmm),
                        route_result.route_sections.destination(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt);

  EXPECT_OK(dp_or);
  SendDrivePassageToCanvas(dp_or.value(), "drive_passage");

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  PlannerObjectManager object_mgr;
  const auto st_traj_manager =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;

  {
    // solid_line_path_boundary_borrow
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(
        route_result.pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(
        route_result.pose.pos_smooth().y());
    plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
    plan_start_point.set_v(route_result.pose.vel_body().x());

    const auto path_bound_or = BuildPathBoundaryFromPose(
        psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_manager,
        lc_state, smooth_result_map,
        /*borrow_lane_boundary=*/true, /*should_smooth=*/false,
        /*unsafe_object_ids=*/{});

    EXPECT_OK(path_bound_or);

    DrawPathSlBoundaryToCanvas(path_bound_or.value(), "borrow/path_boundary");

    const auto avoid_region_or = BuildSolidLineWithinBoundaryConstraint(
        *dp_or, *path_bound_or, plan_start_point);

    EXPECT_OK(avoid_region_or);
    EXPECT_GE(avoid_region_or->size(), 2);
    DrawAvoidLineToCanvas(*avoid_region_or, "borrow/avoid_line");
  }

  {
    // solid_line_path_boundary_lc
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(
        route_result.pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(3.23);
    plan_start_point.mutable_path_point()->set_theta(-0.17);
    plan_start_point.set_v(0.0);

    lc_state.set_lc_left(false);
    lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
    const auto path_bound_or = BuildPathBoundaryFromPose(
        psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_manager,
        lc_state, smooth_result_map,
        /*borrow_lane_boundary=*/false, /*should_smooth=*/false,
        /*unsafe_object_ids=*/{});

    EXPECT_OK(path_bound_or);

    DrawPathSlBoundaryToCanvas(path_bound_or.value(), "lc/path_boundary");

    const auto avoid_region_or = BuildSolidLineWithinBoundaryConstraint(
        *dp_or, *path_bound_or, plan_start_point);

    EXPECT_OK(avoid_region_or);
    EXPECT_EQ(avoid_region_or->size(), 1);

    const auto& xy_points = (*avoid_region_or)[0].xy_points();
    for (const auto& point : xy_points) {
      EXPECT_NEAR(point.y(), 1.7, 0.1);
    }

    DrawAvoidLineToCanvas(*avoid_region_or, "lc/avoid_line");
  }
}

}  // namespace
}  // namespace qcraft::planner
