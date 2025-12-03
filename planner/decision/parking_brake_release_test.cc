#include "onboard/planner/decision/parking_brake_release.h"

#include <memory>
#include <optional>
#include <vector>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/global/clock.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace planner {
namespace {
TEST(BuildParkingBrakeReleaseConstraintTest,
     BuildParkingBrakeReleaseConstraint) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(124.0);
  pose.mutable_pos_smooth()->set_y(63.0);
  const auto route_path =
      RoutingToNameSpot(*smm, cc, pose, /*name_spot=*/"end");
  SendRouteLanePathToCanvas(
      psmm, route_path, "test/route_build_parking_brake_release_constraints");

  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  SendDrivePassageToCanvas(
      *drive_passage,
      "test/drive_passage_build_parking_brake_release_constraints");

  const absl::Time clock_now = Clock::Now();
  const auto& passage = *drive_passage;
  const absl::Time parking_brake_release_time_1 = Clock::Now();
  const auto parking_brake_release_1 = BuildParkingBrakeReleaseConstraint(
      vehicle_geometry_params, passage, parking_brake_release_time_1,
      clock_now);
  EXPECT_TRUE(parking_brake_release_1.ok());

  const absl::Time parking_brake_release_time_2 =
      clock_now - absl::Seconds(1.1);
  const auto parking_brake_release_2 = BuildParkingBrakeReleaseConstraint(
      vehicle_geometry_params, passage, parking_brake_release_time_2,
      clock_now);
  EXPECT_FALSE(parking_brake_release_2.ok());
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
