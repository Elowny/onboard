#include "onboard/planner/assist/planner_odc_generator.h"

#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/params/param_manager.h"
#include "onboard/proto/adasis.pb.h"
// #include "onboard/planner/router/plot_util.h"

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
TEST(PlannerOdcGeneratorTest, WithoutLanePathTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");

  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();

  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  absl::Time plan_time = absl::Now();

  // Normal test.
  {
    const PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(20.0, 0.0),
                   /*heading=*/0.0, Vec2d(11.0, 0.0));

    ASSIGN_OR_DIE(
        auto odc,
        GeneratePlannerOdcByPose(OdcGeneratorInput{
            .psmm = &psmm, .pose = &sdc_pose, .vehicle_geom = &vehicle_geom}));
    EXPECT_FALSE(odc.is_av_overlap_boundary.has_value());
    EXPECT_TRUE(odc.is_valid_both_side_boundary.has_value());
    EXPECT_TRUE(*odc.is_valid_both_side_boundary);
    EXPECT_TRUE(odc.current_lane_width.has_value());
    EXPECT_NEAR(*odc.current_lane_width, 3.57, 1e-2);
    EXPECT_TRUE(odc.current_lane_average_curvature_radius.has_value());
    EXPECT_GT(*odc.current_lane_average_curvature_radius, 1000.0);
    EXPECT_TRUE(odc.current_lane_length.has_value());
    EXPECT_NEAR(*odc.current_lane_length, 100.0, 1e-2);
    EXPECT_TRUE(odc.lane_path_lost.has_value());
    EXPECT_FALSE(*odc.lane_path_lost);
  }

  // No valid both side boundary test.
  {
    const PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(84.0, 0.0),
                   /*heading=*/0.0, Vec2d(11.0, 0.0));

    ASSIGN_OR_DIE(
        auto odc,
        GeneratePlannerOdcByPose(OdcGeneratorInput{
            .psmm = &psmm, .pose = &sdc_pose, .vehicle_geom = &vehicle_geom}));
    EXPECT_FALSE(odc.is_av_overlap_boundary.has_value());
    EXPECT_TRUE(odc.is_valid_both_side_boundary.has_value());
    EXPECT_FALSE(*odc.is_valid_both_side_boundary);
    EXPECT_FALSE(odc.current_lane_width.has_value());
    EXPECT_TRUE(odc.current_lane_average_curvature_radius.has_value());
    EXPECT_GT(*odc.current_lane_average_curvature_radius, 1000.0);
    EXPECT_TRUE(odc.current_lane_length.has_value());
    EXPECT_NEAR(*odc.current_lane_length, 100.0, 1e-2);
  }

  // Curvature radius test.
  {
    const PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(-22.887, 9.579),
                   /*heading=*/-1.067, Vec2d(11.0, 0.0));

    ASSIGN_OR_DIE(
        auto odc,
        GeneratePlannerOdcByPose(OdcGeneratorInput{
            .psmm = &psmm, .pose = &sdc_pose, .vehicle_geom = &vehicle_geom}));
    EXPECT_FALSE(odc.is_av_overlap_boundary.has_value());
    EXPECT_TRUE(odc.is_valid_both_side_boundary.has_value());
    EXPECT_FALSE(*odc.is_valid_both_side_boundary);
    EXPECT_FALSE(odc.current_lane_width.has_value());
    EXPECT_TRUE(odc.current_lane_average_curvature_radius.has_value());
    EXPECT_NEAR(*odc.current_lane_average_curvature_radius, 19.11, 1e-2);
    EXPECT_TRUE(odc.current_lane_length.has_value());
    EXPECT_NEAR(*odc.current_lane_length, 100.0, 1e-2);
  }

  {
    // Lane path lost test.
    const PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(-132.4, -580.5),
                   /*heading=*/-3.13, Vec2d(5.0, 0.0));

    ASSIGN_OR_DIE(
        auto odc,
        GeneratePlannerOdcByPose(OdcGeneratorInput{
            .psmm = &psmm, .pose = &sdc_pose, .vehicle_geom = &vehicle_geom}));
    EXPECT_TRUE(odc.lane_path_lost.has_value());
    EXPECT_TRUE(*odc.lane_path_lost);
  }
}

// NOLINTNEXTLINE(readability-function-size)
TEST(PlannerOdcGeneratorTest, WithLanePathTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");

  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();

  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  absl::Time plan_time = absl::Now();

  // Normal test.
  {
    const PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(20.0, 0.0),
                   /*heading=*/0.0, Vec2d(11.0, 0.0));

    const mapping::LanePath target_lane_path =
        mapping::LanePath(smm, /*lane_ids=*/
                          {mapping::ElementId(2448), mapping::ElementId(1),
                           mapping::ElementId(34)},
                          /*start_fraction=*/0.36, /*end_fraction=*/1.0);

    ASSIGN_OR_DIE(auto odc, GeneratePlannerOdcByLanePath(OdcGeneratorInput{
                                .psmm = &psmm,
                                .pose = &sdc_pose,
                                .vehicle_geom = &vehicle_geom,
                                .target_lane_path = &target_lane_path}));
    EXPECT_FALSE(odc.is_av_overlap_boundary.has_value());
    EXPECT_TRUE(odc.is_valid_both_side_boundary.has_value());
    EXPECT_TRUE(*odc.is_valid_both_side_boundary);
    EXPECT_TRUE(odc.current_lane_width.has_value());
    EXPECT_NEAR(*odc.current_lane_width, 3.58, 1e-2);
    EXPECT_TRUE(odc.current_lane_average_curvature_radius.has_value());
    EXPECT_GT(*odc.current_lane_average_curvature_radius, 1000.0);
    EXPECT_TRUE(odc.current_lane_length.has_value());
    EXPECT_NEAR(*odc.current_lane_length, 95.43, 1e-2);
  }

  // Zero speed test.
  {
    PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(20.0, 0.0),
                   /*heading=*/0.0, Vec2d(0.0, 0.0));
    sdc_pose.mutable_accel_body()->set_x(0.01);

    const mapping::LanePath target_lane_path =
        mapping::LanePath(smm, /*lane_ids=*/
                          {mapping::ElementId(2448), mapping::ElementId(1),
                           mapping::ElementId(34)},
                          /*start_fraction=*/0.36, /*end_fraction=*/1.0);

    ASSIGN_OR_DIE(auto odc, GeneratePlannerOdcByLanePath(OdcGeneratorInput{
                                .psmm = &psmm,
                                .pose = &sdc_pose,
                                .vehicle_geom = &vehicle_geom,
                                .target_lane_path = &target_lane_path}));
    EXPECT_FALSE(odc.is_av_overlap_boundary.has_value());
    EXPECT_TRUE(odc.is_valid_both_side_boundary.has_value());
    EXPECT_TRUE(*odc.is_valid_both_side_boundary);
    EXPECT_TRUE(odc.current_lane_width.has_value());
    EXPECT_NEAR(*odc.current_lane_width, 3.58, 1e-2);
    EXPECT_TRUE(odc.current_lane_average_curvature_radius.has_value());
    EXPECT_GT(*odc.current_lane_average_curvature_radius, 1000.0);
    EXPECT_TRUE(odc.current_lane_length.has_value());
    EXPECT_NEAR(*odc.current_lane_length, 95.43, 1e-2);
  }

  // Lane change test.
  {
    const PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(0.0, 0.0),
                   /*heading=*/0.0, Vec2d(11.0, 0.0));
    const mapping::LanePath target_lane_path =
        mapping::LanePath(smm, /*lane_ids=*/
                          {mapping::ElementId(3), mapping::ElementId(950),
                           mapping::ElementId(35)},
                          /*start_fraction=*/0.0, /*end_fraction=*/1.0);

    ASSIGN_OR_DIE(auto odc, GeneratePlannerOdcByLanePath(OdcGeneratorInput{
                                .psmm = &psmm,
                                .pose = &sdc_pose,
                                .vehicle_geom = &vehicle_geom,
                                .target_lane_path = &target_lane_path}));
    EXPECT_FALSE(odc.is_av_overlap_boundary.has_value());
    EXPECT_TRUE(odc.is_valid_both_side_boundary.has_value());
    EXPECT_TRUE(*odc.is_valid_both_side_boundary);
    EXPECT_TRUE(odc.current_lane_width.has_value());
    EXPECT_NEAR(*odc.current_lane_width, 3.46, 1e-2);
    EXPECT_TRUE(odc.current_lane_average_curvature_radius.has_value());
    EXPECT_GT(*odc.current_lane_average_curvature_radius, 1000.0);
    EXPECT_TRUE(odc.current_lane_length.has_value());
    EXPECT_NEAR(*odc.current_lane_length, 115.47, 1e-2);
  }

  // Curvature radius test.
  {
    const PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(-24.351, 21.0),
                   /*heading=*/-1.067, Vec2d(11.0, 0.0));
    const mapping::LanePath target_lane_path =
        mapping::LanePath(smm, /*lane_ids=*/
                          {mapping::ElementId(57), mapping::ElementId(3),
                           mapping::ElementId(950)},
                          /*start_fraction=*/0.0, /*end_fraction=*/1.0);

    ASSIGN_OR_DIE(auto odc, GeneratePlannerOdcByLanePath(OdcGeneratorInput{
                                .psmm = &psmm,
                                .pose = &sdc_pose,
                                .vehicle_geom = &vehicle_geom,
                                .target_lane_path = &target_lane_path}));
    EXPECT_FALSE(odc.is_av_overlap_boundary.has_value());
    EXPECT_TRUE(odc.is_valid_both_side_boundary.has_value());
    EXPECT_FALSE(*odc.is_valid_both_side_boundary);
    EXPECT_FALSE(odc.current_lane_width.has_value());
    EXPECT_TRUE(odc.current_lane_average_curvature_radius.has_value());
    EXPECT_NEAR(*odc.current_lane_average_curvature_radius, 25.054, 1e-2);
    EXPECT_TRUE(odc.current_lane_length.has_value());
    EXPECT_NEAR(*odc.current_lane_length, 109.71, 1e-2);
  }
}

TEST(PlannerOdcGeneratorTest, LccMinRequiredLaneLength) {
  absl::Time plan_time = absl::Now();
  {
    PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(20.0, 0.0),
                   /*heading=*/0.0, Vec2d(10.0, 0.0));
    const double min_required_lane_length =
        CalculateLccMinRequiredLaneLength(sdc_pose);
    EXPECT_NEAR(min_required_lane_length, 20.0, 1e-2);
  }

  {
    PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(20.0, 0.0),
                   /*heading=*/0.0, Vec2d(25.0, 0.0));
    const double min_required_lane_length =
        CalculateLccMinRequiredLaneLength(sdc_pose);
    EXPECT_NEAR(min_required_lane_length, 40.0, 1e-2);
  }
}

TEST(PlannerOdcGeneratorTest, GeneratePlannerOdcByRouteTest) {
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(0.0, 0.0), 0.0, Vec2d(11.0, 0.0));

  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b7_e2_end");
  auto route_sections = RouteSectionsFromCompositeLanePath(*smm, route_path);

  RouteNaviInfo route_navi_info;
  auto& route_lane_info_map = route_navi_info.route_lane_info_map;
  route_lane_info_map.insert(
      {mapping::ElementId(2448), RouteNaviInfo::RouteLaneInfo()});
  RouteManagerOutput route_mgr_output{
      .route_sections_from_current = std::move(route_sections),
      .route_navi_info = std::move(route_navi_info),
      .status = RouteManagerOutputProto::NORMAL,
  };

  ASSIGN_OR_DIE(auto odc, GeneratePlannerOdcByRoute(&psmm, &route_mgr_output));
  // SendRouteSectionsAreaToCanvas(psmm, route_sections, "route_sections");

  EXPECT_FALSE(odc.distance_to_toll.has_value());
  EXPECT_TRUE(odc.distance_to_traffic_light.has_value());
  EXPECT_NEAR(*odc.distance_to_traffic_light, 69.94, 1e-1);
}

}  // namespace
}  // namespace qcraft::planner
