#include "onboard/planner/router/drive_passage_builder.h"

#include <optional>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft::planner {

namespace {

constexpr double kDrivePassageMaxForwardExtendLength = 10.0;  // m.

TEST(DrivePassageBuilder, BuildStraightDrivePassage) {
  const auto route_result = CreateAStraightForwardRouteInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();

  const auto& lane_path = route_result.route_lane_path.lane_paths().front();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_result.route_sections,
                                            lane_path,
                                            kDrivePassageKeepBehindLength);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, lane_path, *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_result.route_sections.planning_horizon(psmm),
      route_result.route_sections.destination(),
      /*all_lanes_virtual=*/false, /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_OK(dp_or);
  EXPECT_NEAR(dp_or->end_s(),
              lane_path.length() + kDrivePassageMaxForwardExtendLength,
              kRouteStationUnitStep);

  SendDrivePassageToCanvas(dp_or.value(), "test/straight_drive_passage");
}

TEST(DrivePassageBuilder, BuildUturnDrivePassage) {
  const auto route_result = CreateAUturnRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();

  const auto& lane_path = route_result.route_lane_path.lane_paths().front();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_result.route_sections,
                                            lane_path,
                                            kDrivePassageKeepBehindLength);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, lane_path, *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_result.route_sections.planning_horizon(psmm),
      route_result.route_sections.destination(),
      /*all_lanes_virtual=*/false, /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_OK(dp_or);
  EXPECT_NEAR(dp_or->end_s(),
              lane_path.length() + kDrivePassageMaxForwardExtendLength,
              2.0 * kRouteStationUnitStep);

  SendDrivePassageToCanvas(dp_or.value(), "test/uturn_drive_passage");
}

TEST(DrivePassageBuilder, BuildLeftTurnDrivePassage) {
  const auto route_result = CreateALeftTurnRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();

  const auto& lane_path = route_result.route_lane_path.lane_paths().front();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_result.route_sections,
                                            lane_path,
                                            kDrivePassageKeepBehindLength);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, lane_path, *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_result.route_sections.planning_horizon(psmm),
      route_result.route_sections.destination(),
      /*all_lanes_virtual=*/false, /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_OK(dp_or);
  EXPECT_NEAR(dp_or->end_s(),
              lane_path.length() + kDrivePassageMaxForwardExtendLength,
              kRouteStationUnitStep);

  SendDrivePassageToCanvas(dp_or.value(), "test/left_turn_drive_passage");
}

TEST(DrivePassageBuilder, BuildLongDrivePassage) {
  const auto route_result = CreateAForkLaneRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();

  const double planning_horizon =
      route_result.route_sections.planning_horizon(psmm);
  const auto& lane_path = route_result.route_lane_path.lane_paths().front();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_result.route_sections,
                                            lane_path,
                                            kDrivePassageKeepBehindLength);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, lane_path, *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(), planning_horizon,
      route_result.route_sections.destination(),
      /*all_lanes_virtual=*/false, /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_OK(dp_or);
  EXPECT_NEAR(dp_or->end_s(), planning_horizon, 2.0 * kRouteStationUnitStep);

  SendDrivePassageToCanvas(dp_or.value(), "test/long_drive_passage");
}

TEST(DrivePassageBuilder, BuildRampFittedDrivePassage) {
  // Load map and create optimized route path.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  mapping::LanePath lane_path(smm, {mapping::ElementId(30880)},
                              /*start_fraction=*/0.85, /*end_fraction=*/1.0);
  const auto dp_or = BuildDrivePassageFromLanePath(
      psmm, lane_path, /*step_s=*/2.0, /*avoid_loop=*/true,
      /*backward_extend_len=*/0.0, /*required_planning_horizon=*/150.0,
      /*required_backward_len=*/0.0,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_OK(dp_or);
  const auto& passage = *dp_or;

  EXPECT_EQ(passage.last_real_station_index().value(), 23);

  SendDrivePassageToCanvas(passage, "test/ramp_fitted_drive_passage");
}

TEST(DrivePassageBuilder, BuildDrivePassageForPredictionWithLaneBoundaryCache) {
  // Load map and create optimized route path.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  mapping::LanePath lane_path(smm, {mapping::ElementId(30880)},
                              /*start_fraction=*/0.85, /*end_fraction=*/1.0);
  const auto dp_or = BuildDrivePassageForPredictionWithLaneBoundaryCache(
      psmm, lane_path, {}, /*step_s=*/2.0, /*avoid_loop=*/true,
      /*backward_extend_len=*/0.0);
  ASSERT_OK(dp_or);
  const auto& passage = *dp_or;

  EXPECT_EQ(passage.last_real_station_index().value(), 23);

  SendDrivePassageToCanvas(passage, "test/ramp_fitted_drive_passage");
}

}  // namespace

}  // namespace  qcraft::planner
