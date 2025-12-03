#include "onboard/planner/assist/lcc_map_builder.h"

#include <utility>
#include <vector>

#include "absl/status/status.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

// NOLINTNEXTLINE
TEST(LccMapBuilderTest, BuildLocalLaneMap) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  BuildLocalMapInput input;
  input.psmm = &psmm;
  input.alc_state = QALCState::ALC_STANDBY_ENABLE;
  input.lc_direction = LaneChangeDirection::LCD_NONE;
  input.projection_range =
      kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength;
  input.keep_behind_length = kDrivePassageKeepBehindLength;
  // Left/Center/Right lane path exist.
  {
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(30.0);
    pose.mutable_pos_smooth()->set_y(0.0);
    constexpr double kAlccTargetLanePathRequiredLength = 200.0;  // m.
    constexpr double kAlccLaneMapCutOffLength = 20.0;            // m.
    ASSIGN_OR_DIE(auto lane_path,
                  FindNearestLanePathFromEgoPose(
                      pose, psmm, kAlccTargetLanePathRequiredLength));

    ASSIGN_OR_DIE(const auto dm,
                  BuildDrivingMapByRouteOnOfflineMap(
                      psmm, RouteSections::BuildFromLanePath(psmm, lane_path)));
    input.driving_map_topo = &dm;
    input.origin_lane_path = &lane_path;
    input.cut_off_length = kAlccLaneMapCutOffLength;

    ASSIGN_OR_DIE(auto local_lane_map, BuildLocalLaneMap(input));
    const auto left_lane_path = std::move(local_lane_map[0]);
    lane_path = std::move(local_lane_map[1]);
    const auto right_lane_path = std::move(local_lane_map[2]);

    // Left lane path test.
    EXPECT_NEAR(left_lane_path.start_fraction(), 0.536426, 1e-2);
    EXPECT_NEAR(left_lane_path.end_fraction(), 0.962513, 1e-2);
    EXPECT_EQ(left_lane_path.lane_ids().front(), mapping::ElementId(2));
    EXPECT_EQ(left_lane_path.lane_ids().back(), mapping::ElementId(2470));
    // EXPECT_NEAR(left_lane_path.length(), kAlccTargetLanePathRequiredLength,
    //             1e-2);
    // Center lane path test.
    EXPECT_NEAR(lane_path.start_fraction(), 0.536426, 1e-2);
    EXPECT_NEAR(lane_path.end_fraction(), 0.961707, 1e-2);
    EXPECT_EQ(lane_path.lane_ids().front(), mapping::ElementId(2448));
    EXPECT_EQ(lane_path.lane_ids().back(), mapping::ElementId(2471));
    EXPECT_NEAR(lane_path.length(), kAlccTargetLanePathRequiredLength, 1e-2);
    // Right lane path test.
    EXPECT_NEAR(right_lane_path.start_fraction(), 0.536426, 1e-2);
    EXPECT_NEAR(right_lane_path.end_fraction(), 0.962189, 1e-2);
    EXPECT_EQ(right_lane_path.lane_ids().front(), mapping::ElementId(3));
    EXPECT_EQ(right_lane_path.lane_ids().back(), mapping::ElementId(2472));
    // EXPECT_NEAR(right_lane_path.length(), kAlccTargetLanePathRequiredLength,
    //             1e-2);
  }

  // Center/Right lane path exist.
  {
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(0.0);
    pose.mutable_pos_smooth()->set_y(70.0);
    constexpr double kAlccTargetLanePathRequiredLength = 200.0;  // m.
    constexpr double kAlccLaneMapCutOffLength = 20.0;            // m.
    ASSIGN_OR_DIE(auto lane_path,
                  FindNearestLanePathFromEgoPose(
                      pose, psmm, kAlccTargetLanePathRequiredLength));

    ASSIGN_OR_DIE(const auto dm,
                  BuildDrivingMapByRouteOnOfflineMap(
                      psmm, RouteSections::BuildFromLanePath(psmm, lane_path)));
    input.driving_map_topo = &dm;
    input.origin_lane_path = &lane_path;
    input.cut_off_length = kAlccLaneMapCutOffLength;
    ASSIGN_OR_DIE(auto local_lane_map, BuildLocalLaneMap(input));

    const auto left_lane_path = std::move(local_lane_map[0]);
    lane_path = std::move(local_lane_map[1]);
    const auto right_lane_path = std::move(local_lane_map[2]);

    // Left lane path test.
    EXPECT_TRUE(left_lane_path.IsEmpty());
    // Center lane path test.
    EXPECT_NEAR(lane_path.start_fraction(), 0.996994, 1e-2);
    EXPECT_NEAR(lane_path.end_fraction(), 0.335336, 1e-2);
    EXPECT_EQ(lane_path.lane_ids().front(), mapping::ElementId(84));
    EXPECT_EQ(lane_path.lane_ids().back(), mapping::ElementId(81));
    EXPECT_NEAR(lane_path.length(), kAlccTargetLanePathRequiredLength, 1e-2);
    // Right lane path test.
    EXPECT_NEAR(right_lane_path.start_fraction(), 0.996994, 1e-2);
    // EXPECT_NEAR(right_lane_path.end_fraction(), 0.708981, 1e-2);
    EXPECT_EQ(right_lane_path.lane_ids().front(), mapping::ElementId(83));
    // EXPECT_EQ(right_lane_path.lane_ids().back(), mapping::ElementId(2491));
    // EXPECT_NEAR(right_lane_path.length(), kAlccTargetLanePathRequiredLength,
    //             1e-2);
  }

  // Left/Center lane path exist.
  {
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(115.59);
    pose.mutable_pos_smooth()->set_y(-3.73);
    constexpr double kAlccTargetLanePathRequiredLength = 200.0;  // m.
    constexpr double kAlccLaneMapCutOffLength = 20.0;            // m.
    ASSIGN_OR_DIE(auto lane_path,
                  FindNearestLanePathFromEgoPose(
                      pose, psmm, kAlccTargetLanePathRequiredLength));
    ASSIGN_OR_DIE(const auto dm,
                  BuildDrivingMapByRouteOnOfflineMap(
                      psmm, RouteSections::BuildFromLanePath(psmm, lane_path)));
    input.driving_map_topo = &dm;
    input.origin_lane_path = &lane_path;
    input.cut_off_length = kAlccLaneMapCutOffLength;
    ASSIGN_OR_DIE(auto local_lane_map, BuildLocalLaneMap(input));
    const auto left_lane_path = std::move(local_lane_map[0]);
    lane_path = std::move(local_lane_map[1]);
    const auto right_lane_path = std::move(local_lane_map[2]);

    // Left lane path test.
    EXPECT_NEAR(left_lane_path.start_fraction(), 0.0, 1e-2);
    EXPECT_EQ(left_lane_path.lane_ids().front(), mapping::ElementId(2471));
    // Center lane path test.
    EXPECT_NEAR(lane_path.start_fraction(), 0.0, 1e-2);
    EXPECT_NEAR(lane_path.end_fraction(), 0.417644, 1e-2);
    EXPECT_EQ(lane_path.lane_ids().front(), mapping::ElementId(2472));
    EXPECT_EQ(lane_path.lane_ids().back(), mapping::ElementId(159));
    EXPECT_NEAR(lane_path.length(), kAlccTargetLanePathRequiredLength, 1e-2);
    // Right lane path test.
    EXPECT_TRUE(right_lane_path.IsEmpty());
  }
}

TEST(LccMapBuilderTest, UpdateLccDrivingMapByOfflineMap) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  {
    const mapping::LanePath origin_lp(smm, {mapping::ElementId(2448)}, 0.0,
                                      1.0);
    const auto result_or = UpdateLccDrivingMapByOfflineMap(
        psmm, origin_lp, mapping::LanePath(), Vec2d(0.0, 0.0));

    EXPECT_OK(result_or);
    // TODO(weijun): Check lane path
  }

  {
    const mapping::LanePath origin_lp(smm, {mapping::ElementId(13570)}, 0.5,
                                      1.0);
    const mapping::LanePath target_lp(smm, {mapping::ElementId(13575)}, 0.5,
                                      1.0);
    const auto result_or = UpdateLccDrivingMapByOfflineMap(
        psmm, origin_lp, target_lp, Vec2d(234.59, -963.2));

    EXPECT_OK(result_or) << result_or.status().message();
    // TODO(weijun): Check lane path
  }
}

}  // namespace

}  // namespace qcraft::planner
