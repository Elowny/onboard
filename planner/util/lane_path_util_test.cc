#include "onboard/planner/util/lane_path_util.h"

#include <math.h>

#include <memory>
#include <utility>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
TEST(LanePathUtilTest, BuildLanePathFromData) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  {
    auto res_or = BuildLanePathFromData(
        mapping::LanePathData(0.0, 1.0, {mapping::ElementId(2448)}), psmm);

    EXPECT_OK(res_or);
    const auto lane_path = std::move(res_or).value();
    EXPECT_NEAR(lane_path.length(), 56.14, 1e-1);
    EXPECT_NEAR(lane_path.end_s(0), 56.14, 1e-1);
  }

  {
    auto res_or = BuildLanePathFromData(
        mapping::LanePathData(
            0.0, 0.5, {mapping::ElementId(2448), mapping::ElementId(1)}),
        psmm);

    EXPECT_OK(res_or);
    const auto lane_path = std::move(res_or).value();
    EXPECT_NEAR(lane_path.length(), 63.14, 1e-1);
    EXPECT_NEAR(lane_path.end_s(0), 56.14, 1e-1);
    EXPECT_NEAR(lane_path.end_s(1), 63.14, 1e-1);
  }
}

TEST(LanePathUtilTest, IsLanePathConnectedTo) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  {
    const auto lane_path = BuildLanePathFromData(
        mapping::LanePathData(
            0.0, 0.5, {mapping::ElementId(2448), mapping::ElementId(1)}),
        psmm);
    EXPECT_OK(lane_path);
    const auto other_lane_path = BuildLanePathFromData(
        mapping::LanePathData(0.5, 0.6, {mapping::ElementId(1)}), psmm);
    EXPECT_OK(other_lane_path);

    EXPECT_TRUE(IsLanePathConnectedTo(*lane_path, *other_lane_path, psmm));
  }

  {
    const auto lane_path = BuildLanePathFromData(
        mapping::LanePathData(0.0, 1.0, {mapping::ElementId(2448)}), psmm);
    EXPECT_OK(lane_path);
    const auto other_lane_path = BuildLanePathFromData(
        mapping::LanePathData(0.0, 0.6, {mapping::ElementId(1)}), psmm);
    EXPECT_OK(other_lane_path);

    EXPECT_TRUE(IsLanePathConnectedTo(*lane_path, *other_lane_path, psmm));
  }

  {
    const auto lane_path = BuildLanePathFromData(
        mapping::LanePathData(0.0, 1.0, {mapping::ElementId(2448)}), psmm);
    EXPECT_OK(lane_path);
    const auto other_lane_path = BuildLanePathFromData(
        mapping::LanePathData(0.001, 0.6, {mapping::ElementId(1)}), psmm);
    EXPECT_OK(other_lane_path);

    EXPECT_FALSE(IsLanePathConnectedTo(*lane_path, *other_lane_path, psmm));
  }

  {
    const auto lane_path = BuildLanePathFromData(
        mapping::LanePathData(
            0.0, 0.01, {mapping::ElementId(2448), mapping::ElementId(1)}),
        psmm);
    EXPECT_OK(lane_path);
    const auto other_lane_path = BuildLanePathFromData(
        mapping::LanePathData(0.0, 0.6, {mapping::ElementId(1)}), psmm);
    EXPECT_OK(other_lane_path);

    EXPECT_FALSE(IsLanePathConnectedTo(*lane_path, *other_lane_path, psmm));
  }
}

TEST(LanePathUtilTest, ConnectLanePath) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  {
    const auto lane_path = BuildLanePathFromData(
        mapping::LanePathData(
            0.0, 0.01, {mapping::ElementId(2448), mapping::ElementId(1)}),
        psmm);
    EXPECT_OK(lane_path);
    const auto other_lane_path = BuildLanePathFromData(
        mapping::LanePathData(0.01, 0.6, {mapping::ElementId(1)}), psmm);
    EXPECT_OK(other_lane_path);
    const auto new_lane_path =
        ConnectLanePath(*lane_path, *other_lane_path, psmm);
    EXPECT_OK(new_lane_path);
    EXPECT_EQ(new_lane_path->size(), 2);
  }

  {
    const auto lane_path = BuildLanePathFromData(
        mapping::LanePathData(0.0, 1.0, {mapping::ElementId(2448)}), psmm);
    EXPECT_OK(lane_path);
    const auto other_lane_path = BuildLanePathFromData(
        mapping::LanePathData(0.0, 0.6, {mapping::ElementId(1)}), psmm);
    EXPECT_OK(other_lane_path);
    const auto new_lane_path =
        ConnectLanePath(*lane_path, *other_lane_path, psmm);
    EXPECT_OK(new_lane_path);
    EXPECT_EQ(new_lane_path->size(), 2);
  }
}
TEST(LanePathUtilTest, BackwardExtendTargetAlignedRouteLanePathTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.5, 0.3,
                            {mapping::ElementId(2448), mapping::ElementId(1)}),
      psmm);
  EXPECT_OK(lane_path_or);

  const auto res = BackwardExtendTargetAlignedRouteLanePath(
      psmm, /*left=*/false, mapping::LanePoint(mapping::ElementId(2), 0.9),
      *lane_path_or);
  EXPECT_NEAR(res.length(), 22.3, 0.1);
  EXPECT_EQ(res.start_fraction(), 0.5);
}

TEST(LanePathUtilTest, BackwardExtendLanePathTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.5, 0.3,
                            {mapping::ElementId(2448), mapping::ElementId(1)}),
      psmm);
  EXPECT_OK(lane_path_or);

  {
    const auto res_lane_path =
        BackwardExtendLanePath(psmm, *lane_path_or, 20.0);

    const auto* lane_info_ptr =
        psmm.FindLaneInfoOrNull(mapping::ElementId(2448));
    EXPECT_TRUE(lane_info_ptr != nullptr);

    const double dec_fraction = 20.0 / lane_info_ptr->length();
    EXPECT_NEAR(res_lane_path.start_fraction(), 0.5 - dec_fraction, 0.001);
  }

  {
    const auto res_lane_path =
        BackwardExtendLanePath(psmm, *lane_path_or, 50.0);
    EXPECT_EQ(res_lane_path.size(), 3);
    EXPECT_EQ(res_lane_path.front().lane_id(), mapping::ElementId(97));
  }
}

TEST(LanePathUtilTest,
     ForwardExtendLanePathWithMinimumHeadingDiffWithVirtualTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.5, 0.3,
                            {mapping::ElementId(2448), mapping::ElementId(1)}),
      psmm);
  EXPECT_OK(lane_path_or);

  const auto res = ForwardExtendLanePathWithMinimumHeadingDiff(
      psmm, *lane_path_or, 20.0, /*allow_virtual=*/true);
  EXPECT_OK(res);

  EXPECT_NEAR(res->length(), lane_path_or->length() + 20.0, 0.01);
  EXPECT_EQ(res->size(), 3);

  const auto res_2 = ForwardExtendLanePathWithMinimumHeadingDiff(
      psmm, *lane_path_or, /*extend_len=*/0.0, /*allow_virtual=*/true);
  EXPECT_NEAR(res_2->length(), lane_path_or->length(), 0.01);
  EXPECT_EQ(res_2->size(), 2);

  const auto lane_path_or_2 = BuildLanePathFromData(
      mapping::LanePathData(0.1, 0.8, {mapping::ElementId(7018)}), psmm);

  EXPECT_OK(lane_path_or_2);
  const auto res_3 = ForwardExtendLanePathWithMinimumHeadingDiff(
      psmm, *lane_path_or_2, /*extend_len=*/50.0, /*allow_virtual=*/true);
  EXPECT_OK(res_3);
  // Should extend to the end of the lane.
  EXPECT_EQ(res_3->back().fraction(), 1.0);
  EXPECT_NEAR(res_3->length(), lane_path_or_2->length() * 0.9 / 0.7, 0.01);
}

TEST(LanePathUtilTest,
     ForwardExtendLanePathWithMinimumHeadingDiffWithoutVirtualTest) {
  SetMap("dojo");
  auto smm = std::make_shared<SemanticMapManager>();
  smm->LoadWholeMap().Build();

  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.1, 0.8, {mapping::ElementId(6781)}), psmm);
  EXPECT_OK(lane_path_or);
  const auto res = ForwardExtendLanePathWithMinimumHeadingDiff(
      psmm, *lane_path_or, /*extend_len=*/20.0, /*allow_virtual=*/false);
  EXPECT_OK(res);
  // Should not forward extend to virtual lane (id: 6817).
  EXPECT_NEAR(res->length(), lane_path_or->length() * 0.9 / 0.7, 0.01);
  EXPECT_EQ(res->size(), 1);
  EXPECT_EQ(res->back().fraction(), 1.0);

  const auto lane_path_with_virtual_or = BuildLanePathFromData(
      mapping::LanePathData(
          0.2, 0.5, {mapping::ElementId(6781), mapping::ElementId(6817)}),
      psmm);
  EXPECT_OK(lane_path_with_virtual_or);

  // Should return error if do not allow virtual lane.
  const auto res_2 = ForwardExtendLanePathWithMinimumHeadingDiff(
      psmm, *lane_path_with_virtual_or, /*extend_len*/ 20.0,
      /*allow_virtual=*/false);
  EXPECT_FALSE(res_2.ok());
}

TEST(LanePathUtilTest, ForwardExtendLanePathWithoutForkTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.0, 1.0, {mapping::ElementId(2472)}), psmm);
  EXPECT_OK(lane_path_or);

  const auto res = ForwardExtendLanePathWithoutFork(psmm, *lane_path_or, 100.0);

  EXPECT_EQ(res.size(), 2);
  EXPECT_EQ(res.lane_id(0), mapping::ElementId(2472));
  EXPECT_EQ(res.lane_id(1), mapping::ElementId(54));
}

TEST(LanePathUtilTest, FindNearestLanePathFromEgoPoseTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(220);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.95);
  ASSIGN_OR_DIE(const auto lane_path,
                FindNearestLanePathFromEgoPose(pose, psmm,
                                               /*required_min_length=*/100.0));
  EXPECT_THAT(std::vector<mapping::ElementId>(
                  {mapping::ElementId(2471), mapping::ElementId(53),
                   mapping::ElementId(121), mapping::ElementId(160)}),
              testing::ElementsAreArray(lane_path.lane_ids()));
  EXPECT_NEAR(lane_path.length(), 100.0, 1e-1);
}

TEST(LanePathUtilTest, ArclengthToPosTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.0, 1.0, {mapping::ElementId(2448)}), psmm);
  EXPECT_OK(lane_path_or);

  Vec2d res = ArclengthToPos(psmm, *lane_path_or, 10.0);
  const auto* lane_info = psmm.FindLaneInfoOrNull(mapping::ElementId(2448));
  EXPECT_TRUE(lane_info != nullptr);

  mapping::LanePoint lane_point(mapping::ElementId(2448),
                                10.0 / lane_info->length());
  Vec2d expect = ComputeLanePointPos(psmm, lane_point);
  EXPECT_EQ(res, expect);
}

TEST(LanePathUtilTest, ArclengthToLerpThetaTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.0, 1.0,
                            {mapping::ElementId(43), mapping::ElementId(88)}),
      psmm);
  EXPECT_OK(lane_path_or);

  double res = ArclengthToLerpTheta(psmm, *lane_path_or, 10.0);

  const auto lane_index_point = lane_path_or->ArclengthToLaneIndexPoint(10.0);

  const double expect =
      LaneIndexPointToLerpTheta(psmm, *lane_path_or, lane_index_point);
  EXPECT_EQ(res, expect);
}

TEST(LanePathUtilTest, LaneIndexPointToLerpThetaTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.0, 1.0,
                            {mapping::ElementId(43), mapping::ElementId(88)}),
      psmm);
  EXPECT_OK(lane_path_or);

  double res =
      LaneIndexPointToLerpTheta(psmm, *lane_path_or, std::make_pair(0, 0.5));
  mapping::LanePoint lane_point(mapping::ElementId(43), 0.5);
  double expect = ComputeLanePointLerpThetaWithSuccessorLane(
      psmm, mapping::ElementId(88), lane_point);
  EXPECT_EQ(res, expect);
}

TEST(LanePathUtilTest, GetLanesInfoTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.0, 1.0,
                            {mapping::ElementId(43), mapping::ElementId(88)}),
      psmm);
  EXPECT_OK(lane_path_or);

  auto lanes = GetLanesInfoBreakIfNotFound(psmm, *lane_path_or);
  ASSERT_EQ(lanes.size(), 2);
  ASSERT_EQ(lanes[0]->id.value(), 43);

  auto continues_lanes = GetLanesInfoContinueIfNotFound(psmm, *lane_path_or);
  ASSERT_EQ(continues_lanes.size(), 2);
  ASSERT_EQ(continues_lanes[1]->id.value(), 88);
}

TEST(LanePathUtilTest, TrimTrailingNotFoundLanesTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  mapping::LanePath lp(smm,
                       {mapping::ElementId(2448), mapping::ElementId(1),
                        mapping::ElementId(10086)},
                       0.0, 1.0);
  const auto new_lp = TrimTrailingNotFoundLanes(psmm, lp);
  EXPECT_TRUE(new_lp.ok());
  mapping::LanePath expect_lp(
      smm, {mapping::ElementId(2448), mapping::ElementId(1)}, 0.0, 1.0);
  EXPECT_EQ(*new_lp, expect_lp);

  mapping::LanePath lp2(
      smm,
      {mapping::ElementId(2448), mapping::ElementId(1), mapping::ElementId(34)},
      0.0, 1.0);
  const auto new_lp2 = TrimTrailingNotFoundLanes(psmm, lp2);
  EXPECT_TRUE(new_lp2.ok());
  mapping::LanePath expect_lp2(
      smm,
      {mapping::ElementId(2448), mapping::ElementId(1), mapping::ElementId(34)},
      0.0, 1.0);
  EXPECT_EQ(*new_lp2, expect_lp2);

  mapping::LanePath lp3(
      smm, {mapping::ElementId(10010), mapping::ElementId(10000)}, 0.0, 1.0);
  const auto new_lp3 = TrimTrailingNotFoundLanes(psmm, lp3);
  EXPECT_FALSE(new_lp3.ok());
}

TEST(LanePathUtilTest, FindNearLanePathsFromEgoPose) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(-6.3);
  pose.mutable_pos_smooth()->set_y(3.5);
  pose.set_yaw(0.0);
  ASSIGN_OR_DIE(
      const auto lane_paths,
      FindNearLanePathsFromEgoPose(
          psmm, Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()), pose.yaw(),
          /*required_min_length=*/100.0,
          /*heading_penalty_weight=*/0.0,
          /*distance_threshold=*/2.0,
          /*angle_error_threshold=*/M_PI_4));
  EXPECT_EQ(lane_paths.size(), 2);
  EXPECT_THAT(std::vector<mapping::ElementId>(
                  {mapping::ElementId(96), mapping::ElementId(2),
                   mapping::ElementId(938), mapping::ElementId(940)}),
              testing::ElementsAreArray(lane_paths.front().lane_ids()));
  EXPECT_THAT(std::vector<mapping::ElementId>(
                  {mapping::ElementId(55), mapping::ElementId(2),
                   mapping::ElementId(938), mapping::ElementId(940)}),
              testing::ElementsAreArray(lane_paths.at(1).lane_ids()));
}

TEST(LanePathUtilTest, ForwardExtendLanePathTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.0, 1.0, {mapping::ElementId(14)}), psmm);
  EXPECT_OK(lane_path_or);

  const auto forward = ForwardExtendLanePath(psmm, *lane_path_or, 60.0);
  EXPECT_EQ(forward.size(), 3);
  EXPECT_EQ(forward.lane_id(0).value(), 14);
  EXPECT_EQ(forward.lane_id(1).value(), 59);
  EXPECT_EQ(forward.lane_id(2).value(), 2476);
}

TEST(LanePathUtilTest, BuildLanePathsFromDrivingMapTopo) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const Vec2d pos(0.1, 0.0);
  const auto dm_or = BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength(
      psmm, pos, /*heading=*/0.0, /*radius=*/3.0, /*max_forward_length=*/20.0,
      /*max_heading_diff=*/M_PI_4, /*allow_virtual=*/false);
  // 56->2448, 97->2448.
  EXPECT_OK(dm_or);
  const auto lane_paths = BuildLanePathsFromDrivingMapTopo(*dm_or, psmm);
  EXPECT_EQ(lane_paths.size(), 2);
  EXPECT_EQ(lane_paths[0].end_fraction(), 1.0);
  EXPECT_EQ(lane_paths[1].end_fraction(), 1.0);
  EXPECT_EQ(lane_paths[0].lane_id(1), mapping::ElementId(2448));
  EXPECT_EQ(lane_paths[1].lane_id(1), mapping::ElementId(2448));
  EXPECT_EQ(lane_paths[0].start_fraction(), 1.0);
  EXPECT_EQ(lane_paths[1].start_fraction(), 1.0);

  const Vec2d pos2(605.6, -465.4);
  const auto dm2_or = BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength(
      psmm, pos2, /*heading=*/0.0, /*radius=*/3.0, /*max_forward_length=*/20.0,
      /*max_heading_diff=*/M_PI_4, /*allow_virtual=*/false);
  const auto lane_paths_2 = BuildLanePathsFromDrivingMapTopo(*dm2_or, psmm);
  // lane path should terminate before virtual lanes.
  EXPECT_EQ(lane_paths_2.size(), 1);
  EXPECT_EQ(lane_paths_2[0].lane_id(0), mapping::ElementId(6781));
}

TEST(LanePathUtilTest, BuildLanePathFromLaneIdSeqInDrivingMap) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const Vec2d pos(240.2, -3.47);  // On dojo lane 54.
  const auto dm_or = BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength(
      psmm, pos, /*heading=*/0.0, /*radius=*/3.0, /*max_forward_length=*/200.0,
      /*max_heading_diff=*/M_PI_4, /*allow_virtual=*/false);
  const auto lane_paths =
      BuildAllLanePathsFromStartIdInDrivingMapTopoWithDesireLength(
          *dm_or, psmm, mapping::ElementId(54), /*desire_length=*/50,
          /*min_length=*/1.0);
  EXPECT_EQ(lane_paths.size(), 3);

  const auto lane_paths_2 =
      BuildAllLanePathsFromStartIdInDrivingMapTopoWithDesireLength(
          *dm_or, psmm, mapping::ElementId(54), /*desire_length=*/500,
          /*min_length=*/500);
  EXPECT_EQ(lane_paths_2.size(), 0);

  const auto lane_paths_3 =
      BuildAllLanePathsFromStartIdInDrivingMapTopoWithDesireLength(
          *dm_or, psmm, mapping::ElementId(54), /*desire_length=*/3,
          /*min_length=*/1.0);
  EXPECT_EQ(lane_paths_3.size(), 1);
}
}  // namespace
}  // namespace qcraft::planner
