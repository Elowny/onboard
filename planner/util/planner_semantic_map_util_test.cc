#include "onboard/planner/util/planner_semantic_map_util.h"

#include <vector>

#include "gtest/gtest.h"

#include "onboard/maps/semantic_map_util.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft::planner {
namespace {
TEST(PlannerSemanticMapUtilTest, LanePositionTest) {
  SetMap("dojo");
  auto loader =
      mapping::v2::SemanticMapLoader::MakeShared({.map_name = "dojo"});
  auto map_fut = loader->PreloadWholeMap();
  std::shared_ptr<mapping::v2::SemanticMapManager> smm = map_fut.Get();

  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_info_ptr2448 =
      psmm.FindLaneInfoOrNull(mapping::ElementId(2448));
  EXPECT_TRUE(lane_info_ptr2448 != nullptr);

  const mapping::LanePath lp(smm.get(),
                             {mapping::ElementId(3), mapping::ElementId(951)},
                             /*start_fraction=*/0.5, /*end_fraction=*/0.5);

  {
    const bool is_outgoing =
        IsOutgoingLane(psmm, *lane_info_ptr2448, mapping::ElementId(1));
    EXPECT_TRUE(is_outgoing);
    const bool not_outgoing =
        IsOutgoingLane(psmm, *lane_info_ptr2448, mapping::ElementId(2));
    EXPECT_FALSE(not_outgoing);
  }

  {
    const bool is_right_most_s = IsRightMostLane(psmm, lp, 35.0);
    EXPECT_TRUE(is_right_most_s);
    const bool is_right_most_id = IsRightMostLane(psmm, mapping::ElementId(3));
    EXPECT_TRUE(is_right_most_id);
  }

  {
    const bool is_left_most = IsLeftMostLane(psmm, lp, 35.0);
    EXPECT_TRUE(is_left_most);
    const bool is_left_most_id = IsLeftMostLane(psmm, mapping::ElementId(2));
    EXPECT_TRUE(is_left_most_id);
  }
}

TEST(PlannerSemanticMapUtilTest, IsLanePathBlockedByBox2dAtLevelTest) {
  SetMap("dojo");
  auto loader =
      mapping::v2::SemanticMapLoader::MakeShared({.map_name = "dojo"});
  auto map_fut = loader->PreloadWholeMap();
  std::shared_ptr<mapping::v2::SemanticMapManager> smm = map_fut.Get();

  const auto& psmm = CreateDojoTestPSMM();
  const mapping::LanePath lp(smm.get(),
                             {mapping::ElementId(3), mapping::ElementId(950)},
                             /*start_fraction=*/0.5, /*end_fraction=*/0.5);

  const Box2d box(Vec2d(22.24, -2.89), 0.0, 3.0, 3.0);
  const bool is_blocked =
      IsLanePathBlockedByBox2dAtLevel(mapping::LevelId(0), psmm, box, lp, 0.0);
  EXPECT_TRUE(is_blocked);
}

TEST(PlannerSemanticMapUtilTest, SampleLanePathTest) {
  SetMap("dojo");
  auto loader =
      mapping::v2::SemanticMapLoader::MakeShared({.map_name = "dojo"});
  auto map_fut = loader->PreloadWholeMap();
  std::shared_ptr<mapping::v2::SemanticMapManager> smm = map_fut.Get();

  const auto& psmm = CreateDojoTestPSMM();
  const mapping::LanePath lp(smm.get(),
                             {mapping::ElementId(3), mapping::ElementId(950)},
                             /*start_fraction=*/0.5, /*end_fraction=*/0.5);

  {
    mapping::LanePathProto lp_proto;
    lp.ToProto(&lp_proto);
    const auto sample_res_or = SampleLanePathProtoPoints(psmm, lp_proto);
    EXPECT_OK(sample_res_or);
    const auto sample_smm_res_or =
        mapping::SampleLanePathProtoPoints(*smm, lp_proto);
    EXPECT_OK(sample_smm_res_or);
    EXPECT_EQ(sample_res_or->points.size(), sample_smm_res_or->points.size());
  }
}

TEST(PlannerSemanticMapUtilTest, ClampLanePathFromPosTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const mapping::LanePath lp(smm,
                             {mapping::ElementId(3), mapping::ElementId(950)},
                             /*start_fraction=*/0.5, /*end_fraction=*/0.5);

  const auto lane_info_ptr_3 = psmm.FindLaneInfoOrNull(mapping::ElementId(3));
  EXPECT_TRUE(lane_info_ptr_3 != nullptr);

  const auto lane_info_ptr_950 =
      psmm.FindLaneInfoOrNull(mapping::ElementId(950));
  EXPECT_TRUE(lane_info_ptr_950 != nullptr);

  const auto res_or =
      ClampLanePathFromPos(psmm, lp, lane_info_ptr_3->points_smooth.back());
  EXPECT_OK(res_or);
  EXPECT_NEAR(res_or->length(), lane_info_ptr_950->length() / 2, 0.1);
}

TEST(PlannerSemanticMapUtilTest,
     FindOutgoingLanePointWithMinimumHeadingDiffAllowVirtualTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto outgoing_lp_or = FindOutgoingLanePointWithMinimumHeadingDiff(
      psmm, mapping::ElementId(3), /*allow_virtual=*/true);
  EXPECT_OK(outgoing_lp_or);
  EXPECT_TRUE(outgoing_lp_or->lane_id() == mapping::ElementId(950));

  const auto outgoing_lp_or_2 = FindOutgoingLanePointWithMinimumHeadingDiff(
      psmm, mapping::ElementId(7254), /*allow_virtual=*/true);
  EXPECT_OK(outgoing_lp_or_2);
  EXPECT_TRUE(outgoing_lp_or_2->lane_id() == mapping::ElementId(7239));

  // No outgoing lanes.
  const auto outgoing_lp_or_3 = FindOutgoingLanePointWithMinimumHeadingDiff(
      psmm, mapping::ElementId(7241), /*allow_virtual=*/true);
  EXPECT_FALSE(outgoing_lp_or_3.ok());
}

TEST(PlannerSemanticMapUtilTest,
     FindOutgoingLanePointWithMinimumHeadingDiffDoNotAllowVirtualTest) {
  SetMap("dojo");
  auto smm = std::make_shared<SemanticMapManager>();
  smm->LoadWholeMap().Build();

  const auto& psmm = CreateDojoTestPSMM();
  const auto outgoing_lp_or = FindOutgoingLanePointWithMinimumHeadingDiff(
      psmm, mapping::ElementId(6597), /*allow_virtual=*/false);
  EXPECT_FALSE(outgoing_lp_or.ok());

  const auto outgoing_lp_or_2 = FindOutgoingLanePointWithMinimumHeadingDiff(
      psmm, mapping::ElementId(7254), /*allow_virtual=*/false);
  EXPECT_FALSE(outgoing_lp_or_2.ok());

  const auto outgoing_lp_or_3 = FindOutgoingLanePointWithMinimumHeadingDiff(
      psmm, mapping::ElementId(7459), /*allow_virtual=*/false);
  EXPECT_OK(outgoing_lp_or_3);
  EXPECT_TRUE(outgoing_lp_or_3->lane_id() == mapping::ElementId(7484));
}

}  // namespace
}  // namespace qcraft::planner
