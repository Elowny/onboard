#include "onboard/planner/util/lane_point_util.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/async/future.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft::planner {
namespace {
TEST(LanePointUtilTest, ComputeLanePointTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  auto smm = std::make_unique<SemanticMapManager>();
  smm->LoadWholeMap().Build();

  const mapping::LanePoint lp(mapping::ElementId(2448), 0.1);

  {
    const auto smm_pos = lp.ComputePos(*smm);
    const auto psmm_pos = ComputeLanePointPos(psmm, lp);
    EXPECT_NEAR(smm_pos.DistanceTo(psmm_pos), 0.0, 0.01);
  }

  {
    const auto smm_tangent = lp.ComputeTangent(*smm);
    const auto psmm_tangent = ComputeLanePointTangent(psmm, lp);
    EXPECT_NEAR(smm_tangent.CrossProd(psmm_tangent), 0.0, 0.01);
  }

  {
    const double smm_lerp = lp.ComputeLerpTheta(*smm);
    const double psmm_lerp = ComputeLanePointLerpTheta(psmm, lp);
    EXPECT_NEAR(smm_lerp, psmm_lerp, 0.01);
  }

  {
    const double smm_lerp_succ =
        lp.ComputeLerpThetaWithSuccessorLane(*smm, mapping::ElementId(1));
    const double psmm_lerp_succ = ComputeLanePointLerpThetaWithSuccessorLane(
        psmm, mapping::ElementId(1), lp);
    EXPECT_NEAR(smm_lerp_succ, psmm_lerp_succ, 0.01);
  }
}

TEST(LanePointUtilTest, ComputeLanePointGlobalTest) {
  auto loader =
      mapping::v2::SemanticMapLoader::MakeShared({.map_name = "dojo"});
  auto map_fut = loader->PreloadWholeMap();
  std::shared_ptr<mapping::v2::SemanticMapManager> v2smm = map_fut.Get();

  const auto lane = v2smm->FindLane(mapping::ElementId(2448));
  EXPECT_NE(lane, nullptr);
  const auto& segment_points = lane->segment_points();

  const mapping::LanePoint lp1(mapping::ElementId(2448), 0.0);
  const mapping::LanePoint lp2(mapping::ElementId(2448), 1.0);
  const mapping::LanePoint lp3(mapping::ElementId(2448), 0.3);

  {
    const auto pose_or1 = ComputeLanePointGlobalPose(*v2smm, lp1);
    EXPECT_OK(pose_or1);
    EXPECT_DOUBLE_EQ(pose_or1->x(), segment_points.front().x());
    EXPECT_DOUBLE_EQ(pose_or1->y(), segment_points.front().y());

    const auto heading_or1 = ComputeLanePointGlobalHeading(*v2smm, lp1);
    EXPECT_OK(heading_or1);
    const Vec2d expect1 =
        Vec2d(segment_points[1] - segment_points[0]).normalized();
    EXPECT_DOUBLE_EQ(heading_or1->x(), expect1.x());
    EXPECT_DOUBLE_EQ(heading_or1->y(), expect1.y());
  }

  {
    const auto pose_or2 = ComputeLanePointGlobalPose(*v2smm, lp2);
    EXPECT_OK(pose_or2);
    EXPECT_DOUBLE_EQ(pose_or2->x(), segment_points.back().x());
    EXPECT_DOUBLE_EQ(pose_or2->y(), segment_points.back().y());

    const auto heading_or2 = ComputeLanePointGlobalHeading(*v2smm, lp2);
    EXPECT_OK(heading_or2);
    const Vec2d expect2 =
        Vec2d(*segment_points.rbegin() - *(segment_points.rbegin() + 1))
            .normalized();
    EXPECT_DOUBLE_EQ(heading_or2->x(), expect2.x());
    EXPECT_DOUBLE_EQ(heading_or2->y(), expect2.y());
  }

  {
    const auto pose_or3 = ComputeLanePointGlobalPose(*v2smm, lp3);
    EXPECT_OK(pose_or3);
    const double dist =
        segment_points.back().DistanceTo(segment_points.front());
    const double dist_3 = pose_or3->DistanceTo(segment_points.front());
    EXPECT_DOUBLE_EQ(dist_3 / dist, 0.3);
  }
}

TEST(LanePointUtilTest, QueryLanePointBoundaryType) {
  const auto& psmm = CreateDojoTestPSMM();
  {
    const mapping::LanePoint lp(mapping::ElementId(114110120), 0.0);
    const auto type_or = QueryLanePointBoundaryType(psmm, lp, /*left=*/true);
    EXPECT_NOT_OK(type_or);
  }
  {
    const mapping::LanePoint lp(mapping::ElementId(2448), 0.0);
    const auto left_type_or =
        QueryLanePointBoundaryType(psmm, lp, /*left=*/true);
    EXPECT_OK(left_type_or);
    EXPECT_TRUE(left_type_or->has_value());
    EXPECT_EQ(left_type_or->value(), mapping::LaneBoundaryProto::BROKEN_WHITE);
    const auto right_type_or =
        QueryLanePointBoundaryType(psmm, lp, /*left=*/false);
    EXPECT_OK(right_type_or);
    EXPECT_TRUE(right_type_or->has_value());
    EXPECT_EQ(right_type_or->value(), mapping::LaneBoundaryProto::BROKEN_WHITE);
  }
  {
    const mapping::LanePoint lp(mapping::ElementId(9), 0.3);
    const auto left_type_or =
        QueryLanePointBoundaryType(psmm, lp, /*left=*/true);
    EXPECT_OK(left_type_or);
    EXPECT_FALSE(left_type_or->has_value());
    const auto right_type_or =
        QueryLanePointBoundaryType(psmm, lp, /*left=*/false);
    EXPECT_OK(right_type_or);
    EXPECT_FALSE(right_type_or->has_value());
  }
  {
    const mapping::LanePoint lp(mapping::ElementId(2), 0.4);
    const auto left_type_or =
        QueryLanePointBoundaryType(psmm, lp, /*left=*/true);
    EXPECT_OK(left_type_or);
    EXPECT_TRUE(left_type_or->has_value());
    EXPECT_EQ(left_type_or->value(),
              mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW);
    const auto right_type_or =
        QueryLanePointBoundaryType(psmm, lp, /*left=*/false);
    EXPECT_OK(right_type_or);
    EXPECT_TRUE(right_type_or->has_value());
    EXPECT_EQ(right_type_or->value(), mapping::LaneBoundaryProto::BROKEN_WHITE);
  }
}

}  // namespace
}  // namespace qcraft::planner
