#include "onboard/planner/util/spatial_search_util.h"

#include <cmath>
#include <cstdint>

#include "absl/container/flat_hash_map.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/util/lane_path_util.h"

namespace qcraft::planner {
namespace {

constexpr double kEpsilon = 1e-2;

TEST(FindCloseLanePoints, FindCloseLanePointsTest) {
  SetMap("dojo");
  std::shared_ptr<SemanticMapManager> smm =
      std::make_shared<SemanticMapManager>();
  smm->LoadWholeMap().Build();
  const auto& psmm = CreateDojoTestPSMM();
  {
    const Vec2d query_point(-8.5, 4.9);
    const absl::flat_hash_map<int64_t, double> ground_truth{
        {55, 0.692233}, {269, 0.582415}, {96, 0.778052}, {268, 0.583084}};
    const auto close_points =
        FindCloseLanePointsToSmoothPointWithHeadingBoundAmongLanesAtLevel(
            psmm.GetLevel(), psmm, query_point,
            /*heading=*/0.0,
            /*heading_penalty_weight=*/0.0, /*spatial_distance_threshold=*/2.0,
            /*angle_error_threshold=*/M_PI_2);

    EXPECT_EQ(close_points.size(), ground_truth.size());
    for (const auto& pt : close_points) {
      EXPECT_TRUE(ground_truth.contains(pt.lane_id().value()));
      EXPECT_NEAR(pt.fraction(), ground_truth.at(pt.lane_id().value()),
                  kEpsilon);
    }
  }

  {
    const Vec2d query_point(-2.0, -12.0);
    const auto close_points =
        FindCloseLanePointsToSmoothPointWithHeadingBoundAmongLanesAtLevel(
            psmm.GetLevel(), psmm, query_point,
            /*heading=*/0.0,
            /*heading_penalty_weight=*/0.0, /*spatial_distance_threshold=*/2.0,
            /*angle_error_threshold=*/M_PI_2);

    EXPECT_TRUE(close_points.empty());
  }

  {
    const Vec2d query_point(10.0, 0.0);
    const absl::flat_hash_map<int64_t, double> ground_truth{{2448, 0.180212}};
    const auto close_points =
        FindCloseLanePointsToSmoothPointWithHeadingBoundAmongLanesAtLevel(
            psmm.GetLevel(), psmm, query_point,
            /*heading=*/0.0,
            /*heading_penalty_weight=*/0.0, /*spatial_distance_threshold=*/2.0,
            /*angle_error_threshold=*/M_PI_2);

    EXPECT_EQ(close_points.size(), ground_truth.size());
    for (const auto& pt : close_points) {
      EXPECT_TRUE(ground_truth.contains(pt.lane_id().value()));
      EXPECT_NEAR(pt.fraction(), ground_truth.at(pt.lane_id().value()),
                  kEpsilon);
    }
  }
}

TEST(IsPointOnLanePathAtLevel, IsPointOnLanePathAtLevelTest) {
  SetMap("dojo");
  std::shared_ptr<SemanticMapManager> smm =
      std::make_shared<SemanticMapManager>();
  smm->LoadWholeMap().Build();
  const auto& psmm = CreateDojoTestPSMM();

  auto res_or = BuildLanePathFromData(
      mapping::LanePathData(0.0, 1.0,
                            {mapping::ElementId(56), mapping::ElementId(2448)}),
      psmm);

  EXPECT_OK(res_or);
  const auto lane_path = std::move(res_or).value();
  {
    const Vec2d query_point(10.0, 0.0);
    double arc_len_on_lane_path;
    EXPECT_TRUE(IsPointOnLanePathAtLevel(psmm.GetLevel(), psmm, query_point,
                                         lane_path, &arc_len_on_lane_path));
    EXPECT_NEAR(arc_len_on_lane_path, 43.989, kEpsilon);
  }

  {
    const Vec2d query_point(20.0, 0.0);
    double arc_len_on_lane_path;
    EXPECT_TRUE(IsPointOnLanePathAtLevel(psmm.GetLevel(), psmm, query_point,
                                         lane_path, &arc_len_on_lane_path));
    EXPECT_NEAR(arc_len_on_lane_path, 53.989, kEpsilon);
  }

  {
    const Vec2d query_point(10.0, 5.0);
    double arc_len_on_lane_path;
    EXPECT_FALSE(IsPointOnLanePathAtLevel(psmm.GetLevel(), psmm, query_point,
                                          lane_path, &arc_len_on_lane_path));
    EXPECT_NEAR(arc_len_on_lane_path, 43.989, kEpsilon);
  }

  {
    const Vec2d query_point(-20.0, 30.0);
    double arc_len_on_lane_path;
    EXPECT_FALSE(IsPointOnLanePathAtLevel(psmm.GetLevel(), psmm, query_point,
                                          lane_path, &arc_len_on_lane_path));
    EXPECT_NEAR(arc_len_on_lane_path, 0.0, kEpsilon);
  }

  {
    const Vec2d query_point(70.0, 0.0);
    double arc_len_on_lane_path;
    EXPECT_FALSE(IsPointOnLanePathAtLevel(psmm.GetLevel(), psmm, query_point,
                                          lane_path, &arc_len_on_lane_path));
  }
}

TEST(FindClosestLanePoint, SingleLaneTest) {
  SetMap("dojo");
  std::shared_ptr<SemanticMapManager> smm =
      std::make_shared<SemanticMapManager>();
  smm->LoadWholeMap().Build();
  const auto& psmm = CreateDojoTestPSMM();
  {
    const Vec2d query_point(10.0, 0.0);
    Vec2d closest_point;
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
            psmm.GetLevel(), psmm, query_point, mapping::ElementId(2448),
            /*heading=*/0.0, /*heading_penalty_weight=*/0.0, &closest_point,
            /*start_fraction=*/0.0, /*end_fraction=*/1.0);
    EXPECT_TRUE(lane_point.has_value());
    EXPECT_EQ(lane_point->lane_id().value(), 2448);
    EXPECT_NEAR(lane_point->fraction(), 0.180212, kEpsilon);
  }

  {
    const Vec2d query_point(10.0, 1.0);
    Vec2d closest_point;
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
            psmm.GetLevel(), psmm, query_point, mapping::ElementId(2448),
            /*heading=*/0.0, /*heading_penalty_weight=*/0.0, &closest_point,
            /*start_fraction=*/0.0, /*end_fraction=*/1.0);
    EXPECT_TRUE(lane_point.has_value());
    EXPECT_EQ(lane_point->lane_id().value(), 2448);
    EXPECT_NEAR(lane_point->fraction(), 0.180212, kEpsilon);
  }

  {
    const Vec2d query_point(-1.0, 0.0);
    Vec2d closest_point;
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
            psmm.GetLevel(), psmm, query_point, mapping::ElementId(2448),
            /*heading=*/0.0, /*heading_penalty_weight=*/0.0, &closest_point,
            /*start_fraction=*/0.0, /*end_fraction=*/1.0);
    EXPECT_TRUE(lane_point.has_value());
    EXPECT_EQ(lane_point->lane_id().value(), 2448);
    EXPECT_NEAR(lane_point->fraction(), 0.0, kEpsilon);
  }

  {
    const Vec2d query_point(30.0, 0.0);
    Vec2d closest_point;
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
            psmm.GetLevel(), psmm, query_point, mapping::ElementId(2448),
            /*heading=*/0.0, /*heading_penalty_weight=*/0.0, &closest_point,
            /*start_fraction=*/0.0, /*end_fraction=*/0.5);
    EXPECT_TRUE(lane_point.has_value());
    EXPECT_EQ(lane_point->lane_id().value(), 2448);
    EXPECT_NEAR(lane_point->fraction(), 0.5, kEpsilon);
  }

  {
    const Vec2d query_point(10.0, 3.0);
    Vec2d closest_point;
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
            psmm.GetLevel(), psmm, query_point, mapping::ElementId(2448),
            /*heading=*/0.0, /*heading_penalty_weight=*/0.0, &closest_point,
            /*start_fraction=*/0.0, /*end_fraction=*/1.0,
            /*cutoff_distance=*/2.0);
    EXPECT_FALSE(lane_point.has_value());
  }
}

TEST(FindClosestLanePoint, MultipleLaneTest) {
  SetMap("dojo");
  std::shared_ptr<SemanticMapManager> smm =
      std::make_shared<SemanticMapManager>();
  smm->LoadWholeMap().Build();
  const auto& psmm = CreateDojoTestPSMM();

  const std::vector<mapping::ElementId> lane_ids{mapping::ElementId(56),
                                                 mapping::ElementId(2448)};
  {
    const Vec2d query_point(10.0, 0.0);
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
            psmm.GetLevel(), psmm, query_point, lane_ids,
            /*heading=*/0.0);
    EXPECT_OK(lane_point);
    EXPECT_EQ(lane_point->lane_id().value(), 2448);
    EXPECT_NEAR(lane_point->fraction(), 0.180212, kEpsilon);
  }

  {
    const Vec2d query_point(10.0, 1.0);
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
            psmm.GetLevel(), psmm, query_point, lane_ids,
            /*heading=*/0.0);
    EXPECT_OK(lane_point);
    EXPECT_EQ(lane_point->lane_id().value(), 2448);
    EXPECT_NEAR(lane_point->fraction(), 0.180212, kEpsilon);
  }
}

TEST(FindClosestLanePoint, NoCandidateLaneTest) {
  SetMap("dojo");
  std::shared_ptr<SemanticMapManager> smm =
      std::make_shared<SemanticMapManager>();
  smm->LoadWholeMap().Build();
  const auto& psmm = CreateDojoTestPSMM();

  {
    const Vec2d query_point(10.0, 0.0);
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel(
            psmm.GetLevel(), psmm, query_point,
            /*heading=*/0.0);
    EXPECT_OK(lane_point);
    EXPECT_EQ(lane_point->lane_id().value(), 2448);
    EXPECT_NEAR(lane_point->fraction(), 0.180212, kEpsilon);
  }

  {
    const Vec2d query_point(10.0, 1.0);
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel(
            psmm.GetLevel(), psmm, query_point,
            /*heading=*/0.0);
    EXPECT_OK(lane_point);
    EXPECT_EQ(lane_point->lane_id().value(), 2448);
    EXPECT_NEAR(lane_point->fraction(), 0.180212, kEpsilon);
  }

  {
    const Vec2d query_point(-1.0, -13.0);
    Vec2d closest_point;
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel(
            psmm.GetLevel(), psmm, query_point,
            /*heading=*/0.0,
            /*heading_penalty_weight=*/0.0, &closest_point,
            /*cutoff_distance=*/2.0);
    EXPECT_FALSE(lane_point.ok());
  }

  {
    const Vec2d query_point(-1.0, -13.0);
    Vec2d closest_point;
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel(
            psmm.GetLevel(), psmm, query_point,
            /*heading=*/0.0,
            /*heading_penalty_weight=*/0.0, &closest_point,
            /*cutoff_distance=*/20.0);
    EXPECT_OK(lane_point);
    EXPECT_EQ(lane_point->lane_id().value(), 267);
    EXPECT_NEAR(lane_point->fraction(), 0.378031, kEpsilon);
  }
}

}  // namespace
}  // namespace qcraft::planner
