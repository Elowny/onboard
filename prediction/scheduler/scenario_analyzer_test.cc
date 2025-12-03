#include "onboard/prediction/scheduler/scenario_analyzer.h"

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"

namespace qcraft::prediction {
namespace {
TEST(ScenarioAnalyzerTest, OffRoadTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(13.0, -31.0);
  const auto obj = planner::PerceptionObjectBuilder()
                       .set_id("1")
                       .set_type(OT_VEHICLE)
                       .set_pos(pos)
                       .set_timestamp(0.0)
                       .set_box_center(pos)
                       .set_length_width(/*length=*/2.0, /*width=*/1.0)
                       .set_yaw(0.0)
                       .Build();
  const auto scenario = AnalyzeScenarioWithSemanticMapAndObjectProto(psmm, obj);
  EXPECT_EQ(scenario.road_status(), ObjectRoadStatus::ORS_OFF_ROAD);
  EXPECT_EQ(scenario.intersection_status(),
            ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
}

TEST(ScenarioAnalyzerTest, OffRoadTest2) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(-30.0, 47.0);
  const auto obj = planner::PerceptionObjectBuilder()
                       .set_id("1")
                       .set_type(OT_VEHICLE)
                       .set_pos(pos)
                       .set_timestamp(0.0)
                       .set_box_center(pos)
                       .set_length_width(/*length=*/0.1, /*width=*/0.1)
                       .set_yaw(0.0)
                       .Build();
  const auto scenario = AnalyzeScenarioWithSemanticMapAndObjectProto(psmm, obj);
  EXPECT_EQ(scenario.road_status(), ObjectRoadStatus::ORS_OFF_ROAD);
  EXPECT_NEAR(scenario.abs_dist_to_nearest_lane(), 3.7, 1e-1);
  EXPECT_EQ(scenario.intersection_status(),
            ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
}

TEST(ScenarioAnalyzerTest, OnRoadTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(-10.0, -42.0);
  const auto obj = planner::PerceptionObjectBuilder()
                       .set_id("1")
                       .set_type(OT_VEHICLE)
                       .set_pos(pos)
                       .set_timestamp(0.0)
                       .set_box_center(pos)
                       .set_length_width(/*length=*/0.1, /*width=*/0.1)
                       .set_yaw(0.0)
                       .Build();
  const auto scenario = AnalyzeScenarioWithSemanticMapAndObjectProto(psmm, obj);
  EXPECT_EQ(scenario.road_status(), ObjectRoadStatus::ORS_ON_ROAD);
  EXPECT_EQ(scenario.intersection_status(),
            ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  EXPECT_NEAR(scenario.abs_dist_to_nearest_intersection(), 10.4, 1e-1);
}

TEST(ScenarioAnalyzerTest, InIntersectionTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(-14.0, 3.0);
  const auto obj = planner::PerceptionObjectBuilder()
                       .set_id("1")
                       .set_type(OT_VEHICLE)
                       .set_pos(pos)
                       .set_timestamp(0.0)
                       .set_box_center(pos)
                       .set_length_width(/*length=*/2.0, /*width=*/1.0)
                       .set_yaw(0.0)
                       .Build();
  const auto scenario = AnalyzeScenarioWithSemanticMapAndObjectProto(psmm, obj);
  EXPECT_EQ(scenario.road_status(), ObjectRoadStatus::ORS_ON_ROAD);
  EXPECT_EQ(scenario.intersection_status(),
            ObjectIntersectionStatus::OIS_IN_INTERSECTION);
  EXPECT_NEAR(scenario.abs_dist_to_nearest_intersection(), 0.0, 1e-2);
}

}  // namespace
}  // namespace qcraft::prediction
