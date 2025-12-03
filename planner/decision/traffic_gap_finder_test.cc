#include "onboard/planner/decision/traffic_gap_finder.h"

#include <algorithm>
#include <optional>
#include <string>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {
namespace {

PlannerObject BuildPlannerObject(const std::string& obj_id,
                                 const Vec2d& obj_pos, double vel,
                                 double heading = 0.0) {
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id(obj_id)
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(obj_pos)
                            .set_length_width(4.2, 2.1)
                            .set_yaw(heading)
                            .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .set_stationary(false)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.8)
      .set_straight_line(obj_pos,
                         obj_pos + Vec2d::FastUnitFromAngle(heading) * vel *
                                       prediction::kPredictionDuration,
                         /*init_v=*/vel, /*last_v=*/vel);
  return builder.Build();
}

TEST(TrafficGapFinder, TakeBestNTrafficGaps) {
  const auto& psmm = CreateDojoTestPSMM();
  const mapping::LanePath target_lane_path = *BuildLanePathFromData(
      mapping::LanePathData(
          /*start_fraction=*/0.0, /*end_fraction=*/1.0,
          {mapping::ElementId(2476), mapping::ElementId(2491)}),
      psmm);
  DrawLanePathToCanvas(psmm, target_lane_path, "gap_finder/target_lane_path",
                       vis::Color::kGreen);

  const auto frenet_frame =
      *BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, target_lane_path),
                              /*down_sample_raw_points=*/true);

  ObjectVector<PlannerObject> objects;
  objects.push_back(
      BuildPlannerObject("First", Vec2d(160.0, 66.5), /*vel=*/7.0));
  objects.push_back(
      BuildPlannerObject("Second", Vec2d(142.0, 66.0), /*vel=*/5.0));
  objects.push_back(
      BuildPlannerObject("Third", Vec2d(125.5, 67.0), /*vel=*/6.0));
  const PlannerObjectManager obj_mgr(objects);
  DrawPlannerObjectManagerToCanvas(obj_mgr, "gap_finder/objects",
                                   vis::Color::kLightGreen);

  const Box2d ego_box(/*half_length=*/2.1, /*half_width=*/1.05,
                      Vec2d(136.7, 70.0), /*heading=*/0.0);
  SendBox2dToCanvas(ego_box, "gap_finder/ego_vehicle", vis::Color::kOrange);
  const auto ego_frenet_box = *frenet_frame.QueryFrenetBoxAt(ego_box);

  const SpacetimeTrajectoryManager st_traj_mgr(obj_mgr.planner_objects());
  const auto gaps = FindCandidateTrafficGapsOnLanePath(
      frenet_frame, ego_frenet_box, st_traj_mgr);

  EXPECT_EQ(gaps.size(), 2);

  // follow closest
  EXPECT_NEAR(gaps[0].s_start, 12.08, 1e-2);
  EXPECT_NEAR(gaps[0].s_end, 24.37, 1e-2);
  for (const auto* traj_ptr : gaps[0].leader_trajectories) {
    EXPECT_EQ(traj_ptr->object_id(), "Second");
  }
  for (const auto* traj_ptr : gaps[0].follower_trajectories) {
    EXPECT_EQ(traj_ptr->object_id(), "Third");
  }
  // lead closest
  EXPECT_NEAR(gaps[1].s_start, 28.57, 1e-2);
  EXPECT_NEAR(gaps[1].s_end, 42.37, 1e-2);
  for (const auto* traj_ptr : gaps[1].leader_trajectories) {
    EXPECT_EQ(traj_ptr->object_id(), "First");
  }
  for (const auto* traj_ptr : gaps[1].follower_trajectories) {
    EXPECT_EQ(traj_ptr->object_id(), "Second");
  }

  const double ego_v = 6.0;  // m/s.
  const auto result_or =
      EvaluateAndTakeBestTrafficGap(gaps, ego_frenet_box, ego_v);
  ASSERT_OK(result_or);
  ASSERT_TRUE(result_or->leader_id.has_value());
  EXPECT_EQ(result_or->leader_id.value(), "Second");
  ASSERT_TRUE(result_or->follower_id.has_value());
  EXPECT_EQ(result_or->follower_id.value(), "Third");
}
}  // namespace

}  // namespace qcraft::planner
