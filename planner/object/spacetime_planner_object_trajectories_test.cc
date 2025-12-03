#include "onboard/planner/object/spacetime_planner_object_trajectories.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace planner {
namespace {

SpacetimePlannerObjectTrajectories BuildTrajectories() {
  const auto object = PerceptionObjectBuilder().set_id("o").Build();
  ObjectsProto perception_objects;
  *perception_objects.add_objects() = object;

  ObjectsPredictionProto prediction_objects;
  constexpr double kTimestamp = 10.0;
  prediction_objects.mutable_header()->set_timestamp(kTimestamp);

  ObjectPredictionBuilder builder;
  builder.set_object(object);
  constexpr int kNumTrajs = 3;
  constexpr double kProb = (1.0 - 1e-6) / kNumTrajs;
  for (int i = 0; i < kNumTrajs; ++i) {
    builder.add_predicted_trajectory()
        ->set_probability(kProb)
        .set_straight_line(Vec2d::Zero(), Vec2d(30.0, 0.0),
                           /*init_v=*/0.2, /*last_v=*/0.2);
  }
  builder.Build().ToProto(prediction_objects.add_objects());

  const auto planner_objects =
      BuildPlannerObjects(&perception_objects, &prediction_objects,
                          /*align_time=*/0.0,
                          /*thread_pool=*/nullptr);
  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);

  SpacetimePlannerObjectTrajectories trajs;
  for (const auto& traj : st_traj_mgr.trajectories()) {
    trajs.AddSpacetimePlannerObjectTrajectory(
        traj, SpacetimePlannerObjectTrajectoryReason::FRONT);
  }
  return trajs;
}

TEST(SpacetimePlannerObjectTrajectories, AddSpacetimePlannerObjectTrajectory) {
  const auto trajs = BuildTrajectories();

  ASSERT_EQ(trajs.trajectory_infos.size(), 3);
  EXPECT_EQ(trajs.trajectory_infos[0].traj_index, 0);
  EXPECT_EQ(trajs.trajectory_infos[0].object_id, "o");
  EXPECT_EQ(trajs.trajectory_infos[0].reason,
            SpacetimePlannerObjectTrajectoryReason::FRONT);

  EXPECT_EQ(trajs.trajectory_infos[1].traj_index, 1);
  EXPECT_EQ(trajs.trajectory_infos[1].object_id, "o");
  EXPECT_EQ(trajs.trajectory_infos[1].reason,
            SpacetimePlannerObjectTrajectoryReason::FRONT);

  EXPECT_EQ(trajs.trajectory_infos[2].traj_index, 2);
  EXPECT_EQ(trajs.trajectory_infos[2].object_id, "o");
  EXPECT_EQ(trajs.trajectory_infos[2].reason,
            SpacetimePlannerObjectTrajectoryReason::FRONT);
  EXPECT_EQ(trajs.st_start_offset, 0.0);
}

TEST(SpacetimePlannerObjectTrajectories, ToProto) {
  const auto trajs = BuildTrajectories();
  SpacetimePlannerObjectTrajectoriesProto proto;

  trajs.ToProto(&proto);
  EXPECT_THAT(proto, ProtoEqText(R"(
        trajectory {
          reason: FRONT
          id: "o-idx0"
          index: 0
        }
        trajectory {
          reason: FRONT
          id: "o-idx1"
          index: 1
        }
        trajectory {
          reason: FRONT
          id: "o-idx2"
          index : 2
        })"));
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
