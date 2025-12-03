#include "onboard/planner/plan/async_planner_util.h"

#include <algorithm>
#include <initializer_list>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/planner/common/proto/planner_status.pb.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEpsilon = 1e-2;

TEST(AsyncPlannerUtilTest, UpdateAsyncCounter) {
  int counter = -1;
  UpdateAsyncCounter(/*low_freq_cycle=*/2, &counter);
  EXPECT_EQ(counter, 0);
  UpdateAsyncCounter(/*low_freq_cycle=*/2, &counter);
  EXPECT_EQ(counter, 1);
  UpdateAsyncCounter(/*low_freq_cycle=*/2, &counter);
  EXPECT_EQ(counter, 2);
  UpdateAsyncCounter(/*low_freq_cycle=*/2, &counter);
  EXPECT_EQ(counter, 0);
}

TEST(AsyncPlannerUtilTest, ShouldRunLowFreqModule) {
  EXPECT_TRUE(ShouldRunLowFreqModule(/*counter=*/0));
  EXPECT_FALSE(ShouldRunLowFreqModule(/*counter=*/1));
}

TEST(AsyncPlannerUtilTest, IsPlannerAsync) {
  EXPECT_TRUE(IsPlannerAsync(/*async_low_freq_cycle_iterations*/ 1));
  EXPECT_FALSE(IsPlannerAsync(/*async_low_freq_cycle_iterations*/ 0));
}

TEST(AsyncPlannerUtilTest, ShouldRetrieveLowFreqResult) {
  const int async_low_freq_cycle_iterations = 3;
  for (int counter = 0; counter < async_low_freq_cycle_iterations; ++counter) {
    EXPECT_FALSE(
        ShouldRetrieveLowFreqResult(counter, async_low_freq_cycle_iterations));
  }
  EXPECT_TRUE(ShouldRetrieveLowFreqResult(async_low_freq_cycle_iterations,
                                          async_low_freq_cycle_iterations));
}

TEST(AsyncPlannerUtilTest, AlignPathWithPlanStartPoint) {
  const auto mock_path_point = [](double s) -> PathPoint {
    PathPoint p;
    p.set_x(s);
    p.set_y(0.0);
    p.set_z(0.0);
    p.set_theta(0.0);
    p.set_kappa(0.0);
    p.set_lambda(0.0);
    p.set_s(s);
    return p;
  };

  constexpr int kPathPointNum = 5;
  std::vector<PathPoint> path_points;
  path_points.reserve(kPathPointNum);
  for (int i = 0; i < kPathPointNum; ++i) {
    path_points.push_back(mock_path_point(i));
  }

  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(6.0);
    plan_start_point.mutable_path_point()->set_y(0.0);

    const auto res_or =
        AlignPathWithPlanStartPoint(path_points, plan_start_point);
    EXPECT_FALSE(res_or.ok());
  }

  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(-1.0);
    plan_start_point.mutable_path_point()->set_y(0.0);

    const auto res_or =
        AlignPathWithPlanStartPoint(path_points, plan_start_point);
    EXPECT_FALSE(res_or.ok());
  }

  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(2.5);
    plan_start_point.mutable_path_point()->set_y(0.0);

    const auto res_or =
        AlignPathWithPlanStartPoint(path_points, plan_start_point);
    EXPECT_TRUE(res_or.ok());

    const auto& align_path = *res_or;
    EXPECT_EQ(align_path.size(), 3);

    EXPECT_NEAR(align_path[0].x(), 2.5, kEpsilon);
    EXPECT_NEAR(align_path[0].y(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[0].z(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[0].theta(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[0].kappa(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[0].lambda(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[0].s(), 0.0, kEpsilon);

    EXPECT_NEAR(align_path[1].x(), 3.0, kEpsilon);
    EXPECT_NEAR(align_path[1].y(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[1].z(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[1].theta(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[1].kappa(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[1].lambda(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[1].s(), 0.5, kEpsilon);

    EXPECT_NEAR(align_path[2].x(), 4.0, kEpsilon);
    EXPECT_NEAR(align_path[2].y(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[2].z(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[2].theta(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[2].kappa(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[2].lambda(), 0.0, kEpsilon);
    EXPECT_NEAR(align_path[2].s(), 1.5, kEpsilon);
  }
}

TEST(AsyncPlannerUtilTest, ShouldRunAccPlannerInAsyncPlanner) {
  EXPECT_TRUE(ShouldRunAccPlannerInAsyncPlanner(AsyncPlannerState{
      .secondary_counter = 0,
      .latest_multi_task_est_result = nullptr,
  }));

  EXPECT_FALSE(ShouldRunAccPlannerInAsyncPlanner(AsyncPlannerState{
      .secondary_counter = 0,
      .latest_multi_task_est_result =
          std::make_shared<AsyncMultiTaskEstOutput>(),
  }));

  EXPECT_FALSE(ShouldRunAccPlannerInAsyncPlanner(AsyncPlannerState{
      .secondary_counter = std::nullopt,
      .latest_multi_task_est_result =
          std::make_shared<AsyncMultiTaskEstOutput>(),
  }));

  EXPECT_FALSE(ShouldRunAccPlannerInAsyncPlanner(AsyncPlannerState{
      .secondary_counter = std::nullopt,
      .latest_multi_task_est_result = nullptr,
  }));
}

TEST(AsyncPlannerUtilTest, StitchStPathTrajectoryWithPastTrajectory) {
  // Do not stitch previous trajectory test.
  {
    const TrajectoryProto previous_trajectory;
    const std::optional<int> start_index_on_prev_traj = std::nullopt;
    const std::vector<PathPoint> st_path_points;
    std::vector<PathPoint> st_path_points_including_past;

    StitchStPathTrajectoryWithPastTrajectory(
        previous_trajectory, start_index_on_prev_traj, st_path_points,
        &st_path_points_including_past);

    EXPECT_TRUE(st_path_points_including_past.empty());
  }

  const auto mock_apollo_traj_point =
      [](double s) -> ApolloTrajectoryPointProto {
    ApolloTrajectoryPointProto traj_point;
    auto& path_point = *traj_point.mutable_path_point();
    path_point.set_x(s);
    path_point.set_y(0.0);
    path_point.set_z(0.0);
    path_point.set_theta(0.0);
    path_point.set_kappa(0.0);
    path_point.set_lambda(0.0);
    path_point.set_s(s);
    return traj_point;
  };

  const auto mock_path_point = [](double s) -> PathPoint {
    PathPoint p;
    p.set_x(s);
    p.set_y(0.0);
    p.set_z(0.0);
    p.set_theta(0.0);
    p.set_kappa(0.0);
    p.set_lambda(0.0);
    p.set_s(s);
    return p;
  };

  // Stitch previous trajectory test.
  {
    TrajectoryProto previous_trajectory;
    *previous_trajectory.add_past_points() = mock_apollo_traj_point(-5.0);
    *previous_trajectory.add_past_points() = mock_apollo_traj_point(-4.0);
    *previous_trajectory.add_past_points() = mock_apollo_traj_point(-3.0);

    *previous_trajectory.add_trajectory_point() = mock_apollo_traj_point(-2.0);
    *previous_trajectory.add_trajectory_point() = mock_apollo_traj_point(-1.0);
    *previous_trajectory.add_trajectory_point() = mock_apollo_traj_point(0.0);
    *previous_trajectory.add_trajectory_point() = mock_apollo_traj_point(1.0);
    *previous_trajectory.add_trajectory_point() = mock_apollo_traj_point(2.0);

    constexpr int kPathPointNUm = 5;
    std::vector<PathPoint> st_path_points;
    st_path_points.reserve(kPathPointNUm);
    for (int i = 0; i < kPathPointNUm; ++i) {
      st_path_points.push_back(mock_path_point(i));
    }

    const std::optional<int> start_index_on_prev_traj = 2;
    std::vector<PathPoint> st_path_points_including_past;

    StitchStPathTrajectoryWithPastTrajectory(
        previous_trajectory, start_index_on_prev_traj, st_path_points,
        &st_path_points_including_past);

    constexpr int kResPathPointsNium = 10;
    EXPECT_EQ(st_path_points_including_past.size(), kResPathPointsNium);
    // <path_point.x(), path_point.s()>.
    const std::vector<std::pair<double, double>> check_table = {
        {-5.0, 0.0}, {-4.0, 1.0}, {-3.0, 2.0}, {-2.0, 3.0}, {-1.0, 4.0},
        {0.0, 5.0},  {1.0, 6.0},  {2.0, 7.0},  {3.0, 8.0},  {4.0, 9.0}};
    for (int i = 0; i < kResPathPointsNium; ++i) {
      EXPECT_NEAR(st_path_points_including_past[i].x(), check_table[i].first,
                  kEpsilon);
      EXPECT_NEAR(st_path_points_including_past[i].s(), check_table[i].second,
                  kEpsilon);
    }
  }
}

TEST(AsyncPlannerUtilTest, IsNonEmptyPlannerResult) {
  for (const auto status :
       {PlannerStatusProto::OK,
        PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE,
        PlannerStatusProto::INITIALIZER_FAILED,
        PlannerStatusProto::OPTIMIZER_FAILED,
        PlannerStatusProto::PATH_EXTENSION_FAILED,
        PlannerStatusProto::SPEED_OPTIMIZER_FAILED,
        PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
        PlannerStatusProto::SELECTOR_FAILED,
        PlannerStatusProto::MODEL_PRECONDITION_CHECK_FAILED,
        PlannerStatusProto::MODEL_INFERENCE_FAILED,
        PlannerStatusProto::MODEL_OUTPUT_POSTPROCESS_FAILED,
        PlannerStatusProto::BRANCH_RESULT_IGNORED,
        PlannerStatusProto::LC_SAFETY_CHECK_FAILED}) {
    EXPECT_TRUE(IsNonEmptyPlannerResult(PlannerStatus(status, "")));
  }

  for (const auto status :
       {PlannerStatusProto::ROUTE_MSG_UNAVAILABLE,
        PlannerStatusProto::GEAR_NOT_READY,
        PlannerStatusProto::INPUT_INCORRECT,
        PlannerStatusProto::ROUTING_FAILED,
        PlannerStatusProto::OBJECT_MANAGER_FAILED,
        PlannerStatusProto::REFERENCE_PATH_UNAVAILABLE,
        PlannerStatusProto::SCHEDULER_UNAVAILABLE,
        PlannerStatusProto::FALLBACK_PREVIOUS_INFO_UNAVAILABLE,
        PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED,
        PlannerStatusProto::RESET_PREV_TARGET_LANE_PATH_FAILED,
        PlannerStatusProto::UPDATE_TARGET_LANE_PATH_FAILED,
        PlannerStatusProto::LOCAL_LANE_MAP_BUILDER_FAILED,
        PlannerStatusProto::PLANNER_ABNORMAL_EXIT,
        PlannerStatusProto::TRAFFIC_LIGHT_INFO_COLLECTOR_FAILED,
        PlannerStatusProto::PLANNER_STATE_INCOMPLETE,
        PlannerStatusProto::EXPERT_TRAJ_UNAVAILABLE,
        PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED,
        PlannerStatusProto::EXPERT_SPEED_IN_CONTRADICTORY_WITH_SPEED_FINDER,
        PlannerStatusProto::LOW_FREQ_SCHEDULE_FUTURE_FAILED,
        PlannerStatusProto::LOW_FREQ_RESULT_NOT_YET_AVAILABLE,
        PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE,
        PlannerStatusProto::OPTIMIZER_DIFF_INTENTION_EXPERT,
        PlannerStatusProto::LCC_ODC_CHECK_FAIL,
        PlannerStatusProto::BUILD_DRIVING_MAP_FAILED,
        PlannerStatusProto::ACC_CORRIDOR_FAILED,
        PlannerStatusProto::ACC_SPEED_FAILED,
        PlannerStatusProto::ACC_TRAJECTORY_VALIDATION_FAILED,
        PlannerStatusProto::PROJECT_TO_ONLINE_MAP_FAILED,
        PlannerStatusProto::ACC_ALL_PLAN_FAIL}) {
    EXPECT_FALSE(IsNonEmptyPlannerResult(PlannerStatus(status, "")));
  }
}

TEST(AsyncPlannerUtilTest, FillHighFreqPlannerDebug) {
  // TODO(jiayu): Add unit test for FillHighFreqPlannerDebug.
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
