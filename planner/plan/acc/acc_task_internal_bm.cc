#include <memory>
#include <optional>
#include <vector>

#include "absl/types/span.h"
#include "benchmark/benchmark.h"

#include "common/proto/qacc.pb.h"

#include "onboard/global/clock.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_flags.h"
#include "onboard/planner/plan/acc/acc_planner_output.h"
#include "onboard/planner/plan/acc/acc_task_input.h"
#include "onboard/planner/plan/acc/acc_task_internal.h"
#include "onboard/planner/plan/acc/test_util.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

namespace qcraft {
namespace planner {
namespace {
TrajectoryProto BuildStationaryPreviousTrajectory(const Vec2d& pos,
                                                  double heading) {
  prediction::UniCycleState state{
      .x = pos.x(),
      .y = pos.y(),
      .v = 0.0,
      .heading = heading,
  };
  const auto pred_traj_pts =
      prediction::DevelopStaticTrajectory(state, /*dt=*/0.1, /*horizon=*/9.9);
  const auto apollo_traj_pts =
      ToApolloTrajectoryPointProto(absl::MakeSpan(pred_traj_pts));

  TrajectoryProto traj_proto;
  for (const auto& pt : apollo_traj_pts) {
    *traj_proto.add_trajectory_point() = pt;
  }
  return traj_proto;
}

void BM_AccTaskInternal_Map(benchmark::State& state) {  // NOLINT
  const Vec2d kAvPosition = Vec2d(0.0, 0.0);            // NOLINT
  constexpr double kAvHeading = 0.0;
  constexpr double kAvForwardSpeed = 10;
  FLAGS_planner_acc_plan_for_all_sources = false;
  // Generate context inputs.
  auto param_manager = CreateParamManagerFromCarId("Q2506");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();
  const auto& psmm = CreateDojoTestPSMM();
  const auto planner_params = CreateDefaultPlannerParamOnlyFillAccParams();

  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(kAvPosition.x());
  av_pose.mutable_pos_smooth()->set_y(kAvPosition.y());
  av_pose.set_yaw(kAvHeading);
  av_pose.mutable_vel_body()->set_x(kAvForwardSpeed);

  const ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(av_pose);

  PlanStartPointInfo plan_start_point_info{
      .reset = false,
      .start_point = plan_start_point,
      .plan_time = Clock::Now(),
  };
  std::unique_ptr<SpacetimeTrajectoryManager> st_traj_mgr =
      std::make_unique<SpacetimeTrajectoryManager>();
  QACCTaskProto acc_task_proto;
  const auto prev_traj =
      BuildStationaryPreviousTrajectory(kAvPosition, kAvHeading);
  const std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj = {
      prev_traj.trajectory_point().begin(), prev_traj.trajectory_point().end()};
  std::unique_ptr<prediction::AvContext> av_context =
      std::make_unique<prediction::AvContext>(/*capacity=*/200.0, /*len=*/1.5);

  AccTaskInput task_input{
      .planner_semantic_map_manager = &psmm,
      .pose = &av_pose,
      .steering_percentage = std::nullopt,
      .acc_params = &planner_params.acc_params(),
      .vehicle_geometry_params = &vehicle_param_api.vehicle_geometry_params(),
      .vehicle_drive_params = &vehicle_param_api.vehicle_drive_params(),
      .plan_start_point_info = &plan_start_point_info,
      .plan_time = plan_start_point_info.plan_time,
      .st_traj_mgr = st_traj_mgr.get(),
      .prev_trajectory = &prev_traj,
      .time_aligned_prev_traj = &time_aligned_prev_traj,
      .average_kappa = av_context->GetAvKappaCacheAverage(),
      .acc_task_proto = &acc_task_proto,
  };
  AccPlannerOutput planner_output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(RunAccPlanner(task_input, &planner_output));
  }
}

void BM_AccTaskInternal_Combined(benchmark::State& state) {  // NOLINT
  const Vec2d kAvPosition = Vec2d(95.433, -2.44);            // NOLINT
  constexpr double kAvHeading = 0.038;
  constexpr double kAvForwardSpeed = 10;
  FLAGS_planner_acc_plan_for_all_sources = false;
  // Generate context inputs.
  auto param_manager = CreateParamManagerFromCarId("Q2506");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();
  const auto& psmm = CreateDojoTestPSMM();
  const auto planner_params = CreateDefaultPlannerParamOnlyFillAccParams();

  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(kAvPosition.x());
  av_pose.mutable_pos_smooth()->set_y(kAvPosition.y());
  av_pose.set_yaw(kAvHeading);
  av_pose.mutable_vel_body()->set_x(kAvForwardSpeed);

  const ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(av_pose);

  PlanStartPointInfo plan_start_point_info{
      .reset = false,
      .start_point = plan_start_point,
      .plan_time = Clock::Now(),
  };
  std::unique_ptr<SpacetimeTrajectoryManager> st_traj_mgr =
      std::make_unique<SpacetimeTrajectoryManager>();
  QACCTaskProto acc_task_proto;
  const auto prev_traj =
      BuildStationaryPreviousTrajectory(kAvPosition, kAvHeading);
  const std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj = {
      prev_traj.trajectory_point().begin(), prev_traj.trajectory_point().end()};
  std::unique_ptr<prediction::AvContext> av_context =
      std::make_unique<prediction::AvContext>(/*capacity=*/200.0, /*len=*/1.5);

  AccTaskInput task_input{
      .planner_semantic_map_manager = &psmm,
      .pose = &av_pose,
      .steering_percentage = std::nullopt,
      .acc_params = &planner_params.acc_params(),
      .vehicle_geometry_params = &vehicle_param_api.vehicle_geometry_params(),
      .vehicle_drive_params = &vehicle_param_api.vehicle_drive_params(),
      .plan_start_point_info = &plan_start_point_info,
      .plan_time = plan_start_point_info.plan_time,
      .st_traj_mgr = st_traj_mgr.get(),
      .prev_trajectory = &prev_traj,
      .time_aligned_prev_traj = &time_aligned_prev_traj,
      .average_kappa = av_context->GetAvKappaCacheAverage(),
      .acc_task_proto = &acc_task_proto,
  };
  AccPlannerOutput planner_output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(RunAccPlanner(task_input, &planner_output));
  }
}

void BM_AccTaskInternal_Estimated(benchmark::State& state) {  // NOLINT
  const Vec2d kAvPosition = Vec2d(87.97, 2.84);               // NOLINT
  constexpr double kAvHeading = 0.038;
  constexpr double kAvForwardSpeed = 5;
  FLAGS_planner_acc_plan_for_all_sources = false;
  // Generate context inputs.
  auto param_manager = CreateParamManagerFromCarId("Q2506");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();
  const auto& psmm = CreateDojoTestPSMM();
  const auto planner_params = CreateDefaultPlannerParamOnlyFillAccParams();

  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(kAvPosition.x());
  av_pose.mutable_pos_smooth()->set_y(kAvPosition.y());
  av_pose.set_yaw(kAvHeading);
  av_pose.mutable_vel_body()->set_x(kAvForwardSpeed);

  const ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(av_pose);

  PlanStartPointInfo plan_start_point_info{
      .reset = false,
      .start_point = plan_start_point,
      .plan_time = Clock::Now(),
  };
  std::unique_ptr<SpacetimeTrajectoryManager> st_traj_mgr =
      std::make_unique<SpacetimeTrajectoryManager>();
  QACCTaskProto acc_task_proto;
  const auto prev_traj =
      BuildStationaryPreviousTrajectory(kAvPosition, kAvHeading);
  const std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj = {
      prev_traj.trajectory_point().begin(), prev_traj.trajectory_point().end()};
  std::unique_ptr<prediction::AvContext> av_context =
      std::make_unique<prediction::AvContext>(/*capacity=*/200.0, /*len=*/1.5);

  AccTaskInput task_input{
      .planner_semantic_map_manager = &psmm,
      .pose = &av_pose,
      .steering_percentage = std::nullopt,
      .acc_params = &planner_params.acc_params(),
      .vehicle_geometry_params = &vehicle_param_api.vehicle_geometry_params(),
      .vehicle_drive_params = &vehicle_param_api.vehicle_drive_params(),
      .plan_start_point_info = &plan_start_point_info,
      .plan_time = plan_start_point_info.plan_time,
      .st_traj_mgr = st_traj_mgr.get(),
      .prev_trajectory = &prev_traj,
      .time_aligned_prev_traj = &time_aligned_prev_traj,
      .average_kappa = av_context->GetAvKappaCacheAverage(),
      .acc_task_proto = &acc_task_proto,
  };
  AccPlannerOutput planner_output;
  for (auto _ : state) {
    benchmark::DoNotOptimize(RunAccPlanner(task_input, &planner_output));
  }
}

BENCHMARK(BM_AccTaskInternal_Map);
BENCHMARK(BM_AccTaskInternal_Combined);
BENCHMARK(BM_AccTaskInternal_Estimated);
}  // namespace
}  // namespace planner
}  // namespace qcraft

BENCHMARK_MAIN();
