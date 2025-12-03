#include "onboard/planner/plan/async_alcc_planner.h"

#include <memory>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/async/thread_pool.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
// #include "onboard/planner/common/plot_util.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "onboard/async/future.h"
#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/assist/proto/assist_plan_state.pb.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/history_buffer.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
TEST(AsyncAlccPlannerTest, BaseTest) {
  // Set alcc async flag.
  FLAGS_planner_alcc_async_low_freq_cycle_iterations = 2;

  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(0.0, 0.0),
                 /*heading=*/0.0, Vec2d(11.0, 0.0));
  const auto plan_start_point = ConvertToTrajPointProto(sdc_pose);
  const PlanStartPointInfo plan_start_point_info{
      .reset = false,
      .start_point = plan_start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };

  // Construct online map.
  const auto& whole_psmm = CreateDojoTestPSMM();
  ASSIGN_OR_DIE(
      auto online_smm_proto,
      RunOnlineSemanticMapConverter(
          whole_psmm, OnlineSemanticMapConverterOption{
                          .timestamp_s = ToUnixDoubleSeconds(plan_time),
                          .smooth_x = sdc_pose.pos_smooth().x(),
                          .smooth_y = sdc_pose.pos_smooth().y(),
                          .smooth_yaw = sdc_pose.yaw(),
                          .look_ahead_distance = 150.0,
                          .look_back_distance = 20.0}));

  // Construct semantic map manager from online semantic map.
  ASSIGN_OR_DIE(auto psmm_ptr, BuildOnlineMapPsmm(online_smm_proto));
  auto online_smm_proto_ptr =
      std::make_shared<const mapping::OnlineSemanticMapProto>(
          std::move(online_smm_proto));
  auto st_traj_mgr_ptr = std::make_shared<const SpacetimeTrajectoryManager>();
  auto object_manager_ptr = std::make_shared<const PlannerObjectManager>();

  AutonomyStateProto autonomy_state;
  autonomy_state.set_autonomy_state(AutonomyStateProto::READY_TO_AUTO_DRIVE);
  const AssistPlanStateProto assist_plan_state;
  const Chassis chassis;
  const std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj_points;
  const TrajectoryProto log_av_trajectory;
  const TrajectoryProto previous_trajectory;
  const DeciderStateProto decider_state;
  const InitializerStateProto initializer_state;
  const SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;
  const ExternalCommandInfo ext_cmd_info;
  const HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  const MultiTasksAlccPlannerInput input{
      .planner_semantic_map_manager = psmm_ptr,
      .pose = &sdc_pose,
      .chassis = &chassis,
      .autonomy_state = &autonomy_state,
      .alcc_params = &planner_params.alcc_params(),
      .vehicle_params = &vehicle_params,
      .plan_start_point_info = &plan_start_point_info,
      .plan_time = plan_time,
      .st_traj_mgr = st_traj_mgr_ptr,
      .object_manager = object_manager_ptr,
      .time_aligned_prev_traj_points = &time_aligned_prev_traj_points,
      .log_av_trajectory = &log_av_trajectory,
      .online_semantic_map = online_smm_proto_ptr,
      .assist_plan_state = &assist_plan_state,
      .decider_state = &decider_state,
      .initializer_state = &initializer_state,
      .selected_trajectory_optimizer_state_proto = nullptr,
      .st_planner_object_trajectories = &st_planner_object_trajectories,
      .previous_trajectory = &previous_trajectory,
      .ext_cmd_status = &ext_cmd_info.status,
      .prev_low_freq_psmm = nullptr,
      .use_online_semantic_map = true,
      .av_context = nullptr,
      .online_map_drift_buffer = &online_map_drift_buffer,
      .is_standwait = false};

  const QACCTaskProto acc_task_proto;
  std::optional<AccPlannerOutput> acc_output;
  AsyncPlannerOutput async_planner_output;
  AsyncPlannerState async_planner_state;
  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);
  // Run low frequency module test.
  {
    async_planner_state.counter = 0;
    EXPECT_NOT_OK(RunAsyncAlccPlanner(input, acc_task_proto, &acc_output,
                                      &async_planner_output,
                                      &async_planner_state, thread_pool.get()));

    EXPECT_TRUE(async_planner_state.future_multi_task_est_status.IsValid());
    EXPECT_FALSE(async_planner_state.future_multi_task_est_result == nullptr);
    EXPECT_TRUE(async_planner_state.latest_multi_task_est_result == nullptr);
    EXPECT_TRUE(async_planner_output.traj_points.empty());
  }

  // Run high frequency module test.
  {
    async_planner_state.future_multi_task_est_status.Wait();
    async_planner_state.counter = 2;
    EXPECT_OK(RunAsyncAlccPlanner(input, acc_task_proto, &acc_output,
                                  &async_planner_output, &async_planner_state,
                                  thread_pool.get()));

    EXPECT_FALSE(async_planner_state.future_multi_task_est_status.IsValid());
    EXPECT_TRUE(async_planner_state.future_multi_task_est_result == nullptr);
    EXPECT_FALSE(async_planner_state.latest_multi_task_est_result == nullptr);
    EXPECT_FALSE(async_planner_output.traj_points.empty());

    // const auto& traj_points = async_planner_output.traj_points;
    // SendApolloTrajectoryPointsToCanvas(traj_points, "traj",
    //                                    vis::Color::kLightGreen);
  }
}
}  // namespace
}  // namespace qcraft::planner
