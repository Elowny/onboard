#include "onboard/planner/plan/multi_tasks_alcc_planner.h"

#include <utility>

#include "gtest/gtest.h"

#include "onboard/async/thread_pool.h"
#include "onboard/math/geometry/util.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
// #include "onboard/planner/common/plot_util.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"

#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
TEST(MultiTasksAlccPlanner, UseOnlineMapTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");

  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();

  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 0.0),
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

  PathBoundedEstPlannerOutput output;
  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);
  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  const auto planner_status = RunMultiTasksAlccPlanner(
      MultiTasksAlccPlannerInput{
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
          .is_standwait = false,
      },
      &output, thread_pool.get());

  EXPECT_OK(planner_status);
  EXPECT_EQ(output.est_status_list.size(), 1);
  EXPECT_EQ(output.est_planner_output_list.size(), 1);
  EXPECT_EQ(output.est_planner_debug_list.size(), 1);

  const auto& selected_output = output.est_planner_output_list[0];
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_GT(last_traj_pt.x(), 130.0);
  EXPECT_NEAR(last_traj_pt.y(), 0.0, 1e-1);

  //   SendApolloTrajectoryPointsToCanvas(selected_output.traj_points, "traj",
  //                                      vis::Color::kLightGreen);
}
}  // namespace
}  // namespace qcraft::planner
