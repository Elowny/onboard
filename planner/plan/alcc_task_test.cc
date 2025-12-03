#include "onboard/planner/plan/alcc_task.h"

#include <utility>

#include "absl/status/statusor.h"
#include "absl/time/clock.h"

#include "gtest/gtest.h"

#include "onboard/async/future.h"
#include "onboard/async/thread_pool.h"
#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
// std::vector<ApolloTrajectoryPointProto> ConvertToTrajectoryVector(
//     google::protobuf::RepeatedPtrField<ApolloTrajectoryPointProto> proto) {
//   std::vector<ApolloTrajectoryPointProto> res;
//   res.reserve(proto.size());
//   for (auto& pt : proto) {
//     res.push_back(std::move(pt));
//   }
//   return res;
// }
TEST(RunAlccTask, BaseTest) {
  // Set alcc async flag.
  FLAGS_planner_alcc_async_low_freq_cycle_iterations = 2;
  FLAGS_planner_max_alcc_async_iterations = 2;

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
  const Chassis chassis;
  const std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj_points;
  const TrajectoryProto log_av_trajectory;
  const ExternalCommandQueue ext_cmd_queue;
  const AlccTaskInput input{
      .planner_semantic_map_manager = psmm_ptr,
      .pose = &sdc_pose,
      .chassis = &chassis,
      .autonomy_state = &autonomy_state,
      .alcc_params = &planner_params.alcc_params(),
      .acc_params = &planner_params.acc_params(),
      .vehicle_params = &vehicle_params,
      .plan_start_point_info = &plan_start_point_info,
      .ext_cmd_queue = &ext_cmd_queue,
      .plan_time = plan_time,
      .st_traj_mgr = st_traj_mgr_ptr,
      .object_manager = object_manager_ptr,
      .time_aligned_prev_traj_points = &time_aligned_prev_traj_points,
      .log_av_trajectory = &log_av_trajectory,
      .online_semantic_map = online_smm_proto_ptr,
      .use_online_semantic_map = true,
      .av_context = nullptr,
  };

  AlccTaskOutput output;
  PlannerState planner_state;
  ExternalCommandStatus ext_cmd_status;
  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  auto& counter = planner_state.async_planner_state.counter;
  // counter = -1.
  {
    EXPECT_NOT_OK(RunAlccTask(input, &output, &planner_state, &ext_cmd_status,
                              thread_pool.get()));
    EXPECT_EQ(counter, 0);
    planner_state.async_planner_state.future_multi_task_est_status.Wait();
  }
  // counter = 0.
  {
    EXPECT_NOT_OK(RunAlccTask(input, &output, &planner_state, &ext_cmd_status,
                              thread_pool.get()));
    EXPECT_EQ(counter, 1);
  }
  // counter = 1.
  {
    EXPECT_OK(RunAlccTask(input, &output, &planner_state, &ext_cmd_status,
                          thread_pool.get()));
    EXPECT_EQ(counter, -1);
    // auto& traj_points = *output.trajectory_info.mutable_trajectory_point();
    // SendApolloTrajectoryPointsToCanvas(
    //     ConvertToTrajectoryVector(std::move(traj_points)), "traj",
    //     vis::Color::kLightGreen);
  }
  // counter = -1.
  {
    EXPECT_OK(RunAlccTask(input, &output, &planner_state, &ext_cmd_status,
                          thread_pool.get()));
    EXPECT_EQ(counter, 0);
    planner_state.async_planner_state.future_multi_task_est_status.Wait();
  }
}

}  // namespace
}  // namespace qcraft::planner
