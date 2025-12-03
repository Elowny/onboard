#include "onboard/planner/plan/plan_task_dispatcher.h"

#include <algorithm>
#include <deque>
#include <memory>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/time/clock.h"

#include "gtest/gtest.h"

#include "onboard/async/future.h"
#include "onboard/async/thread_pool.h"
#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/run_context.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
// #include "onboard/planner/common/plot_util.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/ml/planner_ml_defs.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/plan/plan_task.h"
#include "onboard/planner/plan/proto/plan_task.pb.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory.pb.h"
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
TEST(RunPlanTaskDispatcher, AlccPlanTest) {
  FLAGS_planner_alcc_async_low_freq_cycle_iterations = 2;
  FLAGS_planner_max_alcc_async_iterations = 2;
  FLAGS_run_mode_type = 1;

  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(0.0, 0.0),
                 /*heading=*/0.0, Vec2d(11.0, 0.0));

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

  // Construct autonomy state.
  AutonomyStateProto autonomy_state;
  autonomy_state.set_autonomy_state(AutonomyStateProto::AUTO_DRIVE);
  autonomy_state.mutable_assist_state()->set_assist_drive_system_state(
      AssistStateProto::ASSIST_LCC_ACTIVE);

  const auto prev_planner_state = std::make_shared<PlannerStateProto>();
  // Construct planner input.
  PlannerInput input{
      .semantic_map_manager = nullptr,
      .planner_semantic_map_manager = psmm_ptr,
      .semantic_map_multilevel_spatial_index = nullptr,
      .planner_state_proto = prev_planner_state,
      .planner_params = planner_params,
      .vehicle_params = vehicle_params,
      .pose = std::make_shared<const PoseProto>(std::move(sdc_pose)),
      .real_objects = nullptr,
      .virtual_objects = nullptr,
      .av_objects = nullptr,
      .autonomy_state =
          std::make_shared<const AutonomyStateProto>(std::move(autonomy_state)),
      .traffic_light_states = std::make_shared<TrafficLightStatesProto>(),
      .driver_action = nullptr,
      .rerouting_request = nullptr,
      .remote_assist_to_car = nullptr,
      .chassis = std::make_shared<const Chassis>(),
      .localization_transform = nullptr,
      .routing_result = nullptr,
      .route_mgr_output = nullptr,
      .semantic_map_modification = nullptr,
      .sensor_fovs = nullptr,
      .online_semantic_map =
          std::make_shared<const mapping::OnlineSemanticMapProto>(
              std::move(online_smm_proto)),
      .prediction = std::make_unique<const ObjectsPredictionProto>(),
      .prediction_debug = nullptr,
      .planner_model_pool = nullptr,
      .av_context = std::make_unique<prediction::AvContext>(
          ml::kAvHistoryBufferSize, ml::kAvHistoryBufferLen),
      .log_prediction = nullptr,
      .log_av_trajectory = std::make_shared<const TrajectoryProto>(),
  };

  const RouteManagerOutput route_output;
  const ObjectsProto objects_proto;
  ExternalCommandInfo ext_cmd_info;
  PlannerState planner_state;
  PlannerOutput output;
  // Mock prev plan task is ACC_PLAN.
  planner_state.plan_task_queue = {PlanTask(ACC_PLAN)};

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  auto& counter = planner_state.async_planner_state.counter;
  // Run acc plan, counter = -1.
  {
    FLAGS_planner_simplify_debug_proto = false;
    EXPECT_OK(RunPlanTaskDispatcher(
        psmm_ptr->coordinate_converter(), input, route_output, &objects_proto,
        plan_time, /*time_interval=*/FLAGS_planner_main_loop_interval,
        &ext_cmd_info, &planner_state, &output, thread_pool.get()));
    EXPECT_EQ(counter, 0);

    const auto& planner_debug = output.planner_debug();
    EXPECT_EQ(planner_debug.planner_status().status(), PlannerStatusProto::OK);
    EXPECT_EQ(planner_debug.est_planner_status().status(),
              PlannerStatusProto::LOW_FREQ_RESULT_NOT_YET_AVAILABLE);
    planner_state.async_planner_state.future_multi_task_est_status.Wait();
  }
  // Run acc plan, counter = 0.
  {
    FLAGS_planner_simplify_debug_proto = true;
    EXPECT_OK(RunPlanTaskDispatcher(
        psmm_ptr->coordinate_converter(), input, route_output, &objects_proto,
        plan_time, /*time_interval=*/FLAGS_planner_main_loop_interval,
        &ext_cmd_info, &planner_state, &output, thread_pool.get()));
    const auto& planner_debug = output.planner_debug();
    EXPECT_EQ(planner_debug.planner_status().status(), PlannerStatusProto::OK);
    EXPECT_EQ(planner_debug.est_planner_status().status(),
              PlannerStatusProto::LOW_FREQ_RESULT_NOT_YET_AVAILABLE);
    EXPECT_EQ(counter, 1);
  }
  // Run alcc plan, counter = 1.
  {
    FLAGS_planner_simplify_debug_proto = false;
    EXPECT_OK(RunPlanTaskDispatcher(
        psmm_ptr->coordinate_converter(), input, route_output, &objects_proto,
        plan_time, /*time_interval=*/FLAGS_planner_main_loop_interval,
        &ext_cmd_info, &planner_state, &output, thread_pool.get()));
    const auto& planner_debug = output.planner_debug();
    EXPECT_EQ(planner_debug.planner_status().status(), PlannerStatusProto::OK);
    EXPECT_EQ(planner_debug.est_planner_status().status(),
              PlannerStatusProto::OK);
    EXPECT_EQ(counter, -1);
    // auto& traj_points =
    //     *output.mutable_trajectory()->mutable_trajectory_point();
    // SendApolloTrajectoryPointsToCanvas(
    //     ConvertToTrajectoryVector(std::move(traj_points)), "traj",
    //     vis::Color::kLightGreen);
  }
}
}  // namespace
}  // namespace qcraft::planner
