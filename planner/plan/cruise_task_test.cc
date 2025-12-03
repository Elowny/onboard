#include "onboard/planner/plan/cruise_task.h"

#include <utility>

#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"

#include "gtest/gtest.h"

#include "onboard/async/future.h"
#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/run_context.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_finder.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/ml/planner_ml_defs.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"
// #include "onboard/planner/common/plot_util.h"
// #include "onboard/planner/router/plot_util.h"
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
TEST(RunCruiseTask, BaseTest) {
  // Set cruise async flag.
  FLAGS_planner_async_low_freq_cycle_iterations = 2;
  // Disable captain net in unit test.
  FLAGS_planner_enable_captain_net_j5 = false;
  FLAGS_planner_enable_captain_net_onnx_trt = false;
  // Disable act net in unit test.
  FLAGS_planner_enable_act_net_speed = false;
  // Set assist mode.
  FLAGS_run_mode_type = 1;

  auto param_manager = CreateParamManagerFromCarId("Q8001");
  auto param_finder = CreateParamFinderWithCarId("Q8001");

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

  // Construct planner semantic map.
  const auto psmm_ptr = CreateDojoTestPSMMSharedPtr();
  const auto& psmm = *psmm_ptr;
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Construct route manager output.
  RouteManagerOutput route_manager_output;
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  route_manager_output.route_sections_from_current = route_sections;
  //   SendRouteSectionsAreaToCanvas(psmm, route_sections, "straight_route");
  const RouteSectionsInfo sections_info(psmm, &route_sections);
  ASSIGN_OR_DIE(route_manager_output.route_navi_info,
                CalcNaviInfoByLaneGraph(*smm, sections_info, /*avoid_lanes=*/{},
                                        /*preview_dist=*/100.0));

  // Construct planner input.
  AutonomyStateProto autonomy_state;
  autonomy_state.set_autonomy_state(AutonomyStateProto::READY_TO_AUTO_DRIVE);

  PlannerInput planner_input{
      .semantic_map_manager =
          psmm_ptr->semantic_map_multilevel_spatial_index()->semantic_manager(),
      .planner_semantic_map_manager = psmm_ptr,
      .semantic_map_multilevel_spatial_index =
          psmm_ptr->semantic_map_multilevel_spatial_index(),
      .planner_state_proto = nullptr,
      .planner_params = planner_params,
      .vehicle_params = vehicle_params,
      .pose = std::make_shared<const PoseProto>(sdc_pose),
      .real_objects = std::make_shared<const ObjectsProto>(),
      .virtual_objects = std::make_shared<const ObjectsProto>(),
      .av_objects = std::make_shared<const ObjectsProto>(),
      .autonomy_state =
          std::make_shared<const AutonomyStateProto>(std::move(autonomy_state)),
      .traffic_light_states = std::make_shared<const TrafficLightStatesProto>(),
      .driver_action = nullptr,
      .rerouting_request = nullptr,
      .remote_assist_to_car = nullptr,
      .chassis = std::make_shared<const Chassis>(),
      .localization_transform = nullptr,
      .routing_result = nullptr,
      .route_mgr_output = nullptr,
      .semantic_map_modification = nullptr,
      .sensor_fovs = nullptr,
      .online_semantic_map = nullptr,
      .prediction = std::make_unique<const ObjectsPredictionProto>(),
      .prediction_debug = std::make_shared<const PredictionDebugProto>(),
      .planner_model_pool =
          std::make_unique<ModelPool>(*param_manager, *param_finder),
      .av_context = std::make_unique<prediction::AvContext>(
          ml::kAvHistoryBufferSize, ml::kAvHistoryBufferLen),
      .log_prediction = std::make_unique<const ObjectsPredictionProto>(),
      .log_av_trajectory = std::make_shared<const TrajectoryProto>(),
  };

  // Construct cruise task input.
  const ExternalCommandQueue ext_cmd_queue;
  const std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj_points;
  const TrajectoryProto previous_trajectory;
  AebPlannerOutput aeb_output;
  aeb_output.trajectory_points = aeb::PlanEmergencyStopTrajectory(
      plan_start_point_info.start_point,
      plan_start_point_info.path_s_increment_from_previous_frame,
      plan_start_point_info.reset, time_aligned_prev_traj_points,
      planner_params);

  const CruiseTaskInput input{
      .rerouted = false,
      .coordinate_converter = &cc,
      .planner_input = &planner_input,
      .planner_params = &planner_params,
      .route_output = &route_manager_output,
      .plan_start_point_info = &plan_start_point_info,
      .ext_cmd_queue = &ext_cmd_queue,
      .plan_time = plan_time,
      .st_traj_mgr = std::make_shared<const SpacetimeTrajectoryManager>(),
      .object_manager = std::make_shared<const PlannerObjectManager>(),
      .time_aligned_prev_traj_points = &time_aligned_prev_traj_points,
      .previous_trajectory = &previous_trajectory,
      .aeb_output = &aeb_output,
  };

  CruiseTaskOutput result;
  PlannerState planner_state;
  ExternalCommandStatus ext_cmd_status;
  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  auto& counter = planner_state.async_planner_state.counter;
  // counter = -1.
  {
    EXPECT_NOT_OK(RunCruiseTask(input, &result, &planner_state, &ext_cmd_status,
                                thread_pool.get()));
    EXPECT_EQ(counter, 0);
    planner_state.async_planner_state.future_multi_task_est_status.Wait();
  }
  // counter = 0.
  {
    EXPECT_NOT_OK(RunCruiseTask(input, &result, &planner_state, &ext_cmd_status,
                                thread_pool.get()));
    EXPECT_EQ(counter, 1);
  }
  // counter = 1.
  {
    EXPECT_OK(RunCruiseTask(input, &result, &planner_state, &ext_cmd_status,
                            thread_pool.get()));
    EXPECT_EQ(counter, -1);
    // auto& traj_points = *result.trajectory_info.mutable_trajectory_point();
    // SendApolloTrajectoryPointsToCanvas(
    //     ConvertToTrajectoryVector(std::move(traj_points)), "traj",
    //     vis::Color::kLightGreen);
  }
  // counter = -1.
  {
    EXPECT_OK(RunCruiseTask(input, &result, &planner_state, &ext_cmd_status,
                            thread_pool.get()));
    EXPECT_EQ(counter, 0);
  }
}

}  // namespace
}  // namespace qcraft::planner
