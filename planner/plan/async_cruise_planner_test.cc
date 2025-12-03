#include "onboard/planner/plan/async_cruise_planner.h"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
// #include "onboard/planner/common/plot_util.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
// #include "onboard/planner/router/plot_util.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "onboard/async/future.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_finder.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/decision/traffic_light/traffic_light_info_collector.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/ml/planner_ml_defs.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
TEST(AsyncCruisePlannerTest, BaseTest) {
  // Set cruise async flag.
  FLAGS_planner_async_low_freq_cycle_iterations = 2;
  // Disable captain net in unit test.
  FLAGS_planner_enable_captain_net_j5 = false;
  FLAGS_planner_enable_captain_net_onnx_trt = false;
  // Disable act net in unit test.
  FLAGS_planner_enable_act_net_speed = false;

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

  const TrajectoryProto previous_trajectory;
  const auto path_look_ahead_time =
      GetStPathPlanLookAheadTime(plan_start_point_info, sdc_pose,
                                 absl::Milliseconds(0), previous_trajectory);

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

  // Construct multi tasks input.
  const auto st_traj_mgr = std::make_shared<const SpacetimeTrajectoryManager>();
  const auto object_manager = std::make_shared<const PlannerObjectManager>();
  const absl::flat_hash_set<std::string> stalled_objects;
  const SceneOutputProto scene_reasoning;
  const ExternalCommandStatus ext_cmd_status;
  const std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  const TrafficLightInfoMap tl_info_map;

  const auto mock_current_lane_path = mapping::LanePath(
      smm, /*lane_ids=*/
      {mapping::ElementId(2448), mapping::ElementId(1), mapping::ElementId(34)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  const auto prev_target_lane_path = mock_current_lane_path;
  const RouteSections prev_route_sections = route_sections;
  const mapping::LanePoint station_anchor(mapping::ElementId(2448),
                                          /*fraction*/ 0.0);
  const LaneChangeStateProto lane_change_state;
  const mapping::LanePath prev_lane_path_before_lc = mock_current_lane_path;
  const mapping::LanePath preferred_lane_path = mock_current_lane_path;
  const SmoothedReferenceLineResultMap smooth_result_map;
  const auto parking_brake_release_time = plan_time - absl::Seconds(2.0);
  const DeciderStateProto decider_state;
  const InitializerStateProto initializer_state;
  const TrajectoryOptimizerStateProto selected_trajectory_optimizer_state_proto;
  const SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;
  const SelectorState selector_state;
  const YellowLightObservationsNew yellow_light_observations;

  const TrafficLightStatesProto traffic_light_states;
  const TrajectoryProto log_av_trajectory;
  const PredictionDebugProto prediction_debug;
  auto planner_model_pool =
      std::make_unique<ModelPool>(*param_manager, *param_finder);

  auto objects_proto = std::make_shared<const ObjectsProto>();
  auto planner_av_context = std::make_unique<prediction::AvContext>(
      ml::kAvHistoryBufferSize, ml::kAvHistoryBufferLen);

  const MultiTasksCruisePlannerInput input{
      .coordinate_converter = &cc,
      .planner_semantic_map_manager = psmm_ptr,
      .online_semantic_map = nullptr,
      .planner_params = &planner_params,
      .vehicle_params = &vehicle_params,
      .rm_output = &route_manager_output,
      .route_sections_from_start = &route_sections,
      .start_point_info = &plan_start_point_info,
      .ego_nearest_lane_id = mapping::ElementId(2448),
      .min_path_look_ahead_duration = path_look_ahead_time,
      .st_traj_mgr = st_traj_mgr,
      .object_manager = object_manager,
      .stalled_objects = &stalled_objects,
      .scene_reasoning = &scene_reasoning,
      .ext_cmd_status = &ext_cmd_status,
      .time_aligned_prev_traj = &time_aligned_prev_traj,
      .tl_info_map = &tl_info_map,
      .pose = nullptr,
      .chassis = nullptr,
      .previous_trajectory = &previous_trajectory,
      .prev_target_lane_path = &prev_target_lane_path,
      .prev_route_sections = &prev_route_sections,
      .prev_length_along_route = std::numeric_limits<double>::max(),
      .prev_max_reach_length = std::numeric_limits<double>::max(),
      .station_anchor = &station_anchor,
      .lane_change_state = &lane_change_state,
      .prev_lane_path_before_lc = &prev_lane_path_before_lc,
      .preferred_lane_path = &preferred_lane_path,
      .new_lc_command = DriverAction::LC_CMD_NONE,
      .alc_confirmation = std::nullopt,
      .smooth_result_map = &smooth_result_map,
      .prev_smooth_state = false,
      .parking_brake_release_time = parking_brake_release_time,
      .decider_state = &decider_state,
      .initializer_state = &initializer_state,
      .selected_trajectory_optimizer_state_proto =
          &selected_trajectory_optimizer_state_proto,
      .st_planner_object_trajectories = &st_planner_object_trajectories,
      .selector_state = &selector_state,
      .yellow_light_observations = &yellow_light_observations,
      .traffic_light_states = &traffic_light_states,
      .log_av_trajectory = &log_av_trajectory,
      .prediction_debug = &prediction_debug,
      .planner_model_pool = planner_model_pool.get(),
      .real_objects = objects_proto,
      .virtual_objects = nullptr,
      .planner_av_context = planner_av_context.get(),
      .context_feature = nullptr,
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
    EXPECT_NOT_OK(RunAsyncCruisePlanner(
        input, acc_task_proto, &acc_output, &async_planner_output,
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
    EXPECT_OK(RunAsyncCruisePlanner(input, acc_task_proto, &acc_output,
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
