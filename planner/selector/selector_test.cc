#include "onboard/planner/selector/selector.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/async/thread_pool.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/decision/traffic_light/traffic_light_info_collector.h"
#include "onboard/planner/est_planner.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"
#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/selector/proto/selector_params.pb.h"
#include "onboard/planner/selector/proto/selector_state.pb.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/lane_change_type.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/history_buffer.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {

namespace {

PlannerObjectManager BuildPhantomVehicle(const Vec2d& obj_pos,
                                         double heading = 0.0) {
  ObjectVector<PlannerObject> objects;
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id("Phantom")
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(obj_pos)
                            .set_length_width(4.5, 2.2)
                            .set_yaw(heading)
                            .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_stationary_traj(obj_pos, heading);

  objects.push_back(builder.Build());
  return PlannerObjectManager(objects);
}

PlannerObjectManager BuildMovingVehicle(const Vec2d& obj_pos,
                                        const Vec2d& speed,
                                        double heading = 0.0) {
  ObjectVector<PlannerObject> objects;
  const double perception_ts = 10.0;
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id("MovingPhantom")
                            .set_timestamp(perception_ts)
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(obj_pos)
                            .set_speed(speed)
                            .set_length_width(4.5, 2.2)
                            .set_yaw(heading)
                            .set_trajectory(20)
                            .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(/*start=*/obj_pos, /*theta=*/heading,
                         /*duration=*/10.0,
                         /*init_v=*/speed.norm(), /*acc=*/0.0);

  objects.push_back(builder.Build());
  return PlannerObjectManager(objects);
}

PlannerObjectManager BuildMovingLargeVehicle(const Vec2d& obj_pos,
                                             const Vec2d& speed,
                                             double heading = 0.0) {
  ObjectVector<PlannerObject> objects;
  const double perception_ts = 10.0;
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id("MovingPhantom")
                            .set_timestamp(perception_ts)
                            .set_type(ObjectType::OT_LARGE_VEHICLE)
                            .set_pos(obj_pos)
                            .set_speed(speed)
                            .set_length_width(6.0, 2.6)
                            .set_yaw(heading)
                            .set_trajectory(20)
                            .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(/*start=*/obj_pos, /*theta=*/heading,
                         /*duration=*/10.0,
                         /*init_v=*/speed.norm(), /*acc=*/0.0);

  objects.push_back(builder.Build());
  return PlannerObjectManager(objects);
}

SchedulerOutput BuildSchedulerOutput(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const LanePathInfo& lp_info, bool borrow,
    const ApolloTrajectoryPointProto& start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const mapping::LanePath& prev_lp, const std::string& channel_prefix,
    bool send_to_canvas) {
  // Build drive passages.
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            lp_info.lane_path(),
                                            kDrivePassageKeepBehindLength);
  EXPECT_OK(backward_extended_lane_path);
  ASSIGN_OR_DIE(auto drive_passage,
                BuildDrivePassage(
                    psmm, /*vision_map_ptr=*/nullptr, lp_info.lane_path(),
                    *backward_extended_lane_path,
                    /*anchor_point=*/mapping::LanePoint(),
                    route_sections.planning_horizon(psmm),
                    route_sections.destination(), /*all_lanes_virtual=*/false,
                    /*override_speed_limit_mps=*/std::nullopt));

  SmoothedReferenceLineResultMap smooth_result_map;
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(start_point);
  ASSIGN_OR_DIE(auto ego_frenet_box,
                drive_passage.QueryFrenetBoxAt(ComputeAvBox(
                    ego_pos, start_point.path_point().theta(), vehicle_geom)));

  const double ref_center_l =
      CalcAvhRefCenterL(psmm, drive_passage, ego_frenet_box, smooth_result_map,
                        /*should_smooth=*/false);
  const auto prev_lc_state = *MakeLaneChangeState(
      drive_passage, ego_pos, ego_frenet_box, prev_lp,
      /*prev_lane_path_before_lc_from_start=*/mapping::LanePath(),
      MakeNoneLaneChangeState(), ref_center_l, AutonomyStateProto::AUTO_DRIVE);
  const auto prev_lp_before_lc =
      prev_lc_state.stage() == LaneChangeStage::LCS_NONE ? mapping::LanePath()
                                                         : prev_lp;
  auto lc_state = *MakeLaneChangeState(
      drive_passage, ego_pos, ego_frenet_box, prev_lp, prev_lp_before_lc,
      prev_lc_state, ref_center_l, AutonomyStateProto::AUTO_DRIVE);
  // Build path boundary.
  ASSIGN_OR_DIE(auto path_boundary,
                BuildPathBoundaryFromPose(
                    psmm, drive_passage, start_point, vehicle_geom, st_traj_mgr,
                    lc_state, smooth_result_map, borrow,
                    /*should_smooth=*/false, /*unsafe_object_ids=*/{}));
  if (send_to_canvas) {
    SendDrivePassageToCanvas(drive_passage, channel_prefix + "/drive_passage");
    DrawPathSlBoundaryToCanvas(path_boundary,
                               channel_prefix + "/path_boundary");
  }

  // Scheduler output.
  return SchedulerOutput{.drive_passage = std::move(drive_passage),
                         .sl_boundary = std::move(path_boundary),
                         .lane_change_state = std::move(lc_state),
                         .length_along_route = lp_info.length_along_route(),
                         .max_reach_length = lp_info.max_reach_length(),
                         .borrow_lane = borrow,
                         .av_frenet_box_on_drive_passage = ego_frenet_box};
}

struct SchedulerAndEstOutput {
  RouteSectionsInfo sections_info;
  MotionConstraintParamsProto motion_constraints;
  VehicleGeometryParamsProto vehicle_geom;
  SelectorParamsProto selector_params;
  ApolloTrajectoryPointProto plan_start_point;
  RouteNaviInfo route_navi_info;
  absl::Time plan_time;
  std::vector<PlannerStatus> est_status;
  std::vector<SpacetimeTrajectoryManager> st_traj_mgr_list;
  std::vector<EstPlannerOutput> results;
};

absl::StatusOr<SchedulerAndEstOutput> RunSchedulerAndEstPlanner(
    const Vec2d& ego_pos, const Vec2d& ego_speed, const double ego_heading,
    bool send_to_canvas, const std::vector<std::string>& channel_prefix,
    const std::string& canvas_prefix, const PlannerObjectManager& object_mgr,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const std::string& route_name, const mapping::LanePoint& destination,
    const mapping::LanePath& prev_lp, const PlannerSemanticMapManager& psmm,
    double preview_route_navi_dist, int begin_lane_idx, int branch_size) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();

  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Load map and create optimized route path.
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(ToUnixDoubleSeconds(plan_time), ego_pos,
                                        ego_heading, ego_speed);
  auto start_point = ConvertToTrajPointProto(sdc_pose);

  const auto route_sections =
      route_name.empty()
          ? RouteSectionsFromCompositeLanePath(
                *smm, *RoutingToLanePoint(*smm, cc, sdc_pose, destination))
          : RouteSectionsFromCompositeLanePath(
                *smm, RoutingToNameSpot(*smm, cc, sdc_pose, route_name));

  RouteSectionsInfo sections_info(psmm, &route_sections);
  auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*smm, sections_info,
                               /*avoid_lanes=*/{}, preview_route_navi_dist);

  const auto lane_graph =
      BuildLaneGraph(psmm, sections_info, object_mgr, stalled_objects,
                     /*avoid_lanes=*/{}, route_navi_info);
  const auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  if (send_to_canvas) {
    DrawPlannerObjectManagerToCanvas(object_mgr, canvas_prefix + "/object",
                                     vis::Color::kLightGreen);
    for (const auto& lp_info : lp_infos) {
      SendLanePathInfoToCanvas(
          lp_info, psmm,
          canvas_prefix + "/lane_paths/" +
              std::to_string(lp_info.start_lane_id().value()));
    }
  }

  const TrajectoryProto previous_trajectory;
  absl::Time parking_brake_release_time;
  const DeciderStateProto decider_state;
  const InitializerStateProto initializer_state;
  const SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;
  SmoothedReferenceLineResultMap smooth_result_map;
  TrafficLightInfoMap tl_info_map;
  SceneOutputProto scene_reasoning;
  std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  ThreadPool* thread_pool = ThreadPool::DefaultPool();

  std::vector<SpacetimeTrajectoryManager> st_traj_mgr_list(branch_size);
  std::vector<SchedulerOutput> multi_tasks(branch_size);
  std::vector<PlannerStatus> status_list(branch_size);
  std::vector<EstPlannerOutput> results(branch_size);
  std::vector<EstPlannerDebug> est_debugs(branch_size);
  std::vector<vis::vantage::ChartDataBundleProto> chart_data_bundles(
      branch_size);

  const PlanStartPointInfo start_point_info{
      .reset = false,
      .start_point = start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };

  const ml::captain_net::CaptainNetOutput captain_net_output;
  for (int i = 0; i < branch_size; ++i) {
    // Scheduler output.
    if (i + begin_lane_idx >= lp_infos.size()) {
      break;
    }
    multi_tasks[i] = BuildSchedulerOutput(
        psmm, route_sections, lp_infos[i + begin_lane_idx], /*borrow=*/false,
        start_point, vehicle_geom, st_traj_mgr, prev_lp,
        canvas_prefix + "/" + channel_prefix[i], send_to_canvas);

    // Create spacetime trajectory manager for each est planner.
    const auto path_look_ahead_time = GetStPathPlanLookAheadTime(
        start_point_info, sdc_pose, absl::Milliseconds(0), previous_trajectory);
    const auto path_start_point_info = GetStPathPlanStartPointInfo(
        path_look_ahead_time, start_point_info, previous_trajectory,
        /*trajectory_optimizer_time_step=*/std::nullopt,
        /*last_st_path_plan_start_time=*/std::nullopt);

    SpacetimeTrajectoryManagerBuilderInput st_mgr_builder_input{
        .passage = &multi_tasks[i].drive_passage,
        .sl_boundary = &multi_tasks[i].sl_boundary,
        .obj_mgr = &object_mgr,
        .on_vision_map = false};

    st_traj_mgr_list[i] =
        BuildSpacetimeTrajectoryManager(st_mgr_builder_input, thread_pool);

    ASSIGN_OR_DIE(const auto driving_map_topo,
                  BuildDrivingMapByRouteOnOfflineMap(psmm, route_sections));
    status_list[i] = RunEstPlanner(
        EstPlannerInput{
            .driving_map_topo = &driving_map_topo,
            .semantic_map_manager = psmm.semantic_map_manager(),
            .planner_semantic_map_manager = &psmm,
            .plan_id = i,
            .vehicle_params = &vehicle_params,
            .parking_brake_release_time = parking_brake_release_time,
            .decider_state = &decider_state,
            .initializer_state = &initializer_state,
            .trajectory_optimizer_state_proto = nullptr,
            .st_planner_object_trajectories = &st_planner_object_trajectories,
            .obj_mgr = &object_mgr,
            .start_point_info = &start_point_info,
            .st_path_start_point_info = &path_start_point_info,
            .tl_info_map = &tl_info_map,
            .smooth_result_map = &smooth_result_map,
            .stalled_objects = &stalled_objects,
            .scene_reasoning = &scene_reasoning,
            .prev_target_lane_path_from_start = &prev_lp,
            .time_aligned_prev_traj = &time_aligned_prev_traj,
            .enable_force_stop = false,
            .st_traj_mgr = &st_traj_mgr_list[i],
            .captain_net_output = &captain_net_output,
            .decision_constraint_config =
                &planner_params.decision_constraint_config(),
            .initializer_params = &planner_params.initializer_params(),
            .trajectory_optimizer_params =
                &planner_params.trajectory_optimizer_params(),
            .speed_finder_params = &planner_params.speed_finder_params(),
            .motion_constraint_params =
                &planner_params.motion_constraint_params(),
            .planner_functions_params =
                &planner_params.planner_functions_params(),
            .vehicle_models_params = &planner_params.vehicle_models_params(),
            .speed_finder_lc_radical_params =
                &planner_params.speed_finder_lc_radical_params(),
            .speed_finder_lc_conservative_params =
                &planner_params.speed_finder_lc_conservative_params(),
            .trajectory_optimizer_lc_radical_params =
                &planner_params.trajectory_optimizer_lc_radical_params(),
            .trajectory_optimizer_lc_normal_params =
                &planner_params.trajectory_optimizer_lc_normal_params(),
            .trajectory_optimizer_lc_conservative_params =
                &planner_params.trajectory_optimizer_lc_conservative_params(),
            .spacetime_planner_object_trajectories_params =
                &planner_params.spacetime_planner_object_trajectories_params()},
        std::move(multi_tasks[i]), &results[i], &est_debugs[i],
        &chart_data_bundles[i], thread_pool);
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(
          results[i].traj_points,
          canvas_prefix + "/" + channel_prefix[i] + "/trajectory",
          vis::Color::kLightOrange);
    }
  }

  return SchedulerAndEstOutput{
      .sections_info = std::move(sections_info),
      .motion_constraints = planner_params.motion_constraint_params(),
      .vehicle_geom = run_params.vehicle_params().vehicle_geometry_params(),
      .selector_params = planner_params.selector_params(),
      .plan_start_point = std::move(start_point),
      .route_navi_info = std::move(route_navi_info),
      .plan_time = plan_time,
      .est_status = std::move(status_list),
      .st_traj_mgr_list = std::move(st_traj_mgr_list),
      .results = std::move(results)};
}

TEST(Selector, OvertakeTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const bool send_to_canvas = true;
  const std::string canvas_prefix = "overtake";
  const std::vector<std::string> channel_prefix = {"nudge", "follow"};
  const Vec2d ego_pos = Vec2d(116.5, 0.0);
  const Vec2d ego_speed = Vec2d(11.0, 0.0);
  const double ego_heading = 0.0;
  const std::string route_name = "b7_e2_end";
  const mapping::LanePoint destination;
  const mapping::LanePath prev_lp(
      psmm.semantic_map_manager(),
      {mapping::ElementId(2471), mapping::ElementId(53)}, 0.0, 1.0);
  const double preview_route_navi_dist = 1000.0;
  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};
  const auto object_mgr = BuildPhantomVehicle(Vec2d(160.0, 0.0));
  const int branch_size = 2;
  const int begin_lane_idx = 0;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];

  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            2471);
}

TEST(Selector, RightTurnOvertakeTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const bool send_to_canvas = true;
  const std::string canvas_prefix = "right_turn_overtake";
  const std::vector<std::string> channel_prefix = {"nudge", "follow"};
  const Vec2d ego_pos = Vec2d(219.0, 63.0);
  const Vec2d ego_speed = Vec2d(6.0, 0.0);
  const double ego_heading = 0.0;
  const mapping::LanePoint destination;
  const std::string route_name = "8c_s3";
  const mapping::LanePath prev_lp(
      psmm.semantic_map_manager(),
      {mapping::ElementId(498), mapping::ElementId(2483),
       mapping::ElementId(500)},
      0.86, 1.0);
  const double preview_route_navi_dist = 1000.0;
  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};
  const auto object_mgr = BuildPhantomVehicle(Vec2d(240.0, 62.6));
  const int branch_size = 2;
  const int begin_lane_idx = 1;
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];

  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            2491);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_GT(last_traj_pt.x(), 229.0);
}

TEST(Selector, LaneChangeTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const bool send_to_canvas = true;
  const std::string canvas_prefix = "lane_change";
  const std::vector<std::string> channel_prefix = {"nudge", "follow"};
  const Vec2d ego_pos = Vec2d(116.5, kDefaultLaneWidth);
  const Vec2d ego_speed = Vec2d(11.0, 0.0);
  const double ego_heading = 0.0;
  const std::string route_name = "b7_e2_end";
  const mapping::LanePoint destination;
  const mapping::LanePath prev_lp(
      psmm.semantic_map_manager(),
      {mapping::ElementId(2470), mapping::ElementId(51)}, 0.0, 1.0);
  const double preview_route_navi_dist = 1000.0;
  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};
  const auto object_mgr = BuildPhantomVehicle(Vec2d(160.0, 0.0));
  const int branch_size = 2;
  const int begin_lane_idx = 0;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];

  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            2470);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_GT(last_traj_pt.x(), 160.0);
}

TEST(Selector, EarlyLaneChangeTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const bool send_to_canvas = true;
  const std::string canvas_prefix = "early_lane_change";
  const std::vector<std::string> channel_prefix = {"lane_keep", "lane_change"};
  const Vec2d ego_pos = Vec2d(180.0, -217.0);
  const Vec2d ego_speed = Vec2d(18.0, 0.0);
  const double ego_heading = 0.0;
  const std::string route_name = "";
  const auto destination = mapping::LanePoint(mapping::ElementId(154), 1.0);
  const mapping::LanePath prev_lp(
      psmm.semantic_map_manager(),
      {mapping::ElementId(2341), mapping::ElementId(2348)}, 0.2, 1.0);
  const double preview_route_navi_dist = 1000.0;
  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};
  const auto object_mgr = BuildPhantomVehicle(Vec2d(160.0, 0.0));
  const int branch_size = 2;
  const int begin_lane_idx = 0;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];

  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            2340);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_LT(last_traj_pt.y(), -218.9);
}

TEST(Selector, TrafficLightTest) {
  // Don't unify because it changes traffic light status.
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();

  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Load map and create optimized route path.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(
      ToUnixDoubleSeconds(plan_time), Vec2d(210.0, 66.5), 0.0, Vec2d(6.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  const auto route_path = *RoutingToLanePoint(
      *smm, cc, sdc_pose, mapping::LanePoint(mapping::ElementId(154), 1.0));
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const RouteSectionsInfo sections_info(psmm, &route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  // Build one object.
  const auto object_mgr = BuildPhantomVehicle(Vec2d(240.0, 66.5));
  DrawPlannerObjectManagerToCanvas(object_mgr, "traffic_light/object",
                                   vis::Color::kLightGreen);
  // ObjectTracer analysis result.
  const absl::flat_hash_set<std::string> stalled_objects;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  const auto lane_graph =
      BuildLaneGraph(psmm, sections_info, object_mgr, stalled_objects,
                     /*avoid_lanes=*/{}, route_navi_info);
  const auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);
  for (const auto& lp_info : lp_infos) {
    SendLanePathInfoToCanvas(
        lp_info, psmm,
        "traffic_light/lane_paths/" +
            std::to_string(lp_info.start_lane_id().value()));
  }

  const TrajectoryProto previous_trajectory;
  absl::Time parking_brake_release_time;
  const DeciderStateProto decider_state;
  const InitializerStateProto initializer_state;
  const SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;
  SmoothedReferenceLineResultMap smooth_result_map;
  // Create traffic light history manager.
  TrafficLightStatesProto tl_states_proto;
  tl_states_proto.mutable_header()->set_timestamp(
      absl::ToUnixMicros(absl::Now()));
  for (int i = 0; i < 3; ++i) {
    auto* new_state = tl_states_proto.add_states();
    new_state->set_traffic_light_id(917);
    new_state->set_color(TL_RED);
    new_state->set_flashing(false);
  }

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto tl_collector_output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &tl_states_proto,
                        .route_sections = &route_sections},
                    yellow_light_observations));
  SceneOutputProto scene_reasoning;
  std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  ThreadPool* thread_pool = ThreadPool::DefaultPool();

  std::vector<SpacetimeTrajectoryManager> st_traj_mgr_list(2);
  std::vector<SchedulerOutput> multi_tasks(2);
  std::vector<PlannerStatus> status_list(2);
  std::vector<EstPlannerOutput> results(2);
  std::vector<EstPlannerDebug> est_debugs(2);
  std::vector<vis::vantage::ChartDataBundleProto> chart_data_bundles(2);
  const std::vector<std::string> channel_prefix{"lane_keep", "lane_change"};
  const PlanStartPointInfo start_point_info{
      .reset = false,
      .start_point = start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };
  const mapping::LanePath prev_lp(
      psmm.semantic_map_manager(),
      {mapping::ElementId(2491), mapping::ElementId(2484),
       mapping::ElementId(62)},
      0.86, 1.0);
  const ml::captain_net::CaptainNetOutput captain_net_output;
  for (int i : {0, 1}) {
    multi_tasks[i] = BuildSchedulerOutput(
        psmm, route_sections, lp_infos[i],
        /*borrow=*/false, start_point, vehicle_geom, st_traj_mgr, prev_lp,
        "traffic_light/" + channel_prefix[i], /*send_to_canvas=*/true);

    // Create spacetime trajectory manager for each est planner.
    const auto path_look_ahead_time = GetStPathPlanLookAheadTime(
        start_point_info, sdc_pose, absl::Milliseconds(0), previous_trajectory);
    const auto path_start_point_info = GetStPathPlanStartPointInfo(
        path_look_ahead_time, start_point_info, previous_trajectory,
        /*trajectory_optimizer_time_step=*/std::nullopt,
        /*last_st_path_plan_start_time=*/std::nullopt);

    SpacetimeTrajectoryManagerBuilderInput st_mgr_builder_input{
        .passage = &multi_tasks[i].drive_passage,
        .sl_boundary = &multi_tasks[i].sl_boundary,
        .obj_mgr = &object_mgr,
        .on_vision_map = false};

    st_traj_mgr_list[i] =
        BuildSpacetimeTrajectoryManager(st_mgr_builder_input, thread_pool);

    ASSIGN_OR_DIE(const auto driving_map_topo,
                  BuildDrivingMapByRouteOnOfflineMap(psmm, route_sections));

    status_list[i] = RunEstPlanner(
        EstPlannerInput{
            .driving_map_topo = &driving_map_topo,
            .semantic_map_manager = psmm.semantic_map_manager(),
            .planner_semantic_map_manager = &psmm,
            .plan_id = i,
            .vehicle_params = &vehicle_params,
            .parking_brake_release_time = parking_brake_release_time,
            .decider_state = &decider_state,
            .initializer_state = &initializer_state,
            .trajectory_optimizer_state_proto = nullptr,
            .st_planner_object_trajectories = &st_planner_object_trajectories,
            .obj_mgr = &object_mgr,
            .start_point_info = &start_point_info,
            .st_path_start_point_info = &path_start_point_info,
            .tl_info_map = &tl_collector_output.tl_info_map,
            .smooth_result_map = &smooth_result_map,
            .stalled_objects = &stalled_objects,
            .scene_reasoning = &scene_reasoning,
            .prev_target_lane_path_from_start = &prev_lp,
            .time_aligned_prev_traj = &time_aligned_prev_traj,
            .enable_force_stop = false,
            .st_traj_mgr = &st_traj_mgr_list[i],
            .captain_net_output = &captain_net_output,
            .decision_constraint_config =
                &planner_params.decision_constraint_config(),
            .initializer_params = &planner_params.initializer_params(),
            .trajectory_optimizer_params =
                &planner_params.trajectory_optimizer_params(),
            .speed_finder_params = &planner_params.speed_finder_params(),
            .motion_constraint_params =
                &planner_params.motion_constraint_params(),
            .planner_functions_params =
                &planner_params.planner_functions_params(),
            .vehicle_models_params = &planner_params.vehicle_models_params(),
            .speed_finder_lc_radical_params =
                &planner_params.speed_finder_lc_radical_params(),
            .speed_finder_lc_conservative_params =
                &planner_params.speed_finder_lc_conservative_params(),
            .trajectory_optimizer_lc_radical_params =
                &planner_params.trajectory_optimizer_lc_radical_params(),
            .trajectory_optimizer_lc_normal_params =
                &planner_params.trajectory_optimizer_lc_normal_params(),
            .trajectory_optimizer_lc_conservative_params =
                &planner_params.trajectory_optimizer_lc_conservative_params(),
            .spacetime_planner_object_trajectories_params =
                &planner_params.spacetime_planner_object_trajectories_params()},
        std::move(multi_tasks[i]), &results[i], &est_debugs[i],
        &chart_data_bundles[i], thread_pool);
    ASSERT_OK(status_list[i]);
    SendApolloTrajectoryPointsToCanvas(
        results[i].traj_points,
        "traffic_light/" + channel_prefix[i] + "/trajectory",
        vis::Color::kLightOrange);
  }

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &planner_params.motion_constraint_params(),
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &planner_params.selector_params()};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(const auto selector_output,
                SelectTrajectory(selector_input, status_list, st_traj_mgr_list,
                                 results, &selector_debug, &selector_state));
  const auto& selected_output = results[selector_output.selected_idx];
  SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                     "traffic_light/selected",
                                     vis::Color::kLightGreen);
  EXPECT_EQ(results[selector_output.selected_idx]
                .scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            2491);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_LT(last_traj_pt.x(), 245.0);
}

TEST(Selector, SingleLaneOvertakeTest) {
  // Don't unify because lane borrow.
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();

  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Load map and create optimized route path.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(-133.0, -160.0), M_PI_2,
                 Vec2d(5.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "4d_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const RouteSectionsInfo sections_info(psmm, &route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  // Build one object.
  const auto object_mgr =
      BuildPhantomVehicle(Vec2d(-132.4, -140.0), /*heading=*/M_PI_2);
  DrawPlannerObjectManagerToCanvas(object_mgr, "single_lane_overtake/object",
                                   vis::Color::kLightGreen);
  // ObjectTracer analysis result.
  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  const auto lane_graph =
      BuildLaneGraph(psmm, sections_info, object_mgr, stalled_objects,
                     /*avoid_lanes=*/{}, route_navi_info);
  const auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);
  for (const auto& lp_info : lp_infos) {
    SendLanePathInfoToCanvas(
        lp_info, psmm,
        "single_lane_overtake/lane_paths/" +
            std::to_string(lp_info.start_lane_id().value()));
  }

  const TrajectoryProto previous_trajectory;
  absl::Time parking_brake_release_time;
  const DeciderStateProto decider_state;
  const InitializerStateProto initializer_state;
  const SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;
  SmoothedReferenceLineResultMap smooth_result_map;
  TrafficLightInfoMap tl_info_map;
  SceneOutputProto scene_reasoning;
  std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  ThreadPool* thread_pool = ThreadPool::DefaultPool();

  std::vector<SpacetimeTrajectoryManager> st_traj_mgr_list(2);
  std::vector<SchedulerOutput> multi_tasks(2);
  std::vector<PlannerStatus> status_list(2);
  std::vector<EstPlannerOutput> results(2);
  std::vector<EstPlannerDebug> est_debugs(2);
  std::vector<vis::vantage::ChartDataBundleProto> chart_data_bundles(2);
  const std::vector<std::string> channel_prefix{"lane_keep", "lane_borrow"};
  const PlanStartPointInfo start_point_info{
      .reset = false,
      .start_point = start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };
  const mapping::LanePath prev_lp(
      psmm.semantic_map_manager(),
      {mapping::ElementId(1224), mapping::ElementId(683)}, 0.2, 0.5);
  const ml::captain_net::CaptainNetOutput captain_net_output;
  for (int i : {0, 1}) {
    multi_tasks[i] = BuildSchedulerOutput(
        psmm, route_sections, lp_infos[0], i, start_point, vehicle_geom,
        st_traj_mgr, prev_lp, "single_lane_overtake/" + channel_prefix[i],
        /*send_to_canvas=*/true);

    // Create spacetime trajectory manager for each est planner.
    const auto path_look_ahead_time = GetStPathPlanLookAheadTime(
        start_point_info, sdc_pose, absl::Milliseconds(0), previous_trajectory);
    const auto path_start_point_info = GetStPathPlanStartPointInfo(
        path_look_ahead_time, start_point_info, previous_trajectory,
        /*trajectory_optimizer_time_step=*/std::nullopt,
        /*last_st_path_plan_start_time=*/std::nullopt);

    SpacetimeTrajectoryManagerBuilderInput st_mgr_builder_input{
        .passage = &multi_tasks[i].drive_passage,
        .sl_boundary = &multi_tasks[i].sl_boundary,
        .obj_mgr = &object_mgr,
        .on_vision_map = false};

    st_traj_mgr_list[i] =
        BuildSpacetimeTrajectoryManager(st_mgr_builder_input, thread_pool);

    ASSIGN_OR_DIE(const auto driving_map_topo,
                  BuildDrivingMapByRouteOnOfflineMap(psmm, route_sections));

    status_list[i] = RunEstPlanner(
        EstPlannerInput{
            .driving_map_topo = &driving_map_topo,
            .semantic_map_manager = psmm.semantic_map_manager(),
            .planner_semantic_map_manager = &psmm,
            .plan_id = i,
            .vehicle_params = &vehicle_params,
            .parking_brake_release_time = parking_brake_release_time,
            .decider_state = &decider_state,
            .initializer_state = &initializer_state,
            .trajectory_optimizer_state_proto = nullptr,
            .st_planner_object_trajectories = &st_planner_object_trajectories,
            .obj_mgr = &object_mgr,
            .start_point_info = &start_point_info,
            .st_path_start_point_info = &path_start_point_info,
            .tl_info_map = &tl_info_map,
            .smooth_result_map = &smooth_result_map,
            .stalled_objects = &stalled_objects,
            .scene_reasoning = &scene_reasoning,
            .prev_target_lane_path_from_start = &prev_lp,
            .time_aligned_prev_traj = &time_aligned_prev_traj,
            .enable_force_stop = false,
            .st_traj_mgr = &st_traj_mgr_list[i],
            .captain_net_output = &captain_net_output,
            .decision_constraint_config =
                &planner_params.decision_constraint_config(),
            .initializer_params = &planner_params.initializer_params(),
            .trajectory_optimizer_params =
                &planner_params.trajectory_optimizer_params(),
            .speed_finder_params = &planner_params.speed_finder_params(),
            .motion_constraint_params =
                &planner_params.motion_constraint_params(),
            .planner_functions_params =
                &planner_params.planner_functions_params(),
            .vehicle_models_params = &planner_params.vehicle_models_params(),
            .speed_finder_lc_radical_params =
                &planner_params.speed_finder_lc_radical_params(),
            .speed_finder_lc_conservative_params =
                &planner_params.speed_finder_lc_conservative_params(),
            .trajectory_optimizer_lc_radical_params =
                &planner_params.trajectory_optimizer_lc_radical_params(),
            .trajectory_optimizer_lc_normal_params =
                &planner_params.trajectory_optimizer_lc_normal_params(),
            .trajectory_optimizer_lc_conservative_params =
                &planner_params.trajectory_optimizer_lc_conservative_params(),
            .spacetime_planner_object_trajectories_params =
                &planner_params.spacetime_planner_object_trajectories_params()},
        std::move(multi_tasks[i]), &results[i], &est_debugs[i],
        &chart_data_bundles[i], thread_pool);
    ASSERT_OK(status_list[i]);
    SendApolloTrajectoryPointsToCanvas(
        results[i].traj_points,
        "single_lane_overtake/" + channel_prefix[i] + "/trajectory",
        vis::Color::kLightOrange);
  }

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &planner_params.motion_constraint_params(),
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &planner_params.selector_params()};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(const auto selector_output,
                SelectTrajectory(selector_input, status_list, st_traj_mgr_list,
                                 results, &selector_debug, &selector_state));
  const auto& selected_output = results[selector_output.selected_idx];
  SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                     "single_lane_overtake/selected",
                                     vis::Color::kLightGreen);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_GT(last_traj_pt.y(), -140.0);
  EXPECT_TRUE(
      results[selector_output.selected_idx].scheduler_output.borrow_lane);
}

TEST(Selector, HighwayRouteCost) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const bool send_to_canvas = true;
  const std::string canvas_prefix = "highway_route_cost";
  const std::vector<std::string> channel_prefix = {"lane_keep", "lane_change"};
  const Vec2d ego_pos = Vec2d(239.1, -485);
  const Vec2d ego_speed = Vec2d(15.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(13596), 1.0);
  const mapping::LanePath prev_lp(
      psmm.semantic_map_manager(),
      {mapping::ElementId(13551), mapping::ElementId(13564)}, 0.1, 0.9);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr = PlannerObjectManager();
  const int branch_size = 2;
  const int begin_lane_idx = 1;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];

  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            13555);
}

TEST(Selector, HighwayProgressCost) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "highway_progress_cost";
  const std::vector<std::string> channel_prefix{"lane_change", "lane_keep"};
  const Vec2d ego_pos = Vec2d(235.1, -600);
  const Vec2d ego_speed = Vec2d(15.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(13596), 1.0);
  const mapping::LanePath prev_lp(
      psmm.semantic_map_manager(),
      {mapping::ElementId(13555), mapping::ElementId(13562)}, 0.1, 0.9);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(235.1, -632), Vec2d(11.0, 0.0), -M_PI_2);
  const int branch_size = 2;
  const int begin_lane_idx = 1;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];

  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            13551);
}

TEST(Selector, HighwayRoutePriority) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "highway_route_priority";
  const std::vector<std::string> channel_prefix{"lane_keep", "lane_change"};
  const Vec2d ego_pos = Vec2d(239.1, -1270);
  const Vec2d ego_speed = Vec2d(20.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(13596), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(13591)}, 0.0, 0.9);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(235.1, -1336), Vec2d(15.0, 0.0), -M_PI_2);
  const int branch_size = 2;
  const int begin_lane_idx = 1;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];

  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            13596);
}

TEST(Selector, PrefilterLeft) {
  // Choose left and middle lane because of far from destination.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "prefilter_left";
  const std::vector<std::string> channel_prefix{"left_change", "lane_keep",
                                                "right_change"};
  const Vec2d ego_pos = Vec2d(-586.4, -456.5);
  const Vec2d ego_speed = Vec2d(12.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(15478), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(15008)}, 0.0, 0.9);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(-586.4, -486), Vec2d(5.0, 0.0), -M_PI_2);
  const int branch_size = 3;
  const int begin_lane_idx = 0;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  selector_flags.planner_enable_prefilter_for_selector = true;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];
  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            15002);
}

TEST(Selector, PrefilterRoute) {
  // Choose Right and Middle lane because of near to destination.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "prefilter_route";
  const std::vector<std::string> channel_prefix{"left_change", "lane_keep",
                                                "right_change"};
  const Vec2d ego_pos = Vec2d(-586.4, -456.5);
  const Vec2d ego_speed = Vec2d(12.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(15460), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(15008)}, 0.0, 0.9);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(-586.4, -486), Vec2d(5.0, 0.0), -M_PI_2);
  const int branch_size = 3;
  const int begin_lane_idx = 0;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  selector_flags.planner_enable_prefilter_for_selector = true;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];
  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            15456);
}

TEST(Selector, PrefilterHighwayLeft) {
  // Choose left and middle lane because of far from destination.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "prefilter_highway_left";
  const std::vector<std::string> channel_prefix{"left_change", "lane_keep",
                                                "right_change"};
  const Vec2d ego_pos = Vec2d(-1746.2, -9032.9);
  const Vec2d ego_speed = Vec2d(16.0, 0.0);
  const double ego_heading = 0;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(34054), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(34030)}, 0.0, 0.9);
  const double preview_route_navi_dist = 3000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(-1693.4, -9033), Vec2d(5.0, 0.0), 0);
  const int branch_size = 3;
  const int begin_lane_idx = 0;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  selector_flags.planner_enable_prefilter_for_selector = true;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];
  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            34024);
}

TEST(Selector, PrefilterHighwayRoute) {
  // Choose right and middle lane because of near to destination.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "prefilter_highway_route";
  const std::vector<std::string> channel_prefix{"left_change", "lane_keep",
                                                "right_change"};
  const Vec2d ego_pos = Vec2d(-1746.2, -9032.9);
  const Vec2d ego_speed = Vec2d(16.0, 0.0);
  const double ego_heading = 0;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(34036), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(34030)}, 0.0, 0.9);
  const double preview_route_navi_dist = 3000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(-1693.4, -9033), Vec2d(5.0, 0.0), 0);
  const int branch_size = 3;
  const int begin_lane_idx = 0;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  selector_flags.planner_enable_prefilter_for_selector = true;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];
  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            34036);
}

TEST(Selector, PrefilterSolidBoundary) {
  // Choose right and middle lane because left change has solid.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "prefilter_solid_boudary";
  const std::vector<std::string> channel_prefix{"left_change", "lane_keep",
                                                "right_change"};
  const Vec2d ego_pos = Vec2d(147.57, -472.2);
  const Vec2d ego_speed = Vec2d(12.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(14213), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(14169)}, 0.0, 0.9);
  const double preview_route_navi_dist = 3000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(147.57, -520.8), Vec2d(5.0, 0.0), -M_PI_2);
  const int branch_size = 3;
  const int begin_lane_idx = 0;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));
  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  selector_flags.planner_enable_prefilter_for_selector = true;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  scheduler_and_est_output.results[0].scheduler_output.is_solid_lane_change =
      true;
  ASSIGN_OR_DIE(
      const auto selector_output,
      SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                       scheduler_and_est_output.st_traj_mgr_list,
                       scheduler_and_est_output.results, &selector_debug,
                       &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];

  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/selected",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            14173);
}

// NOLINTNEXTLINE
TEST(Selector, PrefilterHighwayBlockRight) {
  // Choose right and middle lane because of left is blocking.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "prefilter_highway_block_right";
  const std::vector<std::string> channel_prefix{"left_change", "lane_keep",
                                                "right_change"};
  const Vec2d ego_pos = Vec2d(-1736.2, -9032.9);
  const Vec2d ego_speed = Vec2d(16.0, 0.0);
  const double ego_heading = 0;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(34054), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(34030)}, 0.0, 0.9);
  const double preview_route_navi_dist = 3000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto move_object_mgr =
      BuildMovingVehicle(Vec2d(-1693.4, -9033), Vec2d(5.0, 0.0), 0.0);
  const auto static_object_mgr =
      BuildPhantomVehicle(Vec2d(-1710.6, -9029.2), 0.0);
  ObjectVector<PlannerObject> objects;
  objects.push_back(move_object_mgr.planner_objects().front());
  objects.push_back(static_object_mgr.planner_objects().front());
  const auto object_mgr = PlannerObjectManager(objects);
  const int branch_size = 3;
  const int begin_lane_idx = 0;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  {
    SelectorState empty_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_enable_prefilter_for_selector = true;

    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &empty_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};

    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];

    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/selected",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              34030);
  }

  {
    SelectorState empty_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_enable_prefilter_for_selector = true;
    const auto plan_time = scheduler_and_est_output.plan_time;
    for (double time_offset = -20.0; time_offset < 0.0;
         time_offset = time_offset + 1.0) {
      auto curr_time = plan_time + absl::Seconds(time_offset);
      PrefilterHistoryInfo history_info;
      history_info.left_is_blocked_by_stalled = false;
      history_info.left_safty_check_failed = true;
      history_info.right_is_blocked_by_stalled = false;
      history_info.right_safty_check_failed = false;
      empty_selector_state.prefilter_history_infos.push_back(curr_time,
                                                             history_info);
    }

    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &empty_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};

    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/selected",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              34036);
  }
}

// NOLINTNEXTLINE
TEST(Selector, NoaLaneChangeConfirmationMode) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "noa_lane_change_confirmation_mode";
  const std::vector<std::string> channel_prefix{"lane_keep", "lane_change"};
  const Vec2d ego_pos = Vec2d(239.1, -485);
  const Vec2d ego_speed = Vec2d(15.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(13596), 1.0);
  const mapping::LanePath prev_lp(
      psmm.semantic_map_manager(),
      {mapping::ElementId(13551), mapping::ElementId(13564)}, 0.1, 0.9);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr = PlannerObjectManager();
  const int branch_size = 2;
  const int begin_lane_idx = 1;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));
  {
    // Test: noa mode, successive best lane change frame is 3,
    // no need to lane change confirmation, route lane change.
    // Result: lane change to 13555.
    // No lane change request.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 3;
    selector_flags.planner_begin_radical_lane_change_frame = 3;
    selector_flags.planner_begin_signal_frame = 3;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13551);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13555);
    best_target_lane_state_proto.set_successive_count(2);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test1",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13555);
    EXPECT_EQ(selector_state.lane_change_type,
              LaneChangeType::DEFAULT_ROUTE_CHANGE);
    EXPECT_EQ(selector_state.selector_lane_change_request.lane_change_type(),
              LaneChangeType::NO_CHANGE);
  }

  {
    // Test: noa mode, successive best lane change frame is 2,
    // no need to lane change confirmation, route lane change.
    // Result: lane keep to 13551
    // No lane change request.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 3;
    selector_flags.planner_begin_radical_lane_change_frame = 3;
    selector_flags.planner_begin_signal_frame = 3;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13551);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13555);
    best_target_lane_state_proto.set_successive_count(1);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test2",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13551);
    EXPECT_EQ(selector_state.lane_change_type, LaneChangeType::NO_CHANGE);
    EXPECT_EQ(selector_state.selector_lane_change_request.lane_change_type(),
              LaneChangeType::NO_CHANGE);
  }

  {
    // Test: noa mode, successive best lane change frame is 3,
    // need to lane change confirmation, route lane change.
    // Result: lane keep to 13551
    // right route lane change request.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = true;
    selector_flags.planner_begin_lane_change_frame = 3;
    selector_flags.planner_begin_radical_lane_change_frame = 3;
    selector_flags.planner_begin_signal_frame = 3;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13551);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13555);
    best_target_lane_state_proto.set_successive_count(2);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test3",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13551);
    EXPECT_EQ(selector_state.lane_change_type, LaneChangeType::NO_CHANGE);
    EXPECT_EQ(selector_state.selector_lane_change_request.lane_change_type(),
              LaneChangeType::DEFAULT_ROUTE_CHANGE);
    EXPECT_FALSE(selector_state.selector_lane_change_request.lc_left());
  }

  {
    // Test: noa mode, successive best lane change frame is 4,
    // need to lane change confirmation, route lane change.
    // with alc confirmation input, confirm lane change.
    // Result: lane change.
    // No lane change request.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = true;
    selector_flags.planner_begin_lane_change_frame = 3;
    selector_flags.planner_begin_radical_lane_change_frame = 3;
    selector_flags.planner_begin_signal_frame = 3;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13551);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13555);
    best_target_lane_state_proto.set_successive_count(3);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    input_selector_state.selector_lane_change_request.set_lc_left(false);
    input_selector_state.selector_lane_change_request.set_lane_change_type(
        LaneChangeType::DEFAULT_ROUTE_CHANGE);
    // prefilterd by alc confirmation input.
    auto new_status = scheduler_and_est_output.est_status;
    new_status[0] = PlannerStatus(PlannerStatusProto::BRANCH_RESULT_IGNORED,
                                  " prefiltered.");

    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .alc_confirmation = true,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(const auto selector_output,
                  SelectTrajectory(selector_input, new_status,
                                   scheduler_and_est_output.st_traj_mgr_list,
                                   scheduler_and_est_output.results,
                                   &selector_debug, &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test4",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13555);
    EXPECT_EQ(selector_state.selector_lane_change_request.lane_change_type(),
              LaneChangeType::NO_CHANGE);
  }
}

// NOLINTNEXTLINE
TEST(Selector, NoaRouteLaneChangeRequest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "noa_route_lane_change_request";
  const std::vector<std::string> channel_prefix{"lane_keep", "lane_change"};
  const Vec2d ego_pos = Vec2d(239.1, -804);
  const Vec2d ego_speed = Vec2d(15.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(13596), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(13564)}, 0.0, 1.0);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(235.1, -850), Vec2d(12.0, 0.0), -M_PI_2);
  const int branch_size = 2;
  const int begin_lane_idx = 1;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  {
    // Test: noa mode, successive best lane change frame is 3,
    // no need to lane change confirmation.
    // Tricky scenario, send lane change request.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 3;
    selector_flags.planner_begin_signal_frame = 3;
    selector_flags.planner_begin_radical_lane_change_frame = 3;
    selector_flags.planner_enable_lc_request_in_tricky_scenario = true;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13564);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13562);
    best_target_lane_state_proto.set_successive_count(3);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    SelectorStateProto selector_state_proto;
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test1",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13564);
    EXPECT_EQ(selector_state.selector_lane_change_request.lane_change_type(),
              LaneChangeType::DEFAULT_ROUTE_CHANGE);
    EXPECT_FALSE(selector_state.selector_lane_change_request.lc_left());
    EXPECT_EQ(selector_state.route_ttc_setting.route_request_state(),
              RouteTtcRequestState::WAITING_RESPONSE);
    EXPECT_EQ(selector_state.route_ttc_setting.request_config(),
              RouteTtcConfig::CONSERVATIVE);
  }

  {
    // Test: noa mode, successive best lane change frame is 3,
    // no need to lane change confirmation.
    // already send lane change request
    // receive lane change confirmation.
    // result: lane change.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 3;
    selector_flags.planner_begin_signal_frame = 3;
    selector_flags.planner_begin_radical_lane_change_frame = 3;
    selector_flags.planner_enable_lc_request_in_tricky_scenario = true;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13564);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13562);
    best_target_lane_state_proto.set_successive_count(3);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    input_selector_state.selector_lane_change_request.set_lc_left(false);
    input_selector_state.selector_lane_change_request.set_lane_change_type(
        LaneChangeType::DEFAULT_ROUTE_CHANGE);
    input_selector_state.route_ttc_setting.set_route_request_state(
        RouteTtcRequestState::WAITING_RESPONSE);
    input_selector_state.route_ttc_setting.set_request_config(
        RouteTtcConfig::CONSERVATIVE);
    // prefilterd by alc confirmation input.
    auto new_status = scheduler_and_est_output.est_status;
    new_status[0] = PlannerStatus(PlannerStatusProto::BRANCH_RESULT_IGNORED,
                                  " prefiltered.");
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .alc_confirmation = true,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(const auto selector_output,
                  SelectTrajectory(selector_input, new_status,
                                   scheduler_and_est_output.st_traj_mgr_list,
                                   scheduler_and_est_output.results,
                                   &selector_debug, &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test2",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13562);
    EXPECT_EQ(selector_state.selector_lane_change_request.lane_change_type(),
              LaneChangeType::NO_CHANGE);
    EXPECT_EQ(selector_state.route_ttc_setting.route_request_state(),
              RouteTtcRequestState::RECEIVED_RESPONSE);
    EXPECT_EQ(selector_state.route_ttc_setting.response_config(),
              RouteTtcConfig::CONSERVATIVE);
  }
}

TEST(Selector, OppositePaddleLaneChange) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "opposite_paddle_lane_change";
  const std::vector<std::string> channel_prefix{"lane_change", "lane_keep"};
  const Vec2d ego_pos = Vec2d(-568.5, -452);
  const Vec2d ego_speed = Vec2d(15.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(14989), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(14917)}, 0.0, 0.9);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingLargeVehicle(Vec2d(-568.5, -500), Vec2d(10.0, 0.0), -M_PI_2);
  const int branch_size = 2;
  const int begin_lane_idx = 1;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));

  {  // Disable opposite lane change.
    SelectorState empty_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_allow_lc_time_after_activate_selector = 0.0;
    selector_flags.planner_allow_opposite_lc_time_after_paddle_lc = 30.0;
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    empty_selector_state.last_lc_info.set_lc_left(false);
    qcraft::ToProto(
        scheduler_and_est_output.plan_time - absl::Seconds(10.0),
        empty_selector_state.last_lc_info.mutable_lane_change_time());
    empty_selector_state.last_lc_info.set_lane_change_type(
        LaneChangeType::PADDLE_CHANGE);
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .alc_confirmation = true,
        .selector_state = &empty_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test1",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              14917);
  }

  {  // Active opposite lane change.
    SelectorState empty_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_allow_lc_time_after_activate_selector = 0.0;
    selector_flags.planner_allow_opposite_lc_time_after_paddle_lc = 30.0;
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    empty_selector_state.last_lc_info.set_lc_left(false);
    qcraft::ToProto(
        scheduler_and_est_output.plan_time - absl::Seconds(80.0),
        empty_selector_state.last_lc_info.mutable_lane_change_time());
    empty_selector_state.last_lc_info.set_lane_change_type(
        LaneChangeType::PADDLE_CHANGE);
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .alc_confirmation = true,
        .selector_state = &empty_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test2",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              14911);
  }
}

TEST(Selector, IsGoingForceMerge) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "is_going_force_merge";
  const std::vector<std::string> channel_prefix{"lane_change", "lane_keep"};
  const Vec2d ego_pos = Vec2d(-3164, -1618);
  const Vec2d ego_speed = Vec2d(15.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(36225), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(36175)}, 0.0, 1.0);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr = PlannerObjectManager();
  const int branch_size = 2;
  const int begin_lane_idx = 1;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));
  auto new_status = scheduler_and_est_output.est_status;
  new_status[0] = PlannerStatus(PlannerStatusProto::LC_SAFETY_CHECK_FAILED,
                                " prefiltered.");

  // Lane change because of large vehicle.
  SelectorState empty_selector_state;
  SelectorFlags selector_flags;
  absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
  SelectorInput selector_input{
      .psmm = &psmm,
      .sections_info = &scheduler_and_est_output.sections_info,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &scheduler_and_est_output.motion_constraints,
      .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
      .plan_start_point = &scheduler_and_est_output.plan_start_point,
      .stalled_objects = &stalled_objects,
      .route_navi_info = &scheduler_and_est_output.route_navi_info,
      .avoid_lanes = &empty_avoid_lanes,
      .plan_time = scheduler_and_est_output.plan_time,
      .alc_confirmation = true,
      .selector_state = &empty_selector_state,
      .selector_flags = &selector_flags,
      .config = &scheduler_and_est_output.selector_params};

  SelectorDebugProto selector_debug;
  SelectorState selector_state;
  ASSIGN_OR_DIE(const auto selector_output,
                SelectTrajectory(selector_input, new_status,
                                 scheduler_and_est_output.st_traj_mgr_list,
                                 scheduler_and_est_output.results,
                                 &selector_debug, &selector_state));
  const auto& selected_output =
      scheduler_and_est_output.results[selector_output.selected_idx];
  if (send_to_canvas) {
    SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                       canvas_prefix + "/test1",
                                       vis::Color::kLightGreen);
  }
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            36175);
  EXPECT_TRUE(selector_output.is_going_force_route_change_left.has_value());
  EXPECT_TRUE(*selector_output.is_going_force_route_change_left);
}

// NOLINTNEXTLINE
TEST(Selector, IsGoingForceRouteChange) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "is_going_force_route_change";
  const std::vector<std::string> channel_prefix{"lane_keep", "lane_change"};
  const Vec2d ego_pos = Vec2d(239.1, -1271);
  const Vec2d ego_speed = Vec2d(15.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(13596), 1.0);
  const mapping::LanePath prev_lp(psmm.semantic_map_manager(),
                                  {mapping::ElementId(13591)}, 0.0, 1.0);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(235.1, -1299), Vec2d(20.0, 0.0), -M_PI_2);
  const int branch_size = 2;
  const int begin_lane_idx = 1;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));
  {
    // lane change is not safe.
    // need to compute cost.
    // but we will not choose it.
    // Lane change because of large vehicle.
    SelectorState empty_selector_state;
    SelectorFlags selector_flags;
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &empty_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};

    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    auto new_status = scheduler_and_est_output.est_status;
    new_status[1] = PlannerStatus(PlannerStatusProto::LC_SAFETY_CHECK_FAILED,
                                  " prefiltered.");
    ASSIGN_OR_DIE(const auto selector_output,
                  SelectTrajectory(selector_input, new_status,
                                   scheduler_and_est_output.st_traj_mgr_list,
                                   scheduler_and_est_output.results,
                                   &selector_debug, &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test1",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13591);
    EXPECT_TRUE(selector_output.is_going_force_route_change_left.has_value());
    EXPECT_FALSE(*selector_output.is_going_force_route_change_left);
  }

  {
    // we will choose lane change.
    // Lane change because of large vehicle.
    SelectorState empty_selector_state;
    SelectorFlags selector_flags;
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &empty_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test2",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13596);

    EXPECT_FALSE(selector_output.is_going_force_route_change_left.has_value());
  }

  {
    // Test: noa mode, successive best lane change frame is 2,
    // no need to lane change confirmation, route lane change.
    // Result: lane keep to 13591
    // No lane change request.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 3;
    selector_flags.planner_begin_radical_lane_change_frame = 3;
    selector_flags.planner_begin_signal_frame = 3;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13591);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13596);
    best_target_lane_state_proto.set_successive_count(1);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test3",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13591);
    EXPECT_TRUE(selector_output.is_going_force_route_change_left.has_value());
    EXPECT_FALSE(*selector_output.is_going_force_route_change_left);
  }

  {
    // Test: noa mode, successive best lane change frame is 2,
    // no need to lane change confirmation, route lane change.
    // Result: lane change.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 3;
    selector_flags.planner_begin_radical_lane_change_frame = 2;
    selector_flags.planner_begin_signal_frame = 3;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13591);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13596);
    best_target_lane_state_proto.set_successive_count(1);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test4",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13596);
    EXPECT_FALSE(selector_output.is_going_force_route_change_left.has_value());
  }
}

// NOLINTNEXTLINE
TEST(Selector, ForceChooseLaneKeep) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const bool send_to_canvas = true;
  const std::string canvas_prefix = "force_choose_lane_keep";
  const std::vector<std::string> channel_prefix{"lane_keep", "lane_change"};
  const Vec2d ego_pos = Vec2d(239.1, -485);
  const Vec2d ego_speed = Vec2d(15.0, 0.0);
  const double ego_heading = -M_PI_2;
  const std::string route_name = "";
  const mapping::LanePoint destination(mapping::ElementId(13596), 1.0);
  const mapping::LanePath prev_lp(
      psmm.semantic_map_manager(),
      {mapping::ElementId(13551), mapping::ElementId(13564)}, 0.1, 0.9);
  const double preview_route_navi_dist = 2000.0;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(235.1, -1299), Vec2d(20.0, 0.0), -M_PI_2);
  const int branch_size = 2;
  const int begin_lane_idx = 1;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  ASSIGN_OR_DIE(auto scheduler_and_est_output,
                RunSchedulerAndEstPlanner(
                    ego_pos, ego_speed, ego_heading, send_to_canvas,
                    channel_prefix, canvas_prefix, object_mgr, st_traj_mgr,
                    stalled_objects, route_name, destination, prev_lp, psmm,
                    preview_route_navi_dist, begin_lane_idx, branch_size));
  const auto plan_time = scheduler_and_est_output.plan_time;

  {
    // Test: noa mode, successive best lane change frame is 0,
    // no need to lane change confirmation, route lane change.
    // Just enter selector, choose lane keep.
    // Result: lane keep to 13591
    // No lane change request.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 1;
    selector_flags.planner_begin_radical_lane_change_frame = 1;
    selector_flags.planner_begin_signal_frame = 1;
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test1",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13551);
    EXPECT_TRUE(selector_state.activate_selector_time.has_value());
    EXPECT_EQ(*selector_state.activate_selector_time, plan_time);
    EXPECT_FALSE(selector_state.start_lane_change_time.has_value());
    EXPECT_FALSE(selector_state.give_up_lane_change_time.has_value());
    EXPECT_EQ(selector_state.lane_change_type, LaneChangeType::NO_CHANGE);
    EXPECT_EQ(selector_output.turn_signal, TurnSignal::TURN_SIGNAL_NONE);
  }

  {
    // Test: noa mode, successive best lane change frame is 1,
    // no need to lane change confirmation, route lane change.
    // Activate selector 1s.
    // Result: lane keep.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 1;
    selector_flags.planner_begin_radical_lane_change_frame = 1;
    selector_flags.planner_begin_signal_frame = 1;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13551);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13555);
    best_target_lane_state_proto.set_successive_count(1);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    input_selector_state.activate_selector_time =
        plan_time - absl::Seconds(1.0);
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test2",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13551);
    EXPECT_FALSE(selector_state.start_lane_change_time.has_value());
    EXPECT_FALSE(selector_state.give_up_lane_change_time.has_value());
  }

  {
    // Test: noa mode, successive best lane change frame is 1,
    // no need to lane change confirmation, route lane change.
    // Activate selector 3s.
    // Result: lane change.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 1;
    selector_flags.planner_begin_radical_lane_change_frame = 1;
    selector_flags.planner_begin_signal_frame = 1;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13551);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13555);
    best_target_lane_state_proto.set_successive_count(1);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    input_selector_state.activate_selector_time =
        plan_time - absl::Seconds(3.0);
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test3",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13555);
    EXPECT_TRUE(selector_state.start_lane_change_time.has_value());
    EXPECT_EQ(*selector_state.start_lane_change_time, plan_time);
    EXPECT_FALSE(selector_state.give_up_lane_change_time.has_value());
    EXPECT_EQ(selector_state.lane_change_type,
              LaneChangeType::DEFAULT_ROUTE_CHANGE);
    EXPECT_EQ(selector_state.pre_turn_signal, TurnSignal::TURN_SIGNAL_NONE);
    EXPECT_FALSE(selector_state.lc_prepare_stage_lane_path.has_value());
    EXPECT_EQ(selector_output.turn_signal, TurnSignal::TURN_SIGNAL_RIGHT);
  }

  {
    // Test: noa mode, is lane changing
    // no need to lane change confirmation, route lane change.
    // but lane change is paused.
    // start lane change time is 20s ago.
    // need to continue lane change because give up lane change will be useful
    // next time.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 1;
    selector_flags.planner_begin_radical_lane_change_frame = 1;
    selector_flags.planner_begin_signal_frame = 1;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13555);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13555);
    best_target_lane_state_proto.set_successive_count(1);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    input_selector_state.start_lane_change_time = plan_time - absl::Seconds(20);
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;

    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    scheduler_and_est_output.results[1]
        .scheduler_output.lane_change_state.set_stage(
            LaneChangeStage::LCS_PAUSE);
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    scheduler_and_est_output.results[1]
        .scheduler_output.lane_change_state.set_stage(
            LaneChangeStage::LCS_EXECUTING);
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test4",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13555);
    EXPECT_TRUE(selector_state.start_lane_change_time.has_value());
    EXPECT_EQ(*selector_state.start_lane_change_time,
              plan_time - absl::Seconds(20));
    EXPECT_TRUE(selector_state.give_up_lane_change_time.has_value());
    EXPECT_EQ(*selector_state.give_up_lane_change_time, plan_time);
    EXPECT_EQ(selector_state.lane_change_type,
              LaneChangeType::DEFAULT_ROUTE_CHANGE);
    EXPECT_EQ(selector_output.turn_signal, TurnSignal::TURN_SIGNAL_RIGHT);
  }

  {
    // Test: noa mode, is lane changing
    // no need to lane change confirmation, route lane change.
    // but lane change is paused.
    // will choose lane keep.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 1;
    selector_flags.planner_begin_radical_lane_change_frame = 1;
    selector_flags.planner_begin_signal_frame = 1;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13555);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13555);
    best_target_lane_state_proto.set_successive_count(1);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    input_selector_state.start_lane_change_time = plan_time - absl::Seconds(20);
    input_selector_state.give_up_lane_change_time = plan_time;
    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;

    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    scheduler_and_est_output.results[1]
        .scheduler_output.lane_change_state.set_stage(
            LaneChangeStage::LCS_PAUSE);
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    scheduler_and_est_output.results[1]
        .scheduler_output.lane_change_state.set_stage(
            LaneChangeStage::LCS_EXECUTING);
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test5",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13551);
    EXPECT_FALSE(selector_state.start_lane_change_time.has_value());
    EXPECT_TRUE(selector_state.give_up_lane_change_time.has_value());
    EXPECT_EQ(*selector_state.give_up_lane_change_time, plan_time);
  }

  {
    // Test: noa mode, successive best lane change frame is 2,
    // no need to lane change confirmation, route lane change.
    // begin lane change signal and send lane change reason.
    SelectorState input_selector_state;
    SelectorFlags selector_flags;
    selector_flags.planner_is_l4_mode = false;
    selector_flags.planner_need_to_lane_change_confirmation = false;
    selector_flags.planner_begin_lane_change_frame = 6;
    selector_flags.planner_begin_radical_lane_change_frame = 2;
    selector_flags.planner_begin_signal_frame = 3;
    TargetLaneStateProto last_target_lane_state_proto;
    last_target_lane_state_proto.set_is_borrow(false);
    last_target_lane_state_proto.set_is_fallback(false);
    last_target_lane_state_proto.add_lane_ids(13551);
    input_selector_state.selected_target_lane_state =
        last_target_lane_state_proto;
    TargetLaneStateProto best_target_lane_state_proto;
    best_target_lane_state_proto.set_is_borrow(false);
    best_target_lane_state_proto.set_is_fallback(false);
    best_target_lane_state_proto.add_lane_ids(13555);
    best_target_lane_state_proto.set_successive_count(2);
    input_selector_state.best_target_lane_state =
        std::move(best_target_lane_state_proto);

    absl::flat_hash_set<mapping::ElementId> empty_avoid_lanes;
    SelectorInput selector_input{
        .psmm = &psmm,
        .sections_info = &scheduler_and_est_output.sections_info,
        .prev_lane_path_from_current = &prev_lp,
        .prev_traj = nullptr,
        .motion_constraints = &scheduler_and_est_output.motion_constraints,
        .vehicle_geom = &scheduler_and_est_output.vehicle_geom,
        .plan_start_point = &scheduler_and_est_output.plan_start_point,
        .stalled_objects = &stalled_objects,
        .route_navi_info = &scheduler_and_est_output.route_navi_info,
        .avoid_lanes = &empty_avoid_lanes,
        .plan_time = scheduler_and_est_output.plan_time,
        .selector_state = &input_selector_state,
        .selector_flags = &selector_flags,
        .config = &scheduler_and_est_output.selector_params};
    SelectorDebugProto selector_debug;
    SelectorState selector_state;
    ASSIGN_OR_DIE(
        const auto selector_output,
        SelectTrajectory(selector_input, scheduler_and_est_output.est_status,
                         scheduler_and_est_output.st_traj_mgr_list,
                         scheduler_and_est_output.results, &selector_debug,
                         &selector_state));
    const auto& selected_output =
        scheduler_and_est_output.results[selector_output.selected_idx];
    if (send_to_canvas) {
      SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                         canvas_prefix + "/test6",
                                         vis::Color::kLightGreen);
    }
    EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id()
                  .value(),
              13551);
    EXPECT_EQ(selector_state.lane_change_type,
              LaneChangeType::DEFAULT_ROUTE_CHANGE);
    EXPECT_EQ(selector_state.pre_turn_signal, TurnSignal::TURN_SIGNAL_RIGHT);
    EXPECT_EQ(selector_output.turn_signal, TurnSignal::TURN_SIGNAL_RIGHT);
    EXPECT_TRUE(selector_state.lc_prepare_stage_lane_path.has_value());
    EXPECT_EQ(
        selector_state.lc_prepare_stage_lane_path->front().lane_id().value(),
        13555);
  }
}

}  // namespace
}  // namespace qcraft::planner
