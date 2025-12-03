#include "onboard/planner/assist/alcc_selector.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/async/parallel_for.h"
#include "onboard/async/thread_pool.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/alcc_scheduler.h"
#include "onboard/planner/assist/assist_util.h"
#include "onboard/planner/assist/lcc_map_builder.h"
#include "onboard/planner/common/plan_start_point_info.h"
// #include "onboard/planner/common/plot_util.h"
#include "absl/container/flat_hash_set.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "common/proto/qalc.pb.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/assist/proto/plc_result.pb.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/est_planner.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {

AlccSchedulerOutput BuildAlccSchedulerOutput(
    const VehicleGeometryParamsProto& vehicle_geom,
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const mapping::LanePath& lane_path, QALCState alc_state,
    LaneChangeDirection lc_direction,
    std::optional<double> lcc_cruising_speed_mps) {
  auto drive_passage = *BuildDrivePassageFromLanePath(
      psmm, lane_path,
      /*step_s=*/1.0, /*avoid_loop=*/true, kDrivePassageKeepBehindLength,
      kAlccReferenceLineRequiredLength, kDrivePassageKeepBehindLength,
      lcc_cruising_speed_mps);

  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const Box2d ego_box = ComputeAvBox(
      ego_pos, plan_start_point.path_point().theta(), vehicle_geom);
  ASSIGN_OR_DIE(const auto ego_frenet_box,
                drive_passage.QueryFrenetBoxAt(ego_box));

  const bool lane_change =
      (alc_state == ALC_ONGOING || alc_state == ALC_CROSSING_LANE ||
       alc_state == ALC_RETURNING);

  LaneChangeStateProto lc_state;
  lc_state.set_stage(AlcStateToLaneChangeStage(alc_state));
  if (lane_change) {
    lc_state.set_lc_left(lc_direction == LaneChangeDirection::LCD_LEFT);
  }
  ASSIGN_OR_DIE(
      auto path_boundary,
      lane_change ? BuildPathBoundaryFromPose(
                        psmm, drive_passage, plan_start_point, vehicle_geom,
                        st_traj_mgr, lc_state, SmoothedReferenceLineResultMap(),
                        /*borrow_lane_boundary=*/false, /*should_smooth*/ false,
                        /*unsafe_object_ids=*/{})
                  : BuildPathBoundaryFromDrivePassage(psmm, drive_passage));
  return AlccSchedulerOutput{.drive_passage = std::move(drive_passage),
                             .sl_boundary = std::move(path_boundary),
                             .av_frenet_box_on_drive_passage = ego_frenet_box,
                             .alc_state = alc_state,
                             .lc_direction = lc_direction};
}

std::vector<ApolloTrajectoryPointProto> MakeStraightTrajectory(
    const Vec2d& start, const Vec2d& end, int num_pt) {
  const Vec2d step = (end - start) / (num_pt - 1);
  const double step_s = step.norm();
  const double heading = (end - start).FastAngle();
  const double v = step_s / kTrajectoryTimeStep;

  std::vector<ApolloTrajectoryPointProto> traj_pts;
  traj_pts.reserve(num_pt);

  Vec2d prev_pt = start - step;
  double prev_s = -step_s;
  for (int i = 0; i < num_pt; ++i) {
    ApolloTrajectoryPointProto pt;
    pt.mutable_path_point()->set_x(prev_pt.x() + step.x());
    pt.mutable_path_point()->set_y(prev_pt.y() + step.y());
    pt.mutable_path_point()->set_theta(heading);
    pt.mutable_path_point()->set_s(prev_s += step_s);
    pt.set_v(v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(i * kTrajectoryTimeStep);

    traj_pts.push_back(std::move(pt));
    prev_pt += step;
  }
  return traj_pts;
}
TEST(AlccSelectorTest, LaneChangeTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");

  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();

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

  // Construct online map and psmm.
  const auto& whole_psmm = CreateDojoTestPSMM();
  ASSIGN_OR_DIE(
      const auto online_smm_proto,
      RunOnlineSemanticMapConverter(
          whole_psmm, OnlineSemanticMapConverterOption{
                          .timestamp_s = ToUnixDoubleSeconds(plan_time),
                          .smooth_x = sdc_pose.pos_smooth().x(),
                          .smooth_y = sdc_pose.pos_smooth().y(),
                          .smooth_yaw = sdc_pose.yaw(),
                          .look_ahead_distance = 150.0,
                          .look_back_distance = 20.0}));

  ASSIGN_OR_DIE(const auto psmm_ptr, BuildOnlineMapPsmm(online_smm_proto));
  const auto& psmm = *psmm_ptr;

  // Construct origin lane path.
  auto origin_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(2448), mapping::ElementId(1),
                         mapping::ElementId(34), mapping::ElementId(2471)},
                        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  // Construct target lane path on left.
  auto target_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(2), mapping::ElementId(938),
                         mapping::ElementId(940), mapping::ElementId(2470)},
                        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  // Construct driving map topo.
  ASSIGN_OR_DIE(auto dm_result,
                UpdateLccDrivingMapByOnlineMap(
                    psmm, origin_lane_path, target_lane_path, online_smm_proto,
                    Vec2dFromPoseProto(sdc_pose)));

  const auto dm = std::move(dm_result.driving_map);
  origin_lane_path = std::move(dm_result.aligned_origin_lane_path);
  target_lane_path = std::move(dm_result.aligned_target_lane_path);

  const PlannerObjectManager empty_object_mgr;
  const SpacetimeTrajectoryManager empty_st_traj_mgr;
  std::vector<AlccSchedulerOutput> scheduler_results;
  scheduler_results.reserve(2);
  // Construct ALC_RETURNING scheduler.
  scheduler_results.push_back(BuildAlccSchedulerOutput(
      vehicle_geom, psmm, plan_start_point, empty_st_traj_mgr, origin_lane_path,
      ALC_RETURNING, LaneChangeDirection::LCD_NONE,
      /*lcc_cruising_speed_mps=*/11.0));

  // Construct ALC_ONGOING scheduler.
  scheduler_results.push_back(BuildAlccSchedulerOutput(
      vehicle_geom, psmm, plan_start_point, empty_st_traj_mgr, target_lane_path,
      ALC_ONGOING, LaneChangeDirection::LCD_LEFT,
      /*lcc_cruising_speed_mps=*/11.0));

  const TrajectoryProto previous_trajectory;
  const auto path_look_ahead_time =
      GetStPathPlanLookAheadTime(plan_start_point_info, sdc_pose,
                                 absl::Milliseconds(0), previous_trajectory);
  const auto path_start_point_info = GetStPathPlanStartPointInfo(
      path_look_ahead_time, plan_start_point_info, previous_trajectory,
      /*trajectory_optimizer_time_step=*/std::nullopt,
      /*last_st_path_plan_start_time=*/std::nullopt);

  // Construct empty input.
  const TrafficLightInfoMap empty_tl_map;
  const SmoothedReferenceLineResultMap empty_refline_map;
  const absl::flat_hash_set<std::string> empty_stalled_objects;
  const SceneOutputProto empty_scene_reasoning;
  const ml::captain_net::CaptainNetOutput empty_captain_net_output;
  const std::vector<ApolloTrajectoryPointProto> empty_time_aligned_prev_traj;
  const SpacetimePlannerObjectTrajectoriesProto
      empty_st_planner_object_trajectories;
  const DeciderStateProto empty_decider_state;
  const InitializerStateProto empty_initializer_state;
  absl::Time parking_brake_release_time;

  ThreadPool* thread_pool = ThreadPool::DefaultPool();

  // Construct est planner outputs.
  std::vector<PlannerStatus> status_list(scheduler_results.size());
  std::vector<EstPlannerOutput> est_outputs(scheduler_results.size());
  std::vector<EstPlannerDebug> est_debugs(scheduler_results.size());
  std::vector<vis::vantage::ChartDataBundleProto> chart_data_bundles(
      scheduler_results.size());

  // Run alcc multi tasks planner.
  ParallelFor(0, scheduler_results.size(), thread_pool, [&](int i) {
    status_list[i] = RunEstPlanner(
        EstPlannerInput{
            .driving_map_topo = &dm,
            .semantic_map_manager = psmm.semantic_map_manager(),
            .planner_semantic_map_manager = &psmm,
            .plan_id = 0,
            .vehicle_params = &vehicle_params,
            .parking_brake_release_time = parking_brake_release_time,
            .decider_state = &empty_decider_state,
            .initializer_state = &empty_initializer_state,
            .trajectory_optimizer_state_proto = nullptr,
            .st_planner_object_trajectories =
                &empty_st_planner_object_trajectories,
            .obj_mgr = &empty_object_mgr,
            .start_point_info = &plan_start_point_info,
            .st_path_start_point_info = &path_start_point_info,
            .tl_info_map = &empty_tl_map,
            .smooth_result_map = &empty_refline_map,
            .stalled_objects = &empty_stalled_objects,
            .scene_reasoning = &empty_scene_reasoning,
            .prev_target_lane_path_from_start = &target_lane_path,
            .time_aligned_prev_traj = &empty_time_aligned_prev_traj,
            .enable_force_stop = false,
            .st_traj_mgr = &empty_st_traj_mgr,
            .log_av_trajectory = nullptr,
            .captain_net_output = &empty_captain_net_output,
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
        SchedulerOutput{
            .drive_passage = std::move(scheduler_results[i].drive_passage),
            .sl_boundary = std::move(scheduler_results[i].sl_boundary),
            .lane_change_state = CalculateLaneChangeState(
                scheduler_results[i].av_frenet_box_on_drive_passage,
                scheduler_results[i].alc_state,
                scheduler_results[i].lc_direction),
            .av_frenet_box_on_drive_passage =
                scheduler_results[i].av_frenet_box_on_drive_passage},
        &est_outputs[i], &est_debugs[i], &chart_data_bundles[i], thread_pool);
  });

  // Run alcc selector.
  ASSIGN_OR_DIE(
      const auto selected_id,
      RunAlccSelector(whole_psmm, vehicle_geom, status_list, est_outputs,
                      /*preferred_idx=*/-1, /*plc_result=*/nullptr));

  EXPECT_EQ(selected_id, 1);
  const auto& selected_output = est_outputs[selected_id];
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            2);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_NEAR(last_traj_pt.x(), 119.1, 2e-1);
  EXPECT_NEAR(last_traj_pt.y(), 3.4, 2e-1);

  //   SendApolloTrajectoryPointsToCanvas(
  //       selected_output.traj_points, "lc/lc_left_traj",
  //       vis::Color::kLightGreen);
}

TEST(AlccSelectorTest, NoValidTrajectoryTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  const auto& psmm = CreateDojoTestPSMM();
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();

  const std::vector<PlannerStatus> status_list = {
      PlannerStatus(PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE,
                    "Drive passage on target lane path not available."),
      PlannerStatus(PlannerStatusProto::INITIALIZER_FAILED,
                    "Failed to project ego position on drive passage.")};
  const std::vector<EstPlannerOutput> est_outputs(2);
  EXPECT_NOT_OK(RunAlccSelector(psmm, vehicle_geom, status_list, est_outputs,
                                /*preferred_idx*/ -1, /*plc_result=*/nullptr));
}

TEST(AlccSelectorTest, AllTrajectoryCrossSolidLineTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");

  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();

  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(168.453, -866.666),
                 /*heading=*/-1.573, Vec2d(11.0, 0.0));
  const auto plan_start_point = ConvertToTrajPointProto(sdc_pose);

  // Construct online map and psmm.
  const auto& psmm = CreateDojoTestPSMM();

  // Construct origin lane path.
  const auto origin_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(14133)},
                        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  // Construct target lane path on left.
  const auto target_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(14129)},
                        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  const SpacetimeTrajectoryManager empty_st_traj_mgr;
  std::vector<AlccSchedulerOutput> alcc_scheduler_outputs;
  alcc_scheduler_outputs.reserve(2);
  // Construct ALC_RETURNING scheduler.
  alcc_scheduler_outputs.push_back(BuildAlccSchedulerOutput(
      vehicle_geom, psmm, plan_start_point, empty_st_traj_mgr, origin_lane_path,
      ALC_RETURNING, LaneChangeDirection::LCD_NONE,
      /*lcc_cruising_speed_mps=*/11.0));

  // Construct ALC_ONGOING scheduler.
  alcc_scheduler_outputs.push_back(BuildAlccSchedulerOutput(
      vehicle_geom, psmm, plan_start_point, empty_st_traj_mgr, target_lane_path,
      ALC_ONGOING, LaneChangeDirection::LCD_LEFT,
      /*lcc_cruising_speed_mps=*/11.0));

  std::vector<EstPlannerOutput> est_outputs(2);
  for (int i = 0; i < 2; ++i) {
    auto& est_output = est_outputs[i];
    auto& alcc_scheduler_output = alcc_scheduler_outputs[i];
    est_output.scheduler_output = SchedulerOutput{
        .drive_passage = std::move(alcc_scheduler_output.drive_passage),
        .sl_boundary = std::move(alcc_scheduler_output.sl_boundary),
        .lane_change_state = CalculateLaneChangeState(
            alcc_scheduler_output.av_frenet_box_on_drive_passage,
            alcc_scheduler_output.alc_state,
            alcc_scheduler_output.lc_direction),
        .av_frenet_box_on_drive_passage =
            alcc_scheduler_output.av_frenet_box_on_drive_passage};
  }

  est_outputs[0].traj_points = MakeStraightTrajectory(
      Vec2dFromPoseProto(sdc_pose), Vec2d(166.013, -964.085), kTrajectorySteps);

  est_outputs[1].traj_points = MakeStraightTrajectory(
      Vec2dFromPoseProto(sdc_pose), Vec2d(172.359, -955.377), kTrajectorySteps);

  const std::vector<PlannerStatus> status_list(2);

  ASSIGN_OR_DIE(const auto selected_id,
                RunAlccSelector(psmm, vehicle_geom, status_list, est_outputs,
                                /*preferred_idx=*/-1, /*plc_result=*/nullptr));
  EXPECT_EQ(selected_id, 1);
  const auto& selected_output = est_outputs[selected_id];
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            14129);

  //   SendApolloTrajectoryPointsToCanvas(
  //       selected_output.traj_points, "lc/cross_solid_boundary",
  //       vis::Color::kLightGreen);
}

TEST(AlccSelectorTest, PlcFailedTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");

  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();

  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(166.296, -866.328),
                 /*heading=*/-1.573, Vec2d(11.0, 0.0));
  const auto plan_start_point = ConvertToTrajPointProto(sdc_pose);

  // Construct online map and psmm.
  const auto& psmm = CreateDojoTestPSMM();

  // Construct origin lane path.
  const auto origin_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(14133)},
                        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  // Construct target lane path on left.
  const auto target_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(14129)},
                        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  const SpacetimeTrajectoryManager empty_st_traj_mgr;
  std::vector<AlccSchedulerOutput> alcc_scheduler_outputs;
  alcc_scheduler_outputs.reserve(2);
  // Construct ALC_RETURNING scheduler.
  alcc_scheduler_outputs.push_back(BuildAlccSchedulerOutput(
      vehicle_geom, psmm, plan_start_point, empty_st_traj_mgr, origin_lane_path,
      ALC_RETURNING, LaneChangeDirection::LCD_NONE,
      /*lcc_cruising_speed_mps=*/11.0));

  // Construct ALC_ONGOING scheduler.
  alcc_scheduler_outputs.push_back(BuildAlccSchedulerOutput(
      vehicle_geom, psmm, plan_start_point, empty_st_traj_mgr, target_lane_path,
      ALC_ONGOING, LaneChangeDirection::LCD_LEFT,
      /*lcc_cruising_speed_mps=*/11.0));

  std::vector<EstPlannerOutput> est_outputs(2);
  for (int i = 0; i < 2; ++i) {
    auto& est_output = est_outputs[i];
    auto& alcc_scheduler_output = alcc_scheduler_outputs[i];
    est_output.scheduler_output = SchedulerOutput{
        .drive_passage = std::move(alcc_scheduler_output.drive_passage),
        .sl_boundary = std::move(alcc_scheduler_output.sl_boundary),
        .lane_change_state = CalculateLaneChangeState(
            alcc_scheduler_output.av_frenet_box_on_drive_passage,
            alcc_scheduler_output.alc_state,
            alcc_scheduler_output.lc_direction),
        .av_frenet_box_on_drive_passage =
            alcc_scheduler_output.av_frenet_box_on_drive_passage};
  }

  est_outputs[0].traj_points = MakeStraightTrajectory(
      Vec2dFromPoseProto(sdc_pose), Vec2d(165.968, -968.237), kTrajectorySteps);

  est_outputs[1].traj_points = MakeStraightTrajectory(
      Vec2dFromPoseProto(sdc_pose), Vec2d(169.159, -966.896), kTrajectorySteps);

  const std::vector<PlannerStatus> status_list(2);

  PlcInternalResult plc_result;
  ASSIGN_OR_DIE(const auto selected_id,
                RunAlccSelector(psmm, vehicle_geom, status_list, est_outputs,
                                /*preferred_idx=*/1, &plc_result));
  EXPECT_EQ(selected_id, 0);
  EXPECT_EQ(plc_result.status, PlcInternalStatus::SOLID_BOUNDARY);
  EXPECT_TRUE(plc_result.left_solid_boundary.has_value());
  EXPECT_TRUE(*plc_result.left_solid_boundary);

  const auto& selected_output = est_outputs[selected_id];
  EXPECT_EQ(selected_output.scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id()
                .value(),
            14133);

  //   SendApolloTrajectoryPointsToCanvas(
  //       selected_output.traj_points, "lk/not_cross_solid_boundary",
  //       vis::Color::kLightGreen);
}
}  // namespace
}  // namespace qcraft::planner
