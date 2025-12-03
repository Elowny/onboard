#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "benchmark/benchmark.h"

#include "onboard/proto/perception/fusion/object.pb.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

#include "onboard/async/future.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/clock.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/polygon2d_util.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/est_planner.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
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
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {
namespace {

ObjectProto BuildObject(const std::string& id, const Polygon2d& contour,
                        const Vec2d& pos, double heading, double timestamp) {
  return PerceptionObjectBuilder()
      .set_id(id)
      .set_type(OT_VEHICLE)
      .set_pos(pos)
      .set_timestamp(timestamp)
      .set_velocity(5.0)
      .set_yaw(heading)
      .set_length_width(4.0, 2.0)
      .set_box_center(pos)
      .set_contour(contour)
      .Build();
}

ObjectsProto BuildPerceptionObjects(int num_objects, int num_contour_points,
                                    const Vec2d& pos, double heading,
                                    double timestamp) {
  ObjectsProto objects;
  objects.mutable_header()->set_timestamp(timestamp);
  objects.set_scope(ObjectsProto::SCOPE_REAL);

  const Polygon2d contour = polygon2d::CreateRegularPolygon(
      num_contour_points,
      /*center=*/Vec2d::Zero(), /*radius=*/2.0, /*first_point_angle=*/0.0);
  for (int i = 0; i < num_objects; ++i) {
    *objects.add_objects() =
        BuildObject(std::to_string(i), contour, pos, heading, timestamp);
  }
  return objects;
}

ObjectsPredictionProto BuildPredictionObjects(const ObjectsProto& objects,
                                              double timestamp) {
  ObjectsPredictionProto prediction_proto;
  prediction_proto.mutable_header()->set_timestamp(timestamp);

  constexpr int kNumTrajs = 3;
  constexpr double kProb = (1.0 - 1e-6) / kNumTrajs;
  for (const auto& object : objects.objects()) {
    ObjectPredictionBuilder builder;
    builder.set_object(object);
    const Vec2d pos(object.pos().x(), object.pos().y());
    const Vec2d speed(object.vel().x(), object.vel().y());
    const double theta = speed.Angle();
    constexpr double kTimeRange = 10.0;  // Seconds.
    for (int i = 0; i < kNumTrajs; ++i) {
      builder.add_predicted_trajectory()
          ->set_probability(kProb)
          .set_straight_line(pos, theta, kTimeRange,
                             /*init_v=*/0.2, /*acc=*/0.5);
    }
    builder.Build().ToProto(prediction_proto.add_objects());
  }
  return prediction_proto;
}

SchedulerOutput BuildSchedulerOutput(
    const PlannerSemanticMapManager& psmm,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    const RouteSections& route_sections, const LanePathInfo& lp_info,
    bool borrow, const ApolloTrajectoryPointProto& start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const mapping::LanePath& prev_lp) {
  // Build drive passages.
  ASSIGN_OR_DIE(const auto backward_extended_lane_path,
                BackwardExtendLanePathOnRouteSections(
                    psmm, route_sections, lp_info.lane_path(),
                    kDrivePassageKeepBehindLength));
  ASSIGN_OR_DIE(auto drive_passage,
                BuildDrivePassage(
                    psmm, /*vision_map_ptr=*/nullptr, lp_info.lane_path(),
                    backward_extended_lane_path,
                    /*anchor_point=*/mapping::LanePoint(),
                    route_sections.planning_horizon(psmm),
                    route_sections.destination(), /*all_lanes_virtual=*/false,
                    /*override_speed_limit_mps=*/std::nullopt));

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

  // Scheduler output.
  return SchedulerOutput{.drive_passage = std::move(drive_passage),
                         .sl_boundary = std::move(path_boundary),
                         .lane_change_state = std::move(lc_state),
                         .length_along_route = lp_info.length_along_route(),
                         .max_reach_length = lp_info.max_reach_length(),
                         .borrow_lane = borrow,
                         .av_frenet_box_on_drive_passage = ego_frenet_box};
}

SpacetimePlannerObjectTrajectoriesProto ExportToSpacetimeObjectTrajectories(
    const SpacetimeTrajectoryManager& traj_mgr) {
  SpacetimePlannerObjectTrajectories trajs;
  for (const auto& traj : traj_mgr.trajectories()) {
    trajs.AddSpacetimePlannerObjectTrajectory(
        traj, SpacetimePlannerObjectTrajectoryReason::FRONT);
  }

  SpacetimePlannerObjectTrajectoriesProto proto;
  trajs.ToProto(&proto);
  return proto;
}

void BM_StPathPlannerNoLaneChange(benchmark::State& state) {  // NOLINT
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  auto param_manager = CreateParamManagerFromCarId("Q1001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();

  const absl::Time plan_time = Clock::Now();

  // Plan start point.
  const Vec2d sdc_pos(116.5, 0.0);
  const Vec2d sdc_speed(11.0, 0.0);
  const PoseProto sdc_pose = CreatePose(ToUnixDoubleSeconds(plan_time), sdc_pos,
                                        /*heading=*/0.0, sdc_speed);
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  const StPathPlanStartPointInfo st_path_start_point_info{
      .reset = false,
      .relative_index_from_plan_start_point = 0,
      .start_point = start_point,
      .plan_time = plan_time,
  };
  const PlanStartPointInfo start_point_info{
      .reset = false,
      .start_index_on_prev_traj = 0,
      .start_point = start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };

  // Setup maps.
  auto loader = mapping::v2::SemanticMapLoader::MakeShared();
  auto map_fut = loader->PreloadWholeMap();
  auto smm = map_fut.Get();

  auto cc = CoordinateConverter::FromMap("dojo");
  const auto& psmm = planner::CreateDojoTestPSMM();
  const SmoothedReferenceLineResultMap smooth_result_map;

  // Create spacetime object manager.
  constexpr int kNumContourPoints = 16;
  const int num_perception_objects = state.range(0);
  constexpr double kTimeOffset = 1.0;  // Seconds.
  const auto perception_objects = BuildPerceptionObjects(
      num_perception_objects, kNumContourPoints,
      /*pos=*/sdc_pos + sdc_speed * kTimeOffset, /*heading=*/0.0,
      /*timestamp=*/0.0);
  const auto prediction_objects = BuildPredictionObjects(perception_objects,
                                                         /*timestamp=*/0.0);
  const auto planner_objects =
      BuildPlannerObjects(&perception_objects, &prediction_objects,
                          /*align_time=*/0.0,
                          /*thread_pool=*/nullptr);
  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  const PlannerObjectManager obj_mgr(planner_objects);
  const auto vehicle_geom = DefaultVehicleGeometry();
  const LaneChangeStateProto lane_change_state = MakeNoneLaneChangeState();
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const RouteSectionsInfo sections_info(psmm, &route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);
  const absl::flat_hash_set<std::string> empty_stalled_objects;
  const auto lane_graph =
      BuildLaneGraph(psmm, sections_info, obj_mgr, empty_stalled_objects,
                     /*avoid_lanes=*/{}, route_navi_info);
  const auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  const TrajectoryProto previous_trajectory;
  const absl::Time parking_brake_release_time = plan_time;
  const InitializerStateProto initializer_state;
  const SpacetimePlannerObjectTrajectoriesProto
      st_planner_object_trajectories_proto =
          ExportToSpacetimeObjectTrajectories(st_traj_mgr);

  const TrafficLightInfoMap tl_info_map;
  const std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  const SceneOutputProto scene_reasoning;

  const mapping::LanePath prev_lp(
      smm.get(),
      {mapping::ElementId(498), mapping::ElementId(2483),
       mapping::ElementId(500)},
      0.86, 1.0);
  // Scheduler output.
  const auto scheduler_output = BuildSchedulerOutput(
      psmm, smooth_result_map, route_sections, lp_infos[0], /*borrow=*/false,
      start_point, vehicle_geom, st_traj_mgr, prev_lp);
  const auto path_look_ahead_time = GetStPathPlanLookAheadTime(
      start_point_info, sdc_pose, absl::Milliseconds(0), previous_trajectory);
  const auto path_start_point_info = GetStPathPlanStartPointInfo(
      path_look_ahead_time, start_point_info, previous_trajectory,
      /*trajectory_optimizer_time_step=*/std::nullopt,
      /*last_st_path_plan_start_time=*/std::nullopt);
  ASSIGN_OR_DIE(const auto driving_map_topo,
                BuildDrivingMapByRouteOnOfflineMap(psmm, route_sections));

  const DeciderStateProto decider_state;
  const InitializerStateProto initializer_state_proto;
  const TrajectoryOptimizerStateProto traj_opt_state_proto;
  const ml::captain_net::CaptainNetOutput captain_net_output;

  EstPlannerInput input{
      .driving_map_topo = &driving_map_topo,
      .semantic_map_manager = psmm.semantic_map_manager(),
      .planner_semantic_map_manager = &psmm,
      .plan_id = 1,
      .vehicle_params = &vehicle_params,
      .is_run_model_l4 = true,
      .parking_brake_release_time = parking_brake_release_time,
      .decider_state = &decider_state,
      .initializer_state = &initializer_state_proto,
      .trajectory_optimizer_state_proto = &traj_opt_state_proto,
      .st_planner_object_trajectories = &st_planner_object_trajectories_proto,
      .obj_mgr = &obj_mgr,
      .start_point_info = &start_point_info,
      .st_path_start_point_info = &st_path_start_point_info,
      .tl_info_map = &tl_info_map,
      .smooth_result_map = &smooth_result_map,
      .stalled_objects = &empty_stalled_objects,
      .scene_reasoning = &scene_reasoning,
      .prev_target_lane_path_from_start = &scheduler_output.lane_path_before_lc,
      .time_aligned_prev_traj = &time_aligned_prev_traj,
      .enable_force_stop = false,
      .st_traj_mgr = &st_traj_mgr,
      .log_av_trajectory = nullptr,
      .captain_net_output = &captain_net_output,
      .planner_av_context = nullptr,
      .real_objects = nullptr,
      .virtual_objects = nullptr,
      .planner_model_pool = nullptr,
      .decision_constraint_config =
          &planner_params.decision_constraint_config(),
      .initializer_params = &planner_params.initializer_params(),
      .trajectory_optimizer_params =
          &planner_params.trajectory_optimizer_params(),
      .speed_finder_params = &planner_params.speed_finder_params(),
      .motion_constraint_params = &planner_params.motion_constraint_params(),
      .planner_functions_params = &planner_params.planner_functions_params(),
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
          &planner_params.spacetime_planner_object_trajectories_params()};

  EstPlannerOutput est_output;
  EstPlannerDebug debug_info;
  vis::vantage::ChartDataBundleProto chart_data;
  FLAGS_planner_use_ml_trajectory_as_initializer_ref_traj = false;
  for (auto _ : state) {
    benchmark::DoNotOptimize(RunEstPlanner(input, scheduler_output, &est_output,
                                           &debug_info, &chart_data,
                                           /*thread_pool=*/nullptr));
  }
}
BENCHMARK(BM_StPathPlannerNoLaneChange)->Arg(1)->Arg(5)->Arg(10);
}  // namespace
}  // namespace planner
}  // namespace qcraft

BENCHMARK_MAIN();
