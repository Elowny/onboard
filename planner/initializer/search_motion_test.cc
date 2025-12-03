#include "onboard/planner/initializer/search_motion.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/global/clock.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/astar_motion_searcher.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/dp_motion_searcher.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/geometry/geometry_graph_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_cache.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

namespace {
constexpr double kBuffer = 0.25;  // m.

TEST(SearchMotionCurvyDp, StopLineTestWithTimeConsumptionTest) {
  FLAGS_planner_initializer_debug_level = 2;

  // Construct sdc pose.
  PoseProto sdc_pose;
  sdc_pose.mutable_pos_smooth()->set_x(30.0);
  sdc_pose.mutable_pos_smooth()->set_y(0.0);
  sdc_pose.set_yaw(0.0);
  sdc_pose.mutable_vel_body()->set_x(5.0);
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  // Load default planner params.
  auto planner_params = DefaultPlannerParams();

  // Load map and create optimized route path.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);

  // Build drive passages.
  auto start_time = absl::Now();
  const auto drive_passages = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_TRUE(drive_passages.ok() && !drive_passages.value().empty());
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building DrivePassage.";
  SendDrivePassageToCanvas(drive_passages.value(),
                           "search_motion_test/stopline_drive_passage");

  // Build geometry graph.
  const std::vector<double> sample_strategy_range = {
      1.0, 20.0, 60.0, 100.0, std::numeric_limits<double>::max()};
  const std::vector<double> sample_strategy_layer_gap = {5.0, 5.0, 10.0, 40.0,
                                                         60.0};
  const std::vector<double> sample_strategy_lateral_resolution = {
      0.25, 0.25, 0.5, 0.5, 0.5};
  const std::vector<int> sample_strategy_crossing_layer = {3, 3, 2, 1, 1};
  const std::vector<double> sample_strategy_unit_length_lateral_span = {
      1.0, 0.25, 0.10, 0.05, 0.025};
  const GeometryGraphSamplingStrategy sampling_params = {
      .range_list = sample_strategy_range,
      .layer_gap_list = sample_strategy_layer_gap,
      .lateral_resolution_list = sample_strategy_lateral_resolution,
      .cross_layer_connection_list = sample_strategy_crossing_layer,
      .unit_length_lateral_span_list =
          sample_strategy_unit_length_lateral_span};

  const SpacetimeTrajectoryManager st_traj_mgr;
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  const auto vehicle_geom = DefaultVehicleGeometry();
  const std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_planner_object_traj, &vehicle_geom,
          MotionForm::kConstTimeIntervalSampleStep, kBuffer, kBuffer);

  const auto path_sl_boundary =
      CreateFakePathSlBoundary(drive_passages.value());
  DrawPathSlBoundaryToCanvas(path_sl_boundary,
                             "search_motion_test/path_boundary");

  // Build stopline constraint.
  HalfPlane halfplane(Vec2d(70.0, -5.25), Vec2d(70.0, 9.75));
  const std::vector<double> stop_s_vec{40.0 -
                                       vehicle_geom.front_edge_to_center()};
  std::vector<std::map<std::string, ConstraintProto::LeadingObjectProto>>
      leading_groups;
  leading_groups.push_back({});
  leading_groups.push_back({});

  const auto vehicle_drive = DefaultVehicleDriveParams();
  const bool lc_multiple_traj = false;
  GeometryFormBuilder form_builder(&drive_passages.value(),
                                   drive_passages->end_s(), 0.0);
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &drive_passages.value(),
      .sl_boundary = &path_sl_boundary,
      .stop_s_vec = &stop_s_vec,
      .leading_groups = &leading_groups,
      .st_traj_mgr = &st_traj_mgr,
      .plan_start_point = &start_point,
      .vehicle_geom = &vehicle_geom,
      .collision_checker = collision_checker.get(),
      .sampling_params = &sampling_params,
      .vehicle_drive = &vehicle_drive,
      .form_builder = &form_builder,
      .lc_multiple_traj = lc_multiple_traj};
  ThreadPool* thread_pool = ThreadPool::DefaultPool();
  InitializerDebugProto initializer_debug;
  start_time = absl::Now();
  GeometryGraphCache graph_cache;
  const auto geom_graph_or = BuildCurvyGeometryGraph(
      geom_graph_builder_input, /*retry_collision_checker=*/false, &graph_cache,
      thread_pool, &initializer_debug);
  ASSERT_TRUE(geom_graph_or.ok());
  const auto& geom_graph = geom_graph_or.value();
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building GeometryGraph (without the time of "
                "building inputs)";
  SendGeometryGraphToCanvas(&geom_graph,
                            "search_motion_test/stopline_geom_graph");

  const ml::captain_net::CaptainNetOutput captain_net_output;
  const std::vector<Vec2d> empty_ref_line_points;
  const MotionSearchInput input{
      .planner_semantic_map_manager = &psmm,
      .start_point = &start_point,
      .plan_time = Clock::Now(),
      .drive_passage = &(drive_passages.value()),
      .sl_boundary = &path_sl_boundary,
      .st_traj_mgr = &st_traj_mgr,
      .initializer_params = &planner_params.initializer_params(),
      .motion_constraint_params = &planner_params.motion_constraint_params(),
      .vehicle_geom = &vehicle_geom,
      .geom_graph = &geom_graph,
      .reference_line_points = &empty_ref_line_points,
      .form_builder = &form_builder,
      .collision_checker = collision_checker.get(),
      .stop_s_vec = &stop_s_vec,
      .leading_groups = &leading_groups,
      .blocking_static_traj = nullptr,
      .captain_net_output = &captain_net_output,
      .is_lane_change = false,
      .eval_safety = false,
      .log_av_trajectory = nullptr,
      .is_run_model_l4 = true};

  const std::vector<InitializerConfig::SearchAlgorithm> search_algorithms = {
      InitializerConfig::DP, InitializerConfig::Astar};
  for (const auto& search_algorithm : search_algorithms) {
    start_time = absl::Now();
    absl::StatusOr<MotionSearchOutput> output;
    switch (search_algorithm) {
      case InitializerConfig::DP:
        output = DpSearchForRawTrajectory(input, thread_pool);
        break;
      case InitializerConfig::Astar:
        output = AstarSearchForRawTrajectory(input, thread_pool);
        break;
    }

    ASSERT_OK(output);
    QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
               << " ms consumed in searching motion";
    // EXPECT_EQ(output->motion_graph->geometry_graph(), &geom_graph);

    ParseMotionSearchOutputToMotionSearchDebugProto(
        *output, initializer_debug.mutable_motion_search_debug());

    for (int i = 0; i < output->multi_traj_candidates.size(); i++) {
      const auto& traj_info = output->multi_traj_candidates.at(i);
      SendSingleTrajectoryToCanvas(traj_info, i, 0);
    }

    CanvasDrawTrajectory(output->traj_points,
                         "search_motion_test/dp_stopline_initializer_traj");

    // Construct a rectangle to validate the output is straight. All geometry
    // state points are inside the rectangle to guarantee the straightness
    // because the width of rectangle is small enough.
    constexpr double kYTolerance = 0.1;
    const std::vector<Vec2d> tolerance_polygon_vertexes = {
        Vec2d(70.0, kYTolerance), Vec2d(24.0, kYTolerance),
        Vec2d(24.0, -kYTolerance), Vec2d(70.0, -kYTolerance)};
    const Polygon2d tolerance_polygon(tolerance_polygon_vertexes);

    for (const auto& motion : output->traj_points) {
      EXPECT_TRUE(tolerance_polygon.IsPointIn(
          Vec2d(motion.path_point().x(), motion.path_point().y())))
          << motion.DebugString();
    }

    // Execute trajecory speed constraints check.
    // TODO(zixuan) : introduce global v limit and lane speed limit for checking
    // and check v in decelerating and stopping.
    constexpr double kMaxSpeedLimit = Kph2Mps(40.0);
    for (const auto& motion : output->traj_points) {
      EXPECT_LE(motion.v(), kMaxSpeedLimit);
      EXPECT_GE(motion.v(), 0.0);
    }

    // Execute trajecory acceleration constraints check.
    constexpr double kMaxAcceleration = 2.0;
    constexpr double kMaxDeceleration = -5.0;
    for (const auto& motion : output->traj_points) {
      EXPECT_LE(motion.a(), kMaxAcceleration);
      EXPECT_GE(motion.a(), kMaxDeceleration);
    }

    // Execute trajecory length check.
    constexpr double kMinSearchResultTrajectoryLength = 0.5;
    const double search_result_trajectory_length =
        output->traj_points.back().path_point().s();
    QLOG(INFO) << "Trajectory length: " << search_result_trajectory_length;
    EXPECT_GE(search_result_trajectory_length,
              kMinSearchResultTrajectoryLength);

    // Execute trajecory end time check.
    const double search_result_trajectory_end_time =
        output->traj_points.back().relative_time();
    QLOG(INFO) << "Trajectory end time: " << search_result_trajectory_end_time;
    EXPECT_EQ(search_result_trajectory_end_time, 9.9);
  }
}

}  // namespace
}  // namespace qcraft::planner
