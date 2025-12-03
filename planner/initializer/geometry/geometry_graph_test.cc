#include "onboard/planner/initializer/geometry/geometry_graph.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/async/thread_pool.h"
#include "onboard/base/macros.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_cache.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/initializer/test_util.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {
namespace {
constexpr double kBuffer = 0.25;  // m.

TEST(GeometryGraph, BuildCurvyXYGeometryGraph) {
  const auto route_result = CreateAStraightForwardRouteInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, route_result.route_sections,
          route_result.route_lane_path.lane_paths().front(),
          /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or =
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                        route_result.route_lane_path.lane_paths().front(),
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(psmm),
                        route_result.route_sections.destination(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt);

  EXPECT_TRUE(dp_or.ok());
  SendDrivePassageToCanvas(dp_or.value(), "test/straight_drive_passage");
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

  const SpacetimeTrajectoryManager st_traj_mgr{};
  const auto vehicle_geom = DefaultVehicleGeometry();
  const auto vehicle_drive = DefaultVehicleDriveParams();
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  const std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_planner_object_traj, &vehicle_geom,
          MotionForm::kConstTimeIntervalSampleStep,
          /*stationary_object_buffer=*/kBuffer,
          /*moving_object_buffer=*/2.0 * kBuffer);

  PoseProto sdc_pose;
  sdc_pose.mutable_pos_smooth()->set_x(route_result.pose.pos_smooth().x());
  sdc_pose.mutable_pos_smooth()->set_y(route_result.pose.pos_smooth().y());
  sdc_pose.set_yaw(route_result.pose.yaw());
  sdc_pose.mutable_vel_body()->set_x(0.1);

  const auto traj_point = ConvertToTrajPointProto(sdc_pose);
  const auto path_sl_boundary = CreateFakePathSlBoundary(dp_or.value());
  const std::vector<double> stop_s_vec;
  std::vector<std::map<std::string, ConstraintProto::LeadingObjectProto>>
      leading_groups;

  const GeometryFormBuilder form_builder(&dp_or.value(), dp_or->end_s(), 0.0);
  bool lc_multiple_traj = false;
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .stop_s_vec = &stop_s_vec,
      .leading_groups = &leading_groups,
      .st_traj_mgr = &st_traj_mgr,
      .plan_start_point = &traj_point,
      .vehicle_geom = &vehicle_geom,
      .collision_checker = collision_checker.get(),
      .sampling_params = &sampling_params,
      .vehicle_drive = &vehicle_drive,
      .form_builder = &form_builder,
      .lc_multiple_traj = lc_multiple_traj};
  GeometryGraphCache graph_cache;
  InitializerDebugProto initializer_debug;
  const auto geom_graph_or = BuildCurvyGeometryGraph(
      geom_graph_builder_input, /*retry_collision_checker=*/false, &graph_cache,
      ThreadPool::DefaultPool(), &initializer_debug);
  ASSERT_TRUE(geom_graph_or.ok());
  const auto& geom_graph = geom_graph_or.value();

  LOG(INFO) << geom_graph.GetStartNode().DebugString();

  GeometryGraphProto proto;
  geom_graph.ToProto(&proto);
  EXPECT_EQ(geom_graph.nodes().size(), proto.nodes().size());
  EXPECT_EQ(geom_graph.edges().size(), proto.edges().size());

  SendGeometryGraphToCanvas(&geom_graph, "test/curvy_geograph");
  EXPECT_OK(CheckGeometryGraphConnectivity(geom_graph));
  ASSERT_TRUE(geom_graph.nodes().size() > 0)
      << "Build lateral quintic polynomial geometry graph failed!";
}
}  // namespace
}  // namespace qcraft::planner
