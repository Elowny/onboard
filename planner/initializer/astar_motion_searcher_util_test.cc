#include "onboard/planner/initializer/astar_motion_searcher_util.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/async/thread_pool.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/decision/leading_groups_builder.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/geometry/geometry_graph_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_cache.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_search_util.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
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
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
namespace {

using LeadingTrajs = std::vector<std::string>;

std::vector<LeadingTrajs> BuildLeadingConfigs(
    const std::vector<LeadingGroup>& leading_groups,
    const ConstraintProto::LeadingObjectProto* blocking_static_traj) {
  SCOPED_QTRACE("BuildLeadingConfigs");

  // Construct leading config according to leading groups.
  std::vector<LeadingTrajs> leading_configs;
  leading_configs.reserve(leading_groups.size());
  for (const auto& leading_group : leading_groups) {
    auto& leading_trajs = leading_configs.emplace_back();
    for (const auto& [traj_id, _] : leading_group) {
      leading_trajs.push_back(traj_id);
    }
  }

  if (blocking_static_traj != nullptr) {
    // Add non-stalled blocking static trajectory to all groups.
    for (auto& leading_config : leading_configs) {
      leading_config.push_back(blocking_static_traj->traj_id());
    }
  }

  return leading_configs;
}

class AstarMotionSearcherUtilTest : public testing::Test {
 protected:
  absl::StatusOr<XYGeometryGraph> geom_graph_or_;
  XYGeometryGraph geom_graph_;
  GeometryGraphCache graph_cache_;
  ApolloTrajectoryPointProto start_point_;
  MotionConstraintParamsProto motion_constraint_params_;
  ml::captain_net::CaptainNetOutput captain_net_output_;
  std::vector<Vec2d> reference_line_points_;
  ml::captain_net::CaptainNetOutput captain_net_output_empty_;
  bool is_lane_change_;
  bool eval_safety_;
  SpacetimeTrajectoryManager st_traj_mgr_;
  InitializerConfig initializer_params_;
  std::vector<double> stop_s_vec_;
  std::vector<LeadingGroup> leading_groups_;
  SpacetimePlannerObjectTrajectories st_planner_object_traj_;
  absl::StatusOr<DrivePassage> drive_passages_or_;
  DrivePassage drive_passages_;
  PathSlBoundary path_sl_boundary_;
  VehicleGeometryParamsProto vehicle_geom_;
  GeometryFormBuilder form_builder_;
  std::unique_ptr<CollisionChecker> collision_checker_;
  PlannerParamsProto planner_params_;
  InitializerDebugProto initializer_debug_;
  double traj_horizon_;

  void SetUp() override {
    const double k_buffer = 0.25;

    FLAGS_planner_initializer_debug_level = 2;

    // Construct sdc pose.
    PoseProto sdc_pose;
    sdc_pose.mutable_pos_smooth()->set_x(30.0);
    sdc_pose.mutable_pos_smooth()->set_y(0.0);
    sdc_pose.set_yaw(0.0);
    sdc_pose.mutable_vel_body()->set_x(5.0);
    start_point_ = ConvertToTrajPointProto(sdc_pose);

    // Load default planner params.
    planner_params_ = DefaultPlannerParams();
    planner_params_.mutable_initializer_params()->set_search_algorithm(
        InitializerConfig::Astar);

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
    drive_passages_or_ = BuildDrivePassage(
        psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
        route_path.lane_paths().front(),
        /*anchor_point=*/mapping::LanePoint(),
        route_sections.planning_horizon(psmm), route_sections.destination(),
        /*all_lanes_virtual=*/false,
        /*override_speed_limit_mps=*/std::nullopt);
    ASSERT_TRUE(drive_passages_or_.ok() && !drive_passages_or_.value().empty());
    QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
               << " ms consumed in building DrivePassage.";
    SendDrivePassageToCanvas(drive_passages_or_.value(),
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

    vehicle_geom_ = DefaultVehicleGeometry();
    collision_checker_ = static_cast<std::unique_ptr<CollisionChecker>>(
        std::make_unique<BoxGroupCollisionChecker>(
            &st_planner_object_traj_, &vehicle_geom_,
            MotionForm::kConstTimeIntervalSampleStep, k_buffer, k_buffer));

    path_sl_boundary_ = CreateFakePathSlBoundary(drive_passages_or_.value());
    DrawPathSlBoundaryToCanvas(path_sl_boundary_,
                               "search_motion_test/path_boundary");

    // Build stopline constraint.
    HalfPlane halfplane(Vec2d(70.0, -5.25), Vec2d(70.0, 9.75));
    stop_s_vec_.push_back(40.0 - vehicle_geom_.front_edge_to_center());

    leading_groups_.push_back({});

    const auto vehicle_drive = DefaultVehicleDriveParams();
    const bool lc_multiple_traj = false;
    form_builder_ = GeometryFormBuilder(&drive_passages_or_.value(),
                                        drive_passages_or_->end_s(), 0.0);
    const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
        .passage = &drive_passages_or_.value(),
        .sl_boundary = &path_sl_boundary_,
        .stop_s_vec = &stop_s_vec_,
        .leading_groups = &leading_groups_,
        .st_traj_mgr = &st_traj_mgr_,
        .plan_start_point = &start_point_,
        .vehicle_geom = &vehicle_geom_,
        .collision_checker = collision_checker_.get(),
        .sampling_params = &sampling_params,
        .vehicle_drive = &vehicle_drive,
        .form_builder = &form_builder_,
        .lc_multiple_traj = lc_multiple_traj};
    ThreadPool* thread_pool = ThreadPool::DefaultPool();
    start_time = absl::Now();
    geom_graph_or_ = BuildCurvyGeometryGraph(
        geom_graph_builder_input, /*retry_collision_checker=*/false,
        &graph_cache_, thread_pool, &initializer_debug_);
    ASSERT_TRUE(geom_graph_or_.ok());
    geom_graph_ = geom_graph_or_.value();
    QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
               << " ms consumed in building GeometryGraph (without the time of "
                  "building inputs)";
    SendGeometryGraphToCanvas(&geom_graph_,
                              "search_motion_test/stopline_geom_graph");

    motion_constraint_params_ = planner_params_.motion_constraint_params();
    is_lane_change_ = false;
    eval_safety_ = false;
    initializer_params_ = planner_params_.initializer_params();
    drive_passages_ = drive_passages_or_.value();

    // Construct captainnet trajectory
    std::vector<ApolloTrajectoryPointProto> capnet_ref_traj_points;
    for (int i = 0; i < 11; ++i) {
      PoseProto pose;
      pose.mutable_pos_smooth()->set_x(0.0 + 1.0 * i);
      pose.mutable_pos_smooth()->set_y(0.0);
      auto traj_point = ConvertToTrajPointProto(pose);
      traj_point.set_relative_time(0.0 + 1.0 * i);
      capnet_ref_traj_points.push_back(traj_point);
    }
    captain_net_output_.traj_points = capnet_ref_traj_points;
    captain_net_output_empty_.traj_points = {};

    const int traj_steps = initializer_params_.traj_steps();
    traj_horizon_ = (traj_steps - 1) * kTrajectoryTimeStep;
  }
};

TEST_F(AstarMotionSearcherUtilTest, CreateNodeKey) {
  const auto& nodes_layers = geom_graph_.nodes_layers();
  const int node_idx = 0;
  const double v = 1.0;
  const double t = 2.0;
  constexpr double kAstarVelocityResolutionTest = 3.0;
  constexpr double kAstarTimeResolutionTest = 2.5;
  const auto key = CreateNodeKey(v, t, nodes_layers[0][node_idx]);
  const int v_index = RoundToInt(v / kAstarVelocityResolutionTest);
  const int t_index = RoundToInt(t / kAstarTimeResolutionTest);
  const int v_dir = v_index >= 0 ? 1 : 0;
  const int expect_key = nodes_layers[0][node_idx].value() + (v_index << 10) +
                         (t_index << 20) + (v_dir << 30);
  EXPECT_EQ(key, expect_key);
}

TEST_F(AstarMotionSearcherUtilTest, CreateNode) {
  const auto& nodes_layers = geom_graph_.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  const MotionState sdc_motion =
      PrepareStartMotionNode(geom_graph_, nodes_layers[0], start_point_,
                             &start_node_idx_on_first_layer);
  const int key = CreateNodeKey(sdc_motion.v, sdc_motion.t,
                                nodes_layers[0][start_node_idx_on_first_layer]);
  const auto node = CreateNode(sdc_motion, key,
                               nodes_layers[0][start_node_idx_on_first_layer]);
  EXPECT_EQ(node.layer, 0);
  EXPECT_EQ(node.cost_g, 0.0);
  EXPECT_EQ(node.cost_h, 0.0);
  EXPECT_EQ(node.motion, nullptr);
  EXPECT_EQ(node.state.xy, sdc_motion.xy);
  EXPECT_EQ(node.state.s, sdc_motion.s);
  EXPECT_EQ(node.state.v, sdc_motion.v);
  EXPECT_EQ(node.state.t, sdc_motion.t);
}

TEST_F(AstarMotionSearcherUtilTest, ComputeCaptainNetInspiredHeuristicCost) {
  const auto& nodes_layers = geom_graph_.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  MotionState sdc_motion =
      PrepareStartMotionNode(geom_graph_, nodes_layers[0], start_point_,
                             &start_node_idx_on_first_layer);
  sdc_motion.t = 1.0;
  const int key = CreateNodeKey(sdc_motion.v, sdc_motion.t,
                                nodes_layers[0][start_node_idx_on_first_layer]);
  const auto node = CreateNode(sdc_motion, key,
                               nodes_layers[0][start_node_idx_on_first_layer]);
  const double cost = ComputeCaptainNetInspiredHeuristicCost(
      traj_horizon_, node, captain_net_output_.traj_points);
  // node.x=30.0, node.y=0.0, node.t=0.0
  EXPECT_EQ(cost, 266.0);  // 38.0 when k=1
}

TEST_F(AstarMotionSearcherUtilTest, ComputeReferenceLineInspiredHeuristicCost) {
  const auto& nodes_layers = geom_graph_.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  MotionState sdc_motion =
      PrepareStartMotionNode(geom_graph_, nodes_layers[0], start_point_,
                             &start_node_idx_on_first_layer);
  sdc_motion.t = 1.0;
  const int key = CreateNodeKey(sdc_motion.v, sdc_motion.t,
                                nodes_layers[0][start_node_idx_on_first_layer]);
  const auto node = CreateNode(sdc_motion, key,
                               nodes_layers[0][start_node_idx_on_first_layer]);
  const auto leading_traj_config = BuildLeadingConfigs(leading_groups_, {});
  auto reference_speed_table = std::make_unique<RefSpeedTable>(
      st_traj_mgr_, leading_traj_config[0], drive_passages_, stop_s_vec_);
  const double cost = ComputeReferenceLineInspiredHeuristicCost(
      node, reference_line_points_, reference_speed_table.get());
  // node.x=30.0, node.y=0.0, node.t=0.0
  EXPECT_EQ(cost, 0.0);
}

TEST_F(AstarMotionSearcherUtilTest, ComputeHeuristicCost) {
  const auto& nodes_layers = geom_graph_.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  MotionState sdc_motion =
      PrepareStartMotionNode(geom_graph_, nodes_layers[0], start_point_,
                             &start_node_idx_on_first_layer);
  sdc_motion.t = 1.0;
  const int key = CreateNodeKey(sdc_motion.v, sdc_motion.t,
                                nodes_layers[0][start_node_idx_on_first_layer]);
  const auto node = CreateNode(sdc_motion, key,
                               nodes_layers[0][start_node_idx_on_first_layer]);
  const auto leading_traj_config = BuildLeadingConfigs(leading_groups_, {});
  auto reference_speed_table = std::make_unique<RefSpeedTable>(
      st_traj_mgr_, leading_traj_config[0], drive_passages_, stop_s_vec_);
  bool is_run_model_l4 = true;
  const double cost = ComputeHeuristicCost(
      traj_horizon_, node, reference_line_points_, reference_speed_table.get(),
      captain_net_output_.traj_points, is_run_model_l4);
  // node.x=30.0, node.y=0.0, node.t=0.0
  EXPECT_EQ(cost, 266.0);  // 38.0 when k=1
}

TEST_F(AstarMotionSearcherUtilTest, SampleMotionForms) {
  const auto& nodes_layers = geom_graph_.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  const MotionState sdc_motion =
      PrepareStartMotionNode(geom_graph_, nodes_layers[0], start_point_,
                             &start_node_idx_on_first_layer);
  const int key = CreateNodeKey(sdc_motion.v, sdc_motion.t,
                                nodes_layers[0][start_node_idx_on_first_layer]);
  const auto node = CreateNode(sdc_motion, key,
                               nodes_layers[0][start_node_idx_on_first_layer]);
  const auto outgoing_edge_idxs = geom_graph_.GetOutgoingEdges(node.geom_index);
  const auto geom_edge = geom_graph_.GetEdge(outgoing_edge_idxs[0]);
  const bool sample_const_v = false;
  const auto acc_samples =
      SampleMotionForms(traj_horizon_, node, geom_edge,
                        motion_constraint_params_, sample_const_v);
  EXPECT_GT(acc_samples.size(), 0);
  // Check any motion form
  auto acc_to_check = acc_samples.begin();
  const auto& motion_to_check = std::make_shared<ConstAccelMotion>(
      traj_horizon_, sdc_motion.v, *acc_to_check, geom_edge.geometry);
  const auto motion_start = motion_to_check->GetStartMotionState();
  EXPECT_EQ(motion_start.xy, sdc_motion.xy);
  EXPECT_EQ(motion_start.s, sdc_motion.s);
  EXPECT_EQ(motion_start.v, sdc_motion.v);
  EXPECT_EQ(motion_start.t, sdc_motion.t);
}

TEST_F(AstarMotionSearcherUtilTest, ProcessMotionFormsWithCache) {
  const auto& nodes_layers = geom_graph_.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  const MotionState sdc_motion =
      PrepareStartMotionNode(geom_graph_, nodes_layers[0], start_point_,
                             &start_node_idx_on_first_layer);
  const int key = CreateNodeKey(sdc_motion.v, sdc_motion.t,
                                nodes_layers[0][start_node_idx_on_first_layer]);
  const auto node = CreateNode(sdc_motion, key,
                               nodes_layers[0][start_node_idx_on_first_layer]);
  const auto outgoing_edge_idxs = geom_graph_.GetOutgoingEdges(node.geom_index);
  const auto geom_edge = geom_graph_.GetEdge(outgoing_edge_idxs[0]);

  const bool sample_const_v = false;
  const auto acc_samples =
      SampleMotionForms(traj_horizon_, node, geom_edge,
                        motion_constraint_params_, sample_const_v);

  // Construct cost provider
  const auto leading_traj_config = BuildLeadingConfigs(leading_groups_, {});
  const int leading_group_idx = 0;
  const double geom_graph_max_s = geom_graph_.GetMaxAccumulatedS();
  const double speed_max_s =
      std::max(start_point_.v(), kMinSpeedForFinalCost) * traj_horizon_;
  const double leading_obj_min_s = GetLeadingObjectsEndMinS(
      st_traj_mgr_, drive_passages_, leading_traj_config[leading_group_idx],
      vehicle_geom_.front_edge_to_center());
  const double max_accumulated_s =
      Min(leading_obj_min_s, geom_graph_max_s, speed_max_s);
  const auto ref_speed_table = std::make_unique<RefSpeedTable>(
      st_traj_mgr_, leading_traj_config[leading_group_idx], drive_passages_,
      stop_s_vec_);
  const bool able_to_overtake_leading_behind = true;
  const auto cost_provider = static_cast<std::unique_ptr<CostProviderBase>>(
      std::make_unique<AstarCostProvider>(
          drive_passages_, initializer_params_, motion_constraint_params_,
          stop_s_vec_, st_traj_mgr_, leading_traj_config, leading_group_idx,
          able_to_overtake_leading_behind, vehicle_geom_,
          collision_checker_.get(), &path_sl_boundary_, ref_speed_table.get(),
          &captain_net_output_empty_, is_lane_change_, max_accumulated_s));

  absl::Mutex my_mutex;
  OpenPQ open_pq;
  OpenMap open_map;
  CloseMap close_map;
  const bool is_run_model_l4 = true;
  AstarGraphCache cost_cache;
  std::vector<AstarCacheInfo> new_caches_container;
  ProcessMotionFormsWithCache(
      traj_horizon_, node, geom_edge, acc_samples, *cost_provider,
      reference_line_points_, ref_speed_table.get(),
      captain_net_output_empty_.traj_points, &open_pq, &open_map, close_map,
      cost_cache, &new_caches_container, is_run_model_l4, &my_mutex);
  EXPECT_GT(open_pq.size(), 0);
  EXPECT_GT(open_map.size(), 0);
  const auto node_to_check = open_pq.top();
  EXPECT_EQ(node_to_check->parent, key);
}

TEST_F(AstarMotionSearcherUtilTest, ProcessMotionFormsWithoutCache) {
  const auto& nodes_layers = geom_graph_.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  const MotionState sdc_motion =
      PrepareStartMotionNode(geom_graph_, nodes_layers[0], start_point_,
                             &start_node_idx_on_first_layer);
  const int key = CreateNodeKey(sdc_motion.v, sdc_motion.t,
                                nodes_layers[0][start_node_idx_on_first_layer]);
  const auto node = CreateNode(sdc_motion, key,
                               nodes_layers[0][start_node_idx_on_first_layer]);
  const auto outgoing_edge_idxs = geom_graph_.GetOutgoingEdges(node.geom_index);
  const auto geom_edge = geom_graph_.GetEdge(outgoing_edge_idxs[0]);

  const bool sample_const_v = false;
  const auto acc_samples =
      SampleMotionForms(traj_horizon_, node, geom_edge,
                        motion_constraint_params_, sample_const_v);

  // Construct cost provider
  const auto leading_traj_config = BuildLeadingConfigs(leading_groups_, {});
  const int leading_group_idx = 0;
  const double geom_graph_max_s = geom_graph_.GetMaxAccumulatedS();
  const double speed_max_s =
      std::max(start_point_.v(), kMinSpeedForFinalCost) * traj_horizon_;
  const double leading_obj_min_s = GetLeadingObjectsEndMinS(
      st_traj_mgr_, drive_passages_, leading_traj_config[leading_group_idx],
      vehicle_geom_.front_edge_to_center());
  const double max_accumulated_s =
      Min(leading_obj_min_s, geom_graph_max_s, speed_max_s);
  const auto ref_speed_table = std::make_unique<RefSpeedTable>(
      st_traj_mgr_, leading_traj_config[leading_group_idx], drive_passages_,
      stop_s_vec_);
  const bool able_to_overtake_leading_behind = true;
  const auto cost_provider = static_cast<std::unique_ptr<CostProviderBase>>(
      std::make_unique<AstarCostProvider>(
          drive_passages_, initializer_params_, motion_constraint_params_,
          stop_s_vec_, st_traj_mgr_, leading_traj_config, leading_group_idx,
          able_to_overtake_leading_behind, vehicle_geom_,
          collision_checker_.get(), &path_sl_boundary_, ref_speed_table.get(),
          &captain_net_output_empty_, is_lane_change_, max_accumulated_s));

  absl::Mutex my_mutex;
  OpenPQ open_pq;
  OpenMap open_map;
  CloseMap close_map;
  const bool is_run_model_l4 = true;
  ProcessMotionFormsWithoutCache(
      traj_horizon_, node, geom_edge, acc_samples, *cost_provider,
      reference_line_points_, ref_speed_table.get(),
      captain_net_output_empty_.traj_points, &open_pq, &open_map, close_map,
      is_run_model_l4, &my_mutex);
  EXPECT_GT(open_pq.size(), 0);
  EXPECT_GT(open_map.size(), 0);
  const auto node_to_check = open_pq.top();
  EXPECT_EQ(node_to_check->parent, key);
}

TEST_F(AstarMotionSearcherUtilTest, SearchFailTryStationaryMotion) {
  const auto& nodes_layers = geom_graph_.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  const MotionState sdc_motion =
      PrepareStartMotionNode(geom_graph_, nodes_layers[0], start_point_,
                             &start_node_idx_on_first_layer);
  const int key = CreateNodeKey(sdc_motion.v, sdc_motion.t,
                                nodes_layers[0][start_node_idx_on_first_layer]);
  const auto node = CreateNode(sdc_motion, key,
                               nodes_layers[0][start_node_idx_on_first_layer]);

  // Construct cost provider
  const auto leading_traj_config = BuildLeadingConfigs(leading_groups_, {});
  const int leading_group_idx = 0;
  const double geom_graph_max_s = geom_graph_.GetMaxAccumulatedS();
  const double speed_max_s =
      std::max(start_point_.v(), kMinSpeedForFinalCost) * traj_horizon_;
  const double leading_obj_min_s = GetLeadingObjectsEndMinS(
      st_traj_mgr_, drive_passages_, leading_traj_config[leading_group_idx],
      vehicle_geom_.front_edge_to_center());
  const double max_accumulated_s =
      Min(leading_obj_min_s, geom_graph_max_s, speed_max_s);
  const auto ref_speed_table = std::make_unique<RefSpeedTable>(
      st_traj_mgr_, leading_traj_config[leading_group_idx], drive_passages_,
      stop_s_vec_);
  const bool able_to_overtake_leading_behind = true;
  const auto cost_provider = static_cast<std::unique_ptr<CostProviderBase>>(
      std::make_unique<AstarCostProvider>(
          drive_passages_, initializer_params_, motion_constraint_params_,
          stop_s_vec_, st_traj_mgr_, leading_traj_config, leading_group_idx,
          able_to_overtake_leading_behind, vehicle_geom_,
          collision_checker_.get(), &path_sl_boundary_, ref_speed_table.get(),
          &captain_net_output_empty_, is_lane_change_, max_accumulated_s));
  const bool is_run_model_l4 = true;
  const auto end_node = SearchFailTryStationaryMotion(
      traj_horizon_, *cost_provider, node, reference_line_points_,
      ref_speed_table.get(), captain_net_output_.traj_points, is_run_model_l4);
  EXPECT_EQ(end_node.parent, key);
  const auto motion_end_state = end_node.motion->GetEndMotionState();
  EXPECT_EQ(motion_end_state.xy, sdc_motion.xy);
  EXPECT_EQ(motion_end_state.t, traj_horizon_);
}

TEST_F(AstarMotionSearcherUtilTest, SatisfyEndCondition) {
  const auto& nodes_layers = geom_graph_.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  const MotionState sdc_motion =
      PrepareStartMotionNode(geom_graph_, nodes_layers[0], start_point_,
                             &start_node_idx_on_first_layer);
  const int key = CreateNodeKey(sdc_motion.v, sdc_motion.t,
                                nodes_layers[0][start_node_idx_on_first_layer]);
  auto node = CreateNode(sdc_motion, key,
                         nodes_layers[0][start_node_idx_on_first_layer]);
  bool should_end = SatisfyEndCondition(traj_horizon_, node);
  EXPECT_EQ(should_end, false);
  node.state.t = 11.0;
  should_end = SatisfyEndCondition(traj_horizon_, node);
  EXPECT_EQ(should_end, true);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
