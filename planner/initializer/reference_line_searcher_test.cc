#include "onboard/planner/initializer/reference_line_searcher.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_cache.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/test_util.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {
namespace {
constexpr double kBuffer = 0.25;  // m.
PlannerObjectManager BuildPhantomVehicle(
    const PlannerSemanticMapManager& /*psmm*/, const Vec2d& obj_pos,
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

TEST(ReferenceLineSearcher, StationaryObjectFarAhead) {
  PlannerParamsProto planner_params = DefaultPlannerParams();
  VehicleGeometryParamsProto vehicle_geom = DefaultVehicleGeometry();
  VehicleDriveParamsProto vehicle_drive = DefaultVehicleDriveParams();
  MotionConstraintParamsProto motion_constraint_params =
      DefaultPlannerParams().motion_constraint_params();

  // Create pose.
  PoseProto sdc_pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(0.0, 0.0), 0.0, Vec2d(5.0, 0.0));
  // Plan start point.
  ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(sdc_pose);
  // Routing.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b6_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  // Drive Passage.
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            route_path.lane_paths().front(),
                                            /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false, /*override_speed_limit_mps=*/std::nullopt);
  EXPECT_TRUE(dp_or.ok());
  SendDrivePassageToCanvas(dp_or.value(), "farahead/drive_passage");
  const auto object_mgr = BuildPhantomVehicle(psmm, Vec2d(27.0, -1.0));
  DrawPlannerObjectManagerToCanvas(object_mgr, "farahead/object",
                                   vis::Color::kLightGreen);

  const SpacetimeTrajectoryManager st_traj_mgr(object_mgr.planner_objects());

  // Sampling params.
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

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  const SmoothedReferenceLineResultMap smooth_result_map;
  // Path Sl boundary.
  ASSIGN_OR_DIE(const auto path_sl_boundary,
                BuildPathBoundaryFromPose(
                    psmm, dp_or.value(), plan_start_point, vehicle_geom,
                    st_traj_mgr, lc_state, smooth_result_map,
                    /*borrow_lane_boundary=*/false,
                    /*should_smooth=*/false, /*unsafe_object_ids=*/{}));
  DrawPathSlBoundaryToCanvas(path_sl_boundary, "farahead/path_boundary");

  const std::vector<double> stop_s_vec;
  std::vector<std::map<std::string, ConstraintProto::LeadingObjectProto>>
      leading_groups;

  const GeometryFormBuilder geom_form_builder(&dp_or.value(), dp_or->end_s(),
                                              0.0);
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  for (const auto& trajectory : st_traj_mgr.trajectories()) {
    st_planner_object_traj.AddSpacetimePlannerObjectTrajectory(
        trajectory, SpacetimePlannerObjectTrajectoryReason::STATIONARY);
  }
  const std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_planner_object_traj, &vehicle_geom,
          MotionForm::kConstTimeIntervalSampleStep,
          /*stationary_object_buffer=*/kBuffer,
          /*moving_object_buffer=*/2.0 * kBuffer);
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .stop_s_vec = &stop_s_vec,
      .leading_groups = &leading_groups,
      .st_traj_mgr = &st_traj_mgr,
      .plan_start_point = &plan_start_point,
      .vehicle_geom = &vehicle_geom,
      .collision_checker = collision_checker.get(),
      .sampling_params = &sampling_params,
      .vehicle_drive = &vehicle_drive,
      .form_builder = &geom_form_builder,
      .lc_multiple_traj = false};

  GeometryGraphCache graph_cache;
  InitializerDebugProto initializer_debug;
  auto geom_graph_or = BuildCurvyGeometryGraph(
      geom_graph_builder_input, /*retry_collision_checker=*/false, &graph_cache,
      ThreadPool::DefaultPool(), &initializer_debug);
  ASSERT_OK(geom_graph_or);
  auto& geom_graph = geom_graph_or.value();

  SendGeometryGraphToCanvas(&geom_graph,
                            "farahead/curvy_geograph/before_searcher");
  EXPECT_OK(CheckGeometryGraphConnectivity(geom_graph));

  // Reference line searcher.
  ReferenceLineSearcherInput ref_line_search_input{
      .geometry_graph = &geom_graph,
      .drive_passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .initializer_params = &planner_params.initializer_params(),
      .vehicle_geom = &vehicle_geom,
      .vehicle_drive = &vehicle_drive,
      .st_planner_object_traj = &st_planner_object_traj,
  };

  const auto ref_line_output_or = SearchReferenceLine(
      ref_line_search_input, &initializer_debug, /*thread_pool=*/nullptr);
  ASSERT_OK(ref_line_output_or);
  const auto deactivate_status = DeactivateFarGeometries(
      *ref_line_output_or, path_sl_boundary, &geom_graph);
  ASSERT_OK(deactivate_status);
  SendGeometryGraphToCanvas(&geom_graph,
                            "farahead/curvy_geograph/after_searcher");
  SendPointsToCanvas(ref_line_output_or->ref_line_points,
                     "farahead/reference_line", vis::Color::kLightGreen);
  EXPECT_GT(ref_line_output_or->ref_line_points.back().x(), 30.0);
}

TEST(ReferenceLineSearcher, StationaryObjectClose) {
  PlannerParamsProto planner_params = DefaultPlannerParams();
  VehicleGeometryParamsProto vehicle_geom = DefaultVehicleGeometry();
  VehicleDriveParamsProto vehicle_drive = DefaultVehicleDriveParams();
  MotionConstraintParamsProto motion_constraint_params =
      DefaultPlannerParams().motion_constraint_params();

  // Create pose.
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(0.0, 0.0), 0.0, Vec2d(2.0, 0.0));
  // Plan start point.
  const auto plan_start_point = ConvertToTrajPointProto(sdc_pose);
  // Routing.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b6_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  // Drive Passage.
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            route_path.lane_paths().front(),
                                            /*extend_len=*/15.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false, /*override_speed_limit_mps=*/std::nullopt);
  EXPECT_TRUE(dp_or.ok());
  SendDrivePassageToCanvas(dp_or.value(), "closeahead/drive_passage");
  const auto object_mgr = BuildPhantomVehicle(psmm, Vec2d(10.0, -1.0));
  DrawPlannerObjectManagerToCanvas(object_mgr, "closeahead/object",
                                   vis::Color::kLightGreen);

  const SpacetimeTrajectoryManager st_traj_mgr(object_mgr.planner_objects());

  // Sampling params.
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

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  const SmoothedReferenceLineResultMap smooth_result_map;
  // Path Sl boundary.
  ASSIGN_OR_DIE(const auto path_sl_boundary,
                BuildPathBoundaryFromPose(
                    psmm, dp_or.value(), plan_start_point, vehicle_geom,
                    st_traj_mgr, lc_state, smooth_result_map,
                    /*borrow_lane_boundary=*/false,
                    /*should_smooth=*/false, /*unsafe_object_ids=*/{}));
  DrawPathSlBoundaryToCanvas(path_sl_boundary, "closeahead/path_boundary");

  const std::vector<double> stop_s_vec;
  std::vector<std::map<std::string, ConstraintProto::LeadingObjectProto>>
      leading_groups;

  const GeometryFormBuilder geom_form_builder(&dp_or.value(), dp_or->end_s(),
                                              0.0);
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  for (const auto& trajectory : st_traj_mgr.trajectories()) {
    st_planner_object_traj.AddSpacetimePlannerObjectTrajectory(
        trajectory, SpacetimePlannerObjectTrajectoryReason::STATIONARY);
  }
  const std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_planner_object_traj, &vehicle_geom,
          MotionForm::kConstTimeIntervalSampleStep,
          /*stationary_object_buffer=*/kBuffer,
          /*moving_object_buffer=*/2.0 * kBuffer);
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .stop_s_vec = &stop_s_vec,
      .leading_groups = &leading_groups,
      .st_traj_mgr = &st_traj_mgr,
      .plan_start_point = &plan_start_point,
      .vehicle_geom = &vehicle_geom,
      .collision_checker = collision_checker.get(),
      .sampling_params = &sampling_params,
      .vehicle_drive = &vehicle_drive,
      .form_builder = &geom_form_builder,
      .lc_multiple_traj = false};

  GeometryGraphCache graph_cache;
  InitializerDebugProto initializer_debug;
  auto geom_graph_or = BuildCurvyGeometryGraph(
      geom_graph_builder_input, /*retry_collision_checker=*/false, &graph_cache,
      ThreadPool::DefaultPool(), &initializer_debug);
  ASSERT_OK(geom_graph_or);
  auto& geom_graph = geom_graph_or.value();

  SendGeometryGraphToCanvas(&geom_graph,
                            "closeahead/curvy_geograph/before_searcher");
  EXPECT_OK(CheckGeometryGraphConnectivity(geom_graph));

  // Reference line searcher.
  ReferenceLineSearcherInput ref_line_search_input{
      .geometry_graph = &geom_graph,
      .drive_passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .initializer_params = &planner_params.initializer_params(),
      .vehicle_geom = &vehicle_geom,
      .vehicle_drive = &vehicle_drive,
      .st_planner_object_traj = &st_planner_object_traj,
  };

  const auto ref_line_output_or = SearchReferenceLine(
      ref_line_search_input, &initializer_debug, /*thread_pool=*/nullptr);
  ASSERT_OK(ref_line_output_or);
  const auto deactivate_status = DeactivateFarGeometries(
      *ref_line_output_or, path_sl_boundary, &geom_graph);
  ASSERT_OK(deactivate_status);
  SendGeometryGraphToCanvas(&geom_graph,
                            "closeahead/curvy_geograph/after_searcher");
  SendPointsToCanvas(ref_line_output_or->ref_line_points,
                     "closeahead/reference_line", vis::Color::kLightGreen);
  EXPECT_GT(ref_line_output_or->ref_line_points.back().x(), 15.0);
}

TEST(ReferenceLineSearcher, StationaryObjectFollow) {
  PlannerParamsProto planner_params = DefaultPlannerParams();
  VehicleGeometryParamsProto vehicle_geom = DefaultVehicleGeometry();
  VehicleDriveParamsProto vehicle_drive = DefaultVehicleDriveParams();
  MotionConstraintParamsProto motion_constraint_params =
      DefaultPlannerParams().motion_constraint_params();

  // Create pose.
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(30.0, 0.0), 0.0, Vec2d(2.0, 0.0));
  // Plan start point.
  const auto plan_start_point = ConvertToTrajPointProto(sdc_pose);
  // Routing.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b6_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  // Drive Passage.
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            route_path.lane_paths().front(),
                                            /*extend_len=*/15.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false, /*override_speed_limit_mps=*/std::nullopt);
  EXPECT_TRUE(dp_or.ok());
  SendDrivePassageToCanvas(dp_or.value(), "follow/drive_passage");
  const auto object_mgr = BuildPhantomVehicle(psmm, Vec2d(50.0, 0.0));
  DrawPlannerObjectManagerToCanvas(object_mgr, "follow/object",
                                   vis::Color::kLightGreen);

  const SpacetimeTrajectoryManager st_traj_mgr(object_mgr.planner_objects());

  // Sampling params.
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

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  const SmoothedReferenceLineResultMap smooth_result_map;
  // Path Sl boundary.
  ASSIGN_OR_DIE(const auto path_sl_boundary,
                BuildPathBoundaryFromPose(
                    psmm, dp_or.value(), plan_start_point, vehicle_geom,
                    st_traj_mgr, lc_state, smooth_result_map,
                    /*borrow_lane_boundary=*/false,
                    /*should_smooth=*/false, /*unsafe_object_ids=*/{}));
  DrawPathSlBoundaryToCanvas(path_sl_boundary, "follow/path_boundary");

  const std::vector<double> stop_s_vec;
  std::vector<std::map<std::string, ConstraintProto::LeadingObjectProto>>
      leading_groups;
  ConstraintProto::LeadingObjectProto leading_proto;
  leading_proto.set_traj_id("Phantom-idx0");
  leading_proto.set_reason(
      ConstraintProto::LeadingObjectProto::TRAFFIC_WAITING);
  leading_groups.emplace_back().emplace("Phantom-idx0",
                                        std::move(leading_proto));

  const GeometryFormBuilder geom_form_builder(&dp_or.value(), dp_or->end_s(),
                                              0.0);
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  for (const auto& trajectory : st_traj_mgr.trajectories()) {
    st_planner_object_traj.AddSpacetimePlannerObjectTrajectory(
        trajectory, SpacetimePlannerObjectTrajectoryReason::STATIONARY);
  }
  const std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_planner_object_traj, &vehicle_geom,
          MotionForm::kConstTimeIntervalSampleStep,
          /*stationary_object_buffer=*/kBuffer,
          /*moving_object_buffer=*/2.0 * kBuffer);
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .stop_s_vec = &stop_s_vec,
      .leading_groups = &leading_groups,
      .st_traj_mgr = &st_traj_mgr,
      .plan_start_point = &plan_start_point,
      .vehicle_geom = &vehicle_geom,
      .collision_checker = collision_checker.get(),
      .sampling_params = &sampling_params,
      .vehicle_drive = &vehicle_drive,
      .form_builder = &geom_form_builder,
      .lc_multiple_traj = false};

  GeometryGraphCache graph_cache;
  InitializerDebugProto initializer_debug;
  auto geom_graph_or = BuildCurvyGeometryGraph(
      geom_graph_builder_input, /*retry_collision_checker=*/false, &graph_cache,
      ThreadPool::DefaultPool(), &initializer_debug);
  ASSERT_OK(geom_graph_or);
  auto& geom_graph = geom_graph_or.value();

  SendGeometryGraphToCanvas(&geom_graph,
                            "follow/curvy_geograph/before_searcher");
  EXPECT_OK(CheckGeometryGraphConnectivity(geom_graph));

  // Reference line searcher.
  ReferenceLineSearcherInput ref_line_search_input{
      .geometry_graph = &geom_graph,
      .drive_passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .initializer_params = &planner_params.initializer_params(),
      .vehicle_geom = &vehicle_geom,
      .vehicle_drive = &vehicle_drive,
      .st_planner_object_traj = &st_planner_object_traj,
  };

  const auto ref_line_output_or = SearchReferenceLine(
      ref_line_search_input, &initializer_debug, /*thread_pool=*/nullptr);
  ASSERT_OK(ref_line_output_or);
  const auto deactivate_status = DeactivateFarGeometries(
      *ref_line_output_or, path_sl_boundary, &geom_graph);
  ASSERT_OK(deactivate_status);
  SendGeometryGraphToCanvas(&geom_graph,
                            "follow/curvy_geograph/after_searcher");
  SendPointsToCanvas(ref_line_output_or->ref_line_points,
                     "follow/reference_line", vis::Color::kLightGreen);
  EXPECT_LT(ref_line_output_or->ref_line_points.back().x(), 45.0);
}

}  // namespace

}  // namespace qcraft::planner
