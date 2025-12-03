#include "onboard/planner/optimization/ddp/trajectory_optimizer.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"

#include "gtest/gtest.h"

#include "onboard/async/future.h"
#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/clock.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/end_of_path_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_input.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_output.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/est_planner_debug.pb.h"
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
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DECLARE_bool(use_static_boundary_cost_in_vision_map);
DECLARE_bool(msd_static_boundary_cost_v2);
DECLARE_int32(static_boundary_canvas_level);
namespace qcraft {
namespace planner {
namespace {

struct EgoPose {
  Vec2d pos;     // {x, y}.
  double theta;  // rad.
  double v;      // m/s.
};

PlannerParamsProto planner_params = DefaultPlannerParams();
VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();
VehicleDriveParamsProto veh_drive_params = DefaultVehicleDriveParams();

TEST(TrajectoryOptimizerTest, LaneChangeLeft) {
  EgoPose ego({.pos = {20.0, 0.0}, .theta = 0.0, .v = 10.0});
  const double cos_theta = std::cos(ego.theta);
  const double sin_theta = std::sin(ego.theta);
  const std::string test = "lane_change_left";

  TrajectoryOptimizerInput input;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego.pos.x() +
                      k * kTrajectoryTimeStep * ego.v * cos_theta);
    path_point->set_y(ego.pos.y() +
                      k * kTrajectoryTimeStep * ego.v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego.theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego.v * k * kTrajectoryTimeStep);
    pt.set_v(ego.v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }
  input.trajectory = traj_points;

  // For space_time object and drive_passage.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego.pos.x());
  pose.mutable_pos_smooth()->set_y(ego.pos.y());
  pose.set_yaw(ego.theta);
  pose.mutable_vel_smooth()->set_x(ego.v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego.v * sin_theta);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const auto target_lane_path =
      BackwardExtendTargetAlignedRouteLanePath(
          psmm, !route_path.transitions().front().lc_left,
          route_path.lane_paths()[1].front(), route_path.lane_paths().front())
          .Connect(smm, route_path.lane_paths()[1]);
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            target_lane_path,
                                            kDrivePassageKeepBehindLength);
  EXPECT_OK(backward_extended_lane_path);
  auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, target_lane_path,
      *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  input.drive_passage = &*drive_passage;

  // Build spacetime object manager.
  SpacetimeTrajectoryManager st_traj_mgr;
  SpacetimePlannerObjectTrajectories st_planner_object_traj;

  input.st_traj_mgr = &st_traj_mgr;
  input.st_planner_object_traj = &st_planner_object_traj;

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      psmm, *drive_passage, input.trajectory.front(), veh_geo_params,
      st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false, /*unsafe_object_ids=*/{});
  QCHECK(path_sl_boundary.ok());
  input.path_sl_boundary = &*path_sl_boundary;

  SendDrivePassageToCanvas(
      *drive_passage,
      absl::StrCat("trajectory_optimizer_test/", test, "/drive_passage"));

  EstPlannerDebugProto debug_proto;

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(
      std::move(end_of_path_boundary_constraint).value());
  input.constraint_mgr = &constraint_mgr;

  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs;
  input.leading_trajs = &leading_trajs;

  input.plan_start_point = input.trajectory.front();
  auto fake_time = Clock::Now();
  input.plan_start_time = fake_time;
  input.planner_semantic_map_mgr = &psmm;
  std::vector<TrajectoryPoint> captain_traj = {};
  input.captain_trajectory = &captain_traj;

  input.trajectory_optimizer_params =
      &planner_params.trajectory_optimizer_params();
  input.motion_constraint_params = &planner_params.motion_constraint_params();
  input.planner_functions_params = &planner_params.planner_functions_params();
  input.vehicle_models_params = &planner_params.vehicle_models_params();
  input.veh_geo_params = &veh_geo_params;
  input.veh_drive_params = &veh_drive_params;

  ThreadPool* tp = ThreadPool::DefaultPool();
  auto output =
      OptimizeTrajectory(input, debug_proto.mutable_trajectory_optimizer(),
                         /*charts_data=*/nullptr, tp);

  if (output.ok()) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&input](int index) {
              return Vec2d(input.trajectory[index].path_point().x(),
                           input.trajectory[index].path_point().y());
            },
            input.trajectory.size()),
        vis::Color::kLightRed,
        absl::StrCat("trajectory_optimizer_test/", test, "/input"));

    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&output](int index) {
              return Vec2d(output->trajectory_proto[index].path_point().x(),
                           output->trajectory_proto[index].path_point().y());
            },
            output->trajectory_proto.size()),
        vis::Color::kLightBlue,
        absl::StrCat("trajectory_optimizer_test/", test, "/output"));
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

TEST(TrajectoryOptimizerTest, NearRightCurb) {
  EgoPose ego({.pos = {0.0, -3.8}, .theta = 0.0, .v = 10.0});
  const double cos_theta = std::cos(ego.theta);
  const double sin_theta = std::sin(ego.theta);
  const std::string test = "near_right_curb";

  TrajectoryOptimizerInput input;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego.pos.x() +
                      k * kTrajectoryTimeStep * ego.v * cos_theta);
    path_point->set_y(ego.pos.y() +
                      k * kTrajectoryTimeStep * ego.v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego.theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego.v * k * kTrajectoryTimeStep);
    pt.set_v(ego.v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }

  input.trajectory = traj_points;

  // For space_time object and drive_passage.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego.pos.x());
  pose.mutable_pos_smooth()->set_y(ego.pos.y());
  pose.set_yaw(ego.theta);
  pose.mutable_vel_smooth()->set_x(ego.v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego.v * sin_theta);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            route_path.lane_paths().front(),
                                            kDrivePassageKeepBehindLength);
  EXPECT_OK(backward_extended_lane_path);
  auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  input.drive_passage = &*drive_passage;

  // Build spacetime object manager.
  SpacetimeTrajectoryManager st_traj_mgr;
  SpacetimePlannerObjectTrajectories st_planner_object_traj;

  input.st_traj_mgr = &st_traj_mgr;
  input.st_planner_object_traj = &st_planner_object_traj;

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      psmm, *drive_passage, input.trajectory.front(), veh_geo_params,
      st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false, /*unsafe_object_ids=*/{});
  QCHECK(path_sl_boundary.ok());
  input.path_sl_boundary = &*path_sl_boundary;

  SendDrivePassageToCanvas(
      *drive_passage,
      absl::StrCat("trajectory_optimizer_test/", test, "/drive_passage"));

  EstPlannerDebugProto debug_proto;

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(
      std::move(end_of_path_boundary_constraint).value());
  input.constraint_mgr = &constraint_mgr;

  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs;
  input.leading_trajs = &leading_trajs;

  input.plan_start_point = input.trajectory.front();
  auto fake_time = Clock::Now();
  input.plan_start_time = fake_time;
  input.planner_semantic_map_mgr = &psmm;
  std::vector<TrajectoryPoint> captain_traj = {};
  input.captain_trajectory = &captain_traj;

  input.trajectory_optimizer_params =
      &planner_params.trajectory_optimizer_params();
  input.motion_constraint_params = &planner_params.motion_constraint_params();
  input.planner_functions_params = &planner_params.planner_functions_params();
  input.vehicle_models_params = &planner_params.vehicle_models_params();
  input.veh_geo_params = &veh_geo_params;
  input.veh_drive_params = &veh_drive_params;

  FLAGS_static_boundary_canvas_level = 1;

  ThreadPool* tp = ThreadPool::DefaultPool();
  auto output =
      OptimizeTrajectory(input, debug_proto.mutable_trajectory_optimizer(),
                         /*charts_data=*/nullptr, tp);
  EXPECT_OK(output);

  FLAGS_msd_static_boundary_cost_v2 = false;
  auto output2 =
      OptimizeTrajectory(input, debug_proto.mutable_trajectory_optimizer(),
                         /*charts_data=*/nullptr, tp);
  EXPECT_OK(output2);

  FLAGS_use_static_boundary_cost_in_vision_map = true;
  SetMap("dojo");
  auto loader = mapping::v2::SemanticMapLoader::MakeShared();
  const auto smm_v2 = loader->PreloadWholeMap().Get();
  smm_v2->SetDataSource(OnlineMapProto_DataSource_QCRAFT_VISIONMAP);
  const auto smmsi =
      mapping::v2::SemanticMapMultilevelSpatialIndex::MakeShared(smm_v2);
  static PlannerSemanticMapManager* psmm_online_map =
      new PlannerSemanticMapManager(smmsi);
  psmm_online_map->UpdateCoordinateConverter(
      CoordinateConverter::FromMap("dojo"));
  std::ignore = psmm_online_map->BuildSemanticMapInfo(/*ThreadPool=*/nullptr);
  input.planner_semantic_map_mgr = psmm_online_map;
  auto output3 =
      OptimizeTrajectory(input, debug_proto.mutable_trajectory_optimizer(),
                         /*charts_data=*/nullptr, tp);
  EXPECT_OK(output3);

  if (output.ok()) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&input](int index) {
              return Vec2d(input.trajectory[index].path_point().x(),
                           input.trajectory[index].path_point().y());
            },
            input.trajectory.size()),
        vis::Color::kLightRed,
        absl::StrCat("trajectory_optimizer_test/", test, "/input"));

    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&output](int index) {
              return Vec2d(output->trajectory_proto[index].path_point().x(),
                           output->trajectory_proto[index].path_point().y());
            },
            output->trajectory_proto.size()),
        vis::Color::kLightBlue,
        absl::StrCat("trajectory_optimizer_test/", test, "/output"));
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

TEST(TrajectoryOptimizerTest, NearStopLine) {
  EgoPose ego({.pos = {0.0, 0.0}, .theta = 0.0, .v = 10.0});
  const double cos_theta = std::cos(ego.theta);
  const double sin_theta = std::sin(ego.theta);
  const std::string test = "near_stop_line";

  TrajectoryOptimizerInput input;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego.pos.x() +
                      k * kTrajectoryTimeStep * ego.v * cos_theta);
    path_point->set_y(ego.pos.y() +
                      k * kTrajectoryTimeStep * ego.v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego.theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego.v * k * kTrajectoryTimeStep);
    pt.set_v(ego.v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }
  input.trajectory = traj_points;

  // For space_time object and drive_passage.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego.pos.x());
  pose.mutable_pos_smooth()->set_y(ego.pos.y());
  pose.set_yaw(ego.theta);
  pose.mutable_vel_smooth()->set_x(ego.v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego.v * sin_theta);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            route_path.lane_paths().front(),
                                            kDrivePassageKeepBehindLength);
  EXPECT_OK(backward_extended_lane_path);
  auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  input.drive_passage = &*drive_passage;

  // Build spacetime object manager.
  SpacetimeTrajectoryManager st_traj_mgr;
  SpacetimePlannerObjectTrajectories st_planner_object_traj;

  input.st_traj_mgr = &st_traj_mgr;
  input.st_planner_object_traj = &st_planner_object_traj;

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      psmm, *drive_passage, input.trajectory.front(), veh_geo_params,
      st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false, /*unsafe_object_ids=*/{});
  QCHECK(path_sl_boundary.ok());
  input.path_sl_boundary = &*path_sl_boundary;

  SendDrivePassageToCanvas(
      *drive_passage,
      absl::StrCat("trajectory_optimizer_test/", test, "/drive_passage"));

  EstPlannerDebugProto debug_proto;

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(
      std::move(end_of_path_boundary_constraint).value());
  ConstraintProto::StopLineProto stop_line;
  HalfPlane halfplane(Vec2d(30.0, -1.8), Vec2d(30.0, -5.3));
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_s(30.0 - veh_geo_params.front_edge_to_center());
  stop_line.set_time(0.0);
  stop_line.mutable_source()->mutable_crosswalk()->set_crosswalk_id(0);
  constraint_mgr.AddStopLine(std::move(stop_line));
  input.constraint_mgr = &constraint_mgr;

  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs;
  input.leading_trajs = &leading_trajs;

  input.plan_start_point = input.trajectory.front();
  auto fake_time = Clock::Now();
  input.plan_start_time = fake_time;
  input.planner_semantic_map_mgr = &psmm;
  std::vector<TrajectoryPoint> captain_traj = {};
  input.captain_trajectory = &captain_traj;

  input.trajectory_optimizer_params =
      &planner_params.trajectory_optimizer_params();
  input.motion_constraint_params = &planner_params.motion_constraint_params();
  input.planner_functions_params = &planner_params.planner_functions_params();
  input.vehicle_models_params = &planner_params.vehicle_models_params();
  input.veh_geo_params = &veh_geo_params;
  input.veh_drive_params = &veh_drive_params;

  ThreadPool* tp = ThreadPool::DefaultPool();
  auto output =
      OptimizeTrajectory(input, debug_proto.mutable_trajectory_optimizer(),
                         /*charts_data=*/nullptr, tp);

  if (output.ok()) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&input](int index) {
              return Vec2d(input.trajectory[index].path_point().x(),
                           input.trajectory[index].path_point().y());
            },
            input.trajectory.size()),
        vis::Color::kLightRed,
        absl::StrCat("trajectory_optimizer_test/", test, "/input"));

    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&output](int index) {
              return Vec2d(output->trajectory_proto[index].path_point().x(),
                           output->trajectory_proto[index].path_point().y());
            },
            output->trajectory_proto.size()),
        vis::Color::kLightBlue,
        absl::StrCat("trajectory_optimizer_test/", test, "/output"));
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

TEST(TrajectoryOptimizerTest, NearSpeedRegionAndStopLine) {
  EgoPose ego({.pos = {0.0, 0.0}, .theta = 0.0, .v = 10.0});
  const double cos_theta = std::cos(ego.theta);
  const double sin_theta = std::sin(ego.theta);
  const std::string test = "near_speed_bump_stop_line";

  TrajectoryOptimizerInput input;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego.pos.x() +
                      k * kTrajectoryTimeStep * ego.v * cos_theta);
    path_point->set_y(ego.pos.y() +
                      k * kTrajectoryTimeStep * ego.v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego.theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego.v * k * kTrajectoryTimeStep);
    pt.set_v(ego.v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }

  input.trajectory = traj_points;

  // For space_time object and drive_passage.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego.pos.x());
  pose.mutable_pos_smooth()->set_y(ego.pos.y());
  pose.set_yaw(ego.theta);
  pose.mutable_vel_smooth()->set_x(ego.v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego.v * sin_theta);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            route_path.lane_paths().front(),
                                            kDrivePassageKeepBehindLength);
  EXPECT_OK(backward_extended_lane_path);
  auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  input.drive_passage = &*drive_passage;

  // Build spacetime object manager.
  SpacetimeTrajectoryManager st_traj_mgr;
  SpacetimePlannerObjectTrajectories st_planner_object_traj;

  input.st_traj_mgr = &st_traj_mgr;
  input.st_planner_object_traj = &st_planner_object_traj;

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      psmm, *drive_passage, input.trajectory.front(), veh_geo_params,
      st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false, /*unsafe_object_ids=*/{});
  QCHECK(path_sl_boundary.ok());
  input.path_sl_boundary = &*path_sl_boundary;

  SendDrivePassageToCanvas(
      *drive_passage,
      absl::StrCat("trajectory_optimizer_test/", test, "/drive_passage"));

  EstPlannerDebugProto debug_proto;

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(
      std::move(end_of_path_boundary_constraint).value());
  ConstraintProto::StopLineProto stop_line;
  HalfPlane halfplane(Vec2d(30.0, -1.8), Vec2d(30.0, -5.3));
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_s(30.0);
  stop_line.set_time(0.0);
  stop_line.set_standoff(1.0);
  stop_line.mutable_source()->mutable_crosswalk()->set_crosswalk_id(0);
  constraint_mgr.AddStopLine(std::move(stop_line));
  SourceProto speed_bump_source;
  speed_bump_source.mutable_speed_bump()->set_id(1);

  ConstraintProto::SpeedRegionProto speed_bump;
  speed_bump.set_start_s(15.0);
  speed_bump.set_end_s(17.0);
  speed_bump.set_max_speed(2.0);
  speed_bump.set_min_speed(0.0);
  speed_bump.mutable_source()->mutable_speed_bump()->set_id(1);
  speed_bump.set_id(absl::StrFormat("speed_bump_%d", 1));
  constraint_mgr.AddSpeedRegion(std::move(speed_bump));
  input.constraint_mgr = &constraint_mgr;

  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs;
  input.leading_trajs = &leading_trajs;

  input.plan_start_point = input.trajectory.front();
  auto fake_time = Clock::Now();
  input.plan_start_time = fake_time;
  input.planner_semantic_map_mgr = &psmm;
  std::vector<TrajectoryPoint> captain_traj = {};
  input.captain_trajectory = &captain_traj;

  input.trajectory_optimizer_params =
      &planner_params.trajectory_optimizer_params();
  input.motion_constraint_params = &planner_params.motion_constraint_params();
  input.planner_functions_params = &planner_params.planner_functions_params();
  input.vehicle_models_params = &planner_params.vehicle_models_params();
  input.veh_geo_params = &veh_geo_params;
  input.veh_drive_params = &veh_drive_params;

  ThreadPool* tp = ThreadPool::DefaultPool();
  auto output =
      OptimizeTrajectory(input, debug_proto.mutable_trajectory_optimizer(),
                         /*charts_data=*/nullptr, tp);

  if (output.ok()) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&input](int index) {
              return Vec2d(input.trajectory[index].path_point().x(),
                           input.trajectory[index].path_point().y());
            },
            input.trajectory.size()),
        vis::Color::kLightRed,
        absl::StrCat("trajectory_optimizer_test/", test, "/input"));

    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&output](int index) {
              return Vec2d(output->trajectory_proto[index].path_point().x(),
                           output->trajectory_proto[index].path_point().y());
            },
            output->trajectory_proto.size()),
        vis::Color::kLightBlue,
        absl::StrCat("trajectory_optimizer_test/", test, "/output"));
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

TEST(TrajectoryOptimizerTest, RightNudgeObject) {
  EgoPose ego({.pos = {0.0, 0.0}, .theta = 0.0, .v = 3.0});
  const double cos_theta = std::cos(ego.theta);
  const double sin_theta = std::sin(ego.theta);
  const std::string test = "right_nudge";

  TrajectoryOptimizerInput input;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego.pos.x() +
                      k * kTrajectoryTimeStep * ego.v * cos_theta);
    path_point->set_y(ego.pos.y() +
                      k * kTrajectoryTimeStep * ego.v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego.theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego.v * k * kTrajectoryTimeStep);
    pt.set_v(ego.v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }
  input.trajectory = traj_points;

  // For space_time object and drive_passage.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego.pos.x());
  pose.mutable_pos_smooth()->set_y(ego.pos.y());
  pose.set_yaw(ego.theta);
  pose.mutable_vel_smooth()->set_x(ego.v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego.v * sin_theta);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            route_path.lane_paths().front(),
                                            kDrivePassageKeepBehindLength);
  EXPECT_OK(backward_extended_lane_path);
  auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  input.drive_passage = &*drive_passage;

  // Build spacetime object manager.
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("abc")
                                  .set_type(OT_VEHICLE)
                                  .set_timestamp(1.0)
                                  .set_velocity(2.0)
                                  .set_yaw(0.0)
                                  .set_length_width(4.0, 2.0)
                                  .set_box_center(Vec2d(10.0, 2.0))
                                  .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(Vec2d(10.0, 2.0), Vec2d(10.5, 2.0),
                         /*init_v=*/0.0, /*last_v=*/0.1);
  const PlannerObject object = builder.Build();
  SpacetimeTrajectoryManager st_traj_mgr(absl::MakeSpan(&object, 1));
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  for (const auto& trajectory : st_traj_mgr.trajectories()) {
    st_planner_object_traj.AddSpacetimePlannerObjectTrajectory(
        trajectory, SpacetimePlannerObjectTrajectoryReason::FRONT);
  }

  input.st_traj_mgr = &st_traj_mgr;
  input.st_planner_object_traj = &st_planner_object_traj;

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      psmm, *drive_passage, input.trajectory.front(), veh_geo_params,
      st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false, /*unsafe_object_ids=*/{});
  QCHECK(path_sl_boundary.ok());
  input.path_sl_boundary = &*path_sl_boundary;

  SendDrivePassageToCanvas(
      *drive_passage,
      absl::StrCat("trajectory_optimizer_test/", test, "/drive_passage"));

  EstPlannerDebugProto debug_proto;

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(
      std::move(end_of_path_boundary_constraint).value());
  input.constraint_mgr = &constraint_mgr;

  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs;
  input.leading_trajs = &leading_trajs;

  input.plan_start_point = input.trajectory.front();
  auto fake_time = Clock::Now();
  input.plan_start_time = fake_time;
  input.planner_semantic_map_mgr = &psmm;
  std::vector<TrajectoryPoint> captain_traj = {};
  input.captain_trajectory = &captain_traj;

  input.trajectory_optimizer_params =
      &planner_params.trajectory_optimizer_params();
  input.motion_constraint_params = &planner_params.motion_constraint_params();
  input.planner_functions_params = &planner_params.planner_functions_params();
  input.vehicle_models_params = &planner_params.vehicle_models_params();
  input.veh_geo_params = &veh_geo_params;
  input.veh_drive_params = &veh_drive_params;

  ThreadPool* tp = ThreadPool::DefaultPool();
  auto output =
      OptimizeTrajectory(input, debug_proto.mutable_trajectory_optimizer(),
                         /*charts_data=*/nullptr, tp);

  if (output.ok()) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&input](int index) {
              return Vec2d(input.trajectory[index].path_point().x(),
                           input.trajectory[index].path_point().y());
            },
            input.trajectory.size()),
        vis::Color::kLightRed,
        absl::StrCat("trajectory_optimizer_test/", test, "/input"));

    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&output](int index) {
              return Vec2d(output->trajectory_proto[index].path_point().x(),
                           output->trajectory_proto[index].path_point().y());
            },
            output->trajectory_proto.size()),
        vis::Color::kLightBlue,
        absl::StrCat("trajectory_optimizer_test/", test, "/output"));
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
