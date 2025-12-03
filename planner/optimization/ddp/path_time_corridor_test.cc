#include "onboard/planner/optimization/ddp/path_time_corridor.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/est_planner_debug.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
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
#include "onboard/planner/trajectory_util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {
namespace {

TEST(PathTimeCorridorTest, ObjectQueryTest) {
  const Vec2d ego_pos(0.0, 0.0);
  const double ego_theta = 0.0;
  const double ego_v = 3.0;
  const double cos_theta = std::cos(ego_theta);
  const double sin_theta = std::sin(ego_theta);

  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego_pos.x() +
                      k * kTrajectoryTimeStep * ego_v * cos_theta);
    path_point->set_y(ego_pos.y() +
                      k * kTrajectoryTimeStep * ego_v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego_theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego_v * k * kTrajectoryTimeStep);
    pt.set_v(ego_v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }

  // For space_time object and drive_passage.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego_pos.x());
  pose.mutable_pos_smooth()->set_y(ego_pos.y());
  pose.set_yaw(ego_theta);
  pose.mutable_vel_smooth()->set_x(ego_v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego_v * sin_theta);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "7c_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";

  // Build spacetime object manager.
  PerceptionObjectBuilder perception_builder_right;
  const auto perception_obj_right = perception_builder_right.set_id("right")
                                        .set_type(OT_VEHICLE)
                                        .set_timestamp(1.0)
                                        .set_velocity(0.0)
                                        .set_pos(Vec2d(10.0, -2.5))
                                        .set_yaw(0.0)
                                        .set_length_width(4.0, 2.0)
                                        .set_box_center(Vec2d(10.0, -2.5))
                                        .Build();
  PlannerObjectBuilder builder_right;
  builder_right.set_type(OT_VEHICLE)
      .set_object(perception_obj_right)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_stationary_traj(Vec2d(10.0, -2.5), /*theta=*/0.0);

  const PlannerObject object_right = builder_right.Build();

  PerceptionObjectBuilder perception_builder_left;
  const auto perception_obj_left = perception_builder_left.set_id("left")
                                       .set_type(OT_VEHICLE)
                                       .set_timestamp(0.0)
                                       .set_velocity(5.0)
                                       .set_pos(Vec2d(10.0, 2.0))
                                       .set_yaw(0.0)
                                       .set_length_width(4.0, 2.0)
                                       .set_box_center(Vec2d(10.0, 2.0))
                                       .Build();
  PlannerObjectBuilder builder_left;
  builder_left.set_type(OT_VEHICLE)
      .set_object(perception_obj_left)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(Vec2d(10.0, 2.0), /*theta=*/0.0, /*duration=*/5.0,
                         /*init_v=*/5.0, /*acc=*/0.0);
  const PlannerObject object_left = builder_left.Build();

  std::vector<PlannerObject> planner_objects;
  planner_objects.push_back(object_left);
  planner_objects.push_back(object_right);

  SpacetimeTrajectoryManager st_traj_mgr(absl::MakeSpan(planner_objects));
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  for (const auto& trajectory : st_traj_mgr.trajectories()) {
    st_planner_object_traj.AddSpacetimePlannerObjectTrajectory(
        trajectory, SpacetimePlannerObjectTrajectoryReason::SIDE);
  }
  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      psmm, *drive_passage, traj_points.front(), veh_geo_params, st_traj_mgr,
      lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false, /*unsafe_object_ids=*/{});
  QCHECK(path_sl_boundary.ok());

  EstPlannerDebugProto debug_proto;

  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs;

  std::vector<Vec2d> init_points;
  init_points.reserve(traj_points.size());
  for (const auto& pt : traj_points) {
    init_points.push_back(Vec2d(pt.path_point().x(), pt.path_point().y()));
  }
  const auto init_traj_frenet_frame =
      BuildKdTreeFrenetFrame(init_points,
                             /*down_sample_raw_points=*/true);
  QCHECK(init_traj_frenet_frame.ok());

  const absl::StatusOr<optimizer::PathTimeCorridor> path_time_corridor =
      optimizer::BuildPathTimeCorridor(
          /*base_name=*/"traj_opt", ToTrajectoryPoint(traj_points),
          *drive_passage, *path_sl_boundary, *init_traj_frenet_frame,
          leading_trajs, st_planner_object_traj, veh_geo_params);
  QCHECK(path_time_corridor.ok());
  int idx;
  const auto boundary = path_time_corridor->QueryBoundaryL(
      /*s_start=*/0.0, /*s_end=*/20.0, /*t=*/0.0, &idx);
  constexpr double kEps = 1e-1;
  QCHECK_NEAR(boundary.first->l_boundary, -1.85, kEps);
  QCHECK_NEAR(boundary.second->l_boundary, 1.73, kEps);
  QCHECK_NEAR(boundary.first->l_object, -1.52, kEps);
  QCHECK_NEAR(boundary.second->l_object, 0.98, kEps);
  QCHECK_EQ(idx, 5);

  const auto boudnary = path_time_corridor->QueryNarrowestBoundaryAllTypes(
      /*s_start=*/0.0, /*s_end=*/20.0, /*t=*/0.0);
  QCHECK_NEAR((boudnary.first.l_object + boudnary.second.l_object) * 0.5, -0.27,
              kEps);
}

}  // namespace

}  // namespace planner
}  // namespace qcraft
