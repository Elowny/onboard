#include "onboard/planner/plan/acc/acc_speed_decision.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string_view>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/speed_finder_flags.h"  // IWYU pragma: keep
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {

namespace planner {
namespace {

VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

DiscretizedPath CreatePath() {
  constexpr int kPathCounts = 50;
  std::vector<PathPoint> path_points;
  path_points.reserve(kPathCounts);
  PathPoint start;
  start.set_x(0.0);
  start.set_y(0.0);
  start.set_z(0.0);
  start.set_theta(0.0);
  start.set_kappa(0.001);
  start.set_lambda(0.0);
  start.set_s(0.0);
  for (int k = 0; k < kPathCounts; ++k) {
    path_points.push_back(
        GetPathPointAlongCircle(start, static_cast<double>(k) * 1.0));
  }
  return DiscretizedPath(path_points);
}

SpacetimeObjectTrajectory CreateStBoundaryAndStTraj(
    ObjectType type, StBoundaryProto::ObjectType object_type,
    std::string_view id, double vel, double acc, double yaw,
    const Vec2d& box_center, const DiscretizedPath& path,
    StBoundaryRef* st_boundary) {
  // Build spacetime object manager.
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id(id)
                                  .set_type(type)
                                  .set_timestamp(1.0)
                                  .set_velocity(vel)
                                  .set_yaw(yaw)
                                  .set_length_width(4.0, 2.0)
                                  .set_box_center(box_center)
                                  .set_pos(box_center)
                                  .Build();
  PlannerObjectBuilder builder;
  builder.set_type(type)
      .set_object(perception_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(box_center, yaw, /*duration=*/10.0, vel, acc)
      .set_type(vel == 0.0 ? PredictionType::PT_STATIONARY
                           : PredictionType::PT_CYCV);

  const PlannerObject object = builder.Build();
  SpacetimeObjectTrajectory spacetime_traj(object, /*traj_index=*/0,
                                           /*required_lateral_gap=*/0.0);

  const auto& states = spacetime_traj.states();
  StBoundaryPoints st_boundary_points;

  st_boundary_points.lower_points.reserve(states.size());
  st_boundary_points.upper_points.reserve(states.size());
  st_boundary_points.speed_points.reserve(states.size());
  st_boundary_points.overlap_infos.reserve(states.size());
  int i = 0;
  for (const auto& state : states) {
    const auto& traj_point = *state.traj_point;
    const auto frenet_pt = path.XYToSL(traj_point.pos());
    const double t = state.traj_point->t();
    st_boundary_points.lower_points.push_back(
        StPoint(frenet_pt.s - veh_geo_params.front_edge_to_center(), t));
    st_boundary_points.upper_points.push_back(
        StPoint(frenet_pt.s + veh_geo_params.back_edge_to_center(), t));
    st_boundary_points.speed_points.push_back(VtPoint(traj_point.v(), t));

    const auto& object_box = state.box;
    std::optional<int> av_start_idx, av_end_idx;
    for (int idx = 0; idx < path.size(); ++idx) {
      const auto& pt = path[idx];
      const Box2d av_box =
          ComputeAvBox(Vec2d(pt.x(), pt.y()), pt.theta(), veh_geo_params);
      if (av_box.HasOverlap(object_box)) {
        if (av_start_idx.has_value()) {
          av_end_idx = idx;
        }
        if (!av_start_idx.has_value()) {
          av_start_idx = idx;
        }
      }
    }
    if (av_start_idx.has_value() && av_end_idx.has_value()) {
      st_boundary_points.overlap_infos.push_back(
          OverlapInfo{.time = t,
                      .obj_idx = i,
                      .av_start_idx = *av_start_idx,
                      .av_end_idx = *av_end_idx});
    }
    ++i;
  }
  if (st_boundary_points.overlap_infos.empty()) {
    // Fill a overlap info to create st boundary, have no meaning.
    st_boundary_points.overlap_infos.push_back(OverlapInfo{
        .time = 0, .obj_idx = 0, .av_start_idx = 0, .av_end_idx = 0});
  }
  *st_boundary = StBoundary::CreateInstance(
      st_boundary_points, object_type, std::string(spacetime_traj.traj_id()),
      spacetime_traj.trajectory().probability(), spacetime_traj.is_stationary(),
      StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
  return spacetime_traj;
}

TEST(RunAccSpeedDecision, AnalyzeStOverlapForACCTest0) {
  StBoundaryRef st_boundary;
  const DiscretizedPath path = CreatePath();
  SpacetimeObjectTrajectory spacetime_traj = CreateStBoundaryAndStTraj(
      OT_PEDESTRIAN, StBoundaryProto::PEDESTRIAN,
      /*id=*/"abc",
      /*vel=*/5.0, /*acc=*/0.0, /*yaw=*/M_PI_4,
      /*box_center*/ Vec2d(10.0, -15.0), path, &st_boundary);
  const StOverlapMetaProto res = internal::AnalyzeStOverlapForACC(
      st_boundary, spacetime_traj, path, veh_geo_params);
  QCHECK_EQ(res.source(), StOverlapMetaProto::LANE_CROSS);
  QCHECK_EQ(res.modification_type(), StOverlapMetaProto::NON_MODIFIABLE);
}

TEST(RunAccSpeedDecision, AnalyzeStOverlapForACCTest1) {
  StBoundaryRef st_boundary;
  const DiscretizedPath path = CreatePath();
  SpacetimeObjectTrajectory spacetime_traj = CreateStBoundaryAndStTraj(
      OT_VEHICLE, StBoundaryProto::VEHICLE,
      /*id=*/"abc",
      /*vel=*/5.0, /*acc=*/0.0, /*yaw=*/M_PI / 8.0,
      /*box_center*/ Vec2d(10.0, -10.0), path, &st_boundary);
  const StOverlapMetaProto res = internal::AnalyzeStOverlapForACC(
      st_boundary, spacetime_traj, path, veh_geo_params);
  QCHECK_EQ(res.source(), StOverlapMetaProto::OBJECT_CUTIN);
  QCHECK_EQ(res.modification_type(), StOverlapMetaProto::LON_MODIFIABLE);
}

TEST(RunAccSpeedDecision, AnalyzeStOverlapsForACCTest) {
  StBoundaryRef st_boundary0;
  const DiscretizedPath path = CreatePath();
  SpacetimeObjectTrajectory spacetime_traj0 = CreateStBoundaryAndStTraj(
      OT_PEDESTRIAN, StBoundaryProto::PEDESTRIAN,
      /*id=*/"abc",
      /*vel=*/0.0, /*acc=*/0.0, /*yaw=*/M_PI_4,
      /*box_center*/ Vec2d(10.0, 0.0), path, &st_boundary0);
  StBoundaryRef st_boundary1;
  SpacetimeObjectTrajectory spacetime_traj1 = CreateStBoundaryAndStTraj(
      OT_VEHICLE, StBoundaryProto::VEHICLE,
      /*id=*/"abc",
      /*vel=*/5.0, /*acc=*/0.0, /*yaw=*/M_PI / 8.0,
      /*box_center*/ Vec2d(10.0, -10.0), path, &st_boundary1);
  std::vector<StBoundaryRef> st_boundaries;
  st_boundaries.push_back(std::move(st_boundary0));
  st_boundaries.push_back(std::move(st_boundary1));
  std::vector<PlannerObject> planner_objects;
  planner_objects.push_back(spacetime_traj0.planner_object());
  planner_objects.push_back(spacetime_traj1.planner_object());
  SpacetimeTrajectoryManager st_traj_mgr(planner_objects);

  internal::AnalyzeStOverlapsForACC(path, st_traj_mgr, veh_geo_params,
                                    &st_boundaries);
  QCHECK_EQ(st_boundaries.size(), 2);
  QCHECK(!st_boundaries.front()->overlap_meta().has_value());
  QCHECK(st_boundaries.back()->overlap_meta().has_value());
}

TEST(RunAccSpeedDecision, FilterStBoundariesByCutinTimeTest) {
  const DiscretizedPath path = CreatePath();
  StBoundaryRef st_boundary1;
  SpacetimeObjectTrajectory spacetime_traj1 = CreateStBoundaryAndStTraj(
      OT_PEDESTRIAN, StBoundaryProto::PEDESTRIAN,
      /*id=*/"abc",
      /*vel=*/5.0, /*acc=*/0.0, /*yaw=*/0.0,
      /*box_center*/ Vec2d(10.0, -1.0), path, &st_boundary1);
  StBoundaryRef st_boundary2;
  SpacetimeObjectTrajectory spacetime_traj2 = CreateStBoundaryAndStTraj(
      OT_VEHICLE, StBoundaryProto::VEHICLE,
      /*id=*/"abc2",
      /*vel=*/6.0, /*acc=*/0.0, /*yaw=*/M_PI / 16.0,
      /*box_center*/ Vec2d(10.0, -5.0), path, &st_boundary2);
  std::vector<StBoundaryRef> st_boundaries;
  st_boundaries.push_back(std::move(st_boundary1));
  st_boundaries.push_back(std::move(st_boundary2));
  std::vector<PlannerObject> planner_objects;
  planner_objects.push_back(spacetime_traj1.planner_object());
  planner_objects.push_back(spacetime_traj2.planner_object());
  SpacetimeTrajectoryManager st_traj_mgr(planner_objects);

  const std::vector<StBoundaryRef> res =
      internal::FilterStBoundariesByCutinTime(absl::MakeSpan(st_boundaries),
                                              st_traj_mgr);
  QCHECK_EQ(res.size(), 1);
  QCHECK_EQ(*res.front()->object_id(), std::string("abc"));
}

TEST(RunAccSpeedDecision, FilterOncomingStBoundariesTest) {
  const DiscretizedPath path = CreatePath();
  StBoundaryRef st_boundary1;
  SpacetimeObjectTrajectory spacetime_traj1 = CreateStBoundaryAndStTraj(
      OT_PEDESTRIAN, StBoundaryProto::PEDESTRIAN,
      /*id=*/"abc1",
      /*vel=*/2.0, /*acc=*/0.0, /*yaw=*/M_PI,
      /*box_center*/ Vec2d(30.0, 0.0), path, &st_boundary1);
  StBoundaryRef st_boundary2;
  SpacetimeObjectTrajectory spacetime_traj2 = CreateStBoundaryAndStTraj(
      OT_VEHICLE, StBoundaryProto::VEHICLE,
      /*id=*/"abc2",
      /*vel=*/3.0, /*acc=*/0.0, /*yaw=*/M_PI * 15.0 / 16.0,
      /*box_center*/ Vec2d(10.0, -3.0), path, &st_boundary2);
  std::vector<StBoundaryRef> st_boundaries;
  st_boundaries.push_back(std::move(st_boundary1));
  st_boundaries.push_back(std::move(st_boundary2));
  std::vector<PlannerObject> planner_objects;
  planner_objects.push_back(spacetime_traj1.planner_object());
  planner_objects.push_back(spacetime_traj2.planner_object());
  SpacetimeTrajectoryManager st_traj_mgr(planner_objects);

  const std::vector<StBoundaryRef> res = internal::FilterOncomingStBoundaries(
      absl::MakeSpan(st_boundaries), st_traj_mgr, path);
  QCHECK_EQ(res.size(), 1);
  QCHECK_EQ(*res.front()->object_id(), std::string("abc1"));
}

TEST(RunAccSpeedDecision, IsStBoundaryFromStationaryVehicleTest0) {
  const DiscretizedPath path = CreatePath();
  StBoundaryRef st_boundary;
  SpacetimeObjectTrajectory spacetime_traj = CreateStBoundaryAndStTraj(
      OT_UNKNOWN_STATIC, StBoundaryProto::IMPASSABLE_BOUNDARY,
      /*id=*/"abc1",
      /*vel=*/2.0, /*acc=*/0.0, /*yaw=*/0.0, /*box_center*/ Vec2d(10.0, 2.0),
      path, &st_boundary);
  QCHECK_EQ(internal::IsStBoundaryFromStationaryVehicle(*st_boundary), false);
}

TEST(RunAccSpeedDecision, IsStBoundaryFromStationaryVehicleTest1) {
  const DiscretizedPath path = CreatePath();
  StBoundaryRef st_boundary;
  SpacetimeObjectTrajectory spacetime_traj = CreateStBoundaryAndStTraj(
      OT_VEHICLE, StBoundaryProto::VEHICLE,
      /*id=*/"abc1",
      /*vel=*/2.0, /*acc=*/0.0, /*yaw=*/0.0,
      /*box_center*/ Vec2d(10.0, 2.0), path, &st_boundary);
  QCHECK_EQ(internal::IsStBoundaryFromStationaryVehicle(*st_boundary), false);
}

TEST(RunAccSpeedDecision, IsStBoundaryFromStationaryVehicleTest2) {
  const DiscretizedPath path = CreatePath();
  StBoundaryRef st_boundary;
  SpacetimeObjectTrajectory spacetime_traj = CreateStBoundaryAndStTraj(
      OT_VEHICLE, StBoundaryProto::VEHICLE,
      /*id=*/"abc1",
      /*vel=*/0.0, /*acc=*/0.0, /*yaw=*/0.0,
      /*box_center*/ Vec2d(10.0, 2.0), path, &st_boundary);
  QCHECK_EQ(internal::IsStBoundaryFromStationaryVehicle(*st_boundary), true);
}

TEST(RunAccSpeedDecision, IsStBoundaryFromStationaryVehicleTest3) {
  const DiscretizedPath path = CreatePath();
  StBoundaryRef st_boundary;
  SpacetimeObjectTrajectory spacetime_traj = CreateStBoundaryAndStTraj(
      OT_UNKNOWN_STATIC, StBoundaryProto::STATIC,
      /*id=*/"abc1",
      /*vel=*/0.0, /*acc=*/0.0, /*yaw=*/0.0,
      /*box_center*/ Vec2d(10.0, 2.0), path, &st_boundary);
  QCHECK_EQ(internal::IsStBoundaryFromStationaryVehicle(*st_boundary), false);
}
TEST(RunAccSpeedDecision, IsStBoundaryFromObjectsTest) {
  const DiscretizedPath path = CreatePath();
  StBoundaryRef st_boundary;
  SpacetimeObjectTrajectory spacetime_traj = CreateStBoundaryAndStTraj(
      OT_VEHICLE, StBoundaryProto::VEHICLE,
      /*id=*/"abc1",
      /*vel=*/2.0, /*acc=*/0.0, /*yaw=*/0.0,
      /*box_center*/ Vec2d(10.0, 2.0), path, &st_boundary);
  std::set<std::string> object_ids;
  QCHECK_EQ(internal::IsStBoundaryFromObjects(*st_boundary, object_ids), false);
}

TEST(RunAccSpeedDecision,
     ModifyStationaryObjectStBoundariesForCrowdedSceneTest) {
  StBoundaryRef st_boundary0;
  const DiscretizedPath path = CreatePath();
  SpacetimeObjectTrajectory spacetime_traj0 = CreateStBoundaryAndStTraj(
      OT_PEDESTRIAN, StBoundaryProto::PEDESTRIAN,
      /*id=*/"abc1",
      /*vel=*/0.0, /*acc=*/0.0, /*yaw=*/M_PI_4,
      /*box_center*/ Vec2d(20.0, 0.0), path, &st_boundary0);
  StBoundaryRef st_boundary1;
  SpacetimeObjectTrajectory spacetime_traj1 = CreateStBoundaryAndStTraj(
      OT_VEHICLE, StBoundaryProto::VEHICLE,
      /*id=*/"abc2",
      /*vel=*/0.0, /*acc=*/0.0, /*yaw=*/0.0,
      /*box_center*/ Vec2d(15.0, 2.0), path, &st_boundary1);

  std::set<std::string> crowded_side_objs = {"abc1"};
  StBoundaryWithDecision st_boundary_decision0(std::move(st_boundary0));
  StBoundaryWithDecision st_boundary_decision1(std::move(st_boundary1));
  std::vector<StBoundaryWithDecision> st_boundary_decisions;
  st_boundary_decisions.push_back(std::move(st_boundary_decision0));
  st_boundary_decisions.push_back(std::move(st_boundary_decision1));

  internal::ModifyStationaryObjectStBoundariesForCrowdedScene(
      crowded_side_objs, absl::MakeSpan(st_boundary_decisions));
}

TEST(RunAccSpeedDecision, PostProcessingReverseStBoundariesTest) {
  const DiscretizedPath path = CreatePath();
  StBoundaryRef st_boundary1;
  SpacetimeObjectTrajectory spacetime_traj1 = CreateStBoundaryAndStTraj(
      OT_VEHICLE, StBoundaryProto::VEHICLE,
      /*id=*/"abc1",
      /*vel=*/2.0, /*acc=*/0.0, /*yaw=*/0.0,
      /*box_center*/ Vec2d(10.0, 2.0), path, &st_boundary1);
  StBoundaryRef st_boundary2;
  SpacetimeObjectTrajectory spacetime_traj2 = CreateStBoundaryAndStTraj(
      OT_VEHICLE, StBoundaryProto::VEHICLE,
      /*id=*/"abc2",
      /*vel=*/2.0, /*acc=*/0.0, /*yaw=*/M_PI,
      /*box_center*/ Vec2d(30.0, 1.0), path, &st_boundary2);
  std::vector<StBoundaryRef> st_boundaries;
  st_boundaries.push_back(std::move(st_boundary1));
  st_boundaries.push_back(std::move(st_boundary2));
  internal::PostProcessingReverseStBoundaries(absl::MakeSpan(st_boundaries));
  QCHECK_EQ(st_boundaries.size(), 2);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
