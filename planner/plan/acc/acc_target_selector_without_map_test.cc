#include "onboard/planner/plan/acc/acc_target_selector_without_map.h"

#include "gtest/gtest.h"

#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor_without_map.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft::planner {
namespace {

TEST(SeclectAccTargetWithoutMap, GoStraightNoTarget) {
  auto param_manager = CreateParamManagerFromCarId("Q1001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_param_api = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_param_api.vehicle_geometry_params();
  const auto& vehicle_drive = vehicle_param_api.vehicle_drive_params();

  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(0.0);
  av_pose.mutable_pos_smooth()->set_y(0.0);
  av_pose.set_yaw(0.0);
  av_pose.set_curvature(0.0);
  av_pose.mutable_vel_body()->set_x(Kph2Mps(100.0));

  const auto plan_start_point = ConvertToTrajPointProto(av_pose);

  const auto corridor_or = BuildAccCorridorFromPlanStartPoint(
      av_pose, plan_start_point, /*steering_percentage=*/0.0, vehicle_geom,
      vehicle_drive,
      /*corridor_step_s=*/2.0, kAccTrajectoryTimeHorizon,
      /*av_kappa_cache_average=*/0.0);
  EXPECT_OK(corridor_or.status());

  // Empty object manager.
  PlannerObjectManager object_mgr;
  // Empty spacetime Trajectory Manager.
  SpacetimeTrajectoryManager st_traj_mgr(
      absl::MakeSpan(object_mgr.planner_objects()), /*thread_pool=*/nullptr);

  // Empty prev targets.
  std::vector<AccTargetDecision> prev_targets;

  AccTargetInput target_input{
      .corridor = &(*corridor_or),
      .st_traj_mgr = &st_traj_mgr,
      .prev_primary_targets = &prev_targets,
      .av_pose = &av_pose,
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &plan_start_point,
  };
  const auto target = SelectAccTargetWithoutMap(target_input);
  const auto& leaders = target.leaders;
  EXPECT_EQ(leaders.size(), 0);
}

}  // namespace
}  // namespace qcraft::planner
