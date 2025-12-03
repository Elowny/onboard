#include "onboard/planner/plan/acc/acc_target_selector_with_map.h"

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"
#include "onboard/planner/plan/acc/acc_corridor_with_map.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {

namespace planner {
namespace {

TEST(SelectAccTargetWithMap, GoStraightNoTarget) {
  auto param_manager = CreateParamManagerFromCarId("Q2407");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();

  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(1.1);
  av_pose.mutable_pos_smooth()->set_y(0.08);
  av_pose.set_yaw(0.0);
  av_pose.mutable_vel_body()->set_x(Kph2Mps(80.0));

  const ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(av_pose);

  const auto& vehicle_geom = vehicle_param_api.vehicle_geometry_params();
  const auto& vehicle_drive = vehicle_param_api.vehicle_drive_params();

  const auto dm_or = BuildDrivingMapTopo(av_pose, psmm, plan_start_point.v());
  EXPECT_OK(dm_or.status());
  const auto corridor_or = BuildAccCorridorFromClosestLanePath(
      av_pose, plan_start_point, psmm, *dm_or,
      /*steering_percentage=*/std::nullopt, vehicle_geom, vehicle_drive,
      /*corridor_step_s=*/2.0, kAccTrajectoryTimeHorizon);

  EXPECT_OK(corridor_or.status());

  // Empty object manager.
  PlannerObjectManager object_mgr;

  // Spacetime Trajectory Manager.
  SpacetimeTrajectoryManager st_traj_mgr(
      absl::MakeSpan(object_mgr.planner_objects()), /*thread_pool=*/nullptr);

  // No prev targets.
  std::vector<AccTargetDecision> prev_targets;

  AccTargetInput target_input{
      .corridor = &(*corridor_or),
      .st_traj_mgr = &st_traj_mgr,
      .prev_primary_targets = &prev_targets,
      .av_pose = &av_pose,
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &plan_start_point,
  };

  const auto target = SelectAccTargetWithMap(target_input);
  const auto& leaders = target.leaders;
  EXPECT_EQ(leaders.size(), 0);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
