#include "onboard/planner/plan/acc/acc_preliminary_speed.h"

#include <algorithm>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"
#include "onboard/planner/plan/acc/acc_corridor_with_map.h"
#include "onboard/planner/plan/acc/acc_target.h"
#include "onboard/planner/plan/acc/acc_target_selector_with_map.h"
#include "onboard/planner/plan/acc/test_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {

namespace planner {
namespace {

constexpr double kMinFollowHeadway = 0.8;   // s.
constexpr double kMinStandStillDist = 2.0;  // m.
const PiecewiseLinearFunction<double, double> kFollowHeadwayRatioPLF(
    std::vector<double>{0.0, 1.0, 2.0, 3.0},
    std::vector<double>{0.0, 0.5, 0.7, 1.0});
const PiecewiseLinearFunction<double, double> kStandStillDistRatioPLF(
    std::vector<double>{0.0, 1.0, 2.0, 3.0},
    std::vector<double>{1.0, 1.2, 0.8, 1.0});

TEST(RunAccPreliminarySpeed, GoStraightOneStationaryTarget) {
  auto param_manager = CreateParamManagerFromCarId("Q2506");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();

  const auto& psmm = CreateDojoTestPSMM();

  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(1.430);
  av_pose.mutable_pos_smooth()->set_y(-0.012);
  av_pose.set_yaw(0.0);
  av_pose.mutable_vel_body()->set_x(Kph2Mps(60.0));

  const auto plan_start_point = ConvertToTrajPointProto(av_pose);
  const Vec2d object_pos = {42.678, 0.048};

  PerceptionObjectBuilder percep_builder;
  const auto object = percep_builder.set_id("front 1")
                          .set_type(OT_VEHICLE)
                          .set_velocity(0.0)
                          .set_yaw(0.0)
                          .set_length_width(3.7, 2.0)
                          .set_box_center(object_pos)
                          .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(object)
      .set_pos(object_pos)
      .set_v(0.0)
      .set_theta(0.0)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(object_pos, /*theta=*/0.0);
  auto planner_object = builder.Build();

  std::vector<PlannerObject> objects;
  objects.push_back(std::move(planner_object));

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager({}, absl::MakeSpan(objects),
                                 /*thread_pool=*/nullptr);
  EXPECT_EQ(st_traj_mgr.stationary_object_trajs().size(), 1);

  const auto& vehicle_geom = vehicle_param_api.vehicle_geometry_params();
  const auto& vehicle_drive = vehicle_param_api.vehicle_drive_params();

  const auto dm_or = BuildDrivingMapTopo(av_pose, psmm, plan_start_point.v());
  EXPECT_OK(dm_or.status());
  // Build corridor.
  const auto map_corridor_or = BuildAccCorridorFromClosestLanePath(
      av_pose, plan_start_point, psmm, *dm_or,
      /*steering_percentage=*/std::nullopt, vehicle_geom, vehicle_drive,
      /*corridor_step_s=*/2.0, kAccTrajectoryTimeHorizon);
  EXPECT_OK(map_corridor_or);

  // Select target.
  std::vector<AccTargetDecision> prev_primary_targets = {};
  AccTargetInput target_input{
      .corridor = &(*map_corridor_or),
      .st_traj_mgr = &st_traj_mgr,
      .prev_primary_targets = &prev_primary_targets,
      .av_pose = &av_pose,
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &plan_start_point,
  };
  const auto target_per_corridor = SelectAccTargetWithMap(target_input);
  EXPECT_EQ(target_per_corridor.leaders.size(), 1);
  const auto& leader = target_per_corridor.leaders.front();
  EXPECT_EQ(leader.object_id, "front 1");

  auto planner_params = CreateDefaultPlannerParamOnlyFillAccParams();
  auto& speed_finder_params =
      *planner_params.mutable_acc_params()->mutable_speed_finder_params();
  double standstill_distance = speed_finder_params.follow_standstill_distance();
  double follow_headway = speed_finder_params.follow_time_headway();
  standstill_distance = std::max(
      standstill_distance * kStandStillDistRatioPLF(plan_start_point.v()),
      kMinStandStillDist);
  follow_headway =
      std::max(follow_headway * kFollowHeadwayRatioPLF(plan_start_point.v()),
               kMinFollowHeadway);
  speed_finder_params.set_follow_standstill_distance(standstill_distance);
  speed_finder_params.set_follow_time_headway(follow_headway);

  // Generate preliminary speed.
  AccPreliminarySpeedInput speed_input{
      .plan_start_point = &plan_start_point,
      .corridor = &(*map_corridor_or),
      .targets = &target_per_corridor,
      .av_front_to_rac = vehicle_geom.front_edge_to_center(),
      .ttc = speed_finder_params.follow_time_headway(),
      .standstill_dist = speed_finder_params.follow_standstill_distance(),
      .is_acc_standwait = false,
  };

  const auto speed_output = RunAccPreliminarySpeed(speed_input);
}

TEST(RunAccPreliminarySpeed, EnterRampNoTarget) {
  auto param_manager = CreateParamManagerFromCarId("Q2506");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();

  const auto& psmm = CreateDojoTestPSMM();

  PoseProto av_pose;
  // An point entering ramp on dojo map.
  av_pose.mutable_pos_smooth()->set_x(8.0);
  av_pose.mutable_pos_smooth()->set_y(0.0);
  av_pose.set_yaw(0.0);
  av_pose.mutable_vel_body()->set_x(Kph2Mps(80.0));  // 80kph.

  ApolloTrajectoryPointProto plan_start_point;
  PathPoint path_point;
  path_point.set_x(av_pose.pos_smooth().x());
  path_point.set_y(av_pose.pos_smooth().y());
  path_point.set_theta(av_pose.yaw());
  path_point.set_kappa(0.0);
  path_point.set_s(0.0);
  plan_start_point.set_v(av_pose.vel_body().x());
  plan_start_point.set_a(0.0);
  *plan_start_point.mutable_path_point() = std::move(path_point);

  const auto& vehicle_geom = vehicle_param_api.vehicle_geometry_params();
  const auto& vehicle_drive = vehicle_param_api.vehicle_drive_params();
  const auto dm_or = BuildDrivingMapTopo(av_pose, psmm, plan_start_point.v());
  EXPECT_OK(dm_or.status());
  const auto map_corridor_or = BuildAccCorridorFromClosestLanePath(
      av_pose, plan_start_point, psmm, *dm_or,
      /*steering_percentage=*/std::nullopt, vehicle_geom, vehicle_drive,
      /*corridor_step_s=*/2.0, kAccTrajectoryTimeHorizon);
  EXPECT_OK(map_corridor_or.status());

  AccTargetPerCorridor acc_target_per_corridor{
      .acc_st_traj_mgr = std::make_unique<SpacetimeTrajectoryManager>(),
  };

  auto planner_params = CreateDefaultPlannerParamOnlyFillAccParams();
  auto& speed_finder_params =
      *planner_params.mutable_acc_params()->mutable_speed_finder_params();
  double standstill_distance = speed_finder_params.follow_standstill_distance();
  double follow_headway = speed_finder_params.follow_time_headway();
  standstill_distance = std::max(
      standstill_distance * kStandStillDistRatioPLF(plan_start_point.v()),
      kMinStandStillDist);
  follow_headway =
      std::max(follow_headway * kFollowHeadwayRatioPLF(plan_start_point.v()),
               kMinFollowHeadway);
  speed_finder_params.set_follow_standstill_distance(standstill_distance);
  speed_finder_params.set_follow_time_headway(follow_headway);

  std::set<std::string> crowded_scene_objs;
  AccPreliminarySpeedInput preliminary_speed_input{
      .plan_start_point = &plan_start_point,
      .corridor = &(*map_corridor_or),
      .targets = &acc_target_per_corridor,
      .optional_ext_speed_limit = Kph2Mps(80.0),
      .av_front_to_rac = vehicle_geom.front_edge_to_center(),
      .ttc = speed_finder_params.follow_time_headway(),
      .standstill_dist = speed_finder_params.follow_standstill_distance(),
      .is_acc_standwait = false,
  };

  auto preliminary_speed_out = RunAccPreliminarySpeed(preliminary_speed_input);

  // Acc standwait.
  AccPreliminarySpeedInput preliminary_speed_input_1{
      .plan_start_point = &plan_start_point,
      .corridor = &(*map_corridor_or),
      .targets = &acc_target_per_corridor,
      .optional_ext_speed_limit = Kph2Mps(80.0),
      .av_front_to_rac = vehicle_geom.front_edge_to_center(),
      .ttc = speed_finder_params.follow_time_headway(),
      .standstill_dist = speed_finder_params.follow_standstill_distance(),
      .is_acc_standwait = true,
  };
  auto preliminary_speed_out_1 =
      RunAccPreliminarySpeed(preliminary_speed_input_1);
  constexpr double kEpsilon = 1e-3;
  const auto& speed_limit = preliminary_speed_out_1.speed_limit;
  EXPECT_EQ(speed_limit.speed_limit_ranges().size(), 1);
  EXPECT_NEAR(speed_limit.speed_limit_ranges()[0].speed_limit, 0.0, kEpsilon);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
