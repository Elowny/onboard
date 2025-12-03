#include "onboard/planner/plan/acc/acc_corridor_with_map.h"

#include <memory>
#include <ostream>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/path_generator_util.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"
#include "onboard/planner/plan/acc/acc_flags.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/pole_placement_util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace planner {
namespace {
TEST(BuildAccCorridorFromClosestLanePath, GoStraight) {
  FLAGS_planner_acc_corridor_draw_target_lane_choice_debug = true;
  FLAGS_planner_acc_corridor_draw_target_lane_ref_path = true;

  auto param_manager = CreateParamManagerFromCarId("Q2506");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();

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
  DrawPathSlBoundaryToCanvas(corridor_or->boundary, "GoStraight/path_sl");
}

TEST(BuildAccCorridorFromClosestLanePath, GoStraightNearCurb) {
  auto param_manager = CreateParamManagerFromCarId("Q2506");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();

  const auto& psmm = CreateDojoTestPSMM();
  const Vec2d av_pos = {3.935, -3.472};  // On dojo lane (id = 3).
  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(av_pos.x());
  av_pose.mutable_pos_smooth()->set_y(av_pos.y());
  av_pose.set_yaw(0.0);  // go straight.
  av_pose.mutable_vel_body()->set_x(Kph2Mps(80.0));

  const ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(av_pose);

  const auto dm_or = BuildDrivingMapTopo(av_pose, psmm, plan_start_point.v());
  EXPECT_OK(dm_or.status());
  const auto corridor_or = BuildAccCorridorFromClosestLanePath(
      av_pose, plan_start_point, psmm, *dm_or,
      /*steering_percentage=*/std::nullopt,
      vehicle_param_api.vehicle_geometry_params(),
      vehicle_param_api.vehicle_drive_params(), /*corridor_step_s=*/2.0,
      kAccTrajectoryTimeHorizon);
  EXPECT_OK(corridor_or);
  DrawPathSlBoundaryToCanvas(corridor_or->boundary,
                             "GoStraightNearCurb/path_sl");
}

TEST(CrowdedSceneObjects, Works) {
  SetMap("dojo");
  auto smm = std::make_shared<SemanticMapManager>();
  smm->LoadWholeMap().Build();
  const auto& psmm = CreateDojoTestPSMM();

  // TODO(changqing): acc unit test use fake online map.

  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(57.69);
  av_pose.mutable_pos_smooth()->set_y(3.495);
  av_pose.set_yaw(0.006622);
  av_pose.mutable_vel_body()->set_x(Kph2Mps(0.0));

  // Front obj 1.
  PerceptionObjectBuilder percep_builder_1;
  const auto obj_1 = percep_builder_1.set_id("front 1")
                         .set_type(OT_VEHICLE)
                         .set_timestamp(1.0)
                         .set_velocity(0.0)
                         .set_yaw(0.0)
                         .set_length_width(4.0, 2.0)
                         .set_box_center(Vec2d(66.156, 3.439))
                         .Build();

  PlannerObjectBuilder builder_1;
  builder_1.set_type(OT_VEHICLE)
      .set_object(obj_1)
      .set_pos(Vec2d(66.156, 3.439))
      .set_v(0.0)
      .set_theta(0.0)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(Vec2d(66.156, 3.439), /*theta=*/0.0);
  auto planner_obj_1 = builder_1.Build();

  // Side obj 2.
  PerceptionObjectBuilder percep_builder_2;
  const auto obj_2 = percep_builder_2.set_id("side 2")
                         .set_type(OT_VEHICLE)
                         .set_timestamp(1.0)
                         .set_velocity(0.0)
                         .set_yaw(0.0)
                         .set_length_width(4.0, 2.0)
                         .set_box_center(Vec2d(57.718, -0.026))
                         .Build();

  PlannerObjectBuilder builder_2;
  builder_2.set_type(OT_VEHICLE)
      .set_object(obj_2)
      .set_pos(Vec2d(57.718, -0.026))
      .set_v(0.0)
      .set_theta(0.0)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(Vec2d(57.718, -0.026), /*theta=*/0.0);
  auto planner_obj_2 = builder_2.Build();

  std::vector<PlannerObject> objects;
  objects.push_back(planner_obj_1);
  objects.push_back(planner_obj_2);

  const auto st_traj_mgr = SpacetimeTrajectoryManager(
      {}, absl::MakeSpan(objects), /*thread_pool=*/nullptr);
  EXPECT_EQ(st_traj_mgr.stationary_object_trajs().size(), 2);
  const auto vehicle_geom = DefaultVehicleGeometry();

  const auto side_objs =
      CrowdedSceneObjects(psmm, st_traj_mgr, av_pose, vehicle_geom,
                          /*look_forward_length=*/15.0,
                          /*look_backward_length=*/5.0);
  EXPECT_TRUE(side_objs.empty());

  // Line skipping obj 3.
  PerceptionObjectBuilder percep_builder_3;
  const Vec2d obj_3_pos(61.209, 0.858);
  const double obj_3_heading = 0.3430;
  const auto obj_3 = percep_builder_3.set_id("line skipping 3")
                         .set_type(OT_VEHICLE)
                         .set_timestamp(1.0)
                         .set_velocity(0.0)
                         .set_yaw(obj_3_heading)
                         .set_length_width(3.0, 1.8)
                         .set_box_center(obj_3_pos)
                         .Build();

  PlannerObjectBuilder builder_3;
  builder_3.set_type(OT_VEHICLE)
      .set_object(obj_3)
      .set_pos(obj_3_pos)
      .set_v(0.0)
      .set_theta(obj_3_heading)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(obj_3_pos, obj_3_heading);
  auto planner_obj_3 = builder_3.Build();

  std::vector<PlannerObject> objects_2 = {planner_obj_1, planner_obj_2,
                                          planner_obj_3};
  const auto st_traj_mgr_2 = SpacetimeTrajectoryManager(
      {}, absl::MakeSpan(objects_2), /*thread_pool=*/nullptr);
  EXPECT_EQ(st_traj_mgr_2.stationary_object_trajs().size(), 3);

  const auto side_objs_2 =
      CrowdedSceneObjects(psmm, st_traj_mgr_2, av_pose, vehicle_geom,
                          /*look_forward_length=*/15.0,
                          /*look_backward_length=*/5.0);
  EXPECT_EQ(side_objs_2.size(), 1);
  const auto find = side_objs_2.find("line skipping 3") != side_objs_2.end();
  EXPECT_TRUE(find);
}

TEST(BuildPathSlBoundaryOnReferencePath, GoStraight) {
  const Vec2d av_position = Vec2d(0.0, 0.0);
  constexpr double kAvHeading = 0.0;
  constexpr double kAvSpeed = Kph2Mps(80.0);
  const auto& psmm = CreateDojoTestPSMM();
  auto param_manager = CreateParamManagerFromCarId("Q2506");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_param_api.vehicle_geometry_params();
  const auto& vehicle_drive = vehicle_param_api.vehicle_drive_params();
  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(av_position.x());
  av_pose.mutable_pos_smooth()->set_y(av_position.y());
  av_pose.set_yaw(kAvHeading);
  av_pose.mutable_vel_body()->set_x(kAvSpeed);
  const ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(av_pose);

  const auto dp_opt = BuildDrivePassageFromLaneId(
      psmm, mapping::ElementId(2448), plan_start_point.path_point(),
      /*max_length=*/100.0);
  EXPECT_TRUE(dp_opt.has_value());
  const auto map_sl_boundary_or =
      BuildPathBoundaryFromDrivePassage(psmm, *dp_opt);
  EXPECT_OK(map_sl_boundary_or);
  const auto av_frenet_or = dp_opt->QueryFrenetCoordinateAt(av_position);
  EXPECT_OK(av_frenet_or);
  const auto& av_frenet_on_dp = av_frenet_or.value();
  const prediction::BicycleModelState plan_start_state{
      .x = av_position.x(),
      .y = av_position.y(),
      .v = kAvSpeed,
      .heading = kAvHeading,
      .acc = plan_start_point.a(),
      .front_wheel_angle = 0.0,  // Use VehicleControlStatusProto later.
  };
  const auto t_l_plf =
      PiecewiseLinearFunction<double, double>({0.0, 8.0}, {0.0, 0.0});
  const double av_length = vehicle_geom.length();
  constexpr double kAccCorridorNominalHorizon = 20.0;  // s.
  const double max_steer =
      vehicle_drive.max_steer_angle() / vehicle_drive.steer_ratio();
  const double& wheelbase = vehicle_geom.wheel_base();

  constexpr double kNominalDtForPath = 0.2;
  const auto ref_path_or =
      prediction::DevelopPolePlacementPathWithSmoothConvergence(
          plan_start_state, *dp_opt, t_l_plf, kNominalDtForPath,
          kAccCorridorNominalHorizon, av_length, max_steer, wheelbase);
  EXPECT_OK(ref_path_or);
  const auto path_sl_map_center_or = BuildPathSlBoundaryOnReferencePath(
      *dp_opt, *map_sl_boundary_or, *ref_path_or, av_position, kAvHeading,
      kAvSpeed, av_frenet_on_dp, /*length=*/80.0, /*corridor_step_s=*/2.0,
      vehicle_geom);
  EXPECT_OK(path_sl_map_center_or);

  // Visualize.
  DrawPathSlBoundaryToCanvas(path_sl_map_center_or->first,
                             "go_straight/path_sl_boundary");
  SendDiscretizedPathToCanvas(*ref_path_or, "go_straight/reference_path",
                              vis::Color::kCrimson);
}

TEST(FindForwardLanePathsAtPoint, Works) {
  FLAGS_planner_acc_corridor_draw_target_lane_choice_debug = true;

  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(0.0);
  av_pose.mutable_pos_smooth()->set_y(0.0);
  av_pose.set_yaw(0.0);
  av_pose.mutable_vel_body()->set_y(10.0);  // m/s.

  ApolloTrajectoryPointProto plan_start_point;
  PathPoint path_point;
  path_point.set_x(2.0);
  path_point.set_y(0.0);  // plan ahead 200 ms.
  path_point.set_theta(0.0);
  path_point.set_kappa(0.0);
  path_point.set_s(0.0);
  plan_start_point.set_v(10.0);
  plan_start_point.set_a(0.0);
  *plan_start_point.mutable_path_point() = path_point;

  const auto lane_paths = FindForwardLanePathsAtPoint(
      Vec2d(plan_start_point.path_point().x(),
            plan_start_point.path_point().y()),
      plan_start_point.path_point().theta(), plan_start_point.v(),
      plan_start_point.path_point().kappa(), psmm,
      /*desire_forward_length=*/150.0);
  EXPECT_EQ(lane_paths.size(), 3);
  std::set<mapping::ElementId> ids;
  for (const auto& lane_path : lane_paths) {
    ids.insert(lane_path.lane_ids().front());
  }
  EXPECT_NE(ids.end(), ids.find(mapping::ElementId(2448)));
  EXPECT_NE(ids.end(), ids.find(mapping::ElementId(3)));
  EXPECT_NE(ids.end(), ids.find(mapping::ElementId(2)));
}

TEST(ResampleSmoothedDiscretizedPathOnXY, Works) {
  std::vector<double> x = {
      -3581.84, -3583.87, -3585.91, -3587.94, -3589.98, -3592.01, -3594.04,
      -3596.08, -3598.11, -3600.15, -3602.18, -3604.22, -3606.25, -3608.28,
      -3610.32, -3612.35, -3614.39, -3616.42, -3618.45, -3620.49, -3622.52,
      -3624.56, -3626.59, -3628.63, -3630.66, -3632.69, -3634.73, -3636.76,
      -3638.8,  -3640.83, -3642.86, -3644.9,  -3646.93, -3648.97, -3651,
      -3653.04, -3655.07, -3657.1,  -3659.14, -3661.17, -3663.21, -3665.24,
      -3667.28};
  std::vector<double> y = {
      -204.744, -204.965, -205.187, -205.408, -205.631, -205.853, -206.076,
      -206.299, -206.522, -206.746, -206.969, -207.193, -207.418, -207.642,
      -207.866, -208.091, -208.316, -208.541, -208.766, -208.991, -209.216,
      -209.441, -209.666, -209.891, -210.116, -210.34,  -210.565, -210.79,
      -211.014, -211.239, -211.463, -211.687, -211.911, -212.134, -212.357,
      -212.58,  -212.803, -213.025, -213.248, -213.469, -213.69,  -213.911,
      -214.132};
  std::vector<double> accum_s = {
      0,       2.04651, 4.09302, 6.13953, 8.18604, 10.2326, 12.2791, 14.3256,
      16.3721, 18.4186, 20.4651, 22.5116, 24.5581, 26.6046, 28.6512, 30.6977,
      32.7442, 34.7907, 36.8372, 38.8837, 40.9302, 42.9767, 45.0232, 47.0697,
      49.1163, 51.1628, 53.2093, 55.2558, 57.3023, 59.3488, 61.3953, 63.4418,
      65.4883, 67.5349, 69.5814, 71.6279, 73.6744, 75.7209, 77.7674, 79.8139,
      81.8604, 83.9069, 85.9535};

  EXPECT_EQ(x.size(), y.size());
  const int num = x.size();
  std::vector<Vec2d> xys;
  xys.reserve(num);
  for (int i = 0; i < num; ++i) {
    xys.push_back(Vec2d(x[i], y[i]));
  }

  // Normal sample step.
  double sample_step_s = 2.0;
  auto smoothed_path_opt =
      ResampleSmoothedDiscretizedPathOnXY(xys, accum_s, sample_step_s);
  EXPECT_TRUE(smoothed_path_opt.has_value());
  auto& smoothed_path = *smoothed_path_opt;
  int path_pt_num = smoothed_path.size();
  auto resampled_step_s =
      smoothed_path[path_pt_num - 1].s() - smoothed_path[path_pt_num - 2].s();
  EXPECT_NEAR(resampled_step_s, sample_step_s, 1e-4);

  // If large sample step s.
  sample_step_s = 20.0;  // m.
  smoothed_path_opt =
      ResampleSmoothedDiscretizedPathOnXY(xys, accum_s, sample_step_s);
  EXPECT_TRUE(smoothed_path_opt.has_value());
  smoothed_path = *smoothed_path_opt;
  path_pt_num = smoothed_path.size();
  resampled_step_s =
      smoothed_path[path_pt_num - 1].s() - smoothed_path[path_pt_num - 2].s();
  EXPECT_NEAR(resampled_step_s, sample_step_s, 1e-4);
  EXPECT_EQ(smoothed_path.size(),
            static_cast<int>(accum_s.back() / sample_step_s) + 1);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
