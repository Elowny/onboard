#include "onboard/planner/plan/acc/acc_target_util.h"

#include <algorithm>
#include <optional>

#include "absl/status/status.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "common/proto/qacc.pb.h"

#include "onboard/base/macros.h"
#include "onboard/global/clock.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"
#include "onboard/planner/plan/acc/acc_corridor_with_map.h"
#include "onboard/planner/plan/acc/acc_defs.h"
#include "onboard/planner/plan/acc/acc_target.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {

namespace planner {

namespace {

PlannerObject BuildStationaryPlannerObject(const Vec2d& pos,
                                           const std::string& id,
                                           double heading, double length,
                                           double width, ObjectType type) {
  PerceptionObjectBuilder percep_builder;
  const auto percep_obj = percep_builder.set_id(id)
                              .set_type(type)
                              .set_velocity(0.0)
                              .set_yaw(heading)
                              .set_length_width(length, width)
                              .set_box_center(pos)
                              .Build();
  PlannerObjectBuilder builder;
  builder.set_type(type)
      .set_object(percep_obj)
      .set_v(0.0)
      .set_theta(heading)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(pos, heading);
  return builder.Build();
}

TEST(ObjectFrenetBoxOnPathType, TouchBounds) {
  const int n = 10;
  std::vector<double> s_vec, center_ls, left_ls, right_ls, target_left_ls,
      target_right_ls;
  s_vec.reserve(n);
  center_ls.reserve(n);
  left_ls.reserve(n);
  right_ls.reserve(n);
  target_left_ls.reserve(n);
  target_right_ls.reserve(n);

  std::vector<Vec2d> center_xys, left_xys, right_xys, target_left_xys,
      target_right_xys;
  center_xys.reserve(n);
  left_xys.reserve(n);
  right_xys.reserve(n);
  target_left_xys.reserve(n);
  target_right_xys.reserve(n);

  const double step_s = 1.0;
  const double left_l = 3.0;
  const double right_l = -3.0;
  const double target_left_l = 1.0;
  const double target_right_l = -1.0;
  const double center_l = 0.0;

  for (double s = 0.0; s < step_s * n - step_s * 0.1; s += step_s) {
    s_vec.push_back(s);
    center_ls.push_back(center_l);
    left_ls.push_back(left_l);
    right_ls.push_back(right_l);
    target_left_ls.push_back(target_left_l);
    target_right_ls.push_back(target_right_l);

    center_xys.emplace_back(s, center_l);
    left_xys.emplace_back(s, left_l);
    right_xys.emplace_back(s, right_l);
    target_left_xys.emplace_back(s, target_left_l);
    target_right_xys.emplace_back(s, target_right_l);
  }

  PathSlBoundary path_sl(
      std::move(s_vec), std::move(center_ls), std::move(right_ls),
      std::move(left_ls), std::move(target_right_ls), std::move(target_left_ls),
      std::move(center_xys), std::move(right_xys), std::move(left_xys),
      std::move(target_right_xys), std::move(target_left_xys));

  const double zero_lateral_distance_buffer = 0.0;
  const double lateral_distance_buffer = 0.5;

  // Touch left target bound.
  FrenetBox fbox{
      .s_max = 2.0,
      .s_min = 1.0,
      .l_max = target_left_l + 0.1,
      .l_min = target_left_l,
  };
  auto on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, zero_lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_TARGET_BOUND);
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_TARGET_BOUND);

  // Touch left bound.
  fbox.l_max = left_l + 0.1;
  fbox.l_min = left_l;
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, zero_lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_BOUND);
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_BOUND);

  // Near left bound, no lateral distance buffer should expect off bound type.
  fbox.l_min = left_l + 1e-3;
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, zero_lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_OFF_BOUND);
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_NEAR_BOUND);

  // Touch left buffer.
  fbox.l_min = left_l + lateral_distance_buffer;
  fbox.l_max = fbox.l_min + 0.1;
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_NEAR_BOUND);

  // Off left bound with buffer.
  fbox.l_min = left_l + lateral_distance_buffer + 0.1;
  fbox.l_max = fbox.l_min + 0.1;
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_OFF_BOUND);

  // Touch right target bound.
  fbox.l_max = target_right_l;
  fbox.l_min = target_right_l - 0.1;
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, zero_lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_TARGET_BOUND);
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_TARGET_BOUND);

  // Touch right bound.
  fbox.l_max = right_l;
  fbox.l_min = right_l - 0.1;
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, zero_lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_BOUND);
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_BOUND);

  // Off bound, near right bound.
  fbox.l_max = right_l - 1e-3;
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, zero_lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_OFF_BOUND);
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_NEAR_BOUND);

  // Off bound, touch right bound buffer.
  fbox.l_max = right_l - lateral_distance_buffer;
  fbox.l_min = fbox.l_max - 0.1;
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_NEAR_BOUND);

  // Off bound, near right bound buffer, do not touch.
  fbox.l_max = right_l - lateral_distance_buffer - 1e-3;
  fbox.l_min = fbox.l_max - 0.1;
  on_path_type =
      ObjectFrenetBoxOnPathType(path_sl, fbox, lateral_distance_buffer);
  EXPECT_EQ(on_path_type, OnPathType::OPT_OFF_BOUND);
}

TEST(ComputeBoxLaneInvasionPercentage, Works) {
  const int n = 10;
  std::vector<double> s_vec, center_ls, left_ls, right_ls, target_left_ls,
      target_right_ls;
  s_vec.reserve(n);
  center_ls.reserve(n);
  left_ls.reserve(n);
  right_ls.reserve(n);
  target_left_ls.reserve(n);
  target_right_ls.reserve(n);

  std::vector<Vec2d> center_xys, left_xys, right_xys, target_left_xys,
      target_right_xys;
  center_xys.reserve(n);
  left_xys.reserve(n);
  right_xys.reserve(n);
  target_left_xys.reserve(n);
  target_right_xys.reserve(n);

  const double step_s = 1.0;
  const double left_l = 3.0;
  const double right_l = -3.0;
  const double target_left_l = 1.0;
  const double target_right_l = -1.0;
  const double center_l = 0.0;

  for (double s = 0.0; s < step_s * n - step_s * 0.1; s += step_s) {
    s_vec.push_back(s);
    center_ls.push_back(center_l);
    left_ls.push_back(left_l);
    right_ls.push_back(right_l);
    target_left_ls.push_back(target_left_l);
    target_right_ls.push_back(target_right_l);

    center_xys.emplace_back(s, center_l);
    left_xys.emplace_back(s, left_l);
    right_xys.emplace_back(s, right_l);
    target_left_xys.emplace_back(s, target_left_l);
    target_right_xys.emplace_back(s, target_right_l);
  }

  const auto frenet_frame_or = BuildQtfmEnhancedKdTreeFrenetFrame(
      absl::MakeSpan(center_xys), /*down_sample_raw_points=*/false);
  EXPECT_OK(frenet_frame_or.status());
  const auto& frenet_frame = *frenet_frame_or;

  PathSlBoundary path_sl(
      std::move(s_vec), std::move(center_ls), std::move(right_ls),
      std::move(left_ls), std::move(target_right_ls), std::move(target_left_ls),
      std::move(center_xys), std::move(right_xys), std::move(left_xys),
      std::move(target_right_xys), std::move(target_left_xys));
  EXPECT_EQ(frenet_frame.start_s(), path_sl.start_s());
  EXPECT_EQ(frenet_frame.end_s(), path_sl.end_s());

  // Out, left.
  const Box2d box(Vec2d(1.0, 3.0), /*heading=*/0.0, /*length=*/1.0,
                  /*width=*/1.0);
  const auto percentage_or = ComputeBoxLaneInvasionPercentage(
      path_sl, frenet_frame, box, /*use_obj_width=*/true);
  EXPECT_OK(percentage_or.status());
  EXPECT_EQ(*percentage_or, -1.5);

  // Out, right.
  const Box2d box2(Vec2d(1.0, -3.0), /*heading=*/0.0, /*length=*/1.0,
                   /*width=*/1.0);
  const auto percentage_or_2 = ComputeBoxLaneInvasionPercentage(
      path_sl, frenet_frame, box2, /*use_obj_width=*/true);
  EXPECT_OK(percentage_or_2.status());
  EXPECT_EQ(*percentage_or_2, -1.5);

  // Out, left.
  const Box2d box3(Vec2d(1.0, 5.0), /*heading=*/0.0, /*length=*/1.0,
                   /*width=*/1.0);
  const auto percentage_or_3 = ComputeBoxLaneInvasionPercentage(
      path_sl, frenet_frame, box3, /*use_obj_width=*/true);
  EXPECT_OK(percentage_or_3.status());
  EXPECT_EQ(*percentage_or_3, -3.5);

  // Out, right.
  const Box2d box4(Vec2d(1.0, -5.0), /*heading=*/0.0, /*length=*/1.0,
                   /*width=*/1.0);
  const auto percentage_or_4 = ComputeBoxLaneInvasionPercentage(
      path_sl, frenet_frame, box4, /*use_obj_width=*/true);
  EXPECT_OK(percentage_or_4.status());
  EXPECT_EQ(*percentage_or_4, -3.5);

  // Longitudinally out, left.
  const Box2d box5(Vec2d(20.0, 1.0), /*heading=*/0.0, /*length=*/1.0,
                   /*width=*/1.0);
  const auto percentage_or_5 = ComputeBoxLaneInvasionPercentage(
      path_sl, frenet_frame, box5, /*use_obj_width=*/true);
  EXPECT_FALSE(percentage_or_5.ok());

  // Longitudinally out, right.
  const Box2d box6(Vec2d(20.0, -1.0), /*heading=*/0.0, /*length=*/1.0,
                   /*width=*/1.0);
  const auto percentage_or_6 = ComputeBoxLaneInvasionPercentage(
      path_sl, frenet_frame, box6, /*use_obj_width=*/true);
  EXPECT_FALSE(percentage_or_6.ok());

  // Half in, right.
  const Box2d box7(Vec2d(5.0, -1.0), /*heading=*/0.0, /*length=*/1.0,
                   /*width=*/1.0);
  const auto percentage_or_7 = ComputeBoxLaneInvasionPercentage(
      path_sl, frenet_frame, box7, /*use_obj_width=*/true);
  EXPECT_OK(percentage_or_7.status());
  EXPECT_EQ(*percentage_or_7, 0.5);

  // Half in, left.
  const Box2d box8(Vec2d(5.0, 1.0), /*heading=*/0.0, /*length=*/1.0,
                   /*width=*/1.0);
  const auto percentage_or_8 = ComputeBoxLaneInvasionPercentage(
      path_sl, frenet_frame, box8, /*use_obj_width=*/true);
  EXPECT_OK(percentage_or_8.status());
  EXPECT_EQ(*percentage_or_8, 0.5);

  // Large box.
  const Box2d box9(Vec2d(5.0, -0.1), /*heading=*/0.0, /*length=*/3.0,
                   /*width=*/2.5);
  const auto percentage_or_9 = ComputeBoxLaneInvasionPercentage(
      path_sl, frenet_frame, box9, /*use_obj_width=*/true);
  EXPECT_OK(percentage_or_9.status());
  EXPECT_EQ(*percentage_or_9, 0.86);
}

TEST(ObjectInFrontOfAvFrontCenterByLength, Works) {
  FrenetBox obj_fbox{
      .s_max = 3.0,
      .s_min = 1.0,
      .l_max = 1.0,
      .l_min = -1.0,
  };
  EXPECT_TRUE(internal::ObjectInFrontOfAvFrontCenterByLength(
      /*object_length=*/obj_fbox.s_max - obj_fbox.s_min, obj_fbox,
      ObjectType::OT_VEHICLE,
      /*av_front_s=*/0.0));
  EXPECT_FALSE(internal::ObjectInFrontOfAvFrontCenterByLength(
      /*object_length=*/obj_fbox.s_max - obj_fbox.s_min, obj_fbox,
      ObjectType::OT_VEHICLE,
      /*av_front_s=*/2.5));

  FrenetBox large_obj_fbox{
      .s_max = 6.0,
      .s_min = 0.0,
      .l_max = 1.0,
      .l_min = -1.0,
  };
  EXPECT_TRUE(internal::ObjectInFrontOfAvFrontCenterByLength(
      /*object_length=*/large_obj_fbox.s_max - large_obj_fbox.s_min,
      large_obj_fbox, ObjectType::OT_LARGE_VEHICLE,
      /*av_front_s=*/1.5));
  EXPECT_FALSE(internal::ObjectInFrontOfAvFrontCenterByLength(
      /*object_length=*/large_obj_fbox.s_max - large_obj_fbox.s_min,
      large_obj_fbox, ObjectType::OT_LARGE_VEHICLE,
      /*av_front_s=*/4.5));
}

TEST(FindPotentialTargets, Works) {
  auto param_manager = CreateParamManagerFromCarId("Q2506");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleParamApi& vehicle_param_api = run_params.vehicle_params();

  const auto& psmm = CreateDojoTestPSMM();
  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(0.0);
  av_pose.mutable_pos_smooth()->set_y(0.0);
  av_pose.set_yaw(0.0);
  av_pose.mutable_vel_body()->set_x(Kph2Mps(40.0));  // 80 km/h.

  const auto plan_start_point = ConvertToTrajPointProto(av_pose);

  const auto& vehicle_geom = vehicle_param_api.vehicle_geometry_params();
  const auto& vehicle_drive = vehicle_param_api.vehicle_drive_params();

  // Build corridor.
  const auto dm_or = BuildDrivingMapTopo(av_pose, psmm, plan_start_point.v());
  EXPECT_OK(dm_or.status());
  const auto corridor_or = BuildAccCorridorFromClosestLanePath(
      av_pose, plan_start_point, psmm, *dm_or,
      /*steering_percentage=*/std::nullopt, vehicle_geom, vehicle_drive,
      /*corridor_step_s=*/2.0, kAccTrajectoryTimeHorizon);
  EXPECT_OK(corridor_or);

  // Build objects and build SpacetimeTrajectoryManager.
  const Vec2d obj_pos_1 = {29.135, -1.134};
  const Vec2d obj_pos_2 = {
      23.504, 3.459};  // Neighboring lane. Not leaders but potential.
  const double length = vehicle_geom.length();
  const double width = vehicle_geom.width();
  const double heading = 0.0;

  std::vector<PlannerObject> objects;
  objects.reserve(2);
  objects.push_back(BuildStationaryPlannerObject(
      obj_pos_1, /*id=*/"leader", heading, length, width, /*type=*/OT_VEHICLE));
  objects.push_back(BuildStationaryPlannerObject(obj_pos_2, /*id=*/"potential",
                                                 heading, length, width,
                                                 /*type=*/OT_VEHICLE));
  auto st_traj_mgr =
      std::make_unique<SpacetimeTrajectoryManager>(SpacetimeTrajectoryManager(
          {}, absl::MakeSpan(objects), /*thread_pool=*/nullptr));

  const auto target_infos =
      FindPotentialTargets(*st_traj_mgr, *corridor_or, /*av_front_s=*/0.0,
                           /*enable_filter_oncoming_objects=*/false);
  EXPECT_EQ(target_infos.size(), 2);
  for (const auto& info : target_infos) {
    LOG(INFO) << info.DebugString();
  }

  // Make leader decisions.
  const auto leaders = MakeLeaderDecisions(absl::MakeSpan(target_infos),
                                           /*overlap_threshold=*/0.3);
  EXPECT_EQ(leaders.size(), 1);
  EXPECT_EQ(leaders.front(), "leader-idx0");

  // Build acc target decisions.
  const auto pair =
      BuildAccTargetDecisions(absl::MakeSpan(leaders), *st_traj_mgr);
  const auto& leader_decisions = pair.first;
  const auto& potential_decisions = pair.second;
  EXPECT_EQ(leader_decisions.size(), 1);
  EXPECT_EQ(potential_decisions.size(), 2);

  // Build AccTargetPerCorridor.
  AccTargetPerCorridor target_per_corridor{
      .leaders = leader_decisions,
      .potential_targets = potential_decisions,
      .timestamp = Clock::Now(),
      .source = AccCorridorSource::MAP,
      .acc_st_traj_mgr = std::move(st_traj_mgr),
  };
  EXPECT_TRUE(target_per_corridor.has_leaders());

  // ToProto test.
  QACCTargetsProto targets_proto;
  target_per_corridor.ToProto(&targets_proto);

  EXPECT_TRUE(targets_proto.has_source());
  EXPECT_EQ(targets_proto.source(), AccCorridorSource::MAP);

  EXPECT_EQ(targets_proto.leaders_size(), 1);
  EXPECT_EQ(targets_proto.potential_targets_size(), 2);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
