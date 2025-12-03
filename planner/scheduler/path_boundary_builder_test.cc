#include "onboard/planner/scheduler/path_boundary_builder.h"

// IWYU pragma: no_include "onboard/global/buffered_logger.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {

namespace {

constexpr double kEpsilon = 0.1;  // m

PlannerObjectManager BuildPhantomVehicle(const Vec2d& obj_pos) {
  ObjectVector<PlannerObject> objects;
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id("Phantom")
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(obj_pos)
                            .set_length_width(4.5, 2.2)
                            .set_yaw(0.0)
                            .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_stationary_traj(obj_pos, 0.0);

  objects.push_back(builder.Build());
  return PlannerObjectManager(objects);
}

TEST(BuildPathBoundaryFromPose, BuildPathBoundaryFromPoseTest) {
  const TestRouteResult route_result = CreateAStraightForwardRouteInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  FLAGS_planner_enable_path_boundary_debug = true;
  const auto& psmm = CreateDojoTestPSMM();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, route_result.route_sections,
          route_result.route_lane_path.lane_paths().front(),
          /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or =
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                        route_result.route_lane_path.lane_paths().front(),
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(psmm),
                        route_result.route_sections.destination(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt);

  EXPECT_OK(dp_or);
  SendDrivePassageToCanvas(dp_or.value(), "multi/straight_drive_passage");

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  PlannerObjectManager object_mgr;
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;

  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(
        route_result.pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(
        route_result.pose.pos_smooth().y());
    plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
    plan_start_point.set_v(route_result.pose.vel_body().x());

    const auto path_bound_or = BuildPathBoundaryFromPose(
        psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_mgr,
        lc_state, smooth_result_map,
        /*borrow_lane_boundary=*/false,
        /*<should_smooth_next_left_turn=*/false, /*unsafe_object_ids=*/{});

    EXPECT_OK(path_bound_or);

    DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                               "multi/straight_path_boundary");
  }

  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(118.286);
    plan_start_point.mutable_path_point()->set_y(3.23);
    plan_start_point.mutable_path_point()->set_theta(-0.17);
    plan_start_point.set_v(0.0);

    lc_state.set_lc_left(false);

    {
      lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
      const auto path_bound_or = BuildPathBoundaryFromPose(
          psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_mgr,
          lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/false,
          /*<should_smooth_next_left_turn=*/false, /*unsafe_object_ids=*/{});

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/straight_path_boundary_lc");
    }

    {
      lc_state.set_stage(LaneChangeStage::LCS_PAUSE);
      const auto path_bound_or = BuildPathBoundaryFromPose(
          psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_mgr,
          lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/false,
          /*<should_smooth_next_left_turn=*/false, /*unsafe_object_ids=*/{});

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/straight_path_boundary_lcp");
    }
  }
}

TEST(BuildPathBoundaryFromPose, SolidLineTest) {
  const TestRouteResult route_result =
      CreateAStraightForwardRouteWithSolidInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, route_result.route_sections,
          route_result.route_lane_path.lane_paths().front(),
          /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or =
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                        route_result.route_lane_path.lane_paths().front(),
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(psmm),
                        route_result.route_sections.destination(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt);

  EXPECT_OK(dp_or);

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  PlannerObjectManager object_mgr;
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;

  {
    // solid_line_path_boundary
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(
        route_result.pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(
        route_result.pose.pos_smooth().y());
    plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
    plan_start_point.set_v(route_result.pose.vel_body().x());

    {
      const auto path_bound_or = BuildPathBoundaryFromPose(
          psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_mgr,
          lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/false,
          /*<should_smooth_next_left_turn=*/false, /*unsafe_object_ids=*/{});

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/solid_line_path_boundary");

      const auto [right_l_ext, left_l_ext] =
          path_bound_or->QueryBoundaryL(20.0);
      EXPECT_NEAR(right_l_ext, -1.9, kEpsilon);
      EXPECT_NEAR(left_l_ext, 1.65, kEpsilon);

      const auto [right_l, left_l] = path_bound_or->QueryTargetBoundaryL(20.0);
      EXPECT_NEAR(right_l, -1.9, kEpsilon);
      EXPECT_NEAR(left_l, 1.65, kEpsilon);
    }

    {
      // solid_line_path_boundary_borrow
      const auto path_bound_or = BuildPathBoundaryFromPose(
          psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_mgr,
          lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/true,
          /*<should_smooth_next_left_turn=*/false, /*unsafe_object_ids=*/{});

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/solid_line_path_boundary_borrow");

      const auto [right_l_ext, left_l_ext] =
          path_bound_or->QueryBoundaryL(20.0);
      EXPECT_LT(right_l_ext, -1.9 - kEpsilon);
      EXPECT_GT(left_l_ext, 1.65 + kEpsilon);

      const auto [right_l, left_l] = path_bound_or->QueryTargetBoundaryL(20.0);
      EXPECT_LT(right_l, -1.9 - kEpsilon);
      EXPECT_GT(left_l, 1.65 + kEpsilon);
    }
  }

  {
    // solid_line_path_boundary_lc
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(
        route_result.pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(3.23);
    plan_start_point.mutable_path_point()->set_theta(-0.17);
    plan_start_point.set_v(0.0);

    lc_state.set_lc_left(false);

    {
      lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
      const auto path_bound_or = BuildPathBoundaryFromPose(
          psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_mgr,
          lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/false,
          /*<should_smooth_next_left_turn=*/false, /*unsafe_object_ids=*/{});

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/solid_line_path_boundary_lc");

      const auto [right_l_ext, left_l_ext] =
          path_bound_or->QueryBoundaryL(20.0);
      EXPECT_NEAR(right_l_ext, -1.9, kEpsilon);
      EXPECT_GT(left_l_ext, 1.65 + kEpsilon);

      const auto [right_l, left_l] = path_bound_or->QueryTargetBoundaryL(20.0);
      EXPECT_NEAR(right_l, -1.9, kEpsilon);
      EXPECT_GT(left_l, 1.65 + kEpsilon);
    }

    {
      lc_state.set_stage(LaneChangeStage::LCS_PAUSE);
      const auto path_bound_or = BuildPathBoundaryFromPose(
          psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_mgr,
          lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/false,
          /*<should_smooth_next_left_turn=*/false, /*unsafe_object_ids=*/{});

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/solid_line_path_boundary_lcp");
    }
  }
}

TEST(BuildPathBoundaryFromPose, ObjectTest) {
  const TestRouteResult route_result = CreateAStraightForwardRouteInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, route_result.route_sections,
          route_result.route_lane_path.lane_paths().front(),
          /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or =
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                        route_result.route_lane_path.lane_paths().front(),
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(psmm),
                        route_result.route_sections.destination(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt);

  EXPECT_OK(dp_or);

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(
      route_result.pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(
      route_result.pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
  plan_start_point.set_v(route_result.pose.vel_body().x());

  // Build one object.
  const auto object_mgr = BuildPhantomVehicle(/*obj_pos=*/Vec2d(140.0, -3.5));
  DrawPlannerObjectManagerToCanvas(object_mgr, "multi/static_object",
                                   vis::Color::kLightGreen);
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;

  const auto path_bound_or = BuildPathBoundaryFromPose(
      psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_mgr,
      lc_state, smooth_result_map,
      /*borrow_lane_boundary=*/false, /*<should_smooth_next_left_turn=*/false,
      /*unsafe_object_ids=*/{});

  EXPECT_OK(path_bound_or);

  DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                             "multi/static_object_path_boundary");

  const auto [right_l_ext, left_l_ext] = path_bound_or->QueryBoundaryL(23.5);
  const auto [right_l, left_l] = path_bound_or->QueryTargetBoundaryL(23.5);
  EXPECT_NEAR(right_l_ext, right_l, kEpsilon);
  EXPECT_GT(left_l_ext, left_l + kEpsilon);
}

TEST(BuildPathBoundaryFromPose, SmoothTest) {
  {
    const TestRouteResult route_result =
        CreateALeftTurnWithDirectionInfoRouteInDojo();
    EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

    const auto& psmm = CreateDojoTestPSMM();
    const auto backward_extended_lane_path =
        BackwardExtendLanePathOnRouteSections(
            psmm, route_result.route_sections,
            route_result.route_lane_path.lane_paths().front(),
            /*extend_len=*/10.0);
    EXPECT_OK(backward_extended_lane_path);
    const auto dp_or =
        BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                          route_result.route_lane_path.lane_paths().front(),
                          *backward_extended_lane_path,
                          /*anchor_point=*/mapping::LanePoint(),
                          route_result.route_sections.planning_horizon(psmm),
                          route_result.route_sections.destination(),
                          /*all_lanes_virtual=*/false,
                          /*override_speed_limit_mps=*/std::nullopt);

    EXPECT_OK(dp_or);
    SendDrivePassageToCanvas(dp_or.value(),
                             "multi/smooth/left_turn/drive_passage");

    VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(848.186);
    plan_start_point.mutable_path_point()->set_y(-594.204);
    plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
    plan_start_point.set_v(route_result.pose.vel_body().x());

    PlannerObjectManager object_mgr;
    const auto st_traj_mgr =
        SpacetimeTrajectoryManager(object_mgr.planner_objects());

    LaneChangeStateProto lc_state;
    lc_state.set_stage(LaneChangeStage::LCS_NONE);
    SmoothedReferenceLineResultMap smooth_result_map;

    const auto path_bound_or = BuildPathBoundaryFromPose(
        psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_mgr,
        lc_state, smooth_result_map,
        /*borrow_lane_boundary=*/false, /*<should_smooth_next_left_turn=*/true,
        /*unsafe_object_ids=*/{});

    EXPECT_OK(path_bound_or);

    for (int i = 0; i < path_bound_or->size(); ++i) {
      EXPECT_GT(path_bound_or->target_left_l_vector()[i],
                path_bound_or->reference_center_l_vector()[i]);
      EXPECT_LT(path_bound_or->target_right_l_vector()[i],
                path_bound_or->reference_center_l_vector()[i]);
    }
    DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                               "multi/smooth/left_turn/path_boundary");
  }

  {
    const TestRouteResult route_result =
        CreateARightTurnWithDirectionInfoRouteInDojo();
    EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

    const auto& psmm = CreateDojoTestPSMM();
    const auto backward_extended_lane_path =
        BackwardExtendLanePathOnRouteSections(
            psmm, route_result.route_sections,
            route_result.route_lane_path.lane_paths().front(),
            /*extend_len=*/10.0);
    EXPECT_OK(backward_extended_lane_path);
    const auto dp_or =
        BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                          route_result.route_lane_path.lane_paths().front(),
                          *backward_extended_lane_path,
                          /*anchor_point=*/mapping::LanePoint(),
                          route_result.route_sections.planning_horizon(psmm),
                          route_result.route_sections.destination(),
                          /*all_lanes_virtual=*/false,
                          /*override_speed_limit_mps=*/std::nullopt);

    EXPECT_OK(dp_or);
    SendDrivePassageToCanvas(dp_or.value(),
                             "multi/smooth/right_turn/drive_passage");

    VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(848.186);
    plan_start_point.mutable_path_point()->set_y(-594.204);
    plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
    plan_start_point.set_v(route_result.pose.vel_body().x());

    PlannerObjectManager object_mgr;
    const auto st_traj_mgr =
        SpacetimeTrajectoryManager(object_mgr.planner_objects());

    LaneChangeStateProto lc_state;
    lc_state.set_stage(LaneChangeStage::LCS_NONE);
    SmoothedReferenceLineResultMap smooth_result_map;

    const auto path_bound_or = BuildPathBoundaryFromPose(
        psmm, dp_or.value(), plan_start_point, veh_geo_params, st_traj_mgr,
        lc_state, smooth_result_map,
        /*borrow_lane_boundary=*/false, /*<should_smooth_next_left_turn=*/true,
        /*unsafe_object_ids=*/{});

    EXPECT_OK(path_bound_or);

    for (int i = 0; i < path_bound_or->size(); ++i) {
      EXPECT_GT(path_bound_or->target_left_l_vector()[i],
                path_bound_or->reference_center_l_vector()[i]);
      EXPECT_LT(path_bound_or->target_right_l_vector()[i],
                path_bound_or->reference_center_l_vector()[i]);
    }
    DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                               "multi/smooth/right_turn/path_boundary");
  }
}

TEST(BuildPathBoundaryFromDrivePassage, BuildPathBoundaryFromDrivePassageTest) {
  const TestRouteResult route_result = CreateALeftTurnRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, route_result.route_sections,
          route_result.route_lane_path.lane_paths().front(),
          /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or =
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                        route_result.route_lane_path.lane_paths().front(),
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(psmm),
                        route_result.route_sections.destination(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt);

  EXPECT_OK(dp_or);
  SendDrivePassageToCanvas(dp_or.value(),
                           "multi/build_from_drive_passage_test");

  const auto path_bound_or =
      BuildPathBoundaryFromDrivePassage(psmm, dp_or.value());

  EXPECT_OK(path_bound_or);

  DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                             "multi/build_from_drive_passage");
}

TEST(BuildPathBoundaryForLaneKeeping, BuildPathBoundaryForLaneKeepingTest) {
  const TestRouteResult route_result = CreateAStraightForwardRouteInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, route_result.route_sections,
          route_result.route_lane_path.lane_paths().front(),
          /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or =
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                        route_result.route_lane_path.lane_paths().front(),
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(psmm),
                        route_result.route_sections.destination(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt);

  EXPECT_OK(dp_or);

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(118.286);
  plan_start_point.mutable_path_point()->set_y(0.0);
  plan_start_point.set_v(10.0);

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;
  const auto path_bound_or = BuildPathBoundaryForLaneKeeping(
      psmm, *dp_or, plan_start_point, veh_geo_params, online_map_drift_buffer);

  EXPECT_OK(path_bound_or);

  DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                             "multi/build_for_lane_keeping/HD map");
}

TEST(BuildPathBoundaryForLaneKeeping, OnlineMapTest) {
  const Vec2d ego_pos(118.286, 0.0);

  const auto& whole_psmm = CreateDojoTestPSMM();
  ASSIGN_OR_DIE(const auto online_smm_proto,
                RunOnlineSemanticMapConverter(whole_psmm,
                                              OnlineSemanticMapConverterOption{
                                                  .smooth_x = ego_pos.x(),
                                                  .smooth_y = ego_pos.y(),
                                                  .smooth_yaw = 0.0,
                                                  .look_ahead_distance = 100.0,
                                                  .look_back_distance = 10.0,
                                              }));

  // Build psmm according online semantic map.
  ASSIGN_OR_DIE(const auto psmm_ptr, BuildOnlineMapPsmm(online_smm_proto));
  const auto& psmm = *psmm_ptr;

  const mapping::LanePath lane_path(
      psmm.semantic_map_manager(), {mapping::ElementId(2471)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/10.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(ego_pos.x());
  plan_start_point.mutable_path_point()->set_y(ego_pos.y());
  plan_start_point.set_v(10.0);

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;
  const auto path_bound_or =
      BuildPathBoundaryForLaneKeeping(psmm, drive_passage, plan_start_point,
                                      veh_geo_params, online_map_drift_buffer);

  EXPECT_OK(path_bound_or);

  DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                             "multi/build_for_lane_keeping/vision map");
}

TEST(BuildPathBoundaryForLaneChange, BuildPathBoundaryForLaneChangeTest) {
  const TestRouteResult route_result = CreateAStraightForwardRouteInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, route_result.route_sections,
          route_result.route_lane_path.lane_paths().front(),
          /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or =
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                        route_result.route_lane_path.lane_paths().front(),
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(psmm),
                        route_result.route_sections.destination(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt);

  EXPECT_OK(dp_or);

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  PlannerObjectManager object_mgr;
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
  lc_state.set_lc_left(false);

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(118.286);
  plan_start_point.mutable_path_point()->set_y(2.5);
  plan_start_point.mutable_path_point()->set_theta(-0.17);
  plan_start_point.set_v(10.0);

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;
  const auto path_bound_or = BuildPathBoundaryForLaneChange(
      psmm, *dp_or, plan_start_point, veh_geo_params, st_traj_mgr, lc_state,
      online_map_drift_buffer);

  EXPECT_OK(path_bound_or);

  DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                             "multi/build_for_lane_change/HD map");
}

TEST(BuildPathBoundaryForLaneChange, OnlineMapTest) {
  const Vec2d ego_pos(118.286, 2.5);

  const auto& whole_psmm = CreateDojoTestPSMM();
  ASSIGN_OR_DIE(const auto online_smm_proto,
                RunOnlineSemanticMapConverter(whole_psmm,
                                              OnlineSemanticMapConverterOption{
                                                  .smooth_x = ego_pos.x(),
                                                  .smooth_y = ego_pos.y(),
                                                  .smooth_yaw = 0.0,
                                                  .look_ahead_distance = 100.0,
                                                  .look_back_distance = 10.0,
                                              }));

  // Build psmm according online semantic map.
  ASSIGN_OR_DIE(const auto psmm_ptr, BuildOnlineMapPsmm(online_smm_proto));
  const auto& psmm = *psmm_ptr;

  const mapping::LanePath lane_path(
      psmm.semantic_map_manager(), {mapping::ElementId(2471)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/10.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  PlannerObjectManager object_mgr;
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
  lc_state.set_lc_left(false);

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(ego_pos.x());
  plan_start_point.mutable_path_point()->set_y(ego_pos.y());
  plan_start_point.mutable_path_point()->set_theta(-0.17);
  plan_start_point.set_v(10.0);

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;
  const auto path_bound_or = BuildPathBoundaryForLaneChange(
      psmm, drive_passage, plan_start_point, veh_geo_params, st_traj_mgr,
      lc_state, online_map_drift_buffer);

  EXPECT_OK(path_bound_or);

  DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                             "multi/build_for_lane_change/vision map");
}

}  // namespace
}  // namespace qcraft::planner
