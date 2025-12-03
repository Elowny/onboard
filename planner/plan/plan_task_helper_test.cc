#include "onboard/planner/plan/plan_task_helper.h"

#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/lane_point.pb.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/common/global_pose.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/plan/proto/plan_task.pb.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

namespace {

TEST(SplitCruiseByUturnTask, BasicTest) {
  SetMap("dojo");
  auto smm = std::make_shared<mapping::SemanticMapManager>();
  smm->LoadWholeMap().Build();

  const auto& psmm = planner::CreateDojoTestPSMM();

  {
    const mapping::LanePath lane_path(
        smm.get(),
        {mapping::ElementId(6564), mapping::ElementId(6697),
         mapping::ElementId(6643)},
        0.8, 1.0);
    const mapping::LanePoint dest(mapping::ElementId(6643), 1.0);

    const auto new_tasks_or = SplitCruiseByUturnTask(psmm, lane_path, dest);

    EXPECT_OK(new_tasks_or);
    EXPECT_EQ(new_tasks_or->size(), 3);
    EXPECT_EQ((*new_tasks_or)[0].type(), ON_ROAD_CRUISE_PLAN);
    EXPECT_EQ((*new_tasks_or)[1].type(), UTURN_PLAN);
    EXPECT_EQ((*new_tasks_or)[2].type(), ON_ROAD_CRUISE_PLAN);

    EXPECT_EQ((*new_tasks_or)[1]
                  .destination_info()
                  .dest.lane_points->front()
                  .lane_id()
                  .value(),
              6643);
    EXPECT_TRUE(
        (*new_tasks_or)[1].destination_info().uturn_ref_lane_path.has_value());
  }

  {
    // Destination is on uturn
    const mapping::LanePath lane_path(
        smm.get(), {mapping::ElementId(6564), mapping::ElementId(6697)}, 0.8,
        0.7);
    const mapping::LanePoint dest(mapping::ElementId(6697), 0.7);
    const auto new_tasks_or = SplitCruiseByUturnTask(psmm, lane_path, dest);
    EXPECT_OK(new_tasks_or);
    EXPECT_EQ(new_tasks_or->size(), 2);
    EXPECT_EQ((*new_tasks_or)[1].destination_info().dest.lane_points->front(),
              lane_path.back());
  }
}

TEST(ReachedRouteEnd, BasicTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const Vec2d ego_pos(0.0, 0.0);
  const double ego_v = 0.0;
  SetMap("dojo");
  CoordinateConverter cc = CoordinateConverter::FromMap("dojo");

  {
    ReachDestinationCondition condition;
    condition.set_speed_error(1.0);
    condition.mutable_pose_within_radius()->set_radius(1.0);
    condition.mutable_pose_within_radius()->set_heading_error(0.1);
    std::vector<mapping::LanePoint> lane_points = {
        mapping::LanePoint(mapping::ElementId(2448), 0.0)};

    PlanTask task(ON_ROAD_CRUISE_PLAN,
                  PlanTaskDestinationInfo{
                      .dest = PlanTaskDestination{.lane_points = lane_points},
                      .end_speed = 0.0,
                      .condition = condition});

    EXPECT_TRUE(ReachedRouteEnd(ego_pos, ego_v, task, cc, psmm));
  }

  {
    ReachDestinationCondition condition;
    condition.set_speed_error(1.0);
    condition.mutable_pose_within_radius()->set_radius(1.0);
    condition.mutable_pose_within_radius()->set_heading_error(0.1);

    PlanTask task(
        ON_ROAD_CRUISE_PLAN,
        PlanTaskDestinationInfo{
            .dest =
                PlanTaskDestination{.global_pose =
                                        GlobalPose{.pos = Vec3d(0.0, 0.0, 0.0),
                                                   .heading = 0.0}},
            .end_speed = 0.0,
            .condition = condition});

    EXPECT_TRUE(ReachedRouteEnd(ego_pos, ego_v, task, cc, psmm));
  }
}

TEST(CreatePlanTasksQueueFromRoutingResult, BasicTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  {
    RouteManagerOutput route;
    route.route_sections_from_current =
        RouteSections(0.0, 1.0, {mapping::SectionId(12401)},
                      mapping::LanePoint(mapping::ElementId(3), 1.0));

    auto tasks = CreatePlanTasksQueueFromRoutingResult(route, psmm);

    EXPECT_EQ(tasks.size(), 1);
    EXPECT_EQ(tasks.front().type(), ON_ROAD_CRUISE_PLAN);
  }

  {
    RouteManagerOutput route;
    MultipleStopsRequestProto::StopProto stop;
    stop.mutable_stop_point()
        ->mutable_off_road()
        ->mutable_parking_spot()
        ->add_specified_parking_spot_ids(1);
    route.destination_stop = std::move(stop);
    auto tasks = CreatePlanTasksQueueFromRoutingResult(route, psmm);

    EXPECT_EQ(tasks.size(), 1);
    EXPECT_EQ(tasks.front().type(), OFF_ROAD_PLAN);
  }

  {
    RouteManagerOutput route;
    MultipleStopsRequestProto::StopProto stop;
    stop.mutable_stop_point()
        ->mutable_off_road()
        ->mutable_parking_spot()
        ->add_specified_parking_spot_ids(1);
    route.destination_stop = std::move(stop);
    auto tasks = CreatePlanTasksQueueFromRoutingResult(route, psmm);

    EXPECT_EQ(tasks.size(), 1);
    EXPECT_EQ(tasks.front().type(), OFF_ROAD_PLAN);
  }

  {
    RouteManagerOutput route;
    route.route_sections_from_current =
        RouteSections(0.0, 1.0, {mapping::SectionId(12401)},
                      mapping::LanePoint(mapping::ElementId(3), 1.0));
    MultipleStopsRequestProto::StopProto stop;
    auto* onroad_point = stop.mutable_depart_strategy()
                             ->mutable_off_road()
                             ->add_specified_onroad_points();
    onroad_point->mutable_lane_point()->set_lane_id(3);
    onroad_point->mutable_lane_point()->set_fraction(1.0);

    stop.mutable_stop_strategy()
        ->mutable_parking_spot()
        ->add_specified_parking_spot_ids(1);
    route.destination_stop = std::move(stop);

    auto tasks = CreatePlanTasksQueueFromRoutingResult(route, psmm);

    EXPECT_EQ(tasks.size(), 3);
    EXPECT_EQ(tasks[0].type(), OFF_ROAD_PLAN);
    EXPECT_EQ(tasks[1].type(), ON_ROAD_CRUISE_PLAN);
    EXPECT_EQ(tasks[2].type(), OFF_ROAD_PLAN);
  }
}

TEST(CreateUturnTaskInfo, BasicTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  {
    PoseProto ego_pose;
    ego_pose.mutable_vel_body()->set_x(10.0);
    auto result =
        CreateUturnTask(psmm, ego_pose, mapping::LanePath(),
                        mapping::LanePoint(), std::nullopt, TrajectoryProto());

    EXPECT_NOT_OK(result);
  }

  {
    PoseProto ego_pose;
    ego_pose.mutable_vel_body()->set_x(0.0);

    TrajectoryProto traj;
    for (int i = 0; i < 50; ++i) {
      traj.add_trajectory_point()->set_v(1.0);
    }

    auto result = CreateUturnTask(psmm, ego_pose, mapping::LanePath(),
                                  mapping::LanePoint(), std::nullopt, traj);

    EXPECT_NOT_OK(result);
  }

  {
    PoseProto ego_pose;
    ego_pose.mutable_vel_body()->set_x(0.0);
    const mapping::LanePath prev_target_lane_path(
        psmm.semantic_map_manager(),
        {mapping::ElementId(7266), mapping::ElementId(6675)}, 0.0, 0.5);
    TrajectoryProto traj;
    for (int i = 0; i < 50; ++i) {
      traj.add_trajectory_point()->set_v(0.0);
    }
    TrajectoryEndInfoProto end_info;
    end_info.set_type(StBoundarySourceTypeProto::IMPASSABLE_BOUNDARY);

    auto result = CreateUturnTask(
        psmm, ego_pose, prev_target_lane_path,
        mapping::LanePoint(mapping::ElementId(6671), 1.0), end_info, traj);

    EXPECT_OK(result);
  }
}

TEST(PlanTaskCompeleted, BasicTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  SetMap("dojo");
  CoordinateConverter cc = CoordinateConverter::FromMap("dojo");

  {
    PlanTask task(UTURN_PLAN);
    AutonomyStateProto auto_state;
    auto_state.set_autonomy_state(AutonomyStateProto::NOT_READY);
    EXPECT_TRUE(PlanTaskCompeleted(task, auto_state, CoordinateConverter(),
                                   Vec2d(), 0.0, 0.0, psmm));
  }

  {
    ReachDestinationCondition condition;
    condition.set_speed_error(1.0);
    condition.mutable_pose_within_radius()->set_radius(1.0);
    condition.mutable_pose_within_radius()->set_heading_error(0.1);

    PlanTask task(
        ON_ROAD_CRUISE_PLAN,
        PlanTaskDestinationInfo{
            .dest =
                PlanTaskDestination{.global_pose =
                                        GlobalPose{.pos = Vec3d(0.0, 0.0, 0.0),
                                                   .heading = 0.0}},
            .end_speed = 0.0,
            .condition = condition});

    AutonomyStateProto auto_state;
    auto_state.set_autonomy_state(AutonomyStateProto::AUTO_DRIVE);
    EXPECT_TRUE(PlanTaskCompeleted(task, auto_state, cc, Vec2d(0.0, 0.0), 0.0,
                                   0.0, psmm));

    EXPECT_FALSE(PlanTaskCompeleted(task, auto_state, cc, Vec2d(3.0, 0.0), 0.0,
                                    0.0, psmm));

    EXPECT_FALSE(PlanTaskCompeleted(task, auto_state, cc, Vec2d(0.0, 0.0), 0.4,
                                    0.0, psmm));

    EXPECT_FALSE(PlanTaskCompeleted(task, auto_state, cc, Vec2d(0.0, 0.0), 0.0,
                                    2.0, psmm));
  }
}

TEST(CreateBlockedRoadTask, BasicTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  PoseProto pose_proto;
  pose_proto.mutable_pos_smooth()->set_x(0.0);
  pose_proto.mutable_pos_smooth()->set_y(0.0);
  auto result =
      CreateBlockedRoadTask(psmm, pose_proto, DefaultVehicleGeometry());
  EXPECT_OK(result);
}

TEST(IsHdMapBasedTask, BasicTest) {
  EXPECT_TRUE(IsHdMapBasedTask(ON_ROAD_CRUISE_PLAN));
  EXPECT_TRUE(IsHdMapBasedTask(OFF_ROAD_PLAN));
  EXPECT_TRUE(IsHdMapBasedTask(UTURN_PLAN));
  EXPECT_TRUE(IsHdMapBasedTask(BLOCKED_PLAN));

  EXPECT_FALSE(IsHdMapBasedTask(ALCC_PLAN));
  EXPECT_FALSE(IsHdMapBasedTask(ACC_PLAN));
  EXPECT_FALSE(IsHdMapBasedTask(MAPLESS_NOA));
}

TEST(ClassifyTaskErrorToPlannerStatus, BasicTest) {
  {
    const auto planner_status =
        ClassifyTaskErrorToPlannerStatus(ON_ROAD_CRUISE_PLAN, "Failed!!!",
                                         /*is_driverless_mode=*/true);
    EXPECT_EQ(planner_status.status_code(),
              PlannerStatusProto::DRIVERLESS_PLAN_MAIN_LOOP_FAILED);
  }
  {
    const auto planner_status =
        ClassifyTaskErrorToPlannerStatus(ON_ROAD_CRUISE_PLAN, "Failed!!!",
                                         /*is_driverless_mode=*/false);
    EXPECT_EQ(planner_status.status_code(),
              PlannerStatusProto::NOA_MAIN_LOOP_FAILED);
  }
  {
    const auto planner_status =
        ClassifyTaskErrorToPlannerStatus(OFF_ROAD_PLAN, "Failed!!!",
                                         /*is_driverless_mode=*/true);
    EXPECT_EQ(planner_status.status_code(),
              PlannerStatusProto::DRIVERLESS_PLAN_MAIN_LOOP_FAILED);
  }
  {
    const auto planner_status =
        ClassifyTaskErrorToPlannerStatus(UTURN_PLAN, "Failed!!!",
                                         /*is_driverless_mode=*/true);
    EXPECT_EQ(planner_status.status_code(),
              PlannerStatusProto::DRIVERLESS_PLAN_MAIN_LOOP_FAILED);
  }
  {
    const auto planner_status =
        ClassifyTaskErrorToPlannerStatus(BLOCKED_PLAN, "Failed!!!",
                                         /*is_driverless_mode=*/true);
    EXPECT_EQ(planner_status.status_code(),
              PlannerStatusProto::DRIVERLESS_PLAN_MAIN_LOOP_FAILED);
  }
  {
    const auto planner_status =
        ClassifyTaskErrorToPlannerStatus(ALCC_PLAN, "Failed!!!",
                                         /*is_driverless_mode=*/false);
    EXPECT_EQ(planner_status.status_code(),
              PlannerStatusProto::ALCC_MAIN_LOOP_FAILED);
  }
  {
    const auto planner_status =
        ClassifyTaskErrorToPlannerStatus(ACC_PLAN, "Failed!!!",
                                         /*is_driverless_mode=*/false);
    EXPECT_EQ(planner_status.status_code(),
              PlannerStatusProto::ACC_MAIN_LOOP_FAILED);
  }
  {
    const auto planner_status =
        ClassifyTaskErrorToPlannerStatus(MAPLESS_NOA, "Failed!!!",
                                         /*is_driverless_mode=*/false);
    EXPECT_EQ(planner_status.status_code(),
              PlannerStatusProto::MAPLESS_NOA_MAIN_LOOP_FAILED);
  }
}

}  // namespace

}  // namespace qcraft::planner
