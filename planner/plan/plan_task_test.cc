#include "onboard/planner/plan/plan_task.h"

#include "gtest/gtest.h"

namespace qcraft::planner {

namespace {

TEST(PlanTask, BasicTest) {
  {
    PlanTask task(ACC_PLAN);
    PlanTaskProto proto;
    task.ToProto(&proto, 0);
    PlanTask task_copy = task;
    task.FromProto(proto);
    EXPECT_EQ(task_copy.type(), task.type());
  }

  {
    ReachDestinationCondition condition;
    condition.set_speed_error(1.0);
    condition.mutable_pose_within_radius()->set_radius(10.0);
    condition.mutable_pose_within_radius()->set_heading_error(0.1);
    std::vector<mapping::LanePoint> lane_points = {
        mapping::LanePoint(mapping::ElementId(100), 0.0)};

    PlanTask task(ON_ROAD_CRUISE_PLAN,
                  PlanTaskDestinationInfo{
                      .dest = PlanTaskDestination{.lane_points = lane_points},
                      .end_speed = 0.0,
                      .condition = condition});
    PlanTaskProto proto;
    task.ToProto(&proto, 0);
    task.FromProto(proto);
    PlanTaskProto proto_copy;
    task.ToProto(&proto_copy, 0);

    EXPECT_EQ(proto.DebugString(), proto_copy.DebugString());
  }

  {
    ReachDestinationCondition condition;
    condition.set_speed_error(1.0);
    condition.mutable_pose_within_radius()->set_radius(10.0);
    condition.mutable_pose_within_radius()->set_heading_error(0.1);
    std::vector<mapping::ElementId> parking_spots = {mapping::ElementId(100)};

    PlanTask task(
        ON_ROAD_CRUISE_PLAN,
        PlanTaskDestinationInfo{
            .dest = PlanTaskDestination{.parking_spots = parking_spots},
            .end_speed = 0.0,
            .condition = condition});
    PlanTaskProto proto;
    task.ToProto(&proto, 0);
    task.FromProto(proto);
    PlanTaskProto proto_copy;
    task.ToProto(&proto_copy, 0);

    EXPECT_EQ(proto.DebugString(), proto_copy.DebugString());
  }

  {
    ReachDestinationCondition condition;
    condition.set_speed_error(1.0);
    condition.mutable_pose_within_radius()->set_radius(10.0);
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
    PlanTaskProto proto;
    task.ToProto(&proto, 0);
    task.FromProto(proto);
    PlanTaskProto proto_copy;
    task.ToProto(&proto_copy, 0);

    EXPECT_EQ(proto.DebugString(), proto_copy.DebugString());
  }

  {
    mapping::LanePathProto lane_path;
    lane_path.add_lane_ids(1);
    lane_path.set_start_fraction(0.0);
    lane_path.set_end_fraction(1.0);
    lane_path.set_lane_path_in_forward_direction(true);
    lane_path.add_lane_lengths(10.0);

    PlanTask task(UTURN_PLAN,
                  PlanTaskDestinationInfo{.uturn_ref_lane_path = lane_path});
    PlanTaskProto proto;
    task.ToProto(&proto, 0);
    task.FromProto(proto);
    PlanTaskProto proto_copy;
    task.ToProto(&proto_copy, 0);

    EXPECT_EQ(proto.DebugString(), proto_copy.DebugString());
  }
}

}  // namespace
}  // namespace qcraft::planner
