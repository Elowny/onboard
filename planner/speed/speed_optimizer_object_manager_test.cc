#include "onboard/planner/speed/speed_optimizer_object_manager.h"

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft::planner {
namespace {

SpeedVector BuildConstAccelSpeedVector(double v0, double acc) {
  constexpr int kSpeedPointNum = 100;
  SpeedVector speed_vector;
  for (int i = 0; i < kSpeedPointNum; ++i) {
    const double t = 0.1 * i;
    const double v = v0 + t * acc;
    const double s = v0 * t + 0.5 * acc * t * t;
    speed_vector.emplace_back(t, s, v, acc, /*j=*/0.0);
  }
  return speed_vector;
}

const auto kPlannerParams = DefaultPlannerParams();
const double kPlanTotalTime = 10.0;  // m/s.
const int kKnotNum =
    kPlannerParams.speed_finder_params().speed_optimizer_params().knot_num();
const double kPlanTimeInterval = kPlanTotalTime / (kKnotNum - 1);
constexpr double kEps = 1e-6;

ObjectProto BuildPerceptionObject(const std::string& id, const Vec2d& obj_pos,
                                  ObjectType object_type, double init_v,
                                  double heading, double timestamp) {
  return PerceptionObjectBuilder()
      .set_id(id)
      .set_type(object_type)
      .set_timestamp(timestamp)
      .set_velocity(init_v)
      .set_yaw(heading)
      .set_length_width(/*length=*/4.0, /*width=*/2.0)
      .set_box_center(obj_pos)
      .Build();
}

StBoundaryRef PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
    const std::string& id, const Vec2d& obj_pos, const Vec2d& end_pos,
    double init_v, double end_v, bool is_stationary, double prob,
    PlannerObjectBuilder* builder) {
  if (is_stationary) {
    builder->get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(prob)
        .set_stationary_traj(obj_pos, /*theta=*/0.0);
  } else {
    builder->get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(prob)
        .set_straight_line(obj_pos, end_pos, init_v, end_v);
  }
  constexpr double kBoundaryWidth = 5.0;
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(obj_pos.x(), 0.0),
                                     StPoint(end_pos.x(), 10.0)};
  st_boundary_points.upper_points = {
      StPoint(obj_pos.x() + kBoundaryWidth, 0.0),
      StPoint(end_pos.x() + kBoundaryWidth, 10.0)};
  st_boundary_points.speed_points = {VtPoint(init_v, 0.0),
                                     VtPoint(end_v, 10.0)};
  return StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VEHICLE, id, prob, is_stationary,
      StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
}

void CheckMergeBoundary(const SpeedOptimizerObject& object,
                        const StBoundaryWithDecision& boundary0,
                        const StBoundaryWithDecision& boundary1, bool leading,
                        double prob) {
  for (int idx = 0; idx < kKnotNum; ++idx) {
    const auto& state = object.GetOverlapStateByIndex(idx);
    QCHECK_NEAR(state->prob, prob, kEps);
    const double time = kPlanTimeInterval * static_cast<double>(idx);
    const auto s_range0 = boundary0.raw_st_boundary()->GetBoundarySRange(time);
    const auto speed0 =
        boundary0.raw_st_boundary()->GetStBoundarySpeedAtT(time);
    const auto s_range1 = boundary1.raw_st_boundary()->GetBoundarySRange(time);
    const auto speed1 =
        boundary1.raw_st_boundary()->GetStBoundarySpeedAtT(time);
    QCHECK(s_range0.has_value());
    QCHECK(speed0.has_value());
    QCHECK(s_range1.has_value());
    QCHECK(speed1.has_value());
    QCHECK_GE(state->bound, leading ? s_range1->first : s_range1->second);
    QCHECK_LE(state->bound, leading ? s_range0->first : s_range0->second);
    QCHECK_GE(state->speed, *speed1);
    QCHECK_LE(state->speed, *speed0);
    QCHECK_GE(state->lon_buffer, 0.0);
  }
}

TEST(SpeedOptimizerObjectManagerTest, OneFollowBoundaryMerge) {
  const SpeedVector preliminary_speed =
      BuildConstAccelSpeedVector(/*v0=*/0.0, /*acc=*/1.0);

  const std::string id = "001";
  const Vec2d obj_pos = Vec2d(20.0, 0.0);
  const ObjectType object_type = OT_VEHICLE;
  const double heading = 0.0;
  const double timestamp = 0.0;
  const Vec2d end_pos = Vec2d(120.0, 0.0);
  const double init_v = 10.0;
  const double end_v = 10.0;

  const auto obj = BuildPerceptionObject(id, obj_pos, object_type, init_v,
                                         heading, timestamp);
  PlannerObjectBuilder builder;
  builder.set_type(object_type).set_object(obj).set_stationary(false);

  StBoundaryRef st_boundary_moving =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx0", obj_pos, end_pos, init_v, end_v,
          /*is_stationary=*/false, /*prob=*/1.0, &builder);

  const PlannerObject object = builder.Build();
  const SpacetimeTrajectoryManager traj_mgr({object});

  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving), StBoundaryProto::FOLLOW,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);

  const double av_init_v = 0.0;  // m/s.

  SpeedOptimizerObjectManager opt_obj_mgr(
      st_boundaries_with_decision, &preliminary_speed, traj_mgr, av_init_v,
      kPlanTotalTime, kPlanTimeInterval, kPlannerParams.speed_finder_params());

  QCHECK_EQ(opt_obj_mgr.MovingFollowObjects().size(), 1);
  QCHECK(opt_obj_mgr.MovingLeadObjects().empty());
  QCHECK(opt_obj_mgr.StationaryObjects().empty());
  QCHECK_EQ(opt_obj_mgr.MovingFollowObjects().front().id(), id);
  QCHECK_GE(opt_obj_mgr.MovingFollowObjects().front().standstill(), 0.0);

  CheckMergeBoundary(opt_obj_mgr.MovingFollowObjects().front(),
                     st_boundaries_with_decision.front(),
                     st_boundaries_with_decision.front(), /*leading=*/false,
                     /*prob=*/1.0);
}

TEST(SpeedOptimizerObjectManagerTest, TwoFollowBoundaryMerge) {
  const SpeedVector preliminary_speed =
      BuildConstAccelSpeedVector(/*v0=*/0.0, /*acc=*/1.0);

  const std::string id = "001";
  const Vec2d obj_pos = Vec2d(20.0, 0.0);
  const ObjectType object_type = OT_VEHICLE;
  const double heading = 0.0;
  const double timestamp = 0.0;
  const double init_v = 10.0;
  const auto obj = BuildPerceptionObject(id, obj_pos, object_type, init_v,
                                         heading, timestamp);

  PlannerObjectBuilder builder;
  builder.set_type(object_type).set_object(obj).set_stationary(false);

  StBoundaryRef st_boundary_moving0 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx0", obj_pos, /*end_pos=*/Vec2d(120.0, 0.0), init_v,
          /*end_v=*/10.0,
          /*is_stationary=*/false, /*prob=*/0.6, &builder);
  StBoundaryRef st_boundary_moving1 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx1", obj_pos, /*end_pos=*/Vec2d(100.0, 0.0), /*init_v=*/8.0,
          /*end_v=*/8.0,
          /*is_stationary=*/false, /*prob=*/0.4, &builder);

  const PlannerObject object = builder.Build();
  const SpacetimeTrajectoryManager traj_mgr({object});

  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving0), StBoundaryProto::FOLLOW,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving1), StBoundaryProto::FOLLOW,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);

  const double av_init_v = 0.0;  // m/s.

  SpeedOptimizerObjectManager opt_obj_mgr(
      st_boundaries_with_decision, &preliminary_speed, traj_mgr, av_init_v,
      kPlanTotalTime, kPlanTimeInterval, kPlannerParams.speed_finder_params());

  QCHECK_EQ(opt_obj_mgr.MovingFollowObjects().size(), 1);
  QCHECK(opt_obj_mgr.MovingLeadObjects().empty());
  QCHECK(opt_obj_mgr.StationaryObjects().empty());
  QCHECK_EQ(opt_obj_mgr.MovingFollowObjects().front().id(), id);
  QCHECK_GE(opt_obj_mgr.MovingFollowObjects().front().standstill(), 0.0);

  CheckMergeBoundary(
      opt_obj_mgr.MovingFollowObjects().front(), st_boundaries_with_decision[0],
      st_boundaries_with_decision[1], /*leading=*/false, /*prob=*/1.0);
}

TEST(SpeedOptimizerObjectManagerTest, TwoLeadBoundaryMerge) {
  const SpeedVector preliminary_speed =
      BuildConstAccelSpeedVector(/*v0=*/0.0, /*acc=*/1.0);

  const std::string id = "001";
  const Vec2d obj_pos = Vec2d(20.0, 0.0);
  const ObjectType object_type = OT_VEHICLE;
  const double heading = 0.0;
  const double timestamp = 0.0;
  const double init_v = 10.0;
  const auto obj = BuildPerceptionObject(id, obj_pos, object_type, init_v,
                                         heading, timestamp);

  PlannerObjectBuilder builder;
  builder.set_type(object_type).set_object(obj).set_stationary(false);

  StBoundaryRef st_boundary_moving0 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx0", obj_pos, /*end_pos=*/Vec2d(120.0, 0.0), init_v,
          /*end_v=*/10.0,
          /*is_stationary=*/false, /*prob=*/0.6, &builder);
  StBoundaryRef st_boundary_moving1 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx1", obj_pos, /*end_pos=*/Vec2d(100.0, 0.0), /*init_v=*/8.0,
          /*end_v=*/8.0,
          /*is_stationary=*/false, /*prob=*/0.4, &builder);

  const PlannerObject object = builder.Build();
  const SpacetimeTrajectoryManager traj_mgr({object});

  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving0), StBoundaryProto::LEAD,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving1), StBoundaryProto::LEAD,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);

  const double av_init_v = 0.0;  // m/s.

  SpeedOptimizerObjectManager opt_obj_mgr(
      st_boundaries_with_decision, &preliminary_speed, traj_mgr, av_init_v,
      kPlanTotalTime, kPlanTimeInterval, kPlannerParams.speed_finder_params());

  QCHECK_EQ(opt_obj_mgr.MovingLeadObjects().size(), 1);
  QCHECK(opt_obj_mgr.MovingFollowObjects().empty());
  QCHECK(opt_obj_mgr.StationaryObjects().empty());
  QCHECK_EQ(opt_obj_mgr.MovingLeadObjects().front().id(), id);
  QCHECK_GE(opt_obj_mgr.MovingLeadObjects().front().standstill(), 0.0);

  CheckMergeBoundary(
      opt_obj_mgr.MovingLeadObjects().front(), st_boundaries_with_decision[0],
      st_boundaries_with_decision[1], /*leading=*/true, /*prob=*/1.0);
}

TEST(SpeedOptimizerObjectManagerTest, FollowStationaryMerge) {
  const SpeedVector preliminary_speed =
      BuildConstAccelSpeedVector(/*v0=*/0.0, /*acc=*/1.0);

  const std::string id = "001";
  const Vec2d obj_pos = Vec2d(20.0, 0.0);
  const ObjectType object_type = OT_VEHICLE;
  const double heading = 0.0;
  const double timestamp = 0.0;
  const auto obj = BuildPerceptionObject(id, obj_pos, object_type,
                                         /*init_v=*/0.0, heading, timestamp);

  PlannerObjectBuilder builder;
  builder.set_type(object_type).set_object(obj).set_stationary(false);

  StBoundaryRef st_boundary_moving0 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx0", obj_pos, /*end_pos=*/Vec2d(30.0, 0.0), /*init_v=*/1.0,
          /*end_v=*/1.0, /*is_stationary=*/false, /*prob=*/0.3, &builder);
  StBoundaryRef st_boundary_moving1 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx1", obj_pos, /*end_pos=*/Vec2d(25.0, 0.0), /*init_v=*/0.5,
          /*end_v=*/0.5,
          /*is_stationary=*/false, /*prob=*/0.4, &builder);
  StBoundaryRef st_boundary_stationary0 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx2", obj_pos, obj_pos, /*init_v=*/0.0,
          /*end_v=*/0.0,
          /*is_stationary=*/true, /*prob=*/0.3, &builder);

  const PlannerObject object = builder.Build();
  const SpacetimeTrajectoryManager traj_mgr({object});

  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving0), StBoundaryProto::FOLLOW,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving1), StBoundaryProto::FOLLOW,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_stationary0), StBoundaryProto::FOLLOW,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);

  const double av_init_v = 0.0;  // m/s.

  SpeedOptimizerObjectManager opt_obj_mgr(
      st_boundaries_with_decision, &preliminary_speed, traj_mgr, av_init_v,
      kPlanTotalTime, kPlanTimeInterval, kPlannerParams.speed_finder_params());

  // Check for follow object.
  QCHECK_EQ(opt_obj_mgr.MovingFollowObjects().size(), 1);
  QCHECK(opt_obj_mgr.MovingLeadObjects().empty());
  QCHECK_EQ(opt_obj_mgr.StationaryObjects().size(), 1);
  QCHECK_EQ(opt_obj_mgr.MovingFollowObjects().front().id(), id);
  QCHECK_GE(opt_obj_mgr.MovingFollowObjects().front().standstill(), 0.0);
  const auto& st_boundary_with_decision0 = st_boundaries_with_decision[0];
  const auto& st_boundary_with_decision1 = st_boundaries_with_decision[1];
  for (int idx = 0; idx < kKnotNum; ++idx) {
    const auto& state =
        opt_obj_mgr.MovingFollowObjects().front().GetOverlapStateByIndex(idx);
    QCHECK_NEAR(state->prob, 0.7, kEps);
    const double time = kPlanTimeInterval * static_cast<double>(idx);
    const auto speed0 =
        st_boundary_with_decision0.raw_st_boundary()->GetStBoundarySpeedAtT(
            time);
    const auto speed1 =
        st_boundary_with_decision1.raw_st_boundary()->GetStBoundarySpeedAtT(
            time);
    QCHECK(speed0.has_value());
    QCHECK(speed1.has_value());
    QCHECK_NEAR(state->bound, obj_pos.x(), kEps);
    QCHECK_GE(state->speed, *speed1);
    QCHECK_LE(state->speed, *speed0);
    QCHECK_GE(state->lon_buffer, 0.0);
  }
  // Check for stationary object.
  QCHECK_EQ(opt_obj_mgr.StationaryObjects().front().id(), id);
  QCHECK_GE(opt_obj_mgr.StationaryObjects().front().standstill(), 0.0);
  CheckMergeBoundary(opt_obj_mgr.StationaryObjects().front(),
                     st_boundaries_with_decision[2],
                     st_boundaries_with_decision[2], /*leading=*/false,
                     /*prob=*/0.3);
}

TEST(SpeedOptimizerObjectManagerTest, FollowLeadingMerge) {
  const SpeedVector preliminary_speed =
      BuildConstAccelSpeedVector(/*v0=*/0.0, /*acc=*/1.0);

  const std::string id = "001";
  const Vec2d obj_pos = Vec2d(20.0, 0.0);
  const ObjectType object_type = OT_VEHICLE;
  const double heading = 0.0;
  const double timestamp = 0.0;
  const double init_v = 10.0;
  const auto obj = BuildPerceptionObject(id, obj_pos, object_type, init_v,
                                         heading, timestamp);

  PlannerObjectBuilder builder;
  builder.set_type(object_type).set_object(obj).set_stationary(false);

  StBoundaryRef st_boundary_moving0 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx0", obj_pos, /*end_pos=*/Vec2d(120.0, 0.0), init_v,
          /*end_v=*/10.0,
          /*is_stationary=*/false, /*prob=*/0.3, &builder);
  StBoundaryRef st_boundary_moving1 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx1", obj_pos, /*end_pos=*/Vec2d(100.0, 0.0), /*init_v=*/8.0,
          /*end_v=*/8.0,
          /*is_stationary=*/false, /*prob=*/0.3, &builder);
  StBoundaryRef st_boundary_moving2 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx3", obj_pos, /*end_pos=*/Vec2d(90.0, 0.0), /*init_v=*/7.0,
          /*end_v=*/7.0,
          /*is_stationary=*/false, /*prob=*/0.2, &builder);
  StBoundaryRef st_boundary_moving3 =
      PlannerObjectBuilderAddTrajAndBuildBoundaryRef(
          id + "-idx2", obj_pos, /*end_pos=*/Vec2d(80.0, 0.0), /*init_v=*/6.0,
          /*end_v=*/6.0,
          /*is_stationary=*/false, /*prob=*/0.2, &builder);

  const PlannerObject object = builder.Build();
  const SpacetimeTrajectoryManager traj_mgr({object});

  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving0), StBoundaryProto::FOLLOW,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving1), StBoundaryProto::FOLLOW,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving2), StBoundaryProto::LEAD,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);
  st_boundaries_with_decision.emplace_back(
      std::move(st_boundary_moving3), StBoundaryProto::LEAD,
      StBoundaryProto::ST_BOUNDARY_MODIFIER);

  const double av_init_v = 0.0;  // m/s.

  SpeedOptimizerObjectManager opt_obj_mgr(
      st_boundaries_with_decision, &preliminary_speed, traj_mgr, av_init_v,
      kPlanTotalTime, kPlanTimeInterval, kPlannerParams.speed_finder_params());

  // Check for follow object.
  QCHECK_EQ(opt_obj_mgr.MovingFollowObjects().size(), 1);
  QCHECK_EQ(opt_obj_mgr.MovingLeadObjects().size(), 1);
  QCHECK(opt_obj_mgr.StationaryObjects().empty());
  QCHECK_EQ(opt_obj_mgr.MovingFollowObjects().front().id(), id);
  QCHECK_GE(opt_obj_mgr.MovingFollowObjects().front().standstill(), 0.0);
  QCHECK_EQ(opt_obj_mgr.MovingLeadObjects().front().id(), id);
  QCHECK_GE(opt_obj_mgr.MovingLeadObjects().front().standstill(), 0.0);

  CheckMergeBoundary(
      opt_obj_mgr.MovingFollowObjects().front(), st_boundaries_with_decision[0],
      st_boundaries_with_decision[1], /*leading=*/false, /*prob=*/0.6);
  CheckMergeBoundary(
      opt_obj_mgr.MovingLeadObjects().front(), st_boundaries_with_decision[2],
      st_boundaries_with_decision[3], /*leading=*/true, /*prob=*/0.4);
}

}  // namespace
}  // namespace qcraft::planner
