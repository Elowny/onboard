#include "onboard/planner/plan/acc/acc_task_internal.h"

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/proto/piecewise_linear_function.pb.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/util/path_util.h"

namespace qcraft {

namespace planner {
namespace {

TEST(RunAccTaskInternal, FillRequiredMotionConstraintParamsTest) {
  MotionConstraintParamsProto motion_constraint_params;
  motion_constraint_params.set_default_speed_limit(1000.0);
  motion_constraint_params.set_default_reverse_speed_limit(-3.0);
  motion_constraint_params.set_max_deceleration(-7.0);
  motion_constraint_params.set_max_acceleration(3.0);
  motion_constraint_params.set_max_decel_jerk(-10.0);
  motion_constraint_params.set_max_accel_jerk(10.0);
  motion_constraint_params.set_max_psi(0.06);
  motion_constraint_params.set_max_lateral_accel(3.0);
  motion_constraint_params.set_max_lateral_jerk(2.5);
  motion_constraint_params.set_max_chi(0.1);

  AccReqParamsProto acc_req_params;
  acc_req_params.mutable_max_acc_vel_plf()->add_x(5.0);
  acc_req_params.mutable_max_acc_vel_plf()->add_x(20.0);
  acc_req_params.mutable_max_acc_vel_plf()->add_y(4.0);
  acc_req_params.mutable_max_acc_vel_plf()->add_y(2.0);
  acc_req_params.mutable_max_jerk_vel_plf()->add_x(5.0);
  acc_req_params.mutable_max_jerk_vel_plf()->add_x(20.0);
  acc_req_params.mutable_max_jerk_vel_plf()->add_y(5.0);
  acc_req_params.mutable_max_jerk_vel_plf()->add_y(2.5);
  acc_req_params.mutable_min_decel_vel_plf()->add_x(5.0);
  acc_req_params.mutable_min_decel_vel_plf()->add_x(20.0);
  acc_req_params.mutable_min_decel_vel_plf()->add_y(-5.0);
  acc_req_params.mutable_min_decel_vel_plf()->add_y(-3.5);

  const double plan_start_v = 5.0;
  const MotionConstraintParamsProto res =
      internal::FillRequiredMotionConstraintParams(
          motion_constraint_params, acc_req_params, plan_start_v);
  QCHECK_GE(res.max_accel_jerk(), 0.0);
}

TEST(RunAccTaskInternal, ModifySpeedFinderParamsTest0) {
  SpeedFinderParamsProto speed_finder_params;
  speed_finder_params.set_follow_standstill_distance(4.0);
  speed_finder_params.set_follow_time_headway(1.0);
  speed_finder_params.set_large_vehicle_follow_time_headway(1.3);

  QRunEvent::LccFollowingDistanceLevel following_distance_level =
      QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NONE;
  const bool crowded_scene = true;
  const double plan_start_v = 0.5;
  const SpeedFinderParamsProto res = internal::ModifySpeedFinderParams(
      speed_finder_params, following_distance_level, crowded_scene,
      plan_start_v, /*target_type=*/std::nullopt);
  QCHECK_GE(res.follow_standstill_distance(), 0.0);
}

TEST(RunAccTaskInternal, ModifySpeedFinderParamsTest1) {
  SpeedFinderParamsProto speed_finder_params;
  speed_finder_params.set_follow_standstill_distance(4.0);
  speed_finder_params.set_follow_time_headway(1.0);
  speed_finder_params.set_large_vehicle_follow_time_headway(1.3);

  QRunEvent::LccFollowingDistanceLevel following_distance_level =
      QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NONE;
  const bool crowded_scene = false;
  const double plan_start_v = 5.0;
  const SpeedFinderParamsProto res = internal::ModifySpeedFinderParams(
      speed_finder_params, following_distance_level, crowded_scene,
      plan_start_v, /*target_type=*/std::nullopt);
  QCHECK_GE(res.follow_standstill_distance(), 0.0);
}

TEST(RunAccTaskInternal, GetPrevTargetsFromProtoTest) {
  QACCTargetProto target1, target2;
  target1.set_object_id("100");
  target1.set_traj_id("0");
  target2.set_object_id("101");
  target2.set_traj_id("0");
  QACCTargetsProto prev_acc_target;
  *prev_acc_target.add_leaders() = target1;
  *prev_acc_target.add_leaders() = target2;
  QACCTaskProto acc_task_proto;
  *acc_task_proto.mutable_prev_acc_target() = prev_acc_target;
  const std::vector<AccTargetDecision> res =
      internal::GetPrevTargetsFromProto(&acc_task_proto);
  QCHECK_EQ(res.size(), 2);
}

TEST(RunAccTaskInternal, CreateAccPastPointsListTest) {
  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(0.0);
  plan_start_point.mutable_path_point()->set_y(0.0);
  plan_start_point.mutable_path_point()->set_z(0.0);
  plan_start_point.mutable_path_point()->set_theta(0.5);
  plan_start_point.mutable_path_point()->set_s(0.0);
  plan_start_point.mutable_path_point()->set_kappa(0.02);
  plan_start_point.mutable_path_point()->set_lambda(0.0);
  plan_start_point.set_v(3.0);
  std::vector<ApolloTrajectoryPointProto> origin_past_pts;
  constexpr int kPastPointSize = 20;
  for (int k = kPastPointSize; k > 0; --k) {
    const double s = -static_cast<double>(k) * 0.3;
    const PathPoint pt =
        GetPathPointAlongCircle(plan_start_point.path_point(), s);
    ApolloTrajectoryPointProto ppt;
    *ppt.mutable_path_point() = pt;
    origin_past_pts.push_back(ppt);
  }
  const std::vector<ApolloTrajectoryPointProto> res =
      internal::CreateAccPastPointsList(origin_past_pts, plan_start_point);
  QCHECK_EQ(res.size(), kPastPointSize);
}

TEST(RunAccTaskInternal, PlanAccSpeedForCorridorAndValidateTest) {}

TEST(RunAccTaskInternal, EarlyExitTest) {
  const AccCorridorSource source = AccCorridorSource::MAP;
  const PlannerStatus planner_status(PlannerStatusProto::OK, "OK");
  QCHECK_EQ(internal::EarlyExit(source, planner_status,
                                /*planner_force_no_map=*/false,
                                /*planner_acc_plan_for_all_sources=*/false),
            true);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
