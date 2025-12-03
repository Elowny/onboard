#include "onboard/planner/ml/captain_net/post_process/trajectory_alignment.h"

#include <utility>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/planner/planner_flags.h"

namespace qcraft::planner::ml {

namespace {

ApolloTrajectoryPointProto CalculateIntermediatePoint(
    const ApolloTrajectoryPointProto& first_point,
    const ApolloTrajectoryPointProto& second_point, double two_points_time_gap,
    double intermediate_point_time) {
  ApolloTrajectoryPointProto intermediate_point;
  const auto intermediate_ratio = intermediate_point_time / two_points_time_gap;
  const auto intermediate_x =
      (second_point.path_point().x() - first_point.path_point().x()) *
          intermediate_ratio +
      first_point.path_point().x();
  const auto intermediate_y =
      (second_point.path_point().y() - first_point.path_point().y()) *
          intermediate_ratio +
      first_point.path_point().y();
  intermediate_point.mutable_path_point()->set_x(intermediate_x);
  intermediate_point.mutable_path_point()->set_y(intermediate_y);
  return intermediate_point;
}
}  // namespace

void VanillaTrajectoryAlignment(const ApolloTrajectoryPointProto& start_point,
                                captain_net::CaptainNetOutput* output) {
  SCOPED_QTRACE("CaptainNet::PostProcessTrajectory");
  std::vector<ApolloTrajectoryPointProto> traj;
  constexpr double kEpsilon = 1e-6;
  traj.push_back(start_point);
  auto start_pose =
      Vec2d(start_point.path_point().x(), start_point.path_point().y());
  // Note(Jinyun): use the givn start_point to replace the first inferred point
  for (int i = 1, size = output->traj_points.size(); i < size; ++i) {
    ApolloTrajectoryPointProto traj_point;
    const double cur_x_diff = output->traj_points[i].path_point().x() -
                              output->traj_points[0].path_point().x();
    const double cur_y_diff = output->traj_points[i].path_point().y() -
                              output->traj_points[0].path_point().y();
    const auto restored_pos_diff =
        Vec2d(cur_x_diff, cur_y_diff)
            .Rotate(NormalizeAngle(start_point.path_point().theta()));
    auto restored_pos = restored_pos_diff + start_pose;
    traj_point.mutable_path_point()->set_x(restored_pos.x());
    traj_point.mutable_path_point()->set_y(restored_pos.y());
    const double pre_x = traj.back().path_point().x();
    const double pre_y = traj.back().path_point().y();
    const double pre_theta = traj.back().path_point().theta();
    const double pre_kappa = traj.back().path_point().kappa();
    const double pre_s = traj.back().path_point().s();
    const double cur_theta =
        Vec2d(restored_pos.x() - pre_x, restored_pos.y() - pre_y).Angle();
    traj_point.mutable_path_point()->set_theta(cur_theta);
    const double diff_s = (restored_pos - Vec2d(pre_x, pre_y)).norm();
    traj_point.mutable_path_point()->set_s(diff_s + pre_s);
    const double cur_kappa =
        AngleDifference(pre_theta, cur_theta) / (kEpsilon + diff_s);
    traj_point.mutable_path_point()->set_kappa(cur_kappa);
    traj_point.mutable_path_point()->set_lambda((cur_kappa - pre_kappa) /
                                                (kEpsilon + diff_s));
    const double pre_v = traj.back().v();
    const double pre_a = traj.back().a();
    const double cur_v = diff_s / captain_net::kTimeStep;
    const double cur_a = (cur_v - pre_v) / captain_net::kTimeStep;
    traj_point.set_v(cur_v);
    traj_point.set_a(cur_a);
    traj_point.set_j((cur_a - pre_a) / captain_net::kTimeStep);
    traj_point.set_relative_time(i * captain_net::kTimeStep);
    traj.push_back(std::move(traj_point));
  }
  output->traj_points = std::move(traj);
}

void TimeBasedTrajectoryAlignmentFirstPoint(
    const ApolloTrajectoryPointProto& start_point, const double prediction_time,
    const double plan_start_time, captain_net::CaptainNetOutput* output) {
  SCOPED_QTRACE("CaptainNet::PostProcessTrajectoryBasedOnTime");
  std::vector<ApolloTrajectoryPointProto> traj;
  constexpr double kEpsilon = 1e-6;
  traj.push_back(start_point);
  auto start_pose =
      Vec2d(start_point.path_point().x(), start_point.path_point().y());
  const auto start_time_diff =
      plan_start_time - (prediction_time + captain_net::kTimeStep);
  auto pre_start_point_idx = floor(start_time_diff / captain_net::kTimeStep);
  int start_index = pre_start_point_idx + 1;
  double intermediate_point_time;
  if (start_time_diff < 0.0) {
    intermediate_point_time = start_time_diff;
    pre_start_point_idx = 0;
    start_index = 0;
  } else {
    intermediate_point_time =
        start_time_diff - pre_start_point_idx * captain_net::kTimeStep;
  }
  const auto time_aligned_capnet_start_point = CalculateIntermediatePoint(
      output->traj_points[pre_start_point_idx],
      output->traj_points[pre_start_point_idx + 1], captain_net::kTimeStep,
      intermediate_point_time);
  for (int i = start_index, size = output->traj_points.size(); i < size; ++i) {
    ApolloTrajectoryPointProto traj_point;
    const double cur_x_diff = output->traj_points[i].path_point().x() -
                              time_aligned_capnet_start_point.path_point().x();
    const double cur_y_diff = output->traj_points[i].path_point().y() -
                              time_aligned_capnet_start_point.path_point().y();
    const auto restored_pos_diff =
        Vec2d(cur_x_diff, cur_y_diff)
            .Rotate(NormalizeAngle(start_point.path_point().theta()));
    auto restored_pos = restored_pos_diff + start_pose;
    traj_point.mutable_path_point()->set_x(restored_pos.x());
    traj_point.mutable_path_point()->set_y(restored_pos.y());
    const double pre_x = traj.back().path_point().x();
    const double pre_y = traj.back().path_point().y();
    const double pre_theta = traj.back().path_point().theta();
    const double pre_kappa = traj.back().path_point().kappa();
    const double pre_s = traj.back().path_point().s();
    const double cur_theta =
        Vec2d(restored_pos.x() - pre_x, restored_pos.y() - pre_y).Angle();
    traj_point.mutable_path_point()->set_theta(cur_theta);
    const double diff_s = (restored_pos - Vec2d(pre_x, pre_y)).norm();
    traj_point.mutable_path_point()->set_s(diff_s + pre_s);
    const double cur_kappa =
        AngleDifference(pre_theta, cur_theta) / (kEpsilon + diff_s);
    traj_point.mutable_path_point()->set_kappa(cur_kappa);
    traj_point.mutable_path_point()->set_lambda((cur_kappa - pre_kappa) /
                                                (kEpsilon + diff_s));
    const double pre_v = traj.back().v();
    const double pre_a = traj.back().a();
    const double cur_rel_time =
        (i + 1) * captain_net::kTimeStep + prediction_time - plan_start_time;
    const double cur_time_step = cur_rel_time - traj.back().relative_time();
    const double cur_v = diff_s / cur_time_step;
    const double cur_a = (cur_v - pre_v) / cur_time_step;
    traj_point.set_v(cur_v);
    traj_point.set_a(cur_a);
    traj_point.set_j((cur_a - pre_a) / cur_time_step);
    traj_point.set_relative_time(cur_rel_time);
    traj.push_back(std::move(traj_point));
  }
  output->traj_points = std::move(traj);
}

void TimeBasedTrajectoryAlignmentAllPoints(
    const ApolloTrajectoryPointProto& start_point, const double prediction_time,
    const double plan_start_time, captain_net::CaptainNetOutput* output) {
  SCOPED_QTRACE("CaptainNet::PostProcessTrajectoryBasedOnTimeForAllPoint");
  std::vector<ApolloTrajectoryPointProto> traj;
  constexpr double kEpsilon = 1e-6;
  traj.push_back(start_point);
  auto start_pose =
      Vec2d(start_point.path_point().x(), start_point.path_point().y());
  const auto start_time_diff =
      plan_start_time - (prediction_time + captain_net::kTimeStep);
  auto pre_start_point_idx = floor(start_time_diff / captain_net::kTimeStep);
  double intermediate_point_time;
  ApolloTrajectoryPointProto time_aligned_capnet_start_point;
  if (start_time_diff < 0.0) {
    intermediate_point_time = start_time_diff;
    time_aligned_capnet_start_point = CalculateIntermediatePoint(
        output->traj_points[0], output->traj_points[1], captain_net::kTimeStep,
        intermediate_point_time);
    intermediate_point_time = intermediate_point_time + captain_net::kTimeStep;
  } else {
    intermediate_point_time =
        start_time_diff - pre_start_point_idx * captain_net::kTimeStep;
    time_aligned_capnet_start_point = CalculateIntermediatePoint(
        output->traj_points[pre_start_point_idx],
        output->traj_points[pre_start_point_idx + 1], captain_net::kTimeStep,
        intermediate_point_time);
  }
  ++pre_start_point_idx;
  double relative_time = 0.0;
  while (pre_start_point_idx + 1 < output->traj_points.size()) {
    ApolloTrajectoryPointProto time_aligned_point;
    if (intermediate_point_time < 0.0) {
      time_aligned_point = CalculateIntermediatePoint(
          output->traj_points[0], output->traj_points[1],
          captain_net::kTimeStep, intermediate_point_time);
      intermediate_point_time =
          intermediate_point_time + captain_net::kTimeStep;
    } else {
      time_aligned_point = CalculateIntermediatePoint(
          output->traj_points[pre_start_point_idx],
          output->traj_points[pre_start_point_idx + 1], captain_net::kTimeStep,
          intermediate_point_time);
    }
    relative_time = relative_time + captain_net::kTimeStep;
    ++pre_start_point_idx;

    ApolloTrajectoryPointProto traj_point;
    const double cur_x_diff = time_aligned_point.path_point().x() -
                              time_aligned_capnet_start_point.path_point().x();
    const double cur_y_diff = time_aligned_point.path_point().y() -
                              time_aligned_capnet_start_point.path_point().y();
    const auto restored_pos_diff =
        Vec2d(cur_x_diff, cur_y_diff)
            .Rotate(NormalizeAngle(start_point.path_point().theta()));
    auto restored_pos = restored_pos_diff + start_pose;
    traj_point.mutable_path_point()->set_x(restored_pos.x());
    traj_point.mutable_path_point()->set_y(restored_pos.y());
    const double pre_x = traj.back().path_point().x();
    const double pre_y = traj.back().path_point().y();
    const double pre_theta = traj.back().path_point().theta();
    const double pre_kappa = traj.back().path_point().kappa();
    const double pre_s = traj.back().path_point().s();
    const double cur_theta =
        Vec2d(restored_pos.x() - pre_x, restored_pos.y() - pre_y).Angle();
    traj_point.mutable_path_point()->set_theta(cur_theta);
    const double diff_s = (restored_pos - Vec2d(pre_x, pre_y)).norm();
    traj_point.mutable_path_point()->set_s(diff_s + pre_s);
    const double cur_kappa =
        AngleDifference(pre_theta, cur_theta) / (kEpsilon + diff_s);
    traj_point.mutable_path_point()->set_kappa(cur_kappa);
    traj_point.mutable_path_point()->set_lambda((cur_kappa - pre_kappa) /
                                                (kEpsilon + diff_s));
    const double pre_v = traj.back().v();
    const double pre_a = traj.back().a();
    const double cur_v = diff_s / captain_net::kTimeStep;
    const double cur_a = (cur_v - pre_v) / captain_net::kTimeStep;
    traj_point.set_v(cur_v);
    traj_point.set_a(cur_a);
    traj_point.set_j((cur_a - pre_a) / captain_net::kTimeStep);
    traj_point.set_relative_time(relative_time);
    traj.push_back(std::move(traj_point));
  }
  output->traj_points = std::move(traj);
}

void AlignTrajectory(const ApolloTrajectoryPointProto& plan_start_point,
                     const double prediction_time, const double plan_start_time,
                     captain_net::CaptainNetOutput* output) {
  if (FLAGS_planner_captain_net_align_traj_based_on_time_for_all_points) {
    TimeBasedTrajectoryAlignmentAllPoints(plan_start_point, prediction_time,
                                          plan_start_time, output);
  } else if (
      FLAGS_planner_captain_net_align_traj_based_on_time_for_first_point) {
    TimeBasedTrajectoryAlignmentFirstPoint(plan_start_point, prediction_time,
                                           plan_start_time, output);
  } else {
    VanillaTrajectoryAlignment(plan_start_point, output);
  }
}

}  // namespace qcraft::planner::ml
