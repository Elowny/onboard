#include "onboard/planner/test_util/util.h"

#include <cmath>    // for hypot
#include <utility>  // for move
#include <vector>   // for vector, allocator

#include "onboard/control/proto/controller_msg.pb.h"  // for WheelDriveMode
#include "onboard/global/buffered_logger.h"  // for BufferedLoggerWrapper
#include "onboard/lite/check.h"              // for QCHECK
#include "onboard/math/fast_math.h"          // for CosAndSin
#include "onboard/math/geometry/proto/affine_transformation.pb.h"  // for Vec3dProto
#include "onboard/planner/initializer/proto/initializer_config.pb.h"  // for InitializerConfig
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"  // for SpeedFinderParamsProto
#include "onboard/utils/file_util.h"   // for FileToProto
#include "onboard/utils/proto_util.h"  // for FillInMissingFieldsWithDefault

namespace qcraft {
namespace planner {
namespace {
void FillAlccParamsMissingFieldsWithDefault(
    const PlannerParamsProto& default_planner_params,
    AlccTaskParamsProto* alcc_params) {
  // Fill est planner params.
  FillInMissingFieldsWithDefault(default_planner_params.speed_finder_params(),
                                 alcc_params->mutable_speed_finder_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_params(),
      alcc_params->mutable_trajectory_optimizer_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.decision_constraint_config(),
      alcc_params->mutable_decision_constraint_config());
  FillInMissingFieldsWithDefault(default_planner_params.initializer_params(),
                                 alcc_params->mutable_initializer_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      alcc_params->mutable_motion_constraint_params());
  FillInMissingFieldsWithDefault(default_planner_params.vehicle_models_params(),
                                 alcc_params->mutable_vehicle_models_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.planner_functions_params(),
      alcc_params->mutable_planner_functions_params());

  // Fill lane change style params.
  FillInMissingFieldsWithDefault(
      default_planner_params.speed_finder_lc_radical_params(),
      alcc_params->mutable_speed_finder_lc_radical_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.speed_finder_lc_conservative_params(),
      alcc_params->mutable_speed_finder_lc_conservative_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_radical_params(),
      alcc_params->mutable_trajectory_optimizer_lc_radical_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_normal_params(),
      alcc_params->mutable_trajectory_optimizer_lc_normal_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_conservative_params(),
      alcc_params->mutable_trajectory_optimizer_lc_conservative_params());
}
}  // namespace

PoseProto CreatePose(double timestamp, const Vec2d& pos, double heading,
                     const Vec2d& vel_body) {
  PoseProto pose;
  pose.set_timestamp(timestamp);

  pose.mutable_pos_smooth()->set_x(pos.x());
  pose.mutable_pos_smooth()->set_y(pos.y());
  pose.mutable_pos_smooth()->set_z(0.0);
  pose.set_yaw(heading);
  pose.set_speed(vel_body.norm());

  pose.mutable_vel_body()->set_x(vel_body.x());
  pose.mutable_vel_body()->set_y(vel_body.y());
  pose.mutable_vel_body()->set_z(0.0);

  double heading_cos_sin[2];
  fast_math::CosAndSin<7>(heading, heading_cos_sin);
  const double sin_heading = heading_cos_sin[1],
               cos_heading = heading_cos_sin[0];
  pose.mutable_vel_smooth()->set_x(vel_body.x() * cos_heading -
                                   vel_body.y() * sin_heading);
  pose.mutable_vel_smooth()->set_y(vel_body.x() * sin_heading +
                                   vel_body.y() * cos_heading);
  pose.mutable_vel_smooth()->set_z(0.0);

  return pose;
}

ApolloTrajectoryPointProto ConvertToTrajPointProto(const PoseProto& pose) {
  ApolloTrajectoryPointProto traj_point;
  traj_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  traj_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  traj_point.mutable_path_point()->set_z(pose.pos_smooth().z());
  traj_point.mutable_path_point()->set_theta(pose.yaw());
  traj_point.mutable_path_point()->set_kappa(pose.curvature());
  traj_point.mutable_path_point()->set_lambda(0.0);
  traj_point.mutable_path_point()->set_s(0.0);
  traj_point.set_v(pose.vel_body().x());
  traj_point.set_a(
      std::hypot(pose.accel_smooth().x(), pose.accel_smooth().y()));
  traj_point.set_j(0.0);
  traj_point.set_relative_time(0.0);

  return traj_point;
}

VehicleGeometryParamsProto DefaultVehicleGeometry() {
  VehicleGeometryParamsProto geom;
  geom.set_front_edge_to_center(4.0);
  geom.set_back_edge_to_center(1.0);
  geom.set_left_edge_to_center(1.0);
  geom.set_right_edge_to_center(1.0);
  geom.set_length(5.0);
  geom.set_width(2.0);
  geom.set_height(2.2);
  geom.set_min_turn_radius(6.0);
  geom.set_wheel_base(3.0);
  geom.set_wheel_rolling_radius(0.3);
  return geom;
}

VehicleDriveParamsProto DefaultVehicleDriveParams() {
  VehicleDriveParamsProto drive;
  drive.set_max_steer_angle(8.0);
  drive.set_max_steer_angle_rate(7.0);
  drive.set_min_steer_angle_rate(0.0);
  drive.set_steer_ratio(16.0);
  drive.set_brake_deadzone(10.0);
  drive.set_throttle_deadzone(15.0);
  drive.set_wheel_drive_mode(FRONT_WHEEL_DRIVE);

  return drive;
}

PlannerParamsProto DefaultPlannerParams() {
  PlannerParamsProto default_planner_params;
  file_util::FileToProto("onboard/planner/params/planner_default_params.pb.txt",
                         &default_planner_params);
  // Fill default speed finder params into default planner params.
  SpeedFinderParamsProto default_speed_finder_params;
  file_util::FileToProto(
      "onboard/planner/params/speed_finder_default_params.pb.txt",
      &default_speed_finder_params);
  // Fill default trajectory optimizer params into default planner params.
  TrajectoryOptimizerParamsProto default_trajectory_optimizer_params;
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/trajectory_optimizer_default_params.pb.txt",
      &default_trajectory_optimizer_params));
  // Fill default path params into default planner params.
  FreespacePathFinderParamsProto default_path_finder_params;
  file_util::FileToProto(
      "onboard/planner/params/path_finder_default_params.pb.txt",
      &default_path_finder_params);

  // Fill default local_smoother params into default planner params.
  FreespaceLocalSmootherParamsProto default_local_smoother_params;
  file_util::FileToProto(
      "onboard/planner/params/local_smoother_default_params.pb.txt",
      &default_local_smoother_params);

  // Fill est planner.
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_speed_finder_params());
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params.mutable_trajectory_optimizer_params());

  // Fill freespace planner.
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_speed_finder_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_path_finder_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_path_finder_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_local_smoother_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_local_smoother_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_motion_constraint_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_speed_finder_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_path_finder_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_path_finder_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_local_smoother_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_local_smoother_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_motion_constraint_params());

  // Fill fallback planner.
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_fallback_planner_params()
          ->mutable_speed_finder_params());

  // Fill style params.
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/speed_finder_lc_radical_params.pb.txt",
      default_planner_params.mutable_speed_finder_lc_radical_params()));
  FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_speed_finder_lc_radical_params());

  QCHECK(file_util::FileToProto(
      "onboard/planner/params/speed_finder_lc_conservative_params.pb.txt",
      default_planner_params.mutable_speed_finder_lc_conservative_params()));
  FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_speed_finder_lc_conservative_params());

  QCHECK(file_util::FileToProto(
      "onboard/planner/params/trajectory_optimizer_lc_radical_params.pb.txt",
      default_planner_params.mutable_trajectory_optimizer_lc_radical_params()));
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params.mutable_trajectory_optimizer_lc_radical_params());

  QCHECK(file_util::FileToProto(
      "onboard/planner/params/trajectory_optimizer_lc_normal_params.pb.txt",
      default_planner_params.mutable_trajectory_optimizer_lc_normal_params()));
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params.mutable_trajectory_optimizer_lc_normal_params());

  QCHECK(file_util::FileToProto(
      "onboard/planner/params/"
      "trajectory_optimizer_lc_conservative_params.pb.txt",
      default_planner_params
          .mutable_trajectory_optimizer_lc_conservative_params()));
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params
          .mutable_trajectory_optimizer_lc_conservative_params());

  // Fill noa params.
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/noa_req_params.pb.txt",
      default_planner_params.mutable_noa_params()->mutable_noa_req_params()));

  // Fill alcc params.
  AlccTaskParamsProto default_alcc_params;
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/alcc_default_params.pb.txt",
      &default_alcc_params));
  FillAlccParamsMissingFieldsWithDefault(default_planner_params,
                                         &default_alcc_params);
  FillInMissingFieldsWithDefault(default_alcc_params,
                                 default_planner_params.mutable_alcc_params());

  // Fill acc params.
  FillInMissingFieldsWithDefault(default_planner_params.speed_finder_params(),
                                 default_planner_params.mutable_acc_params()
                                     ->mutable_speed_finder_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      default_planner_params.mutable_acc_params()
          ->mutable_motion_constraint_params());
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/acc_req_params.pb.txt",
      default_planner_params.mutable_acc_params()->mutable_acc_req_params()));
  return default_planner_params;
}

PathSlBoundary CreateFakePathSlBoundary(const DrivePassage& passage) {
  constexpr double kFakeHalfLaneWidth = 1.75;
  const int n = passage.stations().size();
  std::vector<double> s_vec, left_l, right_l, target_left_l, target_right_l;
  s_vec.reserve(n);
  left_l.reserve(n);
  right_l.reserve(n);
  std::vector<Vec2d> left_xy, right_xy, target_left_xy, target_right_xy;
  left_xy.reserve(n);
  right_xy.reserve(n);

  for (const auto& station : passage.stations()) {
    s_vec.push_back(station.accumulated_s());
    left_l.push_back(kFakeHalfLaneWidth);
    right_l.push_back(-kFakeHalfLaneWidth);
    target_left_l.push_back(kFakeHalfLaneWidth);
    target_right_l.push_back(-kFakeHalfLaneWidth);
    left_xy.push_back(station.lat_point(kFakeHalfLaneWidth));
    right_xy.push_back(station.lat_point(-kFakeHalfLaneWidth));
    target_left_xy.push_back(station.lat_point(kFakeHalfLaneWidth));
    target_right_xy.push_back(station.lat_point(-kFakeHalfLaneWidth));
  }
  return PathSlBoundary(std::move(s_vec), std::move(right_l), std::move(left_l),
                        std::move(target_right_l), std::move(target_left_l),
                        std::move(right_xy), std::move(left_xy),
                        std::move(target_right_xy), std::move(target_left_xy));
}

}  // namespace planner
}  // namespace qcraft
