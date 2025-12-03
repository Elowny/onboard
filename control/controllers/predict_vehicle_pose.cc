#include "onboard/control/controllers/predict_vehicle_pose.h"

#include <cmath>

#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/model/single_track_dynamic_model.h"
#include "onboard/control/controllers/model/state_space.h"
#include "onboard/control/proto/dynamic_model_conf.pb.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"

DEFINE_bool(control_enable_yaw_shift_wrt_localize, false,
            "enable yaw reference shift due to localization lateral drift.");

namespace qcraft {
namespace control {

namespace {

// TODO(zhichao): replace the conf setting with vehicle params.
DynamicModelConfProto BuildDynamicModelConf() {
  DynamicModelConfProto conf;
  conf.set_mass(2200);
  conf.mutable_geo_params()->set_cg_ratio(0.35);
  conf.mutable_tire_params()->set_c_af(50000.0);
  conf.mutable_tire_params()->set_c_ar(50000.0);
  return conf;
}

}  // namespace

VehPose::VehPose(const VehicleGeometryParamsProto* vehicle_geometry_params,
                 const VehicleStateProto& vehicle_state)
    : timestamp(vehicle_state.timestamp()),
      x(vehicle_state.x()),
      y(vehicle_state.y()),
      v(vehicle_state.linear_velocity()),
      acc(vehicle_state.linear_acceleration()),
      kappa(vehicle_state.kappa()),
      angular_velocity(vehicle_state.angular_velocity()),
      lateral_velocity(vehicle_state.lateral_velocity()),
      front_wheel_steering_angle(vehicle_state.front_wheel_steering_angle()),
      moving_direction(vehicle_state.moving_direction()),
      geo_param(vehicle_geometry_params) {
  const double heading_shift_wrt_localize =
      FLAGS_control_enable_yaw_shift_wrt_localize
          ? vehicle_state.yaw_diff_wrt_localize_drift()
          : 0.0;
  heading = NormalizeAngle(heading_shift_wrt_localize + vehicle_state.yaw());
}

VehPose::VehPose(double acc_target_last,
                 const VehicleGeometryParamsProto* vehicle_geometry_params,
                 const VehicleStateProto& vehicle_state)
    : timestamp(vehicle_state.timestamp()),
      x(vehicle_state.x()),
      y(vehicle_state.y()),
      v(vehicle_state.linear_velocity()),
      acc(0.5 * (acc_target_last + vehicle_state.linear_acceleration())),
      kappa(vehicle_state.kappa()),
      angular_velocity(vehicle_state.angular_velocity()),
      lateral_velocity(vehicle_state.lateral_velocity()),
      front_wheel_steering_angle(vehicle_state.front_wheel_steering_angle()),
      moving_direction(vehicle_state.moving_direction()),
      geo_param(vehicle_geometry_params) {
  const double heading_shift_wrt_localize =
      FLAGS_control_enable_yaw_shift_wrt_localize
          ? vehicle_state.yaw_diff_wrt_localize_drift()
          : 0.0;
  heading = NormalizeAngle(heading_shift_wrt_localize + vehicle_state.yaw());
}

void VehPose::ToProto(VehPoseProto* proto) const {
  proto->Clear();
  proto->set_x(x);
  proto->set_y(y);
  proto->set_v(v);
  proto->set_heading(heading);
  proto->set_kappa(kappa);
  proto->set_lat_speed(lateral_velocity);
}

void VehPose::AdvancedByKinematicModel(double ts, double acc_cmd,
                                       double kappa_cmd) {
  timestamp += ts;
  const double v_mean = v + 0.5 * acc_cmd * ts;
  const double acc_mean = 0.5 * (acc + acc_cmd);
  v += acc_mean * ts;
  acc = acc_cmd;

  const double kappa_mean = 0.5 * (kappa + kappa_cmd);
  const double delta_heading = v_mean * kappa_mean * ts;
  const double heading_mean = NormalizeAngle(heading + 0.5 * delta_heading);

  x += v_mean * fast_math::Cos(heading_mean) * ts;
  y += v_mean * fast_math::Sin(heading_mean) * ts;
  heading = NormalizeAngle(heading + delta_heading);
  kappa = kappa_cmd;
  moving_direction = NormalizeAngle(moving_direction + delta_heading);
}

void VehPose::AdvancedByDynamicModel(
    double ts, double acc_cmd, double steering_speed_target,
    const SteeringConverter& steering_converter) {
  QCHECK_NOTNULL(geo_param);
  timestamp += ts;
  const double v_mean = v + 0.5 * acc_cmd * ts;
  const double acc_mean = 0.5 * (acc + acc_cmd);
  v += acc_mean * ts;
  acc = acc_cmd;

  DynamicModelMeasurement measurement = {
      .psi = steering_converter.KappaToFrontWheelAngle(kappa),
      .yaw = heading,
      .roll = 0.0,
      .u_0 = lateral_velocity,
      .omega = angular_velocity,
      .v = std::vector<double>{v_mean}};

  const TimeVaryingDiscreteStateSpace dynamic_model =
      BuildSingleTrackDynamicModel(ts, geo_param->wheel_base(),
                                   geo_param->length(), measurement,
                                   BuildDynamicModelConf());

  Eigen::VectorXd init_state(kDynamicModelStateSize);
  init_state << x, y, /*delta_yaw*/ 0.0, angular_velocity, lateral_velocity,
      steering_converter.KappaToFrontWheelAngle(kappa);
  Eigen::VectorXd input(kDynamicModelInputSize);
  input << steering_converter.SteerRateToFrontWheelOmega(
      steering_speed_target, steering_converter.KappaToSteerAngle(kappa));

  std::vector<Eigen::VectorXd> dynamic_state =
      EvaluateTvdStateSpace(init_state, dynamic_model, {input});

  x = dynamic_state[0](0);
  y = dynamic_state[0](1);
  heading = NormalizeAngle(heading + dynamic_state[0](2));
  angular_velocity = dynamic_state[0](3);
  lateral_velocity = dynamic_state[0](4);
  front_wheel_steering_angle = dynamic_state[0](5);
  kappa = steering_converter.FrontWheelAngleToKappa(front_wheel_steering_angle);
  moving_direction =
      (v * Vec2d::FastUnitFromAngle(heading) +
       lateral_velocity * Vec2d::FastUnitFromAngle(heading + M_PI_2))
          .Angle();
}

VehPose PredictControlInitPoseByKM(
    const VehPose& init_pose, const SteeringConverter& /*steering_converter*/,
    const std::vector<double>& kappa_target,
    const std::vector<double>& acc_target) {
  QCHECK_EQ(kappa_target.size(), acc_target.size());
  VehPose predicted_pose = init_pose;
  for (int i = 0; i < kappa_target.size(); ++i) {
    if (init_pose.v * predicted_pose.v <= 0.0) {
      predicted_pose.v = 0.0;
      return predicted_pose;
    }
    predicted_pose.AdvancedByKinematicModel(kControlInterval, acc_target[i],
                                            kappa_target[i]);
  }

  return predicted_pose;
}

VehPose PredictControlInitPoseByDM(
    const VehPose& init_pose, const SteeringConverter& steering_converter,
    const std::vector<double>& steer_speed_target,
    const std::vector<double>& acc_target) {
  QCHECK_EQ(steer_speed_target.size(), acc_target.size());
  constexpr double kMinSpeed = 0.01;  // m/s
  if (std::fabs(init_pose.v) < kMinSpeed) return init_pose;

  VehPose predicted_pose = init_pose;
  for (int i = 0; i < steer_speed_target.size(); ++i) {
    if (init_pose.v * predicted_pose.v <= 0.0) {
      predicted_pose.v = 0.0;
      return predicted_pose;
    }

    predicted_pose.AdvancedByDynamicModel(kControlInterval, acc_target[i],
                                          steer_speed_target[i],
                                          steering_converter);
  }

  return predicted_pose;
}

VehPose VehPose::operator-(const VehPose& comp_vehpose) const {
  VehPose veh_pose_diff;
  veh_pose_diff.x = x - comp_vehpose.x;
  veh_pose_diff.y = y - comp_vehpose.y;
  veh_pose_diff.v = v - comp_vehpose.v;
  veh_pose_diff.heading = NormalizeAngle(heading - comp_vehpose.heading);
  veh_pose_diff.acc = acc - comp_vehpose.acc;
  veh_pose_diff.kappa = kappa - comp_vehpose.kappa;
  veh_pose_diff.angular_velocity =
      angular_velocity - comp_vehpose.angular_velocity;
  veh_pose_diff.lateral_velocity =
      lateral_velocity - comp_vehpose.lateral_velocity;
  veh_pose_diff.front_wheel_steering_angle =
      front_wheel_steering_angle - comp_vehpose.front_wheel_steering_angle;
  veh_pose_diff.moving_direction =
      NormalizeAngle(moving_direction - comp_vehpose.moving_direction);

  return veh_pose_diff;
}

std::string VehPose::DebugString() const {
  return absl::StrCat("x, y: ", x, ", ", y, "; heading: ", heading,
                      "; moving_direction: ", moving_direction,
                      "; lateral_velocity: ", lateral_velocity);
}

}  // namespace control
}  // namespace qcraft
