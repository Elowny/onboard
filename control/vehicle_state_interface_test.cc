#include "onboard/control/vehicle_state_interface.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/control/steering_converter.h"
#include "onboard/localization/visual/proto/data_type.pb.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::control {
namespace {

constexpr int kChassisLossThres = 5;  // times

struct VehicleStateArgument {
  AutonomyStateProto_State autonomy_state;
  PoseProto pose;
  Chassis chassis;
  LocalizationViewerDebugProto localize_debug;
};

VehicleStateArgument BuildCompleteArgument() {
  VehicleStateArgument complete_argument;

  complete_argument.autonomy_state = AutonomyStateProto::AUTO_DRIVE;

  Chassis& chassis = complete_argument.chassis;
  chassis.set_gear_location(Chassis::GEAR_DRIVE);
  chassis.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  chassis.set_steering_percentage(10.0);
  chassis.set_steering_speed(1.0);
  chassis.set_steering_torque_nm(0.5);

  PoseProto& pose = complete_argument.pose;
  pose.mutable_pos_smooth()->set_x(10.0);
  pose.mutable_pos_smooth()->set_y(20.0);
  pose.mutable_pos_smooth()->set_z(0.1);
  pose.set_yaw(3.14);
  pose.set_pitch(0.15);
  pose.set_roll(0.01);
  // set ar_body.
  pose.mutable_ar_body()->set_x(0.1);
  pose.mutable_ar_body()->set_y(0.2);
  pose.mutable_ar_body()->set_z(0.3);
  // set accel_body.
  pose.mutable_accel_body()->set_x(0.4);
  pose.mutable_accel_body()->set_y(0.5);
  pose.mutable_accel_body()->set_z(0.6);
  // set vel_body.
  pose.mutable_vel_body()->set_x(0.7);
  pose.mutable_vel_body()->set_y(0.8);
  pose.mutable_vel_body()->set_z(0.9);
  pose.set_timestamp(1e6);

  LocalizationViewerDebugProto& localize_debug =
      complete_argument.localize_debug;
  localize_debug.mutable_v2_message()
      ->mutable_debug_plot_data()
      ->mutable_offset_xy_vcs()
      ->set_y(0.05);

  return complete_argument;
}

TEST(ConstructVehicleStateTest, EmptyArgumentTest) {
  VehicleStateInterface vehicle_state_interface;
  Chassis empty_chassis;
  PoseProto empty_pose;
  ControllerDebugProto controller_debug_proto;

  VehicleStateArgument valid_argument = BuildCompleteArgument();
  SteeringConverter steering_converter(/*wheel_base*/ 4.0,
                                       /*steer_ratio*/ 16.0,
                                       /*max_steer_angle*/ 3.0 * 3.14);

  const auto valid_state = vehicle_state_interface.Update(
      /*yaw_bias*/ 0.0, valid_argument.autonomy_state, valid_argument.pose,
      valid_argument.chassis, std::make_optional(valid_argument.localize_debug),
      steering_converter, &controller_debug_proto);
  EXPECT_TRUE(valid_state.ok());

  // Incomplete argument;
  const auto invalid_state_0 = vehicle_state_interface.Update(
      /*yaw_bias*/ 0.0, valid_argument.autonomy_state, empty_pose,
      valid_argument.chassis, std::nullopt, steering_converter,
      &controller_debug_proto);
  EXPECT_FALSE(invalid_state_0.ok());

  const auto invalid_state_1 = vehicle_state_interface.Update(
      /*yaw_bias*/ 0.0, valid_argument.autonomy_state, valid_argument.pose,
      empty_chassis, std::nullopt, steering_converter, &controller_debug_proto);
  EXPECT_FALSE(invalid_state_1.ok());

  const auto invalid_state_3 = vehicle_state_interface.Update(
      /*yaw_bias*/ 0.0, valid_argument.autonomy_state, empty_pose,
      empty_chassis, std::nullopt, steering_converter, &controller_debug_proto);
  EXPECT_FALSE(invalid_state_3.ok());

  // Incomplete chassis driving mode:
  Chassis chassis_no_driving_mode = valid_argument.chassis;
  chassis_no_driving_mode.clear_driving_mode();
  const auto invalid_state_4 = vehicle_state_interface.Update(
      /*yaw_bias*/ 0.0, valid_argument.autonomy_state, valid_argument.pose,
      chassis_no_driving_mode,
      std::make_optional(valid_argument.localize_debug), steering_converter,
      &controller_debug_proto);
  EXPECT_FALSE(invalid_state_4.ok());
}

TEST(ConstructVehicleStateTest, SteerPctNanTest) {
  VehicleStateInterface vehicle_state_interface;
  ControllerDebugProto controller_debug_proto;
  VehicleStateArgument valid_argument = BuildCompleteArgument();
  SteeringConverter steering_converter(/*wheel_base*/ 4.0,
                                       /*steer_ratio*/ 16.0,
                                       /*max_steer_angle*/ 3.0 * 3.14);

  const auto valid_state = vehicle_state_interface.Update(
      /*yaw_bias*/ 0.0, valid_argument.autonomy_state, valid_argument.pose,
      valid_argument.chassis, std::nullopt, steering_converter,
      &controller_debug_proto);
  EXPECT_TRUE(valid_state.ok());

  for (int i = 0; i < 10; ++i) {
    // Clear steering angle percentage as nan;
    valid_argument.chassis.set_steering_percentage(
        std::numeric_limits<double>::quiet_NaN());
    const auto state = vehicle_state_interface.Update(
        /*yaw_bias*/ 0.0, valid_argument.autonomy_state, valid_argument.pose,
        valid_argument.chassis,
        std::make_optional(valid_argument.localize_debug), steering_converter,
        &controller_debug_proto);

    if (i >= kChassisLossThres) {
      EXPECT_FALSE(state.ok()) << i;
    } else {
      EXPECT_TRUE(state.ok()) << i;
    }
  }
}

}  // namespace

}  // namespace qcraft::control
