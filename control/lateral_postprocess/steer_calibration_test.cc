#include "onboard/control/lateral_postprocess/steer_calibration.h"

#include <string>

#include "gtest/gtest.h"

#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"
namespace qcraft {
namespace control {

constexpr double kEpsilon = 1e-5;

TEST(SteerCalibrationTest, SteerCalibrationMain) {
  VehicleDriveParamsProto vehicle_drive_params;
  VehicleGeometryParamsProto vehicle_geometry_params;
  constexpr double kMaxSteerAngle = 9.42;  // rad.
  constexpr double kWheelBase = 2.84;      // m.
  constexpr double kSteerRatio = 16.0;
  vehicle_geometry_params.set_wheel_base(kWheelBase);
  vehicle_drive_params.set_steer_ratio(kSteerRatio);
  vehicle_drive_params.set_max_steer_angle(kMaxSteerAngle);
  ControllerConf control_conf;
  SteerCalibrationDebugProto debug;
  SteeringConverter steering_converter(vehicle_geometry_params,
                                       vehicle_drive_params);

  control_conf.mutable_veh_dynamic_model_conf()
      ->set_enable_dynamic_model_compensation(true);
  control_conf.mutable_lat_acc_gain_conf()->set_enable_lat_acc_gain(true);

  SteerCalibration steer_calibration(control_conf, &steering_converter);

  steer_calibration.SteerCalibrationMain(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, &debug);

  EXPECT_NEAR(debug.dynamic_gain(), 1.0, kEpsilon);
  EXPECT_NEAR(debug.post_process_gain(), 1.0, kEpsilon);
  EXPECT_NEAR(debug.steer_gap_kappa_compensate(), 0.0, kEpsilon);
  EXPECT_NEAR(debug.sliding_gain(), 1.0, kEpsilon);
  EXPECT_NEAR(debug.lat_acc_kappa_compensate(), 0.0, kEpsilon);
  EXPECT_NEAR(debug.roll_compensate_kappa(), 0.0, kEpsilon);
  // Todo: Yangu, add some real number for the test.
}

}  // namespace control
}  // namespace qcraft
