#include "onboard/control/calibration/calibration_tools/calibration_force.h"

#include <cmath>

#include "gtest/gtest.h"

TEST(ControlCalibration, FORCE) {
  auto calibration_force_ptr =
      std::make_unique<qcraft::control::calibration::CalibrationForce>();
  EXPECT_EQ(false, calibration_force_ptr->Init(std::nullopt, 0.0, 0.0));

  auto file_name = std::optional<std::string>{
      "onboard/control/calibration/calibration_tools/testdata/Q1002"};
  EXPECT_EQ(true, calibration_force_ptr->Init(file_name, 0.0, 0.0));

  double speed = 10.0;
  double acceleration = 0.0;
  double percentage = 0.0;
  double throttle = 0.0;
  double brake = 0.0;
  for (int i = 0; i < 200; i++) {
    acceleration = std::sin(0.02 * i);
    speed += 0.02 * acceleration;
    percentage = 100.0 * std::sin(0.02 * i);
    if (percentage >= 0.0) {
      throttle = percentage;
    } else {
      brake = -percentage;
    }

    const auto status =
        calibration_force_ptr->Process(speed, acceleration, throttle, brake);
    EXPECT_EQ(status, true);
  }
}
