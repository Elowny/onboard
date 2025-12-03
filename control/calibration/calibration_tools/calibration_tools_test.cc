#include "onboard/control/calibration/calibration_tools/calibration_tools.h"

#include <cmath>

#include "gtest/gtest.h"

constexpr char kDirname[] =
    "onboard/control/calibration/calibration_tools/testdata";
constexpr char kFilename[] = "Q1002";

TEST(ControlCalibration, IDLE) {
  const char method[] = "IDLE";
  auto calibration_ptr =
      std::make_unique<qcraft::control::calibration::CalibrationTools>(
          kDirname, kFilename, method, 0.0, 0.0, true);
  calibration_ptr->Reset();
  double speed = 0.0;
  double acceleration = 0.0;
  qcraft::vis::vantage::ChartDataProto chart;
  for (int i = 0; i < 200; i++) {
    speed = 0.01 * i;
    acceleration =
        -0.12778741 * speed * speed - 0.06507266 * speed + 0.32284199;
    const auto status =
        calibration_ptr->Process(speed, acceleration, 0.0, 0.0, 0.0,
                                 qcraft::Chassis::GEAR_DRIVE, &chart);
    EXPECT_EQ(status, true);
  }
}

TEST(ControlCalibration, SLIDE) {
  const char method[] = "SLIDE";
  auto calibration_ptr =
      std::make_unique<qcraft::control::calibration::CalibrationTools>(
          kDirname, kFilename, method, 0.0, 0.0, true);
  double speed = 0.0;
  double acceleration = 0.0;
  qcraft::vis::vantage::ChartDataProto chart;
  for (int i = 0; i < 150; i++) {
    acceleration = 1.0;
    speed += 0.01 * acceleration;
    const auto status =
        calibration_ptr->Process(speed, acceleration, 0.0, 0.0, 0.0,
                                 qcraft::Chassis::GEAR_DRIVE, &chart);
    EXPECT_EQ(status, true);
  }
  for (int i = 2000; i >= 150; i--) {
    speed = 0.01 * i;
    acceleration = 0.0010 * speed * speed - 0.0670 * speed + 0.0593;
    const auto status =
        calibration_ptr->Process(speed, acceleration, 0.0, 0.0, 0.0,
                                 qcraft::Chassis::GEAR_DRIVE, &chart);
    EXPECT_EQ(status, true);
  }
}

TEST(ControlCalibration, FORCE) {
  const char method[] = "FORCE";
  auto calibration_ptr =
      std::make_unique<qcraft::control::calibration::CalibrationTools>(
          kDirname, kFilename, method, 0.0, 0.0, true);
  double speed = 5.0;
  double acceleration = 0.0;
  double percentage = 0.0;
  double throttle = 0.0;
  double brake = 0.0;
  qcraft::vis::vantage::ChartDataProto chart;
  for (int i = 0; i < 500; i++) {
    acceleration = std::sin(0.02 * i);
    speed += 0.02 * acceleration;
    percentage = 100.0 * std::sin(0.02 * i);
    if (percentage >= 0.0) {
      throttle = percentage;
    } else {
      brake = -percentage;
    }

    const auto status =
        calibration_ptr->Process(speed, acceleration, 0.0, throttle, brake,
                                 qcraft::Chassis::GEAR_DRIVE, &chart);
    calibration_ptr->SaveData();
    EXPECT_EQ(status, true);
  }
}

TEST(ControlCalibration, CheckCounterSuccess) {
  const std::vector<double> list{1.0, 2.0, 3.0, 4.0, 5.0};
  bool status =
      qcraft::control::calibration::CheckCounterSuccess(0.0, 10.0, list, 4);
  EXPECT_EQ(status, true);
  status =
      qcraft::control::calibration::CheckCounterSuccess(0.0, 10.0, list, 6);
  EXPECT_EQ(status, false);
}
