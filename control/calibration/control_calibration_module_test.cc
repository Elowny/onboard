#include "onboard/control/calibration/control_calibration_module.h"

#include "gtest/gtest.h"

#include "onboard/lite/unittest/lite2_unittest_helper.h"

DECLARE_string(savedir);

namespace qcraft {
namespace control {
namespace {

std::shared_ptr<const PoseProto> CreatePose() {
  PoseProto pose;
  pose.set_speed(0.0);
  pose.mutable_accel_body()->set_x(0.1);
  pose.set_pitch(0.0);
  return std::make_shared<const PoseProto>(pose);
}

std::shared_ptr<const Chassis> CreateChassis() {
  Chassis chassis;
  chassis.set_throttle_percentage(10.0);
  chassis.set_brake_percentage(20.0);
  chassis.set_gear_location(qcraft::Chassis::GEAR_DRIVE);
  return std::make_shared<const Chassis>(chassis);
}

void WrapControlCalibrationInput(ControlCalibrationInput* input) {
  input->pose = CreatePose();
  input->chassis = CreateChassis();
}

class ControlCalibrationModuleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    module_.Init("ControlCalibrationModule", CONTROL_CALIBRATION_MODULE);
  }

  void TearDown() override { module_.Destroy(); }

  ControlCalibrationModule* get_control_calibration_module() {
    return dynamic_cast<ControlCalibrationModule*>(module_.GetLiteModule());
  }

 private:
  Lite2UnitTestHelper module_;
};

TEST_F(ControlCalibrationModuleTest, TestProcess) {
  auto* control_calibration_module = get_control_calibration_module();
  FLAGS_savedir = "";

  control_calibration_module->OnInit();
  vis::vantage::ChartsDataProto chart_data;
  WrapControlCalibrationInput(
      &control_calibration_module->control_calibration_input_);
  EXPECT_OK(control_calibration_module->Process(
      control_calibration_module->control_calibration_input_, &chart_data));
}

}  // namespace
}  // namespace control
}  // namespace qcraft
