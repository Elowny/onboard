#include "onboard/control/vehicle_control_module.h"

#include <memory>
#include <utility>

#include "gtest/gtest.h"

// IWYU pragma: no_include "onboard/global/buffered_logger.h"
// IWYU pragma: no_include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/base/macros.h"
#include "onboard/control/benchmark/control_test_config_setup.h"
#include "onboard/global/clock.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/unittest/lite2_unittest_helper.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace control {
namespace {

PoseProto WrapPoseWithCurrentTime() {
  PoseProto pose = BuildPoseForTest();
  pose.set_timestamp(ToUnixDoubleSeconds(Clock::Now()));
  return pose;
}

Chassis WrapChassisWithCurrentTime() {
  Chassis chassis = BuildChassisForTest();
  chassis.mutable_header()->set_timestamp(ToUnixDoubleSeconds(Clock::Now()) *
                                          1e6);
  return chassis;
}

TrajectoryProto WrapTrajectoryWithCurrentTime() {
  TrajectoryProto traj = BuildTrajectoryProtoForTest();
  const double now = ToUnixDoubleSeconds(Clock::Now());
  traj.mutable_header()->set_timestamp(now * 1e6);
  traj.set_trajectory_start_timestamp(now);
  return traj;
}

class ControlModuleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    module_.Init("VehicleControlModule", VEHICLE_CONTROL_MODULE);
  }

  void TearDown() override { module_.Destroy(); }

  VehicleControlModule* get_control_module() {
    return dynamic_cast<VehicleControlModule*>(module_.GetLiteModule());
  }

 private:
  Lite2UnitTestHelper module_;
};

TEST_F(ControlModuleTest, ProduceControlCommandTest) {
  auto* control_module = get_control_module();

  ControlCommand control_command;
  ControllerDebugProto controller_debug_proto;
  const auto state_produce_cmd0 = control_module->ProduceControlCommand(
      &control_command, &controller_debug_proto);
  ASSERT_FALSE(state_produce_cmd0.ok());

  VehicleControlModule::LocalView local_view = {
      .autonomy_state = std::make_shared<AutonomyStateProto>(
          BuildAutonomyStateProtoForTest()),
      .chassis = std::make_shared<Chassis>(WrapChassisWithCurrentTime()),
      .trajectory =
          std::make_shared<TrajectoryProto>(WrapTrajectoryWithCurrentTime()),
      .pose = std::make_shared<PoseProto>(WrapPoseWithCurrentTime())};

  ASSERT_OK(control_module->UpdateInput(local_view, &controller_debug_proto));
  ASSERT_OK(
      control_module->CheckTimestamp(local_view, /*is_input_ready*/ false));
  control_module->local_view_ = std::move(local_view);
  control_module->parameter_identificator_->steer_delay_.steer_delay = 0.3;

  control_module->Proc();
  const auto state_produce_cmd1 = control_module->ProduceControlCommand(
      &control_command, &controller_debug_proto);
  ASSERT_TRUE(state_produce_cmd1.ok());
}

}  // namespace
}  // namespace control
}  // namespace qcraft
