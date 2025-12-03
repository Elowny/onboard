#include "onboard/control/longitudinal_postprocess/speed_mode_manager_v2.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"

namespace qcraft {
namespace control {
namespace {

TEST(SpeedModeManagerTest, AutoDriveTest) {
  ControllerConf controller_conf;
  ControllerDebugProto controller_debug_proto;
  SpeedModeManager speed_mode_manager(&controller_conf);
  speed_mode_manager.UpdateSpeedMode(
      /*is_auto_drive = */ false, /*is_full_stop = */ false,
      /*acc_calibration = */ -2.0,
      /*acc_idle = */ 0.0, &controller_debug_proto);
  EXPECT_EQ(speed_mode_manager.speed_mode(), SpeedMode::IDLE_MODE);
  speed_mode_manager.UpdateSpeedMode(
      /*is_auto_drive = */ false, /*is_full_stop = */ true,
      /*acc_calibration = */ -2.0,
      /*acc_idle = */ 0.0, &controller_debug_proto);
  EXPECT_EQ(speed_mode_manager.speed_mode(), SpeedMode::IDLE_MODE);
}

TEST(SpeedModeManagerTest, FullStopTest) {
  ControllerConf controller_conf;
  ControllerDebugProto controller_debug_proto;
  SpeedModeManager speed_mode_manager(&controller_conf);
  speed_mode_manager.UpdateSpeedMode(
      /*is_auto_drive = */ true, /*is_full_stop = */ true,
      /*acc_calibration = */ 2.0,
      /*acc_idle = */ 0.0, &controller_debug_proto);
  EXPECT_EQ(speed_mode_manager.speed_mode(), SpeedMode::DEC_MODE);
  speed_mode_manager.UpdateSpeedMode(
      /*is_auto_drive = */ true, /*is_full_stop = */ false,
      /*acc_calibration = */ 2.0,
      /*acc_idle = */ 0.0, &controller_debug_proto);
  EXPECT_EQ(speed_mode_manager.speed_mode(), SpeedMode::ACC_MODE);
  EXPECT_EQ(controller_conf.hysteresis_zone(), 0.0);
}

TEST(SpeedModeManagerTest, SpeedModeSwitchTest) {
  auto param_manager = CreateParamManagerFromCarId("Q1001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto controller_conf = run_params.vehicle_params().controller_conf();
  ControllerDebugProto controller_debug_proto;
  SpeedModeManager speed_mode_manager(&controller_conf);
  const double acc_idle = -0.5;

  // ACC_MODE
  double acc_calibration = acc_idle + controller_conf.hysteresis_zone() + 0.01;
  speed_mode_manager.UpdateSpeedMode(
      /*is_auto_drive = */ true, /*is_full_stop = */ false, acc_calibration,
      acc_idle, &controller_debug_proto);
  EXPECT_EQ(speed_mode_manager.speed_mode(), SpeedMode::ACC_MODE);

  // IDLE_MODE
  acc_calibration = acc_idle + controller_conf.hysteresis_zone();
  speed_mode_manager.UpdateSpeedMode(
      /*is_auto_drive = */ true, /*is_full_stop = */ false, acc_calibration,
      acc_idle, &controller_debug_proto);
  EXPECT_EQ(speed_mode_manager.speed_mode(), SpeedMode::IDLE_MODE);
  acc_calibration = acc_idle + controller_conf.hysteresis_zone() - 0.01;
  speed_mode_manager.UpdateSpeedMode(
      /*is_auto_drive = */ true, /*is_full_stop = */ false, acc_calibration,
      acc_idle, &controller_debug_proto);
  EXPECT_EQ(speed_mode_manager.speed_mode(), SpeedMode::IDLE_MODE);
  acc_calibration = acc_idle - controller_conf.hysteresis_zone() + 0.01;
  speed_mode_manager.UpdateSpeedMode(
      /*is_auto_drive = */ true, /*is_full_stop = */ false, acc_calibration,
      acc_idle, &controller_debug_proto);
  EXPECT_EQ(speed_mode_manager.speed_mode(), SpeedMode::IDLE_MODE);
  acc_calibration = acc_idle - controller_conf.hysteresis_zone();
  speed_mode_manager.UpdateSpeedMode(
      /*is_auto_drive = */ true, /*is_full_stop = */ false, acc_calibration,
      acc_idle, &controller_debug_proto);
  EXPECT_EQ(speed_mode_manager.speed_mode(), SpeedMode::IDLE_MODE);

  // DEC_MODE
  acc_calibration = acc_idle - controller_conf.hysteresis_zone() - 0.01;
  speed_mode_manager.UpdateSpeedMode(
      /*is_auto_drive = */ true, /*is_full_stop = */ false, acc_calibration,
      acc_idle, &controller_debug_proto);
  EXPECT_EQ(speed_mode_manager.speed_mode(), SpeedMode::DEC_MODE);

  // Debug proto check.
  const auto debug = controller_debug_proto.speed_mode_debug_proto();
  EXPECT_EQ(debug.acc_bound(), acc_idle);
  EXPECT_EQ(debug.hysteresis_upper_limit(),
            acc_idle + controller_conf.hysteresis_zone());
  EXPECT_EQ(debug.hysteresis_lower_limit(),
            acc_idle - controller_conf.hysteresis_zone());
}

}  // namespace
}  // namespace control
}  // namespace qcraft
