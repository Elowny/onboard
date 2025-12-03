#include "onboard/control/closed_loop_acc/speed_mode_manager.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/proto/control_cmd.pb.h"

namespace qcraft::control {

constexpr double kEpsilon = 1e-5;

TEST(SpeedModeManagerTest, CalculteSpeedModeAccBound) {
  ControllerConf control_conf;
  control_conf.set_hysteresis_zone(0.2);
  ClosedLoopAcc closed_loop_acc(control_conf, /*is_available_idle=*/true, 0, 0);

  closed_loop_acc.CalculteSpeedModeAccBound(-0.5);
  EXPECT_NEAR(closed_loop_acc.acc_lowerbound_, -0.7, kEpsilon);
  EXPECT_NEAR(closed_loop_acc.dec_upperbound_, -0.3, kEpsilon);
}

TEST(SpeedModeManagerTest, ResetSpeedMode) {
  ControllerConf control_conf;
  ClosedLoopAcc closed_loop_acc(control_conf, /*is_available_idle=*/true, 0, 0);
  EXPECT_NEAR(closed_loop_acc.speed_mode_, SpeedMode::ACC_MODE, kEpsilon);
  EXPECT_NEAR(closed_loop_acc.acc_counter_, 0, kEpsilon);
  EXPECT_NEAR(closed_loop_acc.dec_counter_, 0, kEpsilon);
  EXPECT_NEAR(closed_loop_acc.is_first_frame_, true, kEpsilon);
}

TEST(SpeedModeManagerTest, UpdateAccCommand) {
  ControllerConf control_conf;
  ControllerDebugProto controller_debug_proto;
  control_conf.set_hysteresis_zone(0.2);
  control_conf.set_acc2dec_timer(0.1);
  control_conf.set_dec2acc_timer(0.1);
  ClosedLoopAcc closed_loop_acc(control_conf, /*is_available_idle=*/true, 0, 0);
  closed_loop_acc.UpdateAccCommand(/*is_auto_mode=*/true,
                                   /*is_full_stop=*/false, 1.0, 1.0, 0.0, 0.0,
                                   &controller_debug_proto);
  EXPECT_NEAR(closed_loop_acc.speed_mode_, SpeedMode::ACC_MODE, kEpsilon);
  EXPECT_NEAR(closed_loop_acc.acc_counter_, 0, kEpsilon);
  EXPECT_NEAR(closed_loop_acc.dec_counter_, 0, kEpsilon);

  // Uphill with 0 acc
  closed_loop_acc.UpdateAccCommand(/*is_auto_mode=*/true,
                                   /*is_full_stop=*/false, 1.0, 0.0, 1.0, -0.2,
                                   &controller_debug_proto);
  EXPECT_NEAR(closed_loop_acc.speed_mode_, SpeedMode::ACC_MODE, kEpsilon);

  // Downhill
  closed_loop_acc.UpdateAccCommand(/*is_auto_mode=*/true,
                                   /*is_full_stop=*/false, -1.0, 0.0, -1.0,
                                   -0.2, &controller_debug_proto);
  EXPECT_NEAR(closed_loop_acc.speed_mode_, SpeedMode::DEC_MODE, kEpsilon);

  closed_loop_acc.UpdateAccCommand(/*is_auto_mode=*/true,
                                   /*is_full_stop=*/false, -0.3, 0.0, -0.3,
                                   -0.2, &controller_debug_proto);
  EXPECT_NEAR(closed_loop_acc.speed_mode_, SpeedMode::DEC_MODE, kEpsilon);

  closed_loop_acc.UpdateAccCommand(/*is_auto_mode=*/true,
                                   /*is_full_stop=*/false, -0.1, 0.1, -0.2,
                                   -0.2, &controller_debug_proto);
  EXPECT_NEAR(closed_loop_acc.speed_mode_, SpeedMode::DEC_MODE, kEpsilon);
}

}  // namespace qcraft::control
