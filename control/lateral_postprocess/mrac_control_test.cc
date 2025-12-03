#include "onboard/control/lateral_postprocess/mrac_control.h"

#include "gtest/gtest.h"

#include "onboard/control/control_defs.h"
#include "onboard/math/util.h"

namespace qcraft::control {
namespace {
constexpr double kEpsilon = 1e-5;

TEST(MracControl, Reset) {
  MracConfProto mrac_config;
  const double steer_delay = 0.1;
  mrac_config.set_state_weight(0.3);
  mrac_config.set_input_weight(1.0);
  mrac_config.set_ke_weight(0.2);
  mrac_config.set_speed_limit(0.2);
  mrac_config.set_kappa_threshold(0.02);
  mrac_config.set_latacc_threshold(0.4);
  MracControl mrac_control(mrac_config, steer_delay);

  MracDebugProto debug;
  const MracInput mrac_input = {.is_automode = false,
                                .kappa_target = 0.04,
                                .av_kappa = 0.04,
                                .speed = 0.1,
                                .kappa_upper = 0.1,
                                .kappa_lower = -0.1};
  const double kappa_cmd = mrac_control.Compute(mrac_input, &debug);
  EXPECT_NEAR(kappa_cmd, mrac_input.kappa_target, kEpsilon);
  EXPECT_NEAR(debug.av_kappa(), 0.0, kEpsilon);
  EXPECT_NEAR(debug.kappa_input(), 0.0, kEpsilon);
}

TEST(MracControl, LowSpeed) {
  MracConfProto mrac_config;
  const double steer_delay = 0.1;
  mrac_config.set_state_weight(0.3);
  mrac_config.set_input_weight(1.0);
  mrac_config.set_ke_weight(0.2);
  mrac_config.set_speed_limit(0.2);
  mrac_config.set_kappa_threshold(0.02);
  mrac_config.set_latacc_threshold(0.4);
  MracControl mrac_control(mrac_config, steer_delay);

  MracDebugProto debug;
  const MracInput mrac_input = {.is_automode = true,
                                .kappa_target = 0.04,
                                .av_kappa = 0.04,
                                .speed = 0.1,
                                .kappa_upper = 0.1,
                                .kappa_lower = -0.1};
  const double kappa_cmd = mrac_control.Compute(mrac_input, &debug);
  EXPECT_NEAR(kappa_cmd, mrac_input.kappa_target, kEpsilon);
  EXPECT_NEAR(debug.av_kappa(), mrac_input.av_kappa, kEpsilon);
  EXPECT_NEAR(debug.kappa_input(), mrac_input.kappa_target, kEpsilon);
}

TEST(MracControl, Compute) {
  MracConfProto mrac_config;
  const double steer_delay = 0.1;
  mrac_config.set_state_weight(10.0);
  mrac_config.set_input_weight(10.0);
  mrac_config.set_ke_weight(1.0);
  mrac_config.set_speed_limit(0.2);
  mrac_config.set_kappa_threshold(0.02);
  mrac_config.set_latacc_threshold(0.4);
  MracControl mrac_control(mrac_config, steer_delay);

  MracDebugProto debug;
  const MracInput mrac_input = {.is_automode = true,
                                .kappa_target = 0.04,
                                .av_kappa = 0.03,
                                .speed = 5.0,
                                .kappa_upper = 0.1,
                                .kappa_lower = -0.1};
  double kappa_cmd = 0.0;
  for (int i = 0; i < FloorToInt(2.0 * kControlFrequency); ++i) {
    kappa_cmd = mrac_control.Compute(mrac_input, &debug);
  }

  EXPECT_GE(kappa_cmd, mrac_input.kappa_target);
  EXPECT_NEAR(debug.av_kappa(), mrac_input.av_kappa, kEpsilon);
  EXPECT_NEAR(debug.kappa_input(), mrac_input.kappa_target, kEpsilon);
}
}  // namespace
}  // namespace qcraft::control
