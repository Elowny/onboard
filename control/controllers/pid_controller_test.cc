#include "onboard/control/controllers/pid_controller.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/control/control_defs.h"
#include "onboard/math/util.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-3;

TEST(PIDControl, computer) {
  PIDControl pid_control;
  PIDConfig config = {
      .Kp = 1.0,
      .Ki = 0.0,
      .Kd = 0.0,
      .period = kControlInterval,
      .min_value = -0.3,
      .max_value = 0.3,
      .integral_limit = 1.0,
  };
  pid_control.SetConfig(config);
  PIDOut out;

  pid_control.Compute(1.0);
  out = pid_control.GetPIDOut();
  EXPECT_NEAR(out.offset_P, 1.0, kEpsilon);
  EXPECT_NEAR(out.result, 0.3, kEpsilon);

  pid_control.Compute(0.2);
  out = pid_control.GetPIDOut();
  EXPECT_NEAR(out.offset_P, 0.2, kEpsilon);
  EXPECT_NEAR(out.result, 0.2, kEpsilon);

  pid_control.ResetIntegral();
  config = {
      .Kp = 1.0,
      .Ki = 1.0,
      .Kd = 0.0,
      .period = kControlInterval,
      .min_value = -0.3,
      .max_value = 0.3,
      .integral_limit = 0.5,
  };
  pid_control.SetConfig(config);
  for (int i = 0; i < FloorToInt(2.0 * kControlFrequency); ++i) {
    pid_control.Compute(1.0);
    out = pid_control.GetPIDOut();
  }
  EXPECT_NEAR(out.offset_I, 0.5, kEpsilon);
  for (int i = 0; i < FloorToInt(0.1 * kControlFrequency); ++i) {
    pid_control.Compute(-1.0);
    out = pid_control.GetPIDOut();
  }
  EXPECT_NEAR(out.offset_I, 0.4, kEpsilon);
}
}  // namespace
}  // namespace qcraft::control
