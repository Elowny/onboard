#include "onboard/control/lateral_postprocess/compensate_curvature_gain.h"

#include "gtest/gtest.h"

#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

constexpr double kEpsilon = 1e-5;

TEST(CompensateCurvatureGain, ZeroCompensationTest) {
  const KappaGainInput input;
  ControllerConf control_conf;
  SteerCalibrationDebugProto debug;

  EXPECT_NEAR(ComputeKappaGain(input, control_conf, &debug), 1.0, kEpsilon);
}

}  // namespace control
}  // namespace qcraft
