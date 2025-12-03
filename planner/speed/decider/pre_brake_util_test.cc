#include "onboard/planner/speed/decider/pre_brake_util.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEps = 1e-3;

TEST(PreBrakeUtil, GenerateConstAccSpeedLimit) {
  constexpr double kStartTime = 1.0;  // s.
  constexpr double kEndTime = 5.0;    // s.
  constexpr double kStartVel = 10.0;  // m/s.
  constexpr double kMinVel = 1.0;     // m/s.
  constexpr double kMaxVel = 30.0;    // m/s.
  constexpr double kAcc = -3.0;       // m/ss.
  constexpr double kTimeStep = 0.1;   // s.
  constexpr int kStepNum = 100;
  const std::string info = "Test Info";

  const VtSpeedLimit vt_speed_limit =
      GenerateConstAccSpeedLimit(kStartTime, kEndTime, kStartVel, kMinVel,
                                 kMaxVel, kAcc, kTimeStep, kStepNum, info);

  EXPECT_EQ(vt_speed_limit.size(), kStepNum + 1);
  EXPECT_EQ(vt_speed_limit.front().info, info);
  EXPECT_EQ(vt_speed_limit.front().speed_limit, kMaxVel);
  EXPECT_EQ(vt_speed_limit.back().info, info);
  EXPECT_EQ(vt_speed_limit.back().speed_limit, kMaxVel);

  const size_t start_idx = static_cast<size_t>(kStartTime / kTimeStep);
  const size_t end_idx = static_cast<size_t>(kEndTime / kTimeStep);
  EXPECT_NEAR(vt_speed_limit[start_idx].speed_limit, kStartVel, kEps);
  EXPECT_NEAR(vt_speed_limit[end_idx].speed_limit, kMinVel, kEps);
  EXPECT_NEAR((vt_speed_limit[start_idx + 1].speed_limit -
               vt_speed_limit[start_idx].speed_limit) /
                  kTimeStep,
              kAcc, kEps);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
