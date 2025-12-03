#include "onboard/planner/plan/acc/acc_preliminary_speed.h"

#include <algorithm>
#include <ostream>
#include <vector>

#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"

namespace qcraft {
namespace planner {
namespace {
TEST(RunAccPreliminarySpeed, BuildDefaultVSLimitTest) {
  const double speed_limit_val = 5.0;
  const double length = 5.0;
  const SpeedLimit speed_limit =
      internal::BuildDefaultVSLimit(speed_limit_val, length);
  const auto& speed_limit_range = speed_limit.speed_limit_ranges();
  QCHECK_LE(static_cast<int>(speed_limit_range.size()), 1);
  QCHECK_NEAR(speed_limit_range.front().speed_limit, speed_limit_val, 1e-6);
  QCHECK_NEAR(
      speed_limit_range.front().end_s - speed_limit_range.front().start_s,
      length, 1e-6);
}

TEST(RunAccPreliminarySpeed, BuildVSLimitTest) {
  const std::vector<double> kappa = {0.001, 0.001, 0.005, 0.002, 0.001, 0.0015};
  const std::vector<double> s = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0};
  const PiecewiseLinearFunction<double> kappa_s(s, kappa);
  const double cur_v = 5.0;
  AccPreliminarySpeedDebugProto debug_proto;
  const SpeedLimit speed_limit =
      internal::BuildVSLimit(kappa_s, cur_v, &debug_proto);
  const auto& speed_limit_range = speed_limit.speed_limit_ranges();
  QCHECK_GE(speed_limit.speed_limit_ranges().size(), true);
  for (int i = 0; i + 1 < speed_limit_range.size(); ++i) {
    const auto& cur_limit = speed_limit_range[i];
    const auto& next_limit = speed_limit_range[i + 1];
    QCHECK_LE(cur_limit.start_s, cur_limit.end_s);
    QCHECK_LE(cur_limit.end_s, next_limit.start_s);
    QCHECK_LE(next_limit.start_s, next_limit.end_s);
    QCHECK_GE(cur_limit.speed_limit, 0.0);
    QCHECK_GE(next_limit.speed_limit, 0.0);
    // const double kappa0 = kappa_s(cur_limit.end_s);
    // const double kappa1 = kappa_s(next_limit.end_s);
    // if (kappa0 < kappa1) {
    //   QCHECK_GE(cur_limit.speed_limit, next_limit.speed_limit);
    // } else {
    //   QCHECK_LE(cur_limit.speed_limit, next_limit.speed_limit);
    // }
  }
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
