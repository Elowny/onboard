#include "onboard/prediction/util/kinematic_model.h"

#include <math.h>

#include <functional>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kCoef = 1.0 / 6.0;
}  // namespace
std::vector<double> RungeKutta4(
    absl::Span<const double> u0, double dt,
    const std::function<std::vector<double>(absl::Span<const double>)>& model) {
  const int m = u0.size();
  std::vector<double> f0(m, 0.0), f1(m, 0.0), f2(m, 0.0), f3(m, 0.0);
  std::vector<double> u(m, 0.0), u1(m, 0.0), u2(m, 0.0), u3(m, 0.0);
  //  Get four sample values of the derivative.
  f0 = model(u0);

  for (int i = 0; i < m; ++i) {
    u1[i] = u0[i] + dt * f0[i] * 0.5;
  }
  f1 = model(u1);

  for (int i = 0; i < m; ++i) {
    u2[i] = u0[i] + dt * f1[i] * 0.5;
  }
  f2 = model(u2);

  for (int i = 0; i < m; ++i) {
    u3[i] = u0[i] + dt * f2[i];
  }
  f3 = model(u3);
  //
  //  Combine them to estimate the solution.
  //
  for (int i = 0; i < m; ++i) {
    u[i] = u0[i] + dt * (f0[i] + 2.0 * f1[i] + 2.0 * f2[i] + f3[i]) * kCoef;
  }
  return u;
}

// x[]: state (including state and control)
std::vector<double> BicycleModel(absl::Span<const double> x) {
  QCHECK_EQ(x.size(), kBicycleModelStateNum);
  std::vector<double> xprime(x.size(), 0.0);
  const double beta = atan(tan(x[5]) * x[7] / (x[6] + x[7]));
  xprime[0] = x[2] * cos(x[3] + beta);
  xprime[1] = x[2] * sin(x[3] + beta);
  xprime[2] = x[4];
  xprime[3] = (x[2] / x[7]) * sin(beta);
  xprime[4] = 0.0;
  xprime[5] = 0.0;
  xprime[6] = 0.0;
  xprime[7] = 0.0;
  return xprime;
}

BicycleModelState SimulateBicycleModel(const BicycleModelState& prev_state,
                                       double lf, double lr, double dt) {
  QCHECK_GE(lf, 0.0);
  QCHECK_GT(lr, 0.0);

  std::vector<double> x(kBicycleModelStateNum, 0.0);
  x[0] = prev_state.x;
  x[1] = prev_state.y;
  x[2] = prev_state.v;
  x[3] = prev_state.heading;
  x[4] = prev_state.acc;
  x[5] = prev_state.front_wheel_angle;
  x[6] = lf;
  x[7] = lr;
  const std::vector<double> x1 = RungeKutta4(x, dt, BicycleModel);
  return BicycleModelState{.x = x1[0],
                           .y = x1[1],
                           .v = x1[2],
                           .heading = x1[3],
                           .acc = x1[4],
                           .front_wheel_angle = x1[5]};
}

// x[]: state (including state and control)
std::vector<double> UniCycleModel(absl::Span<const double> x) {
  QCHECK_EQ(x.size(), kUniCycleStateNum);
  std::vector<double> xprime(x.size(), 0.0);
  xprime[0] = x[2] * cos(x[3]);
  xprime[1] = x[2] * sin(x[3]);
  xprime[2] = x[5];
  xprime[3] = x[4];
  xprime[4] = 0.0;
  xprime[5] = 0.0;
  return xprime;
}

UniCycleState SimulateUniCycleModel(const UniCycleState& prev_state,
                                    double dt) {
  std::vector<double> x(kUniCycleStateNum, 0.0);
  x[0] = prev_state.x;
  x[1] = prev_state.y;
  x[2] = prev_state.v;
  x[3] = prev_state.heading;
  x[4] = prev_state.yaw_rate;
  x[5] = prev_state.acc;
  const std::vector<double> x1 = RungeKutta4(x, dt, UniCycleModel);
  return UniCycleState{.x = x1[0],
                       .y = x1[1],
                       .v = x1[2],
                       .heading = x1[3],
                       .yaw_rate = x1[4],
                       .acc = x1[5]};
}

}  // namespace prediction
}  // namespace qcraft
