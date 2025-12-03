#include "onboard/planner/math/intelligent_driver_model.h"

#include <algorithm>
#include <cmath>
namespace qcraft {
namespace planner {

namespace idm {
namespace {
constexpr double kEpsilon = 1e-3;
}
double ComputeIDMAcceleration(double v, double ds, double dv,
                              const Parameters& params) {
  const double s_follow =
      params.s_min + v * params.t_desire -
      0.5 * v * dv / (std::sqrt(params.acc_max * params.comfortable_brake));
  const double acc =
      params.acc_max * (1.0 - std::pow(v / params.v_desire, params.delta) -
                        std::pow(s_follow / std::max(ds, kEpsilon), 2.0));
  return std::clamp(acc, -params.brake_max, params.acc_max);
}
}  // namespace idm
}  // namespace planner
}  // namespace qcraft
