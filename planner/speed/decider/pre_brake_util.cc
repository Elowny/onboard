#include "onboard/planner/speed/decider/pre_brake_util.h"

#include <algorithm>

namespace qcraft {
namespace planner {

VtSpeedLimit GenerateConstAccSpeedLimit(double start_t, double end_t,
                                        double start_v, double min_v,
                                        double max_v, double acc,
                                        double time_step, int step_num,
                                        const std::string& info) {
  constexpr double kExtendTime = 1.0;  // s.
  VtSpeedLimit vt_speed_limit;
  vt_speed_limit.reserve(step_num + 1);
  for (int i = 0; i < step_num + 1; ++i) {
    const double current_time = static_cast<double>(i) * time_step;
    double current_v = max_v;
    if (current_time >= start_t && current_time <= end_t) {
      current_v = std::min(start_v + (current_time - start_t) * acc, max_v);
    } else if (current_time > end_t && current_time < end_t + kExtendTime) {
      current_v = std::min(start_v + (end_t - start_t) * acc, max_v);
    }
    current_v = std::max(current_v, min_v);
    vt_speed_limit.emplace_back(current_v, info);
  }
  return vt_speed_limit;
}

}  // namespace planner
}  // namespace qcraft
