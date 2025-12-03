#ifndef ONBOARD_PLANNER_SPEED_DECIDER_PRE_BRAKE_UTIL_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_PRE_BRAKE_UTIL_H_

#include <string>

#include "onboard/planner/speed/vt_speed_limit.h"

namespace qcraft {
namespace planner {

VtSpeedLimit GenerateConstAccSpeedLimit(double start_t, double end_t,
                                        double start_v, double min_v,
                                        double max_v, double acc,
                                        double time_step, int step_num,
                                        const std::string& info);

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_SPEED_DECIDER_PRE_BRAKE_UTIL_H_
