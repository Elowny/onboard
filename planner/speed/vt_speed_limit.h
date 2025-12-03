#ifndef ONBOARD_PLANNER_SPEED_VT_SPEED_LIMIT_H_
#define ONBOARD_PLANNER_SPEED_VT_SPEED_LIMIT_H_

#include <vector>

#include "onboard/planner/speed/speed_limit.h"

namespace qcraft::planner {

// TODO(shiping): Refactor VtSpeedLimit class (e.g. use two vector to represent
// v and t sequence).
using VtSpeedLimit = std::vector<SpeedLimit::SpeedLimitInfo>;

void MergeVtSpeedLimit(const VtSpeedLimit& source, VtSpeedLimit* target);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_VT_SPEED_LIMIT_H_
