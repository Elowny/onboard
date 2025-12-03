#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_

#include <limits>

#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft {
namespace planner {
namespace optimizer {

using Mfob = MixedFourthOrderBicycle;

struct LeadingInfo {
  double s = std::numeric_limits<double>::infinity();
  double v = std::numeric_limits<double>::infinity();
};

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_
