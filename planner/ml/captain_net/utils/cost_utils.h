#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_UTILS_COST_UTILS_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_UTILS_COST_UTILS_H_  // NOLINT

#include <vector>

namespace qcraft::planner::ml {

void CalculateTimeDecayWeight(double time_decay_rate,
                              std::vector<double>* ref_traj_weight);

}  // namespace qcraft::planner::ml

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_UTILS_COST_UTILS_H_
