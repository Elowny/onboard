#include "onboard/planner/ml/captain_net/utils/cost_utils.h"

namespace qcraft::planner::ml {

void CalculateTimeDecayWeight(double time_decay_rate,
                              std::vector<double>* ref_traj_weight) {
  for (int i = 1; i < ref_traj_weight->size(); ++i) {
    (*ref_traj_weight)[i] = (*ref_traj_weight)[i - 1] * time_decay_rate;
  }
}

}  // namespace qcraft::planner::ml
