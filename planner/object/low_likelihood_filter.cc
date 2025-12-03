#include "onboard/planner/object/low_likelihood_filter.h"

#include <optional>

namespace qcraft {
namespace planner {

FilterReason::Type LowLikelihoodFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  if (traj.probability() < min_prob_) {
    return FilterReason::TRAJECTORY_LOW_LIKELIHOOD;
  }

  if (only_use_most_likely_traj_) {
    const auto index_or = object.MostLikelyTrajectory();
    if (index_or.has_value() &&
        traj.probability() < object.traj(*index_or).probability()) {
      return FilterReason::NOT_MOST_LIKELY_TRAJECTORY;
    }
  }

  return FilterReason::NONE;
}

}  // namespace planner
}  // namespace qcraft
