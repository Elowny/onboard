#ifndef ONBOARD_PLANNER_OBJECT_LOW_LIKELIHOOD_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_LOW_LIKELIHOOD_FILTER_H_

#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/prediction/predicted_trajectory.h"

namespace qcraft {
namespace planner {

class LowLikelihoodFilter : public TrajectoryFilter {
 public:
  explicit LowLikelihoodFilter(double min_prob, bool only_use_most_likely_traj)
      : min_prob_(min_prob),
        only_use_most_likely_traj_(only_use_most_likely_traj) {}

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  double min_prob_;
  bool only_use_most_likely_traj_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_LOW_LIKELIHOOD_FILTER_H_
