#ifndef ONBOARD_PLANNER_OBJECT_PARTIAL_SPACETIME_OBJECT_TRAJECTORY_H_
#define ONBOARD_PLANNER_OBJECT_PARTIAL_SPACETIME_OBJECT_TRAJECTORY_H_

#include <set>
#include <utility>
#include <vector>

#include "onboard/math/range1d.h"
#include "onboard/math/util.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"

namespace qcraft::planner {

class PartialSpacetimeObjectTrajectory {
 public:
  enum class DecisionType {
    FOLLOW = 1,
    LEAD = 2,
  };

  explicit PartialSpacetimeObjectTrajectory(SpacetimeObjectTrajectory st_traj)
      : st_traj_(std::move(st_traj)) {}

  const SpacetimeObjectTrajectory& st_traj() const { return st_traj_; }

  void AppendTimeRangeAndDecisonType(double low, double high,
                                     DecisionType decision_type) {
    Range1d range(low, high);
    const auto it = std::lower_bound(
        considered_time_ranges_.begin(), considered_time_ranges_.end(), range,
        [](const auto& lhs, const auto& rhs) { return lhs.low() < rhs.low(); });
    considered_time_ranges_.insert(it, std::move(range));
    decision_types_.push_back(decision_type);
  }

  std::optional<DecisionType> GetDecisionTypeAtTime(double time) const {
    QCHECK_EQ(considered_time_ranges_.size(), decision_types_.size());
    for (int i = 0; i < considered_time_ranges_.size(); ++i) {
      if (considered_time_ranges_[i].Contains(time)) {
        return decision_types_[i];
      }
    }
    return std::nullopt;
  }

 private:
  SpacetimeObjectTrajectory st_traj_;
  std::vector<Range1d<double>> considered_time_ranges_;
  std::vector<DecisionType> decision_types_;
};

}  // namespace qcraft::planner

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OBJECT_PARTIAL_SPACETIME_OBJECT_TRAJECTORY_H_
