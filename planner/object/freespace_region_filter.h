#ifndef ONBOARD_PLANNER_OBJECT_FREESPACE_REGION_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_FREESPACE_REGION_FILTER_H_

#include "onboard/lite/check.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/prediction/predicted_trajectory.h"

namespace qcraft {
namespace planner {

class FreespaceRegionFilter : public TrajectoryFilter {
 public:
  explicit FreespaceRegionFilter(const AABox2d* region)
      : region_(*QCHECK_NOTNULL(region)) {}

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  AABox2d region_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_FREESPACE_REGION_FILTER_H_
