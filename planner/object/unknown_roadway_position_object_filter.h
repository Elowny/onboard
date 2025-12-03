#ifndef ONBOARD_PLANNER_OBJECT_UNKNOWN_ROADWAY_POSITION_OBJECT_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_UNKNOWN_ROADWAY_POSITION_OBJECT_FILTER_H_

#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/prediction/predicted_trajectory.h"

namespace qcraft {
namespace planner {

// This class filters certain types of objects that are classified as
// RoadWayPositionType::RWPT_UNKNOWN
class UnknownRoadwayPositionObjectFilter : public TrajectoryFilter {
 public:
  UnknownRoadwayPositionObjectFilter() = default;

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_UNKNOWN_ROADWAY_POSITION_OBJECT_FILTER_H_
