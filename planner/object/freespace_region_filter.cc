#include "onboard/planner/object/freespace_region_filter.h"

#include <vector>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace planner {
namespace {

bool TrajectoryOverlapWithRegion(const prediction::PredictedTrajectory& traj,
                                 const AABox2d& region, double padding) {
  const auto padded_region =
      AABox2d(region.half_length() + padding, region.half_width() + padding,
              region.center());
  for (const auto& p : traj.points()) {
    if (padded_region.IsPointIn(p.pos())) {
      return true;
    }
  }
  return false;
}

}  // namespace

FilterReason::Type FreespaceRegionFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  const auto& box = object.bounding_box();
  if (prediction::IsStationaryTrajectory(traj) && !box.HasOverlap(region_)) {
    return FilterReason::STATIONARY_OBJECT_NOT_IN_FREESPACE_REGION;
  }

  constexpr double kDistanceBuffer = 1.0;  // m.
  const double padding = object.contour().CircleRadius() + kDistanceBuffer;
  if (!TrajectoryOverlapWithRegion(traj, region_, padding)) {
    return FilterReason::TRAJECTORY_NOT_IN_FREESPACE_REGION;
  }

  return FilterReason::NONE;
}
}  // namespace planner
}  // namespace qcraft
