#include "onboard/planner/object/path_boundary_filter.h"

#include "onboard/planner/plan/acc/acc_defs.h"
#include "onboard/planner/plan/acc/acc_target_util.h"

namespace qcraft {
namespace planner {

FilterReason::Type PathBoundaryFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& /*pred_traj*/) const {
  // Filter object by it's current position w.r.t. outer path boundaries or
  // explicitly defined "near path boundary distance buffer".
  if (path_ff_ == nullptr) {
    return FilterReason::NONE;
  }
  const auto& contour = object.contour();
  const auto& fbox_or =
      path_ff_->QueryLongitudinallyBoundedFrenetBoxAtContour(contour);
  if (!fbox_or.ok()) {
    return FilterReason::OBJECT_NOT_ON_OR_NEAR_PATH_BOUNDARY;
  }
  const auto& fbox = *fbox_or;
  const auto on_path_type =
      ObjectFrenetBoxOnPathType(*path_sl_, fbox, lateral_distance_buffer_);
  if (on_path_type == OnPathType::OPT_OFF_BOUND) {
    return FilterReason::OBJECT_NOT_ON_OR_NEAR_PATH_BOUNDARY;
  }
  return FilterReason::NONE;
}

}  // namespace planner
}  // namespace qcraft
