#include "onboard/planner/decision/end_of_current_lane_path.h"

#include <utility>

#include "absl/status/status.h"

#include "onboard/maps/lane_path.h"
#include "onboard/math/geometry/halfplane.h"

namespace qcraft {
namespace planner {
// This function returns end of current route constraint.
absl::StatusOr<ConstraintProto::StopLineProto>
BuildEndOfCurrentLanePathConstraint(const DrivePassage& passage) {
  if (!passage.beyond_lane_path()) {
    return absl::NotFoundError("End of current lane path not in range.");
  }
  // Otherwise, lane path length is less than the drive passage length, meaning
  // that the current lane path ends.
  const double end_of_lane_path =
      passage.lane_path().length() + passage.lane_path_start_s();
  const auto curbs = passage.QueryCurbPointAtS(end_of_lane_path);
  if (!curbs.ok()) {
    return absl::NotFoundError("Curb boundaries not found.");
  }
  const HalfPlane halfplane(curbs->first, curbs->second);

  ConstraintProto::StopLineProto end_of_current_lane_path_constraint;
  end_of_current_lane_path_constraint.set_s(end_of_lane_path);
  end_of_current_lane_path_constraint.set_standoff(0.0);
  end_of_current_lane_path_constraint.set_time(0.0);
  halfplane.ToProto(end_of_current_lane_path_constraint.mutable_half_plane());
  if (passage.reach_destination()) {
    end_of_current_lane_path_constraint.set_id("route_destination");
    end_of_current_lane_path_constraint.mutable_source()
        ->mutable_route_destination()
        ->set_id(0);
  } else {
    end_of_current_lane_path_constraint.set_id("end_of_current_lane_path");
    end_of_current_lane_path_constraint.mutable_source()
        ->mutable_end_of_current_lane_path()
        ->set_id(0);
  }
  return end_of_current_lane_path_constraint;
}

}  // namespace planner
}  // namespace qcraft
