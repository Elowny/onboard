#include "onboard/planner/decision/parking_brake_release.h"

#include <utility>

#include "absl/status/status.h"
#include "onboard/math/geometry/halfplane.h"

namespace qcraft {
namespace planner {
// This function returns parking brake release constraint.
absl::StatusOr<ConstraintProto::StopLineProto>
BuildParkingBrakeReleaseConstraint(
    const qcraft::VehicleGeometryParamsProto& vehicle_geom,
    const DrivePassage& passage, const absl::Time parking_brake_release_time,
    const absl::Time plan_time) {
  // TODO(all): Read this time from vehicle params.
  constexpr absl::Duration kWaitForParkBrakeDuration = absl::Seconds(1);
  if (parking_brake_release_time + kWaitForParkBrakeDuration < plan_time) {
    return absl::OutOfRangeError("No need to care this parking brake release.");
  }

  const auto curbs = passage.QueryCurbPointAtS(
      passage.lane_path_start_s() + vehicle_geom.front_edge_to_center());
  if (!curbs.ok()) {
    return absl::NotFoundError("Curb boundaries not found.");
  }
  const HalfPlane halfplane(curbs->first, curbs->second);
  ConstraintProto::StopLineProto parking_brake_release_constraint;
  parking_brake_release_constraint.set_s(vehicle_geom.front_edge_to_center());
  parking_brake_release_constraint.set_standoff(0.0);
  parking_brake_release_constraint.set_time(0.0);
  halfplane.ToProto(parking_brake_release_constraint.mutable_half_plane());
  parking_brake_release_constraint.set_id("parking_brake_release");
  parking_brake_release_constraint.mutable_source()
      ->mutable_parking_brake_release()
      ->set_id(0);
  return parking_brake_release_constraint;
}

}  // namespace planner
}  // namespace qcraft
