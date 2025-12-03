#ifndef ONBOARD_PLANNER_DECISION_PARKING_BRAKE_RELEASE_H_
#define ONBOARD_PLANNER_DECISION_PARKING_BRAKE_RELEASE_H_

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// This function returns parking brake release constraint.
absl::StatusOr<ConstraintProto::StopLineProto>
BuildParkingBrakeReleaseConstraint(
    const qcraft::VehicleGeometryParamsProto& vehicle_geom,
    const DrivePassage& passage, const absl::Time parking_brake_release_time,
    const absl::Time plan_time);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_PARKING_BRAKE_RELEASE_H_
