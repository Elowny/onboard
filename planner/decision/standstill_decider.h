#ifndef ONBOARD_PLANNER_DECISION_STANDSTILL_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_STANDSTILL_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
absl::StatusOr<std::vector<ConstraintProto::StopLineProto>>
BuildStandstillConstraints(
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage,
    absl::Span<const ConstraintProto::StopLineProto> stop_lines);
}  // namespace planner
}  // namespace qcraft
#endif
