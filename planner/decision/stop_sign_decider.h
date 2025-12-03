#ifndef ONBOARD_PLANNER_DECISION_STOP_SIGN_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_STOP_SIGN_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/proto/stop_sign_state.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
namespace qcraft {
namespace planner {

struct StopSignDeciderOutput {
  // The stop lines created because of stop sign.
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  // The stop sign states for each stop sign seen on current drive passage.
  std::vector<StopSignStateProto> stop_sign_states;
};

// This function builds stop lines and stop sign states from the stop signs on
// current drive passage. The states should be passed to next iteration.
// TODO(lidong): When two cars arrived at the same time, there is a set of
// precedence rules, such as turn should yield to straight. This rule is not
// implemented in this decider. This implementation may cause two autonomous
// vehicles block each other if they arrived at the stop sign at similar time.
absl::StatusOr<StopSignDeciderOutput> BuildStopSignConstraints(
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geom,
    const DrivePassage& drive_passage, double now_in_seconds,
    const ApolloTrajectoryPointProto& plan_start_point,
    const ::google::protobuf::RepeatedPtrField<StopSignStateProto>&
        last_stop_sign_states);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_STOP_SIGN_DECIDER_H_
