#ifndef ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
#define ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_limit_provider.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_graph.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::Status MakeInteractiveSpeedDecision(
    std::string_view base_name, const VehicleGeometryParamsProto& vehicle_geom,
    const MotionConstraintParamsProto& motion_constraint_params,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const DiscretizedPath& path, double current_v, double current_a,
    const SpeedFinderParamsProto& speed_finder_params, double speed_cap,
    int traj_steps, SpeedLimitProvider* speed_limit_provider,
    SpeedVector* preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    InteractiveSpeedDebugProto* interactive_speed_debug);

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
