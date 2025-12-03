#ifndef ONBOARD_PLANNER_SPEED_DECIDER_PRE_BRAKE_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_PRE_BRAKE_DECIDER_H_

#include <optional>
#include <vector>

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/vt_speed_limit.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

std::optional<VtSpeedLimit> MakePedestrainPreBrakeDecision(
    const SpeedFinderParamsProto::PreBrakeDeciderParamsProto& params,
    const SpacetimeTrajectoryManager& st_traj_mgr, double current_v,
    double max_v, double time_step, int step_num,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

std::optional<VtSpeedLimit> MakeUncertainVehiclePreBrakeDecision(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_params, double current_v,
    double max_v, double time_step, int step_num,
    const SpeedVector& preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_SPEED_PRE_BRAKE_DECIDER_H_
