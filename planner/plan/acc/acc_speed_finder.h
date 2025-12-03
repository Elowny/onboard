#ifndef ONBOARD_PLANNER_SPEED_ACC_SPEED_FINDER_H_
#define ONBOARD_PLANNER_SPEED_ACC_SPEED_FINDER_H_

#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "onboard/planner/plan/acc/acc_speed_finder_input.h"
#include "onboard/planner/plan/acc/acc_speed_finder_output.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_bound.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_optimizer_object_manager.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/proto/trajectory_point.pb.h"

// IWYU pragma: no_include <algorithm>

namespace qcraft {
namespace planner {

namespace internal {
absl::Status OptimizeAccSpeed(
    std::string_view base_name, double init_v, double init_a, double delta_t,
    const SpeedOptimizerObjectManager& opt_obj_mgr, double path_length,
    const SpeedBoundMapType& speed_bound_map,
    const SpeedVector& reference_speed,
    const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    SpeedVector* optimized_speed,
    SpeedFinderDebugProto* speed_finder_debug_proto);

SpeedBoundMapType EstimateDefaultSpeedBound(
    const SpeedLimit& speed_limit,
    const SpeedVector& dp_sampling_preliminary_speed, double init_v,
    double allowed_max_speed, int knot_num, double delta_t);

}  // namespace internal

absl::StatusOr<AccSpeedFinderOutput> FindAccSpeed(
    const AccSpeedFinderInput& input);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_ACC_SPEED_FINDER_H_
