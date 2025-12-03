#ifndef ONBOARD_PLANNER_PLAN_AEB_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_AEB_PLANNER_H_

#include <optional>
#include <vector>

#include "onboard/planner/emergency_stop.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct AebPlannerOutput {
  std::optional<aeb::EmergencyStopInfo> emergency_stop_info;
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
};

namespace aeb {
std::optional<aeb::EmergencyStopInfo> GenerateEmergencyStopInfo(
    const PlannerInput& input,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points);
std::vector<ApolloTrajectoryPointProto> PlanEmergencyStopTrajectory(
    const ApolloTrajectoryPointProto& plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points,
    const PlannerParamsProto& planner_params);
}  // namespace aeb

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_AEB_PLANNER_H_
