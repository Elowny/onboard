#ifndef ONBOARD_PLANNER_SPEED_SPEED_DECISION_UTIL_H_
#define ONBOARD_PLANNER_SPEED_SPEED_DECISION_UTIL_H_

#include <vector>

#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"

namespace qcraft::planner {
inline constexpr double kSpeedLimitProviderTimeStep = 0.1;  // s.

std::vector<StBoundaryWithDecision> InitializeStBoundaryWithDecision(
    std::vector<StBoundaryRef> raw_st_boundaries);

void KeepNearestStationarySpacetimeTrajectoryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_DECISION_UTIL_H_
