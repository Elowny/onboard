#ifndef ONBOARD_PLANNER_SCHEDULER_STRAIGHTEN_REFERENCE_LINE_H_
#define ONBOARD_PLANNER_SCHEDULER_STRAIGHTEN_REFERENCE_LINE_H_

#include "absl/status/statusor.h"

#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"

namespace qcraft::planner {

absl::StatusOr<SmoothedReferenceLineResultMap>
BuildStraightenedRampResultMapFromRouteSections(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    double half_av_width, SmoothedReferenceLineResultMap results);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_STRAIGHTEN_REFERENCE_LINE_H_
