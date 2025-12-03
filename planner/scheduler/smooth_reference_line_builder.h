#ifndef ONBOARD_PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_BUILDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"

namespace qcraft::planner {

absl::StatusOr<SmoothedReferenceCenterResult> SmoothLanePathByLaneIds(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width);

absl::StatusOr<SmoothedReferenceLineResultMap>
BuildSmoothedResultMapFromRouteSections(const PlannerSemanticMapManager& psmm,
                                        const RouteSections& route_sections,
                                        double half_av_width,
                                        SmoothedReferenceLineResultMap results);

absl::StatusOr<std::vector<std::vector<mapping::ElementId>>>
FindLanesToSmoothFromRoute(const PlannerSemanticMapManager& psmm,
                           const RouteSections& route_sections);

// TODO(zixuan): Move it to anonymous namespace.
absl::StatusOr<SmoothedReferenceCenterResult>
SmoothLanePathBoundedByPathBoundary(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const PathSlBoundary& boundary,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_BUILDER_H_
