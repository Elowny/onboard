#ifndef ONBOARD_PLANNER_UTIL_SCENE_UTIL_H_
#define ONBOARD_PLANNER_UTIL_SCENE_UTIL_H_

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections_info.h"

namespace qcraft {
namespace planner {

bool IsInHighWay(const mapping::v2::SemanticMapManager& smm,
                 const RouteSectionsInfo& sections_info,
                 double preview_distance);

bool IsRightMostDrivableLane(const PlannerSemanticMapManager& psmm,
                             mapping::ElementId lane_id);

int CountSectionDrivableLanesFromRight(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo::RouteSectionSegmentInfo& section_segment,
    double preview_dist);

int PreviewMinDrivableLanes(const PlannerSemanticMapManager& psmm,
                            const RouteSectionsInfo& sections_info,
                            double preview_dist, int start_index);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_UTIL_SCENE_UTIL_H_
