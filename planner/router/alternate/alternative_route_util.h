#ifndef ONBOARD_PLANNER_ROUTER_ALTERNATE_ALTERNATIVE_ROUTE_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_ALTERNATE_ALTERNATIVE_ROUTE_UTIL_H_

#include "absl/container/flat_hash_map.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {

absl::flat_hash_map<mapping::SectionId, double> GenerateDecreasingCostSeq(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& sections, double look_ahead_dist,
    double cost_per_meter);

bool IsAlternateRouteDiffPrimary(
    const RouteSectionSequenceProto& primary_sections,
    const RouteSectionSequenceProto& alternate_sections);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ALTERNATE_ALTERNATIVE_ROUTE_UTIL_H_
