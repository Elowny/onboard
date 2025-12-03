#ifndef ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_UTIL_H_

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/route_sections_info.h"

namespace qcraft {
namespace planner {
struct MergeLaneInfo {
  double len_before_merge_lane = 0.0;
  absl::flat_hash_set<mapping::ElementId> merge_targets;
};

absl::flat_hash_map<mapping::ElementId, double> CalculateMaxDrivingDistance(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    bool from_lane_beginning);

absl::flat_hash_map<mapping::ElementId, int> FindLcNumToTargets(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_length, int start_section_idx);

absl::flat_hash_map<mapping::ElementId, MergeLaneInfo>
CalculateLengthBeforeMergeLane(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_length);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_UTIL_H_
