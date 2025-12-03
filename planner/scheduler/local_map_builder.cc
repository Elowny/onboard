#include "onboard/planner/scheduler/local_map_builder.h"

#include <algorithm>
#include <vector>

#include "absl/status/status.h"

#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_util.h"

namespace qcraft::planner {

namespace {

// The max length of lane path is consistent with the effective range of
// perception temporarily.
constexpr double kDefaultLanePathLength = 150.0;

}  // namespace

absl::StatusOr<std::vector<mapping::LanePath>> BuildLocalMap(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const RouteNaviInfo& route_navi_info) {
  const RouteSectionsInfo route_sections_info(psmm, &route_sections);
  if (route_sections_info.empty() ||
      route_sections_info.front().lane_ids.empty()) {
    return absl::NotFoundError("Route section is empty.");
  }

  std::vector<mapping::LanePath> lane_paths;
  const auto& current_section_info = route_sections_info.front();
  lane_paths.reserve(current_section_info.lane_ids.size());

  // Build lane paths start from left most lane to right most lane in current
  // route section.
  for (const auto lane_id : current_section_info.lane_ids) {
    lane_paths.emplace_back(FindLanePathFromLaneAlongRouteSections(
        psmm, route_sections_info, route_navi_info, lane_id,
        route_sections_info.start_fraction(), kDefaultLanePathLength));
  }
  return lane_paths;
}

}  // namespace qcraft::planner
