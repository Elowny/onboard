#include "onboard/planner/util/scene_util.h"

#include <limits.h>

#include <algorithm>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/global/logging.h"
#include "onboard/maps/compatibility_layer.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/utils/source_location.h"

namespace qcraft {
namespace planner {

bool IsInHighWay(const mapping::v2::SemanticMapManager& smm,
                 const RouteSectionsInfo& sections_info,
                 const double preview_distance) {
  bool in_high_way = false;
  if (sections_info.empty()) {
    return in_high_way;
  }
  double accum_dist = 0.0;
  for (int i = 0, n = sections_info.size(); i < n; ++i) {
    const auto& section_segment = sections_info.section_segment(i);
    SMM_SECTION_PROTO_OR_BREAK(section_ptr, smm, section_segment.id);
    const auto road_class = section_ptr->road_class();
    in_high_way = in_high_way ||
                  road_class == mapping::SectionProto_RoadClass_CITY_EXPRESS ||
                  road_class == mapping::SectionProto_RoadClass_HIGHWAY;
    accum_dist += section_segment.length();
    if (in_high_way) break;
    if (accum_dist >= preview_distance) break;
  }

  return in_high_way;
}

bool IsRightMostDrivableLane(const PlannerSemanticMapManager& psmm,
                             mapping::ElementId lane_id) {
  SMM_ASSIGN_LANE_OR_RETURN(lane_info, psmm, lane_id, false);
  if (lane_info.lane_neighbors_on_right.empty()) {
    return true;
  }
  SMM_ASSIGN_LANE_OR_RETURN(right_lane_info, psmm,
                            lane_info.lane_neighbors_on_right.front().other_id,
                            false);
  return mapping::compat::IsPassengerVehicleAvoidLaneType(
      right_lane_info.Type());
}

int CountSectionDrivableLanesFromRight(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo::RouteSectionSegmentInfo& section_segment,
    double preview_dist) {
  int num_drivable_lanes = 0;
  for (auto lane_id_pt = section_segment.lane_ids.rbegin();
       lane_id_pt != section_segment.lane_ids.rend(); ++lane_id_pt) {
    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm, *lane_id_pt);
    if (mapping::compat::IsPassengerVehicleAvoidLaneType(lane_info.Type())) {
      continue;
    }
    ++num_drivable_lanes;
    // Ignore left lane with solid boundary.
    const mapping::LanePoint check_lane_point(*lane_id_pt,
                                              section_segment.start_fraction);
    const auto crossing_type_or =
        QueryLanePointBoundaryType(psmm, check_lane_point, /*left=*/true);
    if (crossing_type_or.ok() && *crossing_type_or != std::nullopt &&
        IsBoundarySolid(**crossing_type_or, mapping::BoundarySide::kRight)) {
      break;
    }
    const double preview_fraction =
        std::min(section_segment.end_fraction,
                 preview_dist / std::max(0.01, section_segment.average_length) +
                     section_segment.start_fraction);
    const mapping::LanePoint preview_lane_point(*lane_id_pt, preview_fraction);
    const auto preview_crossing_type_or =
        QueryLanePointBoundaryType(psmm, preview_lane_point, /*left=*/true);
    if (preview_crossing_type_or.ok() &&
        *preview_crossing_type_or != std::nullopt &&
        IsBoundarySolid(**preview_crossing_type_or,
                        mapping::BoundarySide::kRight)) {
      break;
    }
  }
  return num_drivable_lanes;
}

int PreviewMinDrivableLanes(const PlannerSemanticMapManager& psmm,
                            const RouteSectionsInfo& sections_info,
                            double preview_dist, int start_index) {
  int min_drivable_lane = INT_MAX;
  for (int i = start_index; i < sections_info.size(); ++i) {
    min_drivable_lane =
        std::min(min_drivable_lane,
                 CountSectionDrivableLanesFromRight(
                     psmm, sections_info.section_segment(i), preview_dist));
    preview_dist -= sections_info.section_segment(i).length();
    if (preview_dist < 0.0) break;
  }
  return min_drivable_lane;
}

}  // namespace planner
}  // namespace qcraft
