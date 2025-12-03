#ifndef ONBOARD_PLANNER_ROUTER_ROAD_CONDITIONS_PROCESS_
#define ONBOARD_PLANNER_ROUTER_ROAD_CONDITIONS_PROCESS_

// IWYU pragma: no_include <algorithm>
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <stdint.h>

#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/compatibility_layer.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/map_internal.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/road_constraints.h"
#include "onboard/proto/route.pb.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner::route {

struct RouteRestrictDistrict {
  // Try not to drive on the lanes.
  absl::flat_hash_set<mapping::ElementId> avoid_lanes;
  // Forbidden, must avoid.
  absl::flat_hash_set<mapping::SectionId> restrict_sections;

  // For alternate route.
  absl::flat_hash_map<mapping::SectionId, double> extra_sections_cost;

  void Clear() {
    avoid_lanes.clear();
    restrict_sections.clear();
    extra_sections_cost.clear();
  }
};

absl::flat_hash_set<mapping::SectionId> FindRestrictSectionsFromRestrictRegions(
    const mapping::v2::SemanticMapManager& smm,
    const google::protobuf::RepeatedPtrField<mapping::GeoPolygonProto>&
        regions);

absl::StatusOr<RouteRestrictDistrict> ParseRestrictRoadsFromRoutingRequest(
    const mapping::v2::SemanticMapManager& smm,
    const RoutingRequestProto& routing_request);

absl::flat_hash_set<mapping::SectionId> ParseRestrictProtoToSectionId(
    const mapping::v2::SemanticMapManager& smm,
    const RestrictProto& restrict_proto);

bool IsFallInIntervalsOrTrueIfEmpty(
    absl::Time time,
    const google::protobuf::RepeatedPtrField<mapping::Interval>& intervals,
    absl::TimeZone tz = absl::LocalTimeZone());

/// @brief  Change the blacklist with bus lane
/// @return the change list.
template <typename Container>
std::vector<mapping::ElementId> MayUpdateAvoidLanesByRestrict(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const Container& section_ids, absl::Time begin, absl::Time end,
    absl::flat_hash_set<mapping::ElementId>* blacklist) {
  std::vector<mapping::ElementId> added_lanes;
  for (const auto& section_id : section_ids) {
    SMM_SECTION_PROTO_OR_CONTINUE(section_proto, semantic_map_manager,
                                  section_id);
    for (const auto& lane_id : section_proto->lanes()) {
      SMM_LANE_PROTO_OR_CONTINUE(lane_proto, semantic_map_manager, lane_id);
      const auto& restrict_range = lane_proto->bus_only_day_intervals();
      mapping::ElementId lane_element_id(lane_id);
      if (lane_proto->type() == mapping::LaneProto::BUS_ONLY) {
        if (IsFallInIntervalsOrTrueIfEmpty(begin, restrict_range) ||
            IsFallInIntervalsOrTrueIfEmpty(end, restrict_range)) {
          auto added_pair = blacklist->insert(lane_element_id);
          if (added_pair.second) {
            added_lanes.push_back(lane_element_id);
          }
        } else {
          // Now can on bus-lane
          const auto remove_count = blacklist->erase(lane_element_id);
          if (remove_count > 0) {
            added_lanes.push_back(lane_element_id);
          }
        }
      }

      if (mapping::compat::IsPassengerVehicleAvoidLaneType(
              lane_proto->type())) {
        const auto [_, inserted] = blacklist->insert(lane_element_id);
        if (inserted) {
          added_lanes.push_back(lane_element_id);
        }
      }
    }
  }
  return added_lanes;
}

void UpdateAvoidLanes(
    const mapping::v2::SemanticMapManager& smm,
    const google::protobuf::RepeatedField<int64_t>& section_ids,
    google::protobuf::RepeatedField<int64_t>* avoid_lanes);

// BANDAID(zuowei): A hack to process AvoidLaneSegmentMap.
google::protobuf::RepeatedField<int64_t> UpdateDynamicOddAvoidLanes(
    const mapping::v2::SemanticMapManager& v2smm,
    const AvoidLaneSegmentMap& lane_segment_map,
    const RouteSectionSequenceProto& sections_proto,
    const google::protobuf::RepeatedField<int64_t>& avoid_lanes);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ROAD_CONDITIONS_PROCESS_
