#include "onboard/planner/router/road_conditions_process.h"

#include <algorithm>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_join.h"

#include "onboard/global/clock.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/maps/v2/semantic_map_definition.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/geometry/polygon2d.h"

namespace qcraft::planner::route {
namespace {
constexpr static auto kLookFutureRestrictDuration = absl::Minutes(5);

using AvoidLanes = absl::flat_hash_set<mapping::ElementId>;
using RestrictSections = absl::flat_hash_set<mapping::SectionId>;

}  // namespace

absl::StatusOr<RouteRestrictDistrict> ParseRestrictRoadsFromRoutingRequest(
    const mapping::v2::SemanticMapManager& smm,
    const RoutingRequestProto& routing_request) {
  AvoidLanes avoid_lanes;
  if (!routing_request.avoid_lanes().empty()) {
    for (const auto avoid_lane : routing_request.avoid_lanes()) {
      avoid_lanes.insert(mapping::ElementId(avoid_lane));
    }
  }

  RestrictSections restrict_sections;

  if (!routing_request.avoid_regions().empty()) {
    const auto sections_in_avoid_regions =
        FindRestrictSectionsFromRestrictRegions(
            smm, routing_request.avoid_regions());
    restrict_sections.insert(sections_in_avoid_regions.begin(),
                             sections_in_avoid_regions.end());
  }

  if (routing_request.has_forbidden()) {
    const auto forbidden_sections =
        ParseRestrictProtoToSectionId(smm, routing_request.forbidden());
    restrict_sections.insert(forbidden_sections.begin(),
                             forbidden_sections.end());
  }

  return RouteRestrictDistrict{
      .avoid_lanes = std::move(avoid_lanes),
      .restrict_sections = std::move(restrict_sections)};
}

// TODO(zuowei): Improve efficiency.
absl::flat_hash_set<mapping::SectionId> FindRestrictSectionsFromRestrictRegions(
    const mapping::v2::SemanticMapManager& smm,
    const google::protobuf::RepeatedPtrField<mapping::GeoPolygonProto>&
        regions) {
  if (regions.empty()) return {};
  RestrictSections restrict_sections_in_regions;

  std::vector<Polygon2d> polygons;
  polygons.reserve(regions.size());
  for (const auto& avoid_region : regions) {
    polygons.push_back(
        Polygon2d(mapping::ConverterGeoPoints(avoid_region.points())));
  }
  for (const auto& lane : smm.semantic_map().lanes) {
    for (const auto& polygon : polygons) {
      bool lane_in_polygon = true;
      for (const auto& global_point : lane->Proto().polyline().points()) {
        if (!polygon.IsPointIn(
                {global_point.longitude(), global_point.latitude()})) {
          lane_in_polygon = false;
          break;
        }
      }
      if (lane_in_polygon &&
          !restrict_sections_in_regions.contains(
              mapping::SectionId(lane->Proto().section_id()))) {
        restrict_sections_in_regions.insert(
            mapping::SectionId(lane->Proto().section_id()));
      }
    }
  }
  return restrict_sections_in_regions;
}

absl::flat_hash_set<mapping::SectionId> ParseRestrictProtoToSectionId(
    const mapping::v2::SemanticMapManager& smm,
    const RestrictProto& restrict_proto) {
  absl::flat_hash_set<mapping::SectionId> restrict_sections;
  if (!restrict_proto.sections().empty()) {
    for (const auto section_id : restrict_proto.sections()) {
      restrict_sections.insert(mapping::SectionId(section_id));
    }
  }

  if (!restrict_proto.regions().empty()) {
    const auto sections_in_restrict_regions =
        FindRestrictSectionsFromRestrictRegions(smm, restrict_proto.regions());
    restrict_sections.insert(sections_in_restrict_regions.begin(),
                             sections_in_restrict_regions.end());
  }
  return restrict_sections;
}

bool IsFallInIntervalsOrTrueIfEmpty(
    absl::Time time,
    const google::protobuf::RepeatedPtrField<mapping::Interval>& intervals,
    absl::TimeZone tz) {
  return intervals.empty() ||
         mapping::SemanticMapManager::IsFallInIntervals(time, intervals, tz);
}

void UpdateAvoidLanes(
    const mapping::v2::SemanticMapManager& smm,
    const google::protobuf::RepeatedField<int64_t>& section_ids,
    google::protobuf::RepeatedField<int64_t>* avoid_lanes) {
  absl::Time now = Clock::Now();
  absl::Time now_plus_5min = now + kLookFutureRestrictDuration;
  absl::flat_hash_set<mapping::ElementId> blacklist;
  blacklist.insert(avoid_lanes->begin(), avoid_lanes->end());
  const auto changed_lane_ids = route::MayUpdateAvoidLanesByRestrict(
      smm, section_ids, now, now_plus_5min, &blacklist);
  if (!changed_lane_ids.empty()) {
    QLOG_EVERY_N_SEC(INFO, 3)
        << " Changed avoid lanes:" << absl::StrJoin(changed_lane_ids, ",");
    avoid_lanes->Clear();
    if (!blacklist.empty()) {
      avoid_lanes->Reserve(blacklist.size());
      for (const auto lane_id : blacklist) {
        avoid_lanes->Add(lane_id.value());
      }
    }
  }
}

google::protobuf::RepeatedField<int64_t> UpdateDynamicOddAvoidLanes(
    const mapping::v2::SemanticMapManager& v2smm,
    const AvoidLaneSegmentMap& lane_segment_map,
    const RouteSectionSequenceProto& sections_proto,
    const google::protobuf::RepeatedField<int64_t>& avoid_lanes) {
  if (lane_segment_map.IsEmpty()) return avoid_lanes;

  const auto& section_ids = sections_proto.section_id();
  if (section_ids.empty()) return avoid_lanes;
  const int64_t front_section_id = section_ids[0];
  absl::flat_hash_set<int64_t> sections_set{section_ids.begin(),
                                            section_ids.end()};

  absl::flat_hash_set<int64_t> avoid_lanes_set{avoid_lanes.begin(),
                                               avoid_lanes.end()};

  for (const auto& [lane_id, lane_segment] :
       lane_segment_map.lane_segments_map) {
    const auto lane_ptr = v2smm.FindLane(lane_id);
    if (lane_ptr == nullptr) {
      if (avoid_lanes_set.contains(lane_id.value())) {
        avoid_lanes_set.erase(lane_id.value());
      }
      continue;
    }
    const auto lane_section_id = lane_ptr->proto().section_id();
    if (sections_set.contains(lane_section_id)) {
      if (lane_section_id == front_section_id) {
        if (lane_segment.end_fraction <= sections_proto.start_fraction() &&
            avoid_lanes_set.contains(lane_id.value())) {
          avoid_lanes_set.erase(lane_id.value());
        } else if (lane_segment.end_fraction >
                   sections_proto.start_fraction()) {
          avoid_lanes_set.insert(lane_id.value());
        }
      } else {
        avoid_lanes_set.insert(lane_id.value());
      }
    }
  }

  google::protobuf::RepeatedField<int64_t> updated_avoid_lanes;
  updated_avoid_lanes.Reserve(avoid_lanes_set.size());
  std::for_each(avoid_lanes_set.begin(), avoid_lanes_set.end(),
                [&updated_avoid_lanes](const int64_t avoid_lane) {
                  updated_avoid_lanes.Add(avoid_lane);
                });
  return updated_avoid_lanes;
}

}  // namespace qcraft::planner::route
