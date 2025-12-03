#include "onboard/planner/router/noa/hd_hmi_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/maps/ehr/proto/odd.pb.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/planner/router/noa/hdmap_util.h"
#include "onboard/planner/router/router_defs.h"
#include "onboard/utils/source_location.h"
namespace qcraft::planner::route::noa {

namespace {

struct FindLaneTypeResult {
  mapping::SectionId entry_section_id;
  mapping::ElementId entry_lane_id;
  double dist = 0.0;
  int section_index = -1;
  bool found = false;
  // TODO(xiang) shoule add via_cross_sections info.
};

FindLaneTypeResult FindLaneAlongSections(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& route_sections, double horizon_at_most,
    const std::function<bool(const mapping::SectionProto&,
                             const mapping::LaneProto&)>& pred) {
  const auto& sections = route_sections.section_id();
  const double start_fraction = route_sections.start_fraction();
  const double end_fraction = route_sections.end_fraction();

  FindLaneTypeResult find_result;
  double len = 0.0;
  for (int i = 0; i < sections.size(); i++) {
    const auto& section_id = sections[i];
    SMM_SECTION_PROTO_OR_CONTINUE(section_proto, smm, section_id);
    for (auto lane_id : section_proto->lanes()) {
      SMM_LANE_PROTO_OR_CONTINUE(lane_proto, smm, lane_id);
      if (pred(*section_proto, *lane_proto)) {
        find_result.found = true;
        find_result.entry_section_id = mapping::SectionId(section_id);
        find_result.entry_lane_id = mapping::ElementId(lane_id);
        find_result.dist = len;
        find_result.section_index = i;
        return find_result;
      }
    }

    if (len > horizon_at_most || find_result.found) {
      break;
    }
    if (i == 0) {
      len += section_proto->average_length() * (1.0 - start_fraction);
    } else if (i == sections.size() - 1) {
      len += section_proto->average_length() * end_fraction;
    } else {
      len += section_proto->average_length();
    }
  }
  return find_result;
}

bool CheckDistToEndCondition(int dist_to_sd_end, int dist_to_map_boundary) {
  return dist_to_sd_end < odc::kHdDistHorizon &&
         dist_to_map_boundary > odc::kHdMinBoundary;
}

bool IsLaneEnteryRamp(const mapping::LaneProto& lane_proto) {
  return lane_proto.type() == mapping::LaneProto::EXIT ||
         lane_proto.type() == mapping::LaneProto::RAMP_ENTRANCE;
}

inline double ClampFraction(int index, int min_index, int max_index,
                            double start_fraction, double end_fraction) {
  if (min_index == max_index) {
    return end_fraction - start_fraction;
  }
  return std::clamp(index == min_index
                        ? (1.0 - start_fraction)
                        : (index == max_index ? end_fraction : 1.0),
                    0.0, 1.0);
}
struct ExitSectionIndexAndDist {
  int section_index = -1;
  double dist_to_exit = 0.0;
  mapping::SectionId exit_section_id = mapping::kInvalidSectionId;
};

absl::StatusOr<ExitSectionIndexAndDist> CalcFirstExitSectionIndexAndDist(
    const mapping::v2::SemanticMapManager& smm,
    const int ramp_jct_section_index, mapping::ElementId ramp_jct_lane_id,
    const double dist_to_ramp_jct, int sections_size, double start_fraction,
    double end_fraction) {
  SMM_LANE_PROTO_OR_ERROR(lane_proto, smm, ramp_jct_lane_id);
  auto* cur_proto_ptr = lane_proto;
  bool look_backwards = true;
  int section_index = ramp_jct_section_index;
  double dist_to_exit = dist_to_ramp_jct;
  while (look_backwards && section_index >= 0) {
    look_backwards = false;
    for (auto ic_lane_id : cur_proto_ptr->incoming_lanes()) {
      SMM_LANE_PROTO_OR_CONTINUE(ic_lane_proto, smm, ic_lane_id);
      if (IsLaneEnteryRamp(*ic_lane_proto)) {
        cur_proto_ptr = ic_lane_proto;
        look_backwards = true;
        double fraction = ClampFraction(section_index, 0, sections_size - 1,
                                        start_fraction, end_fraction);
        SMM_SECTION_PROTO_OR_CONTINUE(section, smm,
                                      cur_proto_ptr->section_id());
        dist_to_exit -= section->average_length() * fraction;
        --section_index;
        break;  // break for
      }
    }
  }
  return ExitSectionIndexAndDist{
      .section_index = section_index,
      .dist_to_exit = dist_to_exit,
      .exit_section_id = mapping::SectionId(cur_proto_ptr->section_id()),
  };
}

}  // namespace

// Now only calc the ramp action, To remove if sdk-HMI provided
void UpdateNavInstruction(const mapping::v2::SemanticMapManager& smm,
                          const RouteSectionSequenceProto& route_sections,
                          int horizon, NaviInstructionProto* nav_instruction) {
  const auto& sections = route_sections.section_id();
  const double start_fraction = route_sections.start_fraction();
  const double end_fraction = route_sections.end_fraction();
  if (sections.size() == 0) {
    return;
  }
  if (sections.size() == 1) {
    SMM_SECTION_PROTO_OR_RETURN(section_proto, smm,
                                mapping::SectionId(sections[0]), void());
    double diff_faction = std::clamp(end_fraction - start_fraction, 0.0, 1.0);
    nav_instruction->set_distance(diff_faction *
                                  section_proto->average_length());
    nav_instruction->mutable_navi_action()->set_navi_action_type(
        NaviActionProto::DESTINATION);
    nav_instruction->set_is_last_navi_action(true);
    return;
  }

  const auto find_result = FindLaneAlongSections(
      smm, route_sections, horizon,
      [](const auto& /*section_proto*/, const auto& lane_proto) {
        return lane_proto.type() == mapping::LaneProto::RAMP_JCT;
      });

  if (find_result.found && find_result.dist < horizon) {
    // Find the first EXIT, if parallel EXIT lanes exist here, select one by
    // random.
    const auto exit_section_index_and_dist = CalcFirstExitSectionIndexAndDist(
        smm, find_result.section_index, find_result.entry_lane_id,
        find_result.dist, sections.size(), start_fraction, end_fraction);
    if (!exit_section_index_and_dist.ok()) {
      return;
    }
    auto* nav_action = nav_instruction->mutable_navi_action();
    constexpr double kDistEpisilon = 1E-6;
    if (exit_section_index_and_dist->dist_to_exit > kDistEpisilon) {
      nav_instruction->set_distance(
          exit_section_index_and_dist->section_index >= 0
              ? exit_section_index_and_dist->dist_to_exit
              : 0.0);
      nav_action->set_sub_action_type(NaviActionProto::ENTER_RAMP);
      nav_action->set_section_id(
          exit_section_index_and_dist->exit_section_id.value());
    } else {
      // The AV is on ramp now, TODO(xiang): Need a different nav action.
      nav_action->Clear();
    }
  } else if (nav_instruction->navi_action().sub_action_type() ==
             NaviActionProto::ENTER_RAMP) {
    nav_instruction->mutable_navi_action()->set_sub_action_type(
        NaviActionProto::SUB_ACTION_NONE);
  }
}

std::optional<int> CaclDistanceToEnd(
    const mapping::v2::SemanticMapSpatialIndex& smm_index,
    const RouteSectionSequenceProto& route_sections,
    const Vec2d& sd_destination, int dist_to_sd_end, int dist_to_map_boundary) {
  if (!CheckDistToEndCondition(dist_to_sd_end, dist_to_map_boundary)) {
    return std::nullopt;
  }
  auto pt_lanes =
      PointToNearLanes(smm_index, sd_destination, kSearchLaneRadius);
  if (pt_lanes.empty()) {
    return std::nullopt;
  }
  const auto& smm = *smm_index.semantic_manager();
  const auto& sections = route_sections.section_id();
  const double start_fraction = route_sections.start_fraction();
  const double end_fraction = route_sections.end_fraction();
  // GroupByAndFilter
  absl::flat_hash_set<int64_t> dest_section_ids;
  std::vector<PointToLaneT<mapping::v2::Lane>> filtered_pt_lanes;
  for (const auto& pt_lane : pt_lanes) {
    auto [_, inserted] =
        dest_section_ids.insert(pt_lane.lane_info->proto().section_id());
    if (inserted) {
      filtered_pt_lanes.push_back(pt_lane);
    }
  }
  // Only assume that the section ids in the mpp are unique.
  std::vector<int64_t> joined_section_ids;
  for (const auto section_id : sections) {
    if (dest_section_ids.contains(section_id)) {
      joined_section_ids.push_back(section_id);
    }
  }
  if (joined_section_ids.empty()) {
    return std::nullopt;
  }
  const auto section_id_of_dest =
      filtered_pt_lanes[0].lane_info->proto().section_id();
  const auto match_frac = filtered_pt_lanes[0].fraction;
  double length = 0.0;
  for (int i = 0; i < static_cast<int>(sections.size()); ++i) {
    double start_frac = i == 0 ? start_fraction : 0.0;
    double end_frac = i == sections.size() - 1 ? end_fraction : 1.0;
    SMM_ASSIGN_SECTION_OR_RETURN(sec_info, smm, sections[i], length);
    if (sections[i] == section_id_of_dest) {
      start_frac = std::max(start_frac, match_frac);
      end_frac = std::min(end_frac, match_frac);
    }
    length += sec_info.proto().average_length() *
              std::max(0.0, end_frac - start_frac);
    if (sections[i] == section_id_of_dest) {
      break;
    }
  }
  if (std::abs(length - dist_to_sd_end) > odc::kSdHdDistDiffMaxError) {
    return std::nullopt;
  }
  return std::max(0, static_cast<int>(length));
}

void UpdateHmiOddEvent(const mapping::v2::SemanticMapManager& smm,
                       const RouteSectionSequenceProto& route_sections,
                       int horizon, NcaOdcProto* nca_odc_proto) {
  if (route_sections.section_id().empty()) {
    return;
  }
  if (nca_odc_proto == nullptr) {
    return;
  }
  const auto& sections = route_sections.section_id();
  double start_fraction = route_sections.start_fraction();
  double end_fraction = route_sections.end_fraction();

  nca_odc_proto->mutable_odd_custom_events()->Clear();
  nca_odc_proto->mutable_odd_traffic_events()->Clear();

  double length = 0.0;
  for (int i = 0; i < sections.size(); i++) {
    const auto start_frac = (i == 0) ? start_fraction : 0.0;
    const auto end_frac = (i == sections.size() - 1) ? end_fraction : 1.0;
    SMM_SECTION_PROTO_OR_CONTINUE(section_proto, smm, sections[i]);
    const auto& section = *section_proto;
    if (section.has_odd()) {
      auto transform_events_fn = [&section, length, start_frac](
                                     const auto& events, auto* hmi_events) {
        for (const auto& event : events) {
          const auto event_start_fraction =
              event.has_start_fraction() ? event.start_fraction() : 0.0;
          const auto dist_to_start =
              length +
              section.average_length() * (event_start_fraction - start_frac);
          VLOG(3) << "length: " << length
                  << ", dist_to_start: " << dist_to_start
                  << ", section: " << section.id()
                  << ", section_length: " << section.average_length()
                  << ", event: " << event.type();

          if (dist_to_start > 0) {
            auto* hmi_event = hmi_events->Add();
            hmi_event->set_distance(dist_to_start);
            hmi_event->set_type(event.type());
          }
        }
      };
      nca_odc_proto->mutable_odd_custom_events()->Reserve(
          section.odd().custom_events_size());
      transform_events_fn(section.odd().custom_events(),
                          nca_odc_proto->mutable_odd_custom_events());

      nca_odc_proto->mutable_odd_traffic_events()->Reserve(
          section.odd().traffic_events_size());
      transform_events_fn(section.odd().traffic_events(),
                          nca_odc_proto->mutable_odd_traffic_events());
    }
    length += section.average_length() * (end_frac - start_frac);
    if (length > horizon) {
      break;
    }
  }
}

}  // namespace qcraft::planner::route::noa
