#include "onboard/planner/router/alternate/alternative_route_util.h"

#include <algorithm>

#include "absl/hash/hash.h"

#include "onboard/global/logging.h"
#include "onboard/maps/map_or_die_macros.h"

namespace qcraft::planner::route {

absl::flat_hash_map<mapping::SectionId, double> GenerateDecreasingCostSeq(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& sections, double look_ahead_dist,
    double cost_per_meter) {
  absl::flat_hash_map<mapping::SectionId, double> modify_sections;

  double accum_length = 0.0;

  for (int i = 0; i < sections.section_id_size(); ++i) {
    SMM_SECTION_PROTO_OR_CONTINUE(cur_sec_proto, smm, sections.section_id()[i]);
    if (cur_sec_proto->lanes().empty()) {
      continue;
    }
    SMM_LANE_PROTO_OR_CONTINUE(first_lane_proto, smm,
                               cur_sec_proto->lanes()[0]);

    if (first_lane_proto->is_in_intersection()) {
      const double extra_cost =
          std::max(0.0, look_ahead_dist - accum_length) * cost_per_meter;
      modify_sections[mapping::SectionId(sections.section_id()[i])] =
          extra_cost;
    }

    const double start_frac = i == 0 ? sections.start_fraction() : 0.0;
    const double end_frac =
        i + 1 == sections.section_id_size() ? sections.end_fraction() : 1.0;

    accum_length +=
        cur_sec_proto->average_length() * std::max(0.0, end_frac - start_frac);
    if (accum_length >= look_ahead_dist) break;
  }

  return modify_sections;
}

bool IsAlternateRouteDiffPrimary(
    const RouteSectionSequenceProto& primary_sections,
    const RouteSectionSequenceProto& alternate_sections) {
  if (primary_sections.section_id_size() !=
      alternate_sections.section_id_size()) {
    return true;
  }
  for (int i = 0; i < primary_sections.section_id_size(); ++i) {
    if (primary_sections.section_id(i) != alternate_sections.section_id(i)) {
      return true;
    }
  }
  return false;
}

}  // namespace qcraft::planner::route
