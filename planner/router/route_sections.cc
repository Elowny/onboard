#include "onboard/planner/router/route_sections.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <memory>

#include "absl/status/status.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner {

RouteSections RouteSections::BuildFromLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  std::vector<mapping::SectionId> section_ids;
  section_ids.reserve(lane_path.size());

  for (const auto& seg : lane_path) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, seg.lane_id);
    section_ids.push_back(lane_info.section_id);
  }

  return RouteSections(lane_path.front().fraction(),
                       lane_path.back().fraction(), std::move(section_ids),
                       lane_path.back());
}

RouteSections RouteSections::BuildFromProto(
    const RouteSectionSequenceProto& proto) {
  std::vector<mapping::SectionId> section_ids;

  section_ids.reserve(proto.section_id_size());

  for (const auto& section_id : proto.section_id()) {
    section_ids.push_back(mapping::SectionId(section_id));
  }
  const mapping::LanePoint dest =
      proto.has_destination()
          ? mapping::LanePoint(proto.destination())
          : mapping::LanePoint(mapping::kInvalidElementId, 0.0);
  return RouteSections(proto.start_fraction(), proto.end_fraction(),
                       std::move(section_ids), dest);
}

RouteSections::RouteSectionSegment RouteSections::route_section_segment(
    int index) const {
  QCHECK_GE(index, 0);
  QCHECK_LT(index, size());

  return RouteSectionSegment{
      .id = section_ids_[index],
      .start_fraction = index == 0 ? start_fraction_ : 0.0,
      .end_fraction = index + 1 == size() ? end_fraction_ : 1.0};
}

absl::StatusOr<int> RouteSections::FindSectionSegment(
    const RouteSectionSegment& segment) const {
  for (int i = 0; i < static_cast<int>(section_ids_.size()); ++i) {
    if (section_ids_[i] == segment.id) {
      const auto this_seg = route_section_segment(i);

      if (segment.start_fraction >= this_seg.start_fraction &&
          segment.end_fraction <= this_seg.end_fraction) {
        return i;
      } else {
        return absl::OutOfRangeError("");
      }
    }
  }
  return absl::NotFoundError("");
}

double RouteSections::planning_horizon(
    const PlannerSemanticMapManager& psmm) const {
  if (!planning_horizon_.has_value()) {
    // Calculate only once.
    double horizon = 0.0;
    double accum_time = 0.0, last_speed_limit = 0.0;
    bool stopped_early = false;
    size_t idx = 0;
    for (; idx < section_ids_.size(); ++idx) {
      SMM_ASSIGN_SECTION_OR_BREAK(sec_info, psmm, section_ids_[idx]);
      if (const auto* first_lane_ptr =
              psmm.FindLaneInfoOrNull(sec_info.lane_ids[0]);
          first_lane_ptr == nullptr) {
        break;
      }

      const double start_frac = (idx == 0 ? start_fraction_ : 0.0);
      const double end_frac =
          (idx + 1 == section_ids_.size() ? end_fraction_ : 1.0);
      const double seg_len = sec_info.average_length * (end_frac - start_frac);
      const auto& lane_ids = sec_info.lane_ids;

      double max_speed_limit = 0.0;
      for (const auto lane_id : lane_ids) {
        max_speed_limit =
            std::max(max_speed_limit, psmm.QueryMaxLaneSpeedLimitById(lane_id));
      }
      last_speed_limit = max_speed_limit;
      const double time_inc = seg_len / (max_speed_limit + 1e-9);
      if (accum_time + time_inc > kPlanningTimeHorizon) {
        horizon += (kPlanningTimeHorizon - accum_time) * max_speed_limit;
        stopped_early = true;
        break;
      }
      accum_time += time_inc;
      horizon += seg_len;
    }
    if (!stopped_early) {
      horizon += (kPlanningTimeHorizon - accum_time) * last_speed_limit;
    }
    planning_horizon_ = horizon;
  }
  return *planning_horizon_;
}

double RouteSections::planning_horizon(
    const mapping::v2::SemanticMapManager& v2smm) const {
  if (!planning_horizon_.has_value()) {
    // Calculate only once.
    constexpr double kDefaultVel = 40.0;  // kph
    double horizon = 0.0;
    double accum_time = 0.0, last_speed_limit = 0.0;
    bool stopped_early = false;
    size_t idx = 0;
    for (; idx < section_ids_.size(); ++idx) {
      SMM2_ASSIGN_SECTION_OR_BREAK(sec_info, v2smm, section_ids_[idx]);
      if (sec_info.proto().lanes().empty()) {
        break;
      }
      if (const auto first_lane_ptr =
              v2smm.FindLane(mapping::ElementId(sec_info.proto().lanes(0)));
          first_lane_ptr == nullptr) {
        break;
      }

      const double start_frac = (idx == 0 ? start_fraction_ : 0.0);
      const double end_frac =
          (idx + 1 == section_ids_.size() ? end_fraction_ : 1.0);
      const double seg_len =
          sec_info.proto().average_length() * (end_frac - start_frac);
      const auto& lane_ids = sec_info.proto().lanes();

      double max_speed_limit = 0.0;
      for (const auto lane_id : lane_ids) {
        SMM2_ASSIGN_LANE_OR_CONTINUE(lane, v2smm, mapping::ElementId(lane_id));
        // TODO(zuowei): move to lane_speed_limit.h
        double cur_speed_limit = lane.proto().has_speed_limit_kph()
                                     ? lane.proto().speed_limit_kph()
                                     : kDefaultVel;
        if (!lane.proto().speed_limits_kph().empty()) {
          const auto iter = std::max_element(
              lane.proto().speed_limits_kph().begin(),
              lane.proto().speed_limits_kph().end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.speed_limit_kph() < rhs.speed_limit_kph();
              });
          cur_speed_limit = iter->speed_limit_kph();
        }
        max_speed_limit = std::max(max_speed_limit, Kph2Mps(cur_speed_limit));
      }
      last_speed_limit = max_speed_limit;
      const double time_inc = seg_len / max_speed_limit;
      if (accum_time + time_inc > kPlanningTimeHorizon) {
        horizon += (kPlanningTimeHorizon - accum_time) * max_speed_limit;
        stopped_early = true;
        break;
      }
      accum_time += time_inc;
      horizon += seg_len;
    }
    if (!stopped_early) {
      horizon += (kPlanningTimeHorizon - accum_time) * last_speed_limit;
    }
    planning_horizon_ = horizon;
  }
  return *planning_horizon_;
}

void RouteSections::ToProto(RouteSectionSequenceProto* proto) const {
  proto->Clear();

  proto->set_start_fraction(start_fraction_);

  proto->set_end_fraction(end_fraction_);

  proto->mutable_section_id()->Reserve(section_ids_.size());

  for (const auto& id : section_ids_) {
    proto->add_section_id(id.value());
  }

  destination_.ToProto(proto->mutable_destination());
}

}  // namespace qcraft::planner
