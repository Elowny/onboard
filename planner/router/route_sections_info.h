#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_INFO_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_INFO_H_

#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"

namespace qcraft::planner {

class RouteSectionsInfo {
 public:
  explicit RouteSectionsInfo(const PlannerSemanticMapManager& psmm,
                             const RouteSections* sections);

  explicit RouteSectionsInfo(const mapping::v2::SemanticMapManager& smm,
                             const RouteSections* sections);
  struct RouteSectionSegmentInfo {
    mapping::SectionId id;
    double start_fraction;
    double end_fraction;
    double average_length;
    std::vector<mapping::ElementId> lane_ids;

    absl::flat_hash_map<mapping::ElementId, int> id_idx_map;

    double length() const {
      return average_length * (end_fraction - start_fraction);
    }
    bool contains(const mapping::LanePoint& lane_pt) const {
      constexpr double kEpsilon = 1E-9;
      return id_idx_map.contains(lane_pt.lane_id()) &&
             start_fraction <= lane_pt.fraction() + kEpsilon &&
             lane_pt.fraction() <= end_fraction + kEpsilon;
    }
  };

  bool empty() const { return section_segments_.empty(); }
  int size() const { return section_segments_.size(); }
  mapping::LanePoint destination() const {
    return route_sections_->destination();
  }
  const RouteSectionSegmentInfo& front() const {
    return section_segments_.front();
  }
  const RouteSectionSegmentInfo& back() const {
    return section_segments_.back();
  }
  double start_fraction() const { return front().start_fraction; }
  double end_fraction() const { return back().end_fraction; }
  double planning_horizon() const { return planning_horizon_; }

  double length_between(int start_idx, int end_idx) const {
    CHECK_GE(start_idx, 0);
    CHECK_LE(end_idx, section_segments_.size());
    double len = 0.0;
    for (int i = start_idx; i < end_idx; ++i) {
      len += section_segments_[i].length();
    }
    return len;
  }
  double length() const { return length_between(0, section_segments_.size()); }

  const RouteSections* route_sections() const { return route_sections_; }

  bool IsValid() const { return length() > 0.0; }

  absl::Span<const RouteSectionSegmentInfo> section_segments() const {
    return section_segments_;
  }
  const RouteSectionSegmentInfo& section_segment(int index) const {
    return section_segments_[index];
  }

  const RouteSectionSegmentInfo* FindSegmentContainingLanePointOrNull(
      const mapping::LanePoint& lane_pt) const {
    if (lane_pt.lane_id() == mapping::kInvalidElementId) return nullptr;

    for (const auto& section : section_segments_) {
      if (section.id_idx_map.contains(lane_pt.lane_id())) {
        return section.start_fraction <= lane_pt.fraction() &&
                       lane_pt.fraction() <= section.end_fraction
                   ? &section
                   : nullptr;
      }
    }
    return nullptr;
  }

 public:
  explicit RouteSectionsInfo(const RouteSections* sections,
                             std::vector<RouteSectionSegmentInfo> section_segs,
                             double planning_horizon)
      : route_sections_(QCHECK_NOTNULL(sections)),
        section_segments_(std::move(section_segs)),
        planning_horizon_(planning_horizon) {}

 private:
  const RouteSections* route_sections_;
  std::vector<RouteSectionSegmentInfo> section_segments_;
  double planning_horizon_ = 0.0;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_INFO_H_
