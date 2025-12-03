#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <stddef.h>

#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner {

class RouteSections {
 public:
  struct RouteSectionSegment {
    mapping::SectionId id;
    double start_fraction;
    double end_fraction;
  };

  RouteSections() {}
  explicit RouteSections(double start_fraction, double end_fraction,
                         std::vector<mapping::SectionId> section_ids,
                         mapping::LanePoint destination)
      : start_fraction_(start_fraction),
        end_fraction_(end_fraction),
        section_ids_(std::move(section_ids)),
        destination_(destination) {}

  static RouteSections BuildFromLanePath(const PlannerSemanticMapManager& psmm,
                                         const mapping::LanePath& lane_path);

  static RouteSections BuildFromProto(const RouteSectionSequenceProto& proto);

  bool empty() const { return section_ids_.empty(); }
  int size() const { return section_ids_.size(); }
  int section_id_size() const { return section_ids_.size(); }

  bool operator==(const RouteSections& other) const {
    constexpr double kEpsilon = 1E-9;
    if (std::fabs(start_fraction_ - other.start_fraction_) > kEpsilon)
      return false;
    if (std::fabs(end_fraction_ - other.end_fraction_) > kEpsilon) return false;
    if (size() != other.size()) return false;
    if (destination_ != other.destination()) return false;

    for (size_t i = 0; i < section_ids_.size(); ++i) {
      if (section_ids_[i] != other.section_id(i)) return false;
    }
    return true;
  }
  bool operator!=(const RouteSections& other) const {
    return !(*this == other);
  }

  double start_fraction() const { return start_fraction_; }
  double end_fraction() const { return end_fraction_; }

  absl::Span<const mapping::SectionId> section_ids() const {
    return section_ids_;
  }
  mapping::SectionId section_id(size_t index) const {
    QCHECK_LT(index, section_ids_.size());
    return section_ids_[index];
  }

  RouteSectionSegment route_section_segment(int index) const;
  RouteSectionSegment front() const {
    QCHECK_GE(section_ids_.size(), 1) << "Route sections empty!";
    return route_section_segment(0);
  }
  RouteSectionSegment back() const {
    QCHECK_GE(section_ids_.size(), 1) << "Route sections empty!";
    return route_section_segment(section_ids_.size() - 1);
  }

  mapping::LanePoint destination() const { return destination_; }

  // Returns section index if ok
  absl::StatusOr<int> FindSectionSegment(
      const RouteSectionSegment& segment) const;

  void Clear() { *this = RouteSections(); }
  std::string DebugString() const {
    return absl::StrCat("section ids:", absl::StrJoin(section_ids_, ","),
                        ", start fraction:", start_fraction_,
                        ", end_fraction:", end_fraction_);
  }

  double planning_horizon(const PlannerSemanticMapManager& psmm) const;
  double planning_horizon(const mapping::v2::SemanticMapManager& v2smm) const;

  void ToProto(RouteSectionSequenceProto* proto) const;

 private:
  double start_fraction_;
  double end_fraction_;
  std::vector<mapping::SectionId> section_ids_;

  mapping::LanePoint destination_;

  mutable std::optional<double> planning_horizon_ = std::nullopt;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_H_
