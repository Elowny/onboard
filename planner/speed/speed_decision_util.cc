#include "onboard/planner/speed/speed_decision_util.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary.h"

namespace qcraft::planner {
std::vector<StBoundaryWithDecision> InitializeStBoundaryWithDecision(
    std::vector<StBoundaryRef> raw_st_boundaries) {
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.reserve(raw_st_boundaries.size());
  for (auto& st_boundary : raw_st_boundaries) {
    // Initialize with unknown-decision.
    st_boundaries_with_decision.emplace_back(std::move(st_boundary));
  }
  return st_boundaries_with_decision;
}

void KeepNearestStationarySpacetimeTrajectoryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  double nearest_stationary_s = std::numeric_limits<double>::max();
  const std::string* nearest_stationary_id_ptr = nullptr;
  constexpr double kEpsilon = 0.01;
  for (const auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    const auto& st_boundary = *st_boundary_with_decision.raw_st_boundary();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (!st_boundary.is_stationary()) continue;
    if (st_boundary.min_s() < kEpsilon) continue;
    if (const double follow_s =
            st_boundary.min_s() -
            st_boundary_with_decision.follow_standstill_distance();
        follow_s < nearest_stationary_s) {
      nearest_stationary_s = follow_s;
      nearest_stationary_id_ptr = &st_boundary.id();
    }
  }
  if (nearest_stationary_id_ptr != nullptr) {
    st_boundaries_with_decision->erase(
        std::remove_if(
            st_boundaries_with_decision->begin(),
            st_boundaries_with_decision->end(),
            [nearest_stationary_id_ptr](
                const StBoundaryWithDecision& st_boundary_with_decision) {
              const auto& st_boundary =
                  *st_boundary_with_decision.raw_st_boundary();
              if (st_boundary.source_type() !=
                  StBoundarySourceTypeProto::ST_OBJECT) {
                return false;
              }
              if (!st_boundary.is_stationary()) return false;
              if (st_boundary.min_s() < kEpsilon) return false;
              return st_boundary.id() != *nearest_stationary_id_ptr;
            }),
        st_boundaries_with_decision->end());
  }
}
}  // namespace qcraft::planner
