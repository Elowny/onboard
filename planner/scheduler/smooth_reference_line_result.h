#ifndef ONBOARD_PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_RESULT_H_
#define ONBOARD_PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_RESULT_H_

#include <map>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

struct SmoothedReferenceCenterResult {
  // PLF is from lane fraction to lateral offset.
  absl::flat_hash_map<qcraft::mapping::ElementId,
                      PiecewiseLinearFunction<double, double>>
      lane_id_to_smoothed_lateral_offset;

  absl::StatusOr<double> GetSmoothedLateralOffset(
      const mapping::LanePoint& lane_point) const {
    const PiecewiseLinearFunction<double, double>* plf =
        FindOrNull(lane_id_to_smoothed_lateral_offset, lane_point.lane_id());
    if (plf == nullptr) {
      return absl::NotFoundError("");
    } else {
      return (*plf)(lane_point.fraction());
    }
  }
};

using SmoothedResultMap =
    std::map<std::vector<mapping::ElementId>, SmoothedReferenceCenterResult>;

class SmoothedReferenceLineResultMap {
 public:
  SmoothedReferenceLineResultMap() = default;

  bool Contains(const std::vector<mapping::ElementId>& lane_ids) const {
    return FindOverlapSmoothedLaneIds(lane_ids).ok();
  }

  void AddResult(std::vector<mapping::ElementId> lane_ids,
                 SmoothedReferenceCenterResult smoothed_results) {
    if (smoothed_result_map_.find(lane_ids) == smoothed_result_map_.end()) {
      smoothed_result_map_.emplace(std::move(lane_ids),
                                   std::move(smoothed_results));
    } else {
      smoothed_result_map_[lane_ids] = std::move(smoothed_results);
    }
  }

  void DeleteResult(const std::vector<mapping::ElementId>& lane_ids) {
    smoothed_result_map_.erase(lane_ids);
  }

  void Clear() { smoothed_result_map_.clear(); }

  const SmoothedResultMap& smoothed_result_map() const {
    return smoothed_result_map_;
  }

  absl::StatusOr<std::vector<mapping::ElementId>> FindOverlapSmoothedLaneIds(
      const std::vector<mapping::ElementId>& lane_ids) const;

  absl::StatusOr<SmoothedReferenceCenterResult> FindOverlapSmoothedResult(
      const std::vector<mapping::ElementId>& lane_ids) const {
    ASSIGN_OR_RETURN(const auto overlap_lane_ids,
                     FindOverlapSmoothedLaneIds(lane_ids));
    return smoothed_result_map_.at(overlap_lane_ids);
  }

 private:
  SmoothedResultMap smoothed_result_map_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_RESULT_H_
