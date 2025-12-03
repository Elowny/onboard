#ifndef ONBOARD_PLANNER_DECISION_CONSTRAINT_MANAGER_H_
#define ONBOARD_PLANNER_DECISION_CONSTRAINT_MANAGER_H_

#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/types/span.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_gap_finder.h"
#include "onboard/planner/decision/traffic_gap_v2/proto/traffic_gap.pb.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner {

class ConstraintManager {
 public:
  ConstraintManager() = default;

  void AddSpeedRegion(ConstraintProto::SpeedRegionProto speed_region) {
    const auto it = std::lower_bound(
        speed_region_.begin(), speed_region_.end(), speed_region,
        [](const ConstraintProto::SpeedRegionProto& elem,
           const ConstraintProto::SpeedRegionProto& val) {
          return elem.start_s() < val.start_s();
        });
    speed_region_.insert(it, std::move(speed_region));
  }

  void AddStopLine(ConstraintProto::StopLineProto stop_line) {
    const auto it =
        std::lower_bound(stop_line_.begin(), stop_line_.end(), stop_line,
                         [](const ConstraintProto::StopLineProto& elem,
                            const ConstraintProto::StopLineProto& val) {
                           return elem.s() < val.s();
                         });
    stop_line_.insert(it, std::move(stop_line));
  }

  void AddPathSpeedRegion(
      ConstraintProto::PathSpeedRegionProto path_speed_region) {
    const auto it = std::lower_bound(
        path_speed_region_.begin(), path_speed_region_.end(), path_speed_region,
        [](const ConstraintProto::PathSpeedRegionProto& elem,
           const ConstraintProto::PathSpeedRegionProto& val) {
          return elem.start_s() < val.start_s();
        });
    path_speed_region_.insert(it, std::move(path_speed_region));
  }

  void AddPathStopLine(ConstraintProto::PathStopLineProto path_stop_line) {
    const auto it = std::lower_bound(
        path_stop_line_.begin(), path_stop_line_.end(), path_stop_line,
        [](const ConstraintProto::PathStopLineProto& elem,
           const ConstraintProto::PathStopLineProto& val) {
          return elem.s() < val.s();
        });
    path_stop_line_.insert(it, std::move(path_stop_line));
  }

  void AddAvoidLine(ConstraintProto::AvoidLineProto avoid_line) {
    avoid_line_.push_back(std::move(avoid_line));
  }

  void AddSpeedProfile(ConstraintProto::SpeedProfileProto speed_profile) {
    speed_profiles_.emplace_back(std::move(speed_profile));
  }

  void SetTrafficGap(TrafficGapResult traffic_gap) {
    traffic_gap_ = std::move(traffic_gap);
  }
  void SetTrafficGapDebug(TrafficGapDebugProto traffic_gap_debug) {
    traffic_gap_debug_ = std::move(traffic_gap_debug);
  }

  absl::Span<const ConstraintProto::SpeedRegionProto> SpeedRegion() const {
    return speed_region_;
  }

  absl::Span<const ConstraintProto::StopLineProto> StopLine() const {
    return stop_line_;
  }

  absl::Span<const ConstraintProto::AvoidLineProto> AvoidLine() const {
    return avoid_line_;
  }

  const TrafficGapResult& TrafficGap() const { return traffic_gap_; }
  const TrafficGapDebugProto& TrafficGapDebug() const {
    return traffic_gap_debug_;
  }

  // Same as above, but use s on path instead of on drive passage.
  absl::Span<const ConstraintProto::PathSpeedRegionProto> PathSpeedRegion()
      const {
    return path_speed_region_;
  }
  absl::Span<const ConstraintProto::PathStopLineProto> PathStopLine() const {
    return path_stop_line_;
  }

  absl::Span<const ConstraintProto::SpeedProfileProto> SpeedProfiles() const {
    return speed_profiles_;
  }

 private:
  std::vector<ConstraintProto::SpeedRegionProto> speed_region_;
  std::vector<ConstraintProto::StopLineProto> stop_line_;
  std::vector<ConstraintProto::PathStopLineProto> path_stop_line_;
  std::vector<ConstraintProto::PathSpeedRegionProto> path_speed_region_;
  std::vector<ConstraintProto::AvoidLineProto> avoid_line_;
  std::vector<ConstraintProto::SpeedProfileProto> speed_profiles_;

  TrafficGapResult traffic_gap_;
  TrafficGapDebugProto traffic_gap_debug_;
};
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_DECISION_CONSTRAINT_MANAGER_H_
