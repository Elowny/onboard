#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FILTER_H_

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"

#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

class SpacetimePlannerObjectTrajectoriesFilter {
 public:
  virtual bool Filter(const SpacetimeObjectTrajectory& traj) const = 0;
  virtual ~SpacetimePlannerObjectTrajectoriesFilter() = default;
};

// Filter cut-in trajs on highway for st planner.
class CutInVehicleSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  CutInVehicleSpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage* drive_passage, const PathSlBoundary* path_sl_boundary,
      const Box2d& av_box, double av_speed);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;       // Not owned.
  const PathSlBoundary* path_sl_boundary_;  // Not owned.
  Box2d av_box_;
  std::optional<FrenetBox> av_sl_box_;
  double av_speed_;
};

// Filter Crossing trajs for st planner.
class CrossingSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 private:
  using CrossingFilterParamsProto =
      SpacetimePlannerObjectTrajectoriesParamsProto::CrossingFilterParamsProto;

 public:
  CrossingSpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage* drive_passage,
      const CrossingFilterParamsProto* crossing_filter_params);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;        // Not owned.
  const CrossingFilterParamsProto* params_;  // Not owned.
};

// Filter vehicle trajs direction reverse to current lane for st planner.
class ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  explicit ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage* drive_passage,
      const PathSlBoundary* path_sl_boundary);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;       // Not owned.
  const PathSlBoundary* path_sl_boundary_;  // Not owned.
};

// Filter vehicles occluded.
class OccludedVehicleSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  OccludedVehicleSpacetimePlannerObjectTrajectoriesFilter();
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;
};

// Filter trajs beyond stop line for st planner.
class BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage* drive_passage,
      absl::Span<const ConstraintProto::StopLineProto> stop_lines);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;  // Not owned.
  double first_stop_line_s_ = std::numeric_limits<double>::infinity();
};

class BehaviorConflictSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  BehaviorConflictSpacetimePlannerObjectTrajectoriesFilter(
      const DrivePassage& drive_passage, const PathSlBoundary& sl_boundary,
      absl::Span<const PlannerObject> planner_objects, const Box2d& av_box,
      double av_speed);

  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  absl::flat_hash_map<std::string, std::vector<int>> object_cutin_traj_set_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FILTER_H_
