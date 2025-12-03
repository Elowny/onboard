#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FINDER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FINDER_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"

#include "onboard/maps/lane_path.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

class SpacetimePlannerObjectTrajectoriesFinder {
 public:
  virtual SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const = 0;
  virtual ~SpacetimePlannerObjectTrajectoriesFinder() = default;
};

// Find all trajs for st planner.
class AllSpacetimePlannerObjectTrajectoriesFinder
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  AllSpacetimePlannerObjectTrajectoriesFinder() = default;
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& /*traj*/) const override {  // NOLINT
    return SpacetimePlannerObjectTrajectoryReason::ALL;
  };
};

// Find stationary trajs for st planner.
class StationarySpacetimePlannerObjectTrajectoriesFinder
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  StationarySpacetimePlannerObjectTrajectoriesFinder(
      const PlannerSemanticMapManager* psmm,
      const mapping::LanePath& lane_path);
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

  // BANDAID(runbing): This is a hack to identify a gate boom barrier. The box
  // of boom area.
  std::optional<Box2d> bool_barrier_box_or_;
};

// Find side trajs (trajs that move along the direction of sdc) for st planner.
class FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  static constexpr double kComfortableNudgeCheckTime = 5.0;          // s.
  static constexpr double kComfortableNudgeLatSpeedCheckTime = 2.0;  // s.
  explicit FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder(
      const Box2d& av_box, const DrivePassage* drive_passage,
      const PathSlBoundary* sl_boundary, double av_speed,
      const SpacetimePlannerObjectTrajectoriesProto* prev_st_trajs,
      const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
      const VehicleGeometryParamsProto* veh_geo);
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

 private:
  Box2d av_box_;
  const DrivePassage* drive_passage_;          // Not owned.
  const PathSlBoundary* path_sl_boundary_;     // Not owned.
  const VehicleGeometryParamsProto* veh_geo_;  // Not owned.
  double av_speed_;
  std::optional<FrenetBox> av_sl_box_;
  absl::flat_hash_set<std::string> prev_st_planner_obj_id_;

  // Is time_aligned_prev_traj inside path sl boundary from time 0 to
  // kComfortableNudgeCheckTime.
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj_;
  mutable std::optional<bool> is_prev_traj_effective_;
  mutable std::vector<Box2d> time_aligned_prev_traj_boxes_;
};

// Find Object drive in.
class DrivingInSpacetimePlannerObjectTrajectoriesFinder
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  explicit DrivingInSpacetimePlannerObjectTrajectoriesFinder(
      const Box2d& av_box, double av_speed,
      const LaneChangeStateProto* lane_change_state,
      const PlannerSemanticMapManager* psmm, const DrivePassage* drive_passage);
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;  // Not owned.
  std::optional<FrenetBox> av_sl_box_;
  double av_speed_;

  std::optional<double> s_merge_range_min_;
  std::optional<double> s_merge_range_max_;
};

// TODO(renjie): Delete this finder.
// Find very close side obj for st planner.
class DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  explicit DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder(
      const Box2d& av_box, const DrivePassage* drive_passage,
      double av_velocity);
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

 private:
  Box2d av_box_;
  const DrivePassage* drive_passage_;  // Not owned.
  Vec2d av_tangent_;
  double av_velocity_;
  std::optional<FrenetBox> av_sl_box_;
};

// Find all trajs (in front of sdc) for st planner.
// Use with caution, not fully tested yet.
class FrontMovingSpacetimePlannerObjectTrajectoriesFinder
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  explicit FrontMovingSpacetimePlannerObjectTrajectoriesFinder(
      const DrivePassage* drive_passage,
      const ApolloTrajectoryPointProto* plan_start_point, double av_length);
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;                   // Not owned.
  const ApolloTrajectoryPointProto* plan_start_point_;  // Not owned.
  FrenetCoordinate av_sl_;
  double av_length_;  // AV length.
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FINDER_H_
