#ifndef ONBOARD_PLANNER_PLAN_ACC_TARGET_UTIL_H_
#define ONBOARD_PLANNER_PLAN_ACC_TARGET_UTIL_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"

#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_defs.h"
#include "onboard/planner/plan/acc/acc_target.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace internal {

inline bool IsAccTargetType(ObjectType type) {
  switch (type) {
    case ObjectType::OT_VEHICLE:
    case ObjectType::OT_LARGE_VEHICLE:
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_CYCLIST:
    case ObjectType::OT_TRICYCLIST:
    case ObjectType::OT_PEDESTRIAN:
    case ObjectType::OT_CONE:
    case ObjectType::OT_UNKNOWN_MOVABLE:
    case ObjectType::OT_UNKNOWN_STATIC:
    case ObjectType::OT_WARNING_TRIANGLE:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_BUCKET:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_POST:
      return true;
    case ObjectType::OT_FOD:
    case ObjectType::OT_VEGETATION:
      return false;
  }
}

SpacetimeObjectTrajectory AlignSpacetimeObjectTrajectoryToCorridor(
    const SpacetimeObjectTrajectory& st_traj, const AccCorridor& corridor);

inline bool ObjectInFrontOfAvFrontCenterByLength(double object_length,
                                                 const FrenetBox& object_fbox,
                                                 ObjectType object_type,
                                                 double av_front_s) {
  constexpr double kVehicleAheadLengthFactor = 0.5;
  constexpr double kLargeVehicleAheadLengthFactor = 0.33;
  const double length_factor = object_type == ObjectType::OT_LARGE_VEHICLE
                                   ? kLargeVehicleAheadLengthFactor
                                   : kVehicleAheadLengthFactor;
  return (object_fbox.s_max - av_front_s) > (object_length * length_factor);
}

}  // namespace internal

struct AccTargetInfo {
  absl::string_view traj_id;
  const SpacetimeObjectTrajectory* st_traj = nullptr;
  FrenetCoordinate f_coord;
  FrenetBox fbox;
  double heading_diff;
  OnPathType on_path_type;
  Vec2d v_decomp;
  double overlap_obj = -std::numeric_limits<double>::infinity();
  double overlap_lane = -std::numeric_limits<double>::infinity();

  std::string DebugString() const {
    return absl::StrFormat(
        "Potential ACC target: %s, type: %s, f_coord (%f, %f), s_min: %f, "
        "s_max: %f, "
        "heading_diff: %f degree, "
        "on_path_type: %d, v_decomp: %s, overlap_obj: %.3f, overlap_lane: %.3f",
        traj_id, ObjectType_Name(st_traj->object_type()), f_coord.s, f_coord.l,
        fbox.s_min, fbox.s_max, NormalizeAngle(heading_diff) / M_PI * 180.0,
        on_path_type, v_decomp.DebugString(), overlap_obj, overlap_lane);
  }
};

// Only check fbox lateral distance.
OnPathType ObjectFrenetBoxOnPathType(const PathSlBoundary& path_sl,
                                     const FrenetBox& fbox,
                                     double lateral_distance_buffer);

std::unique_ptr<SpacetimeTrajectoryManager>
BuildSpacetimeTrajectoryManagerWithAlignmentAndRemoveLateralGap(
    const absl::flat_hash_set<std::string>& targets_need_alignment,
    absl::Span<const AccTargetInfo> potential_targets,
    const AccCorridor& corridor,
    const SpacetimeTrajectoryManager& planner_st_traj_mgr);

template <typename FrenetFrameType>
absl::StatusOr<double> ComputeBoxLaneInvasionPercentage(
    const PathSlBoundary& path_sl, const FrenetFrameType& path_ff,
    const Box2d& box, bool use_obj_width) {
  ASSIGN_OR_RETURN(
      const FrenetBox fbox,
      path_ff.QueryLongitudinallyBoundedFrenetBoxAtContour(Polygon2d(box)));
  const bool from_right = std::fabs(fbox.l_min) > std::fabs(fbox.l_max);
  const double center_s = 0.5 * (fbox.s_min + fbox.s_max);
  // first: right_target_l, second: left_target_l
  const auto target_bound_l = path_sl.QueryTargetBoundaryL(center_s);
  const double target_bound_width =
      target_bound_l.second - target_bound_l.first;
  constexpr double kMaxObjWidth = 3.4;
  double max_invasion = -std::numeric_limits<double>::infinity();
  for (const auto& pt : box.GetCornersCounterClockwise()) {
    const auto pt_sl = path_ff.XYToSL(pt);
    // First: right, second: left.
    const auto target_bound_l_at_p = path_sl.QueryTargetBoundaryL(pt_sl.s);
    const double ref_width =
        use_obj_width ? std::max(std::min(box.width(), kMaxObjWidth),
                                 from_right ? -target_bound_l_at_p.first
                                            : target_bound_l_at_p.second)
                      : target_bound_width;
    const double invasion_l_range = from_right
                                        ? pt_sl.l - target_bound_l_at_p.first
                                        : target_bound_l_at_p.second - pt_sl.l;
    max_invasion = std::max(max_invasion, invasion_l_range / ref_width);
  }
  return std::min(max_invasion, 1.0);
}

// Return leaders and potential targets.
std::pair<std::vector<AccTargetDecision>, std::vector<AccTargetDecision>>
BuildAccTargetDecisions(absl::Span<const std::string> leader_traj_ids,
                        const SpacetimeTrajectoryManager& st_traj_mgr);

std::vector<AccTargetInfo> FindPotentialTargets(
    const SpacetimeTrajectoryManager& planner_st_traj_mgr,
    const AccCorridor& corridor, double av_front_s,
    bool enable_filter_oncoming_objects);

// Separate leaders and potential targets.
std::vector<std::string> MakeLeaderDecisions(
    absl::Span<const AccTargetInfo> potential_targets,
    double overlap_threshold);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ACC_TARGET_UTIL_H_
