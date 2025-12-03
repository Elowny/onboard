#include "onboard/planner/plan/acc/acc_target_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_target.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

namespace internal {

SpacetimeObjectTrajectory AlignSpacetimeObjectTrajectoryToCorridor(
    const SpacetimeObjectTrajectory& st_traj, const AccCorridor& corridor) {
  SpacetimeObjectTrajectory aligned_traj(st_traj);  // Deep copy.
  const auto& ff = corridor.frenet_frame;

  // Use first state contour, box and perception box to rotate and shift.
  const auto& init_contour = st_traj.contour();
  const auto& init_contour_circle_center = init_contour.CircleCenter();
  const auto& init_box = st_traj.bounding_box();

  const auto init_pos_sl = ff.XYToSL(init_box.center());
  for (auto& mutable_state : aligned_traj.mutable_states()) {
    // Shift state pos onto corridor path.
    // Rotate state box, contours to align with path tangent.
    auto pos_sl = ff.XYToSL(mutable_state.box.center());
    pos_sl.l = init_pos_sl.l;
    // TODO(changqing): Maybe need to truncate states according to heading
    // difference.
    const auto pos_on_path = ff.SLToXY(pos_sl);
    const auto heading_vec_on_path = ff.InterpolateTangentByS(pos_sl.s);
    const auto rot_rad =
        NormalizeAngle2D(Vec2d(init_box.cos_heading(), init_box.sin_heading()),
                         heading_vec_on_path);
    const auto translation =
        pos_on_path - init_box.center();  // Translation vector.
    double rot_rad_cos_sin[2];
    fast_math::CosAndSin<7>(rot_rad, rot_rad_cos_sin);
    mutable_state.contour =
        init_contour.Transform(init_contour_circle_center, rot_rad_cos_sin[0],
                               rot_rad_cos_sin[1], translation);
    mutable_state.box = init_box.Transform(translation, rot_rad);
  }

  return aligned_traj;
}

}  // namespace internal

OnPathType ObjectFrenetBoxOnPathType(const PathSlBoundary& path_sl,
                                     const FrenetBox& fbox,
                                     double lateral_distance_buffer) {
  QCHECK_GE(lateral_distance_buffer, 0.0);
  // pair.first: right, pair.second: left.
  const auto center_sl = fbox.center();
  const auto bound_l = path_sl.QueryBoundaryL(center_sl.s);
  const auto target_bound_l = path_sl.QueryTargetBoundaryL(center_sl.s);
  if (fbox.l_min <= target_bound_l.second &&
      fbox.l_max >= target_bound_l.first) {
    return OnPathType::OPT_TARGET_BOUND;
  }

  const auto l_left_buffer = bound_l.second + lateral_distance_buffer;
  const auto l_right_buffer = bound_l.first - lateral_distance_buffer;

  if (fbox.l_min > l_left_buffer || fbox.l_max < l_right_buffer) {
    return OnPathType::OPT_OFF_BOUND;
  }

  if (lateral_distance_buffer > 0.0) {
    if ((fbox.l_min <= l_left_buffer && fbox.l_min > bound_l.second) ||
        (fbox.l_max >= l_right_buffer && fbox.l_max < bound_l.first)) {
      return OnPathType::OPT_NEAR_BOUND;
    }
  }
  return OnPathType::OPT_BOUND;
}

std::unique_ptr<SpacetimeTrajectoryManager>
BuildSpacetimeTrajectoryManagerWithAlignmentAndRemoveLateralGap(
    const absl::flat_hash_set<std::string>& targets_need_alignment,
    absl::Span<const AccTargetInfo> potential_targets,
    const AccCorridor& corridor,
    const SpacetimeTrajectoryManager& planner_st_traj_mgr) {
  std::vector<SpacetimeObjectTrajectory> all_trajs;
  all_trajs.reserve(potential_targets.size());

  for (const auto& potential_target : potential_targets) {
    const auto& traj_id = potential_target.traj_id;
    const auto* traj = planner_st_traj_mgr.FindTrajectoryById(traj_id);
    if (traj == nullptr) continue;  // Should never happen.
    auto new_traj = *traj;
    // ACC do not use lateral buffer for trajectory.
    new_traj.set_required_lateral_gap(/*lateral_gap=*/0.0);
    if (targets_need_alignment.contains(traj_id)) {
      all_trajs.push_back(internal::AlignSpacetimeObjectTrajectoryToCorridor(
          new_traj, corridor));
    } else {
      all_trajs.push_back(std::move(new_traj));
    }
  }
  auto acc_st_traj_mgr =
      std::make_unique<SpacetimeTrajectoryManager>(absl::MakeSpan(all_trajs));
  return acc_st_traj_mgr;
}

std::pair<std::vector<AccTargetDecision>, std::vector<AccTargetDecision>>
BuildAccTargetDecisions(absl::Span<const std::string> leader_traj_ids,
                        const SpacetimeTrajectoryManager& st_traj_mgr) {
  std::pair<std::vector<AccTargetDecision>, std::vector<AccTargetDecision>> res;
  auto& leaders = res.first;
  auto& potential_targets = res.second;
  leaders.reserve(leader_traj_ids.size());
  for (const auto& traj_id : leader_traj_ids) {
    leaders.push_back(AccTargetDecision{
        .object_id =
            SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(traj_id),
        .traj_id = traj_id});
  }

  potential_targets.reserve(st_traj_mgr.trajectories().size());
  for (const auto& traj : st_traj_mgr.trajectories()) {
    potential_targets.push_back(AccTargetDecision{
        .object_id = SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
            traj.traj_id()),
        .traj_id = std::string(traj.traj_id())});
  }
  return res;
}

// Filter targets by relative s & credible length & outer boundary & object
// type.
std::vector<AccTargetInfo> FindPotentialTargets(
    const SpacetimeTrajectoryManager& planner_st_traj_mgr,
    const AccCorridor& corridor, double av_front_s,
    bool /*enable_filter_oncoming_objects*/) {
  const auto& path_ff = corridor.frenet_frame;
  const auto& path_sl = corridor.boundary;
  const auto& path = corridor.path;
  const double credible_length = corridor.credible_length;

  std::vector<AccTargetInfo> potential_targets;
  for (const auto& st_traj : planner_st_traj_mgr.trajectories()) {
    if (!internal::IsAccTargetType(st_traj.object_type())) {
      continue;
    }
    const auto& pose = st_traj.pose();
    const double heading = pose.theta();
    const auto h_vec = Vec2d::FastUnitFromAngle(heading);
    const auto object_v = pose.v() * h_vec;
    // Check object frenet box against path sl bound to determine on path.
    const auto object_fbox_or =
        path_ff.QueryFrenetBoxAtContour(st_traj.contour());
    if (!object_fbox_or.ok()) {
      continue;
    }
    if (!internal::ObjectInFrontOfAvFrontCenterByLength(
            st_traj.bounding_box().length(), *object_fbox_or,
            st_traj.object_type(), av_front_s) ||
        object_fbox_or->s_min > credible_length) {
      // Ignore objects which are behind of av or front of credible corridor.
      continue;
    }
    const auto on_path_type =
        ObjectFrenetBoxOnPathType(path_sl, *object_fbox_or,
                                  /*lateral_distance_buffer=*/0.0);
    if (on_path_type == OnPathType::OPT_OFF_BOUND) {
      continue;
    }

    const auto fbox_center = object_fbox_or->center();
    const auto proj_path_point = path.Evaluate(fbox_center.s);
    const auto path_unit_vec =
        Vec2d::FastUnitFromAngle(proj_path_point.theta());
    const auto heading_diff = NormalizeAngle(heading - proj_path_point.theta());
    const auto overlap_obj = ComputeBoxLaneInvasionPercentage(
        path_sl, path_ff, st_traj.bounding_box(), /*use_obj_width=*/true);
    const auto overlap_lane = ComputeBoxLaneInvasionPercentage(
        path_sl, path_ff, st_traj.bounding_box(), /*use_obj_width=*/false);

    // Filter moving oncoming targets with insufficient overlap
    constexpr double kAccTargetMaxHeadingDiff = 3.0 * M_PI_4;
    constexpr double kIgnoreOncomingOverlapPercentage = 0.3;
    constexpr double kIsMovingSpeedThreshold = 1.0;
    const bool is_overlap_insufficient =
        !overlap_obj.ok() ||
        (overlap_obj.value() < kIgnoreOncomingOverlapPercentage);
    if (std::fabs(heading_diff) > kAccTargetMaxHeadingDiff &&
        is_overlap_insufficient && pose.v() > kIsMovingSpeedThreshold) {
      continue;
    }

    potential_targets.emplace_back(AccTargetInfo{
        .traj_id = st_traj.traj_id(),
        .st_traj = &st_traj,
        .f_coord = fbox_center,
        .fbox = *object_fbox_or,
        .heading_diff = heading_diff,
        .on_path_type = on_path_type,
        .v_decomp = Vec2d(path_unit_vec.Dot(object_v),
                          path_unit_vec.CrossProd(object_v)),
        .overlap_obj =
            overlap_obj.value_or(-std::numeric_limits<double>::infinity()),
        .overlap_lane =
            overlap_lane.value_or(-std::numeric_limits<double>::infinity()),
    });
  }
  std::sort(potential_targets.begin(), potential_targets.end(),
            [](const AccTargetInfo& lhs, const AccTargetInfo& rhs) {
              return lhs.fbox.s_min < rhs.fbox.s_min;
            });
  return potential_targets;
}

std::vector<std::string> MakeLeaderDecisions(
    absl::Span<const AccTargetInfo> potential_targets,
    double overlap_threshold) {
  std::vector<std::string> leader_traj_ids;
  for (const auto& potential_target : potential_targets) {
    if (potential_target.overlap_obj >= overlap_threshold) {
      // Collect leaders id.
      leader_traj_ids.push_back(std::string(potential_target.traj_id));
    }
  }
  return leader_traj_ids;
}

}  // namespace qcraft::planner
