#include "onboard/planner/decision/traffic_gap_v2/traffic_gap_finder_impl.h"

#include <float.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"

#include "onboard/math/fast_math.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/lane_change_safety_params.h"
#include "onboard/planner/common/lane_change_safety_util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

constexpr double kLonFollowBuffer = 3.0;     // m.
constexpr double kAccelDecelAbsValue = 1.0;  // m/s^2.
constexpr double kInvAccelDecelAbsValue = 1.0 / kAccelDecelAbsValue;
constexpr double kAlignSpeedDiff = 3.0;  // m/s.
constexpr double kFinalSpeedDiff = 1.0;  // m/s.

struct ObjectProjectionInfo {
  const PlannerObject* object;
  FrenetBox frenet_box;
};

inline bool HasEnteredLane(double l_max, double l_min) {
  constexpr double kLateralThreshold = 1.0;  // m.
  return l_max > -kLateralThreshold && l_min < kLateralThreshold;
}

inline double ComputeCatchDist(double v_start, double v_end, double v_ref,
                               bool gap_behind) {
  const double rel_s =
      std::abs(Sqr(v_end) - Sqr(v_start)) * 0.5 * kInvAccelDecelAbsValue -
      std::abs(v_end - v_start) * kInvAccelDecelAbsValue * v_ref;
  return gap_behind ? rel_s : -rel_s;
}

inline double SolveImmediateCatchTime(double v_start, double v_ref,
                                      double lon_offset) {
  const double c =
      std::sqrt(Sqr(v_start - v_ref) - 2.0 * kAccelDecelAbsValue * lon_offset);
  return kInvAccelDecelAbsValue * (std::abs(v_start - v_ref) - c);
}

double SolveIntermediateSpeed(double v_start, double v_end, double v_ref,
                              double lon_offset, bool gap_behind) {
  const double c = 2.0 * v_ref * (v_start + v_end) - Sqr(v_start) - Sqr(v_end) -
                   2.0 * kAccelDecelAbsValue * lon_offset;
  const double val = 0.5 * std::sqrt(4.0 * Sqr(v_ref) - 2.0 * c);

  return gap_behind ? v_ref - val : v_ref + val;
}

double ComputeEgoPreviewV(const ObjectProjectionInfo& leader,
                          const ObjectProjectionInfo& follower,
                          const FrenetBox& ego_frenet_box, double ego_v,
                          double ego_a, double gap_finder_preview_time) {
  if (leader.object != nullptr &&
      ego_frenet_box.s_max > leader.frenet_box.s_min) {
    return leader.object->pose().v();
  }
  if (follower.object != nullptr &&
      follower.frenet_box.s_max > ego_frenet_box.s_min) {
    return follower.object->pose().v();
  }
  return ego_v + ego_a * gap_finder_preview_time;
}

double ComputeLeaderBuffer(const ObjectProjectionInfo& leader,
                           const FrenetBox& ego_frenet_box, double ego_v,
                           double /*speed_limit*/, double preview_time) {
  if (leader.object == nullptr) return kLonFollowBuffer;
  const bool is_leader_ahead_ego =
      ego_frenet_box.s_max < leader.frenet_box.s_min;
  const auto& object = *leader.object;
  const double obj_v = is_leader_ahead_ego
                           ? object.pose().v()
                           : EstimateObjectSpeed(object, preview_time);
  const double ego_follow_time = ComputeEgoFollowTime(obj_v, ego_v);
  const double safe_response_interval = ComputeSafeResponseInterval(
      obj_v, ego_v, kEgoResponseTime, ego_follow_time);
  return safe_response_interval;
}

double ComputeFollowerBuffer(const ObjectProjectionInfo& follower,
                             const FrenetBox& ego_frenet_box, double ego_v,
                             double speed_limit, double preview_time) {
  if (follower.object == nullptr) return kLonFollowBuffer;
  const bool is_ego_ahead_follower =
      follower.frenet_box.s_max < ego_frenet_box.s_min;
  const auto& object = *follower.object;
  const double obj_v = is_ego_ahead_follower
                           ? EstimateObjectSpeed(object, preview_time)
                           : object.pose().v();
  const double ego_lead_time = ComputeEgoLeadTime(speed_limit, ego_v, obj_v);
  const double safe_response_interval = ComputeSafeResponseInterval(
      ego_v, obj_v, kFollowerStandardResponseTime, ego_lead_time);
  return safe_response_interval;
}
}  // namespace

absl::StatusOr<GapAlignInfo> ComputeGapAlignInfo(double lon_offset,
                                                 double target_v, double cap_v,
                                                 double ego_v, bool gap_behind,
                                                 bool gap_aligned) {
  if (gap_behind && target_v < kAlignSpeedDiff) {
    return absl::OutOfRangeError("Target speed too low to follow.");
  }
  if (!gap_behind && cap_v < target_v) {
    return absl::OutOfRangeError("Cap speed too low to lead follower.");
  }
  constexpr double kMinCoastingSpeed = 1.0;  // m/s.
  const double final_v =
      gap_behind ? std::max(kMinCoastingSpeed, target_v - kFinalSpeedDiff)
                 : std::min(cap_v, target_v + kFinalSpeedDiff);
  if (gap_aligned && gap_behind != (ego_v > final_v)) {
    return GapAlignInfo{.align_time = 0.0, .align_s = 0.0, .align_v = ego_v};
  }

  const double aligned_sign = gap_aligned ? -1.0 : 1.0;
  const double min_remain_dist =
      lon_offset +
      aligned_sign * ComputeCatchDist(ego_v, final_v, target_v, gap_behind);
  if (gap_aligned && min_remain_dist > 0.0) {
    const double align_time =
        std::abs(final_v - ego_v) * kInvAccelDecelAbsValue;
    return GapAlignInfo{.align_time = align_time,
                        .align_s = 0.5 * (ego_v + final_v) * align_time,
                        .align_v = final_v};
  }
  if (!gap_aligned && min_remain_dist < 0.0) {
    const double align_time =
        SolveImmediateCatchTime(ego_v, target_v, lon_offset);
    const double align_v =
        ego_v + align_time * (ego_v > target_v ? -kAccelDecelAbsValue
                                               : kAccelDecelAbsValue);
    return GapAlignInfo{.align_time = align_time,
                        .align_s = 0.5 * (ego_v + align_v) * align_time,
                        .align_v = align_v};
  }

  const double align_v =
      gap_behind ? std::max(kMinCoastingSpeed, target_v - kAlignSpeedDiff)
                 : std::min(cap_v, target_v + kAlignSpeedDiff);
  if (!gap_aligned && gap_behind != (ego_v > align_v)) {
    const double t1 = min_remain_dist / std::abs(target_v - ego_v);
    const double t2 = std::abs(final_v - ego_v) * kInvAccelDecelAbsValue;
    return GapAlignInfo{.align_time = t1 + t2,
                        .align_s = ego_v * t1 + 0.5 * (final_v + ego_v) * t2,
                        .align_v = final_v};
  }

  const double remain_dist =
      lon_offset +
      aligned_sign * (ComputeCatchDist(ego_v, align_v, target_v, gap_behind) +
                      ComputeCatchDist(align_v, final_v, target_v, gap_behind));
  if (gap_aligned != (remain_dist < 0.0)) {
    const double mid_v = SolveIntermediateSpeed(
        ego_v, final_v, target_v, aligned_sign * lon_offset, gap_behind);
    return GapAlignInfo{.align_time = std::abs(ego_v + final_v - 2.0 * mid_v) *
                                      kInvAccelDecelAbsValue,
                        .align_s = 0.5 * kInvAccelDecelAbsValue *
                                   (std::abs(Sqr(mid_v) - Sqr(ego_v)) +
                                    std::abs(Sqr(mid_v) - Sqr(final_v))),
                        .align_v = final_v};
  }

  const double t1 = std::abs(remain_dist / (target_v - align_v));
  const double t2 =
      std::abs(ego_v + final_v - 2.0 * align_v) * kInvAccelDecelAbsValue;
  return GapAlignInfo{
      .align_time = t1 + t2,
      .align_s = align_v * t1 + 0.5 * kInvAccelDecelAbsValue *
                                    (std::abs(Sqr(align_v) - Sqr(ego_v)) +
                                     std::abs(Sqr(align_v) - Sqr(final_v))),
      .align_v = final_v};
}

// TODO(boqian): reduce duplicate code.
absl::StatusOr<TrafficGap> ComputeTrafficGapInfo(
    const ObjectProjectionInfo& leader, const ObjectProjectionInfo& follower,
    double cap_v, double cap_offset, double lane_max_v,
    const FrenetBox& ego_frenet_box, double ego_v, double ego_a,
    double max_reach_length, double speed_limit) {
  const double gap_finder_preview_time =
      FLAGS_planner_main_loop_interval;  // s.
  const double ego_preview_v = ComputeEgoPreviewV(
      leader, follower, ego_frenet_box, ego_v, ego_a, gap_finder_preview_time);
  const double max_front_s =
      leader.frenet_box.s_min - ComputeLeaderBuffer(leader, ego_frenet_box,
                                                    ego_preview_v, speed_limit,
                                                    gap_finder_preview_time);
  const double min_rear_s =
      follower.frenet_box.s_max +
      ComputeFollowerBuffer(follower, ego_frenet_box, ego_preview_v,
                            speed_limit, gap_finder_preview_time);

  constexpr double kLcPreviewTime = 4.0;  // s.
  const double leader_v =
      leader.object == nullptr ? DBL_MAX : leader.object->pose().v();
  const double follower_v =
      follower.object == nullptr ? -DBL_MAX : follower.object->pose().v();
  // Not aligned yet.
  if (ego_frenet_box.s_max > max_front_s || ego_frenet_box.s_min < min_rear_s) {
    const bool gap_behind = ego_frenet_box.s_max > max_front_s;
    const double lon_offset = gap_behind ? ego_frenet_box.s_max - max_front_s
                                         : min_rear_s - ego_frenet_box.s_min;
    const double target_v = gap_behind ? leader_v : follower_v;
    ASSIGN_OR_RETURN(
        const auto align_info,
        ComputeGapAlignInfo(lon_offset, target_v, lane_max_v, ego_v, gap_behind,
                            /*gap_aligned=*/false));
    if (align_info.align_s > cap_v * align_info.align_time + cap_offset) {
      return absl::OutOfRangeError("Ego speed capped by leading object.");
    }
    const double total_s = align_info.align_s + target_v * kLcPreviewTime;
    if (total_s > max_reach_length) {
      return absl::NotFoundError(
          absl::StrCat("Max reach length ", max_reach_length,
                       " shorter than total travel dist ", total_s, "."));
    }
    const double gap_length = max_front_s - min_rear_s +
                              (leader_v - follower_v) * align_info.align_time;
    if (gap_length < ego_frenet_box.length()) {
      return absl::NotFoundError("Gap length not enough after align time.");
    }
    const double gap_cap_v =
        gap_behind
            ? align_info.align_v
            : (leader.object == nullptr
                   ? DBL_MAX
                   : SolveIntermediateSpeed(ego_v, leader_v, leader_v,
                                            max_front_s - ego_frenet_box.s_max,
                                            /*gap_behind=*/false));
    return TrafficGap{.leader_object = leader.object,
                      .follower_object = follower.object,
                      .lon_offset = lon_offset,
                      .target_v = align_info.align_v,
                      .align_time = align_info.align_time,
                      .gap_length = gap_length,
                      .gap_cap_v = gap_cap_v};
  }
  // Already aligned.
  TrafficGap aligned_gap{.leader_object = leader.object,
                         .follower_object = follower.object,
                         .lon_offset = 0.0,
                         .align_time = -DBL_MAX};
  for (bool gap_behind : {true, false}) {
    const double lon_offset = gap_behind ? max_front_s - ego_frenet_box.s_max
                                         : ego_frenet_box.s_min - min_rear_s;
    const double target_v = gap_behind ? leader_v : follower_v;
    ASSIGN_OR_RETURN(
        const auto align_info,
        ComputeGapAlignInfo(lon_offset, target_v, lane_max_v, ego_v, gap_behind,
                            /*gap_aligned=*/true));
    if (align_info.align_s > cap_v * align_info.align_time + cap_offset) {
      return absl::OutOfRangeError("Ego speed capped by leading object.");
    }
    const double total_s = align_info.align_s + target_v * kLcPreviewTime;
    if (total_s > max_reach_length) {
      return absl::NotFoundError(
          absl::StrCat("Max reach length ", max_reach_length,
                       " shorter than total travel dist ", total_s, "."));
    }
    const double gap_length = max_front_s - min_rear_s +
                              (leader_v - follower_v) * align_info.align_time;
    if (gap_length < ego_frenet_box.length()) {
      return absl::NotFoundError("Gap length not enough after align time.");
    }
    if (align_info.align_time > aligned_gap.align_time) {
      aligned_gap.align_time = align_info.align_time;
      aligned_gap.target_v = align_info.align_v;
      aligned_gap.gap_length = gap_length;
      aligned_gap.gap_cap_v =
          leader.object == nullptr
              ? DBL_MAX
              : SolveIntermediateSpeed(ego_v, leader_v, leader_v,
                                       max_front_s - ego_frenet_box.s_max,
                                       /*gap_behind=*/false);
    }
  }
  return aligned_gap;
}

// Leader speed, lon_offset.
std::pair<double, double> FindNearestLeadingObject(
    const FrenetFrame& frenet_frame, const FrenetBox& ego_frenet_box,
    double /*ego_v*/, const SpacetimeTrajectoryManager& st_traj_mgr) {
  const PlannerObject* nearest = nullptr;
  double min_front_s = DBL_MAX;
  double nearest_angle_diff = 0.0;
  for (const auto& [_, trajectories] : st_traj_mgr.object_trajectories_map()) {
    if (trajectories.empty()) continue;

    const PlannerObject& obj = trajectories.front()->planner_object();
    ASSIGN_OR_CONTINUE(const auto frenet_box,
                       frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));

    const double angle_diff = AngleDifference(
        obj.pose().theta(),
        frenet_frame.InterpolateTangentByS(frenet_box.center_s()).FastAngle());
    if (angle_diff > M_PI_4 ||
        frenet_box.s_min - kLonFollowBuffer < ego_frenet_box.s_max ||
        !HasEnteredLane(frenet_box.l_max, frenet_box.l_min)) {
      continue;
    }
    if (frenet_box.s_min < min_front_s) {
      nearest = &obj;
      min_front_s = frenet_box.s_min;
      nearest_angle_diff = angle_diff;
    }
  }
  if (nearest == nullptr) return {DBL_MAX, DBL_MAX};

  const double obj_lon_v =
      nearest->pose().v() * fast_math::Cos(nearest_angle_diff);
  const double lon_offset =
      min_front_s - kLonFollowBuffer - ego_frenet_box.s_max;
  return {obj_lon_v, lon_offset};
}

std::vector<TrafficGap> FindCandidateTrafficGapsOnLanePath(
    const FrenetFrame& target_frenet_frame, double cap_v, double cap_offset,
    const FrenetBox& ego_frenet_box, double ego_v, double ego_a,
    const SpacetimeTrajectoryManager& st_traj_mgr, double max_reach_length,
    double speed_limit) {
  std::vector<ObjectProjectionInfo> objects_on_lane;
  for (const auto& [_, trajectories] : st_traj_mgr.object_trajectories_map()) {
    if (trajectories.empty()) continue;

    const PlannerObject& obj = trajectories.front()->planner_object();
    ASSIGN_OR_CONTINUE(
        const auto frenet_box,
        target_frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));
    if (!HasEnteredLane(frenet_box.l_max, frenet_box.l_min)) continue;

    objects_on_lane.emplace_back(
        ObjectProjectionInfo{.object = &obj, .frenet_box = frenet_box});
  }
  if (objects_on_lane.empty()) return {};

  objects_on_lane.emplace_back(ObjectProjectionInfo{
      .object = nullptr,
      .frenet_box = FrenetBox{
          .s_max = -DBL_MAX, .s_min = -DBL_MAX, .l_max = 0.0, .l_min = 0.0}});
  std::stable_sort(objects_on_lane.begin(), objects_on_lane.end(),
                   [](const auto& a, const auto& b) {
                     return a.frenet_box.s_min < b.frenet_box.s_min;
                   });
  objects_on_lane.emplace_back(ObjectProjectionInfo{
      .object = nullptr,
      .frenet_box = FrenetBox{
          .s_max = DBL_MAX, .s_min = DBL_MAX, .l_max = 0.0, .l_min = 0.0}});

  constexpr double kConsiderAsLaneSpeedLimitDist = 1e3;
  const double lane_max_v =
      cap_offset > kConsiderAsLaneSpeedLimitDist
          ? cap_v
          : SolveIntermediateSpeed(ego_v, cap_v, cap_v, cap_offset,
                                   /*gap_behind=*/false);
  std::vector<TrafficGap> gaps;
  for (int i = 0; i + 1 < objects_on_lane.size(); ++i) {
    ASSIGN_OR_CONTINUE(
        gaps.emplace_back(),
        ComputeTrafficGapInfo(objects_on_lane[i + 1], objects_on_lane[i], cap_v,
                              cap_offset, lane_max_v, ego_frenet_box, ego_v,
                              ego_a, max_reach_length, speed_limit));
  }

  return gaps;
}

}  // namespace qcraft::planner
