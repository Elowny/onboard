#include "onboard/planner/decision/traffic_gap_finder.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

const std::vector<double> kDiscreteAccelerations = {0.0, -0.5, -1.0, -1.5,
                                                    -2.0};
const std::vector<double> kEvaluationKeys = {0.0, 1.0, 2.0, 3.0};

constexpr double kMinCoastingSpeed = 3.0;  // m/s

struct SVState {
  double s, v;
};

std::vector<SVState> FindObjectStates(
    const SpacetimeObjectTrajectory* object_traj, double start_s) {
  if (object_traj == nullptr) return std::vector<SVState>();

  if (object_traj->is_stationary()) {
    std::vector<SVState> result;
    result.reserve(kEvaluationKeys.size());
    for (int i = 0; i < kEvaluationKeys.size(); ++i) {
      result.emplace_back(SVState{.s = start_s, .v = 0.0});
    }
    return result;
  }

  std::vector<SVState> states;
  states.reserve(kEvaluationKeys.size());

  for (const double t : kEvaluationKeys) {
    const auto nearest_it = std::min_element(
        object_traj->states().begin(), object_traj->states().end(),
        [t](const SpacetimeObjectState& lhs, const SpacetimeObjectState& rhs) {
          return std::abs(t - lhs.traj_point->t()) <
                 std::abs(t - rhs.traj_point->t());
        });

    // This addition is correct only if the closest two gaps are considered.
    states.emplace_back(SVState{.s = start_s + nearest_it->traj_point->s(),
                                .v = nearest_it->traj_point->v()});
  }

  return states;
}

absl::StatusOr<int> EvaluateTrafficGap(const TrafficGap& candidate,
                                       const FrenetBox& ego_frenet_box,
                                       double ego_init_v) {
  std::vector<std::vector<SVState>> all_leader_sv;
  for (const auto* traj : candidate.leader_trajectories) {
    all_leader_sv.emplace_back(FindObjectStates(traj, candidate.s_end));
  }

  std::vector<std::vector<SVState>> all_follower_sv;
  for (const auto* traj : candidate.follower_trajectories) {
    all_follower_sv.emplace_back(FindObjectStates(traj, candidate.s_start));
  }

  for (int strategy_index = 0; strategy_index < kDiscreteAccelerations.size();
       ++strategy_index) {
    const double accel = kDiscreteAccelerations[strategy_index];
    for (int i = 0; i < kEvaluationKeys.size(); ++i) {
      const double t = kEvaluationKeys[i];
      // TODO(weijun): consider accelerating later.
      const double accel_v = ego_init_v + accel * t;
      const double ego_v = std::max(kMinCoastingSpeed, accel_v);
      double ego_delta_s;
      if (accel_v < kMinCoastingSpeed) {
        const double break_t = std::abs(accel) < 1e-7
                                   ? 0.0
                                   : (kMinCoastingSpeed - ego_init_v) / accel;
        ego_delta_s = (kMinCoastingSpeed + ego_init_v) * break_t * 0.5 +
                      kMinCoastingSpeed * (t - break_t);
      } else {
        ego_delta_s = (ego_v + ego_init_v) * t * 0.5;
      }
      const double ego_front_s = ego_delta_s + ego_frenet_box.s_max;
      const double ego_rear_s = ego_delta_s + ego_frenet_box.s_min;

      const auto ttc_ok =
          [i, ego_v](const std::vector<std::vector<SVState>>& all_states,
                     double ego_s, bool leader) {
            for (const auto& states : all_states) {
              if (states.empty()) continue;

              if ((leader && ego_s > states[i].s) ||
                  (!leader && ego_s < states[i].s)) {
                return false;
              }
              constexpr double kMinTTC = 3.0;  // seconds.
              const double ttc = (states[i].s - ego_s) / (ego_v - states[i].v);
              if (ttc >= 0.0 && ttc < kMinTTC) return false;
            }
            return true;
          };

      const bool leader_ok =
          ttc_ok(all_leader_sv, ego_front_s, /*leader=*/true);
      const bool follower_ok =
          ttc_ok(all_follower_sv, ego_rear_s, /*leader=*/false);

      if (leader_ok && follower_ok) return strategy_index;
    }
  }

  return absl::NotFoundError("");
}

}  // namespace

std::vector<TrafficGap> FindCandidateTrafficGapsOnLanePath(
    const FrenetFrame& target_frenet_frame, const FrenetBox& ego_frenet_box,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  struct PlannerObjectProjectionInfo {
    absl::Span<const SpacetimeObjectTrajectory* const> st_trajectories;
    FrenetBox frenet_box;
  };

  std::vector<PlannerObjectProjectionInfo> objects_on_lane;
  for (const auto& [_, trajectories] : st_traj_mgr.object_trajectories_map()) {
    if (trajectories.empty()) continue;
    const PlannerObject& obj = trajectories.front()->planner_object();
    if (obj.is_stationary()) continue;
    // TODO(weijun): replace with contour.
    ASSIGN_OR_CONTINUE(
        const auto frenet_box,
        target_frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));

    constexpr double kLateralThreshold = 1.0;  // m.
    if (frenet_box.l_min > kLateralThreshold ||
        frenet_box.l_max < -kLateralThreshold) {
      continue;
    }

    objects_on_lane.emplace_back(PlannerObjectProjectionInfo{
        .st_trajectories = trajectories, .frenet_box = frenet_box});
  }
  if (objects_on_lane.empty()) return {};

  std::stable_sort(objects_on_lane.begin(), objects_on_lane.end(),
                   [](const auto& a, const auto& b) {
                     return a.frenet_box.s_min < b.frenet_box.s_min;
                   });

  const double ego_center_s = ego_frenet_box.center_s();
  const auto closest_object_it = std::min_element(
      objects_on_lane.begin(), objects_on_lane.end(),
      [ego_center_s](const auto& lhs, const auto& rhs) {
        return std::abs(lhs.frenet_box.center_s() - ego_center_s) <
               std::abs(rhs.frenet_box.center_s() - ego_center_s);
      });

  // Go behind closest object
  TrafficGap follow;
  follow.leader_trajectories = closest_object_it->st_trajectories;
  follow.s_end = closest_object_it->frenet_box.s_min;
  if (closest_object_it == objects_on_lane.begin()) {
    follow.s_start = std::numeric_limits<double>::lowest();
  } else {
    follow.s_start = std::prev(closest_object_it)->frenet_box.s_max;
    follow.follower_trajectories =
        std::prev(closest_object_it)->st_trajectories;
  }

  // Overtake closest object
  TrafficGap lead;
  lead.follower_trajectories = closest_object_it->st_trajectories;
  lead.s_start = closest_object_it->frenet_box.s_max;
  if (std::next(closest_object_it) == objects_on_lane.end()) {
    lead.s_end = std::numeric_limits<double>::max();
  } else {
    lead.leader_trajectories = std::next(closest_object_it)->st_trajectories;
    lead.s_end = std::next(closest_object_it)->frenet_box.s_min;
  }

  return {follow, lead};
}

absl::StatusOr<TrafficGapResult> EvaluateAndTakeBestTrafficGap(
    absl::Span<const TrafficGap> candidate_gaps,
    const FrenetBox& ego_frenet_box, double ego_init_v) {
  if (candidate_gaps.empty()) {
    return absl::NotFoundError("Input candidate gaps empty!");
  }

  int best_strategy = INT_MAX;
  int best_idx = -1;
  for (int i = 0; i < candidate_gaps.size(); ++i) {
    ASSIGN_OR_CONTINUE(
        const int strategy_index,
        EvaluateTrafficGap(candidate_gaps[i], ego_frenet_box, ego_init_v));

    if (strategy_index < best_strategy) {
      best_strategy = strategy_index;
      best_idx = i;
    }
  }
  if (best_idx == -1) {
    return absl::NotFoundError("No viable candidate after evaluation!");
  }

  const auto& best_gap = candidate_gaps[best_idx];
  TrafficGapResult result;
  if (!best_gap.leader_trajectories.empty()) {
    result.leader_id = best_gap.leader_trajectories.front()->object_id();
  }
  if (!best_gap.follower_trajectories.empty()) {
    result.follower_id = best_gap.follower_trajectories.front()->object_id();
  }
  return result;
}

}  // namespace qcraft::planner
