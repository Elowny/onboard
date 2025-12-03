#include "onboard/planner/decision/traffic_gap_v2/traffic_gap_finder.h"

#include <float.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/math/util.h"
#include "onboard/planner/decision/traffic_gap_v2/traffic_gap_finder_impl.h"
#include "onboard/planner/object/planner_object.h"

namespace qcraft::planner {

namespace {

double EvaluateTrafficGap(const TrafficGap& candidate, double ego_length,
                          double ego_v, double speed_limit,
                          const std::string& prev_leader,
                          const std::string& prev_follower,
                          TrafficGapDebugProto::TrafficGapInfo* gap_info) {
  constexpr double kWeightGapLength = 1.0;
  constexpr double kWeightAlignTime = 1.0;
  constexpr double kWeightLonOffset = 5.0;
  constexpr double kWeightSpeedDiffToEgo = 1.0;
  constexpr double kWeightSpeedDiffToLimit = 10.0;
  constexpr double kWeightSameWithPrev = -1.0;

  double cost = 0.0;

  // Gap length after alignment.
  constexpr double kGapLengthCutoffMultiplier = 3.0;
  const double std_cutoff_len = kGapLengthCutoffMultiplier * ego_length;
  const double gap_length_cost =
      kWeightGapLength *
      (candidate.gap_length > std_cutoff_len
           ? 0.0
           : Sqr(1.0 - candidate.gap_length / std_cutoff_len));
  cost += gap_length_cost;
  gap_info->mutable_gap_length()->set_val(candidate.gap_length);
  gap_info->mutable_gap_length()->set_cost(gap_length_cost);

  // Time for alignment.
  const double align_time_cost = kWeightAlignTime * candidate.align_time;
  cost += align_time_cost;
  gap_info->mutable_align_time()->set_val(candidate.align_time);
  gap_info->mutable_align_time()->set_cost(align_time_cost);

  // Current longitudinal offset to ego.
  const double lon_offset_cost = kWeightLonOffset * candidate.lon_offset;
  cost += lon_offset_cost;
  gap_info->mutable_lon_offset()->set_val(candidate.lon_offset);
  gap_info->mutable_lon_offset()->set_cost(lon_offset_cost);

  // Speed diff between ego and target.
  const double speed_to_ego_cost =
      kWeightSpeedDiffToEgo * std::abs(ego_v - candidate.target_v);
  cost += speed_to_ego_cost;
  gap_info->mutable_speed_to_ego()->set_val(candidate.target_v);
  gap_info->mutable_speed_to_ego()->set_cost(speed_to_ego_cost);

  // Speed diff between speed limit and target.
  const double speed_to_limit_cost =
      kWeightSpeedDiffToLimit *
      std::max(0.0, speed_limit - candidate.gap_cap_v);
  cost += speed_to_limit_cost;
  gap_info->mutable_speed_to_limit()->set_val(candidate.gap_cap_v);
  gap_info->mutable_speed_to_limit()->set_cost(speed_to_limit_cost);

  // Continuity with previous.
  double continuity = 0.0;
  if (candidate.leader_object != nullptr &&
      candidate.leader_object->id() == prev_leader) {
    continuity += 1.0;
  }
  if (candidate.follower_object != nullptr &&
      candidate.follower_object->id() == prev_follower) {
    continuity += 1.0;
  }
  const double continuity_cost = kWeightSameWithPrev * continuity;
  cost += continuity_cost;
  gap_info->mutable_continuity()->set_val(continuity);
  gap_info->mutable_continuity()->set_cost(continuity_cost);

  gap_info->set_leader_id(candidate.leader_object == nullptr
                              ? "None"
                              : candidate.leader_object->id());
  gap_info->set_follower_id(candidate.follower_object == nullptr
                                ? "None"
                                : candidate.follower_object->id());
  gap_info->set_total_cost(cost);

  return cost;
}

}  // namespace

TrafficGapResult FindBestTrafficGapOnRouteTarget(
    const FrenetFrame& cur_frenet_frame, const FrenetBox& cur_ego_fbox,
    const SpacetimeTrajectoryManager& cur_st_traj_mgr,
    const FrenetFrame& target_frenet_frame, const FrenetBox& target_ego_fbox,
    const SpacetimeTrajectoryManager& target_st_traj_mgr, double ego_v,
    double ego_a, double max_reach_length, double speed_limit,
    const std::string& prev_leader, const std::string& prev_follower,
    TrafficGapDebugProto* debug_info) {
  TrafficGapResult result;

  auto [cap_v, cap_offset] = FindNearestLeadingObject(
      cur_frenet_frame, cur_ego_fbox, ego_v, cur_st_traj_mgr);
  if (cap_v > speed_limit) {
    cap_v = speed_limit;
    cap_offset = DBL_MAX;
  }
  const auto candidate_gaps = FindCandidateTrafficGapsOnLanePath(
      target_frenet_frame, cap_v, cap_offset, target_ego_fbox, ego_v, ego_a,
      target_st_traj_mgr, max_reach_length, speed_limit);
  if (candidate_gaps.empty()) return result;

  QEVENT_EVERY_N_SECONDS("jiayu", "traffic_gap_finder_trigger",
                         /*seconds=*/6.0, [](QEvent* /*qevent*/) {});

  std::vector<double> gap_costs(candidate_gaps.size());
  const double ego_length = target_ego_fbox.length();
  for (int i = 0; i < candidate_gaps.size(); ++i) {
    gap_costs[i] = EvaluateTrafficGap(candidate_gaps[i], ego_length, ego_v,
                                      speed_limit, prev_leader, prev_follower,
                                      debug_info->add_gap_candidates());
  }

  const int best_index =
      std::min_element(gap_costs.begin(), gap_costs.end()) - gap_costs.begin();
  const auto& best_gap = candidate_gaps[best_index];
  if (best_gap.leader_object != nullptr) {
    result.leader_id = best_gap.leader_object->id();
  }
  if (best_gap.follower_object != nullptr) {
    result.follower_id = best_gap.follower_object->id();
  }
  return result;
}

}  // namespace qcraft::planner
