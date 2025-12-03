#include "onboard/planner/speed/dp_svt_cost.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <optional>
#include <ostream>
#include <utility>

#include "gflags/gflags.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/svt_point.h"
#include "onboard/utils/map_util.h"

DECLARE_bool(enable_sampling_dp_reference_speed);

namespace qcraft::planner {
namespace {
constexpr double kMaxSpeedLimitDiff = 10.0;    // m/s
constexpr double kComfortableBrakeAcc = -3.0;  // m/ss.

using DecisionType = StBoundaryProto::DecisionType;

inline DecisionType MakeStBoundaryDecision(double s, double s_lower,
                                           double s_upper) {
  constexpr double kFollowLeadRatio = 0.5;
  if (s < s_lower + kFollowLeadRatio * (s_upper - s_lower)) {
    return StBoundaryProto::FOLLOW;
  } else {
    return StBoundaryProto::LEAD;
  }
}

double GetStBoundaryCost(
    DecisionType decision, double comfortable_brake_s, double current_av_speed,
    const StBoundary& st_boundary, double s, double s_lower, double s_upper,
    double t, double follow_standstill_distance,
    const PiecewiseLinearFunction<double>& follow_distance_rel_speed_plf,
    const SpeedFinderParamsProto& speed_finder_params,
    const SpeedFinderParamsProto::SamplingDpSpeedParamsProto& params) {
  double cost = 0.0;
  if (decision == StBoundaryProto::FOLLOW) {
    double follow_distance_s = 0.0;
    if (st_boundary.source_type() == StBoundarySourceTypeProto::VIRTUAL ||
        st_boundary.source_type() ==
            StBoundarySourceTypeProto::IMPASSABLE_BOUNDARY ||
        st_boundary.source_type() == StBoundarySourceTypeProto::PATH_BOUNDARY ||
        st_boundary.protection_type() ==
            StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT) {
      follow_distance_s = follow_standstill_distance;
    } else {
      const auto object_v = st_boundary.GetStBoundarySpeedAtT(t);
      QCHECK(object_v.has_value());
      const double follow_time_headway =
          st_boundary.is_large_vehicle()
              ? speed_finder_params.large_vehicle_follow_time_headway()
              : speed_finder_params.follow_time_headway();
      follow_distance_s = follow_time_headway * std::max(0.0, *object_v) +
                          follow_standstill_distance;
      const double follow_distance_min_s_lower =
          std::max(comfortable_brake_s - follow_standstill_distance, 0.0);
      const double rel_speed_gain =
          follow_distance_rel_speed_plf(*object_v - current_av_speed);
      follow_distance_s = std::clamp(follow_distance_s * rel_speed_gain,
                                     follow_standstill_distance,
                                     s_lower - follow_distance_min_s_lower);
    }
    if (s + follow_distance_s > s_lower) {
      const double s_diff = follow_distance_s - s_lower + s;
      cost =
          st_boundary.probability() * params.object_weight() * s_diff * s_diff;
    }
  } else if (decision == StBoundaryProto::LEAD) {
    const auto object_v = st_boundary.GetStBoundarySpeedAtT(t);
    QCHECK(object_v.has_value());
    const double overtake_distance_s =
        st_boundary.protection_type() ==
                StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT
            ? 0.0
            : speed_finder_params.lead_time_headway() *
                      std::max(*object_v, 0.0) +
                  speed_finder_params.lead_standstill_distance();
    if (s < s_upper + overtake_distance_s) {  // or calculated from velocity
      const double s_diff = overtake_distance_s + s_upper - s;
      cost =
          st_boundary.probability() * params.object_weight() * s_diff * s_diff;
    }
  } else {
    // We must have lead or follow decision here.
    QLOG(FATAL) << "Bad unknown decision for st-boundary " << st_boundary.id()
                << " at s = " << s << " t = " << t;
  }
  return cost;
}

bool IsUncertainDecision(
    const SvtGraphPoint& svt_graph_point, const StBoundary& st_boundary,
    const absl::flat_hash_map<std::string, const StBoundaryWithDecision*>&
        original_st_boundary_wd_map,
    DecisionType decision, double init_v) {
  if (st_boundary.object_type() == StBoundaryProto::PEDESTRIAN) {
    return false;
  }
  // If we need a large acceleration/deceleration to execute the decision,
  // then this decision may be an uncertain decision.
  const StBoundary* original_st_boundary = &st_boundary;
  if (st_boundary.is_protective()) {
    if (const auto& protected_st_boundary_id =
            st_boundary.protected_st_boundary_id();
        protected_st_boundary_id.has_value()) {
      if (const auto original_st_boundary_wd = FindOrNull(
              original_st_boundary_wd_map, *protected_st_boundary_id);
          nullptr != original_st_boundary_wd) {
        original_st_boundary = (*original_st_boundary_wd)->st_boundary();
      }
    }
  }

  if (decision == StBoundaryProto::FOLLOW) {
    constexpr double kMaxMildAcc = 0.8;  // m/ss.
    const double avg_acc =
        (svt_graph_point.point().v() - init_v) / svt_graph_point.point().t();
    if (avg_acc > kMaxMildAcc) return false;
    constexpr double kMinAcc = -4.0;  // m/ss.
    const auto& upper_right_point = original_st_boundary->upper_right_point();
    const double delta_t = upper_right_point.t() - svt_graph_point.point().t();
    if (delta_t <= 0.0) return false;
    const double delta_s = upper_right_point.s() - svt_graph_point.point().s();
    const double t_decel =
        std::min(-svt_graph_point.point().v() / kMinAcc, delta_t);
    const double max_s =
        svt_graph_point.point().v() * t_decel + 0.5 * kMinAcc * Sqr(t_decel);
    return max_s > delta_s;
  } else if (decision == StBoundaryProto::LEAD) {
    constexpr double kMaxAcc = 2.0;  // m/ss.
    const auto& bottom_right_point = original_st_boundary->bottom_right_point();
    const double delta_t = bottom_right_point.t() - svt_graph_point.point().t();
    if (delta_t <= 0.0) return false;
    const double delta_s = bottom_right_point.s() - svt_graph_point.point().s();
    const double max_s =
        svt_graph_point.point().v() * delta_t + 0.5 * kMaxAcc * Sqr(delta_t);
    return max_s < delta_s;
  } else {
    QLOG(FATAL) << "Invalid decision";
  }
  return false;
}

inline DecisionType GetBackupDecision(DecisionType decision) {
  switch (decision) {
    case StBoundaryProto::FOLLOW:
      return StBoundaryProto::LEAD;
    case StBoundaryProto::LEAD:
      return StBoundaryProto::FOLLOW;
    case StBoundaryProto::UNKNOWN:
    case StBoundaryProto::IGNORE:
      QLOG(FATAL) << "Invalid decision to get backup decision";
      return StBoundaryProto::UNKNOWN;
  }
}

}  // namespace

DpSvtCost::DpSvtCost(const SpeedFinderParamsProto* speed_finder_params,
                     double total_t, double total_s, double init_v,
                     const std::vector<StBoundaryWithDecision*>*
                         sorted_st_boundaries_with_decision)
    : speed_finder_params_(QCHECK_NOTNULL(speed_finder_params)),
      params_(&speed_finder_params->sampling_dp_speed_params()),
      sorted_st_boundaries_with_decision_(
          QCHECK_NOTNULL(sorted_st_boundaries_with_decision)),
      unit_t_(params_->unit_t()),
      total_s_(total_s),
      total_t_(total_t) {
  int index = 0;
  for (const auto* st_boundary_with_decision :
       *sorted_st_boundaries_with_decision_) {
    boundary_map_[st_boundary_with_decision->id()] = index++;
  }
  const auto dimension_t = CeilToInt(total_t / unit_t_) + 1;
  comfortable_brake_s_.reserve(dimension_t);
  comfortable_brake_s_.push_back(0.0);
  const double max_brake_time = -init_v / kComfortableBrakeAcc;
  const double max_brake_s = init_v * max_brake_time +
                             0.5 * kComfortableBrakeAcc * Sqr(max_brake_time);
  for (int i = 1; i < dimension_t; ++i) {
    const double t = static_cast<double>(i) * unit_t_;
    const double comfortable_brake_s =
        t > max_brake_time ? max_brake_s
                           : init_v * t + 0.5 * kComfortableBrakeAcc * Sqr(t);
    comfortable_brake_s_.push_back(comfortable_brake_s);
  }
  boundary_cost_.resize(sorted_st_boundaries_with_decision_->size());
  for (auto& vec : boundary_cost_) {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }
  // Get the map of original st-boundaries.
  for (const auto* st_boundary_wd : *sorted_st_boundaries_with_decision_) {
    if (st_boundary_wd->raw_st_boundary()->is_protective()) {
      continue;
    }
    original_st_boundary_wd_map_[st_boundary_wd->id()] = st_boundary_wd;
  }
  speed_large_accel_penalty_factor_plf_ = PiecewiseLinearFunctionFromProto(
      params_->speed_large_accel_penalty_factor_plf());

  exceed_speed_unit_cost_ =
      params_->exceed_speed_penalty() * params_->speed_weight() * unit_t_;
  low_speed_unit_cost_ =
      params_->low_speed_penalty() * params_->speed_weight() * unit_t_;

  // Prerequisite: the max_t of the protective st-boundary must be less than or
  // equal to the min_t of the corresponding original st-boundary.
  for (const auto* st_boundary_wd : *sorted_st_boundaries_with_decision_) {
    const auto& st_boundary = *st_boundary_wd->st_boundary();
    if (!st_boundary.is_protective()) continue;
    const auto& protected_st_boundary_id =
        st_boundary.protected_st_boundary_id();
    if (!protected_st_boundary_id.has_value()) continue;
    if (const auto original_st_boundary_wd =
            FindOrNull(original_st_boundary_wd_map_, *protected_st_boundary_id);
        nullptr != original_st_boundary_wd) {
      const auto& original_st_boundary =
          *(*original_st_boundary_wd)->st_boundary();
      QCHECK_LE(st_boundary.max_t(), original_st_boundary.min_t());
    }
  }
}

std::vector<SvtGraphPoint::StBoundaryDecision>
DpSvtCost::GetStBoundaryDecisionsForInitPoint(
    const SvtGraphPoint& svt_graph_point) {
  std::vector<SvtGraphPoint::StBoundaryDecision> st_boundary_decisions;
  const double s = svt_graph_point.point().s();  // Should be zero.
  const double t = svt_graph_point.point().t();  // Should be zero.

  for (const auto* st_boundary_with_decision :
       *sorted_st_boundaries_with_decision_) {
    const auto& st_boundary = *st_boundary_with_decision->st_boundary();
    if (t < st_boundary.min_t() || t > st_boundary.max_t()) {
      continue;
    }
    if (st_boundary_with_decision->decision_type() == StBoundaryProto::IGNORE) {
      // Skip computing cost for st_boundary that has a prior IGNORE decision.
      continue;
    }

    double s_upper = 0.0;
    double s_lower = 0.0;
    const int boundary_index = boundary_map_[st_boundary.id()];
    {
      absl::MutexLock lock(&boundary_cost_mutex_);
      if (boundary_cost_[boundary_index][svt_graph_point.index_t()].first <
          0.0) {
        const auto s_range = st_boundary.GetBoundarySRange(t);
        QCHECK(s_range.has_value());
        boundary_cost_[boundary_index][svt_graph_point.index_t()] = *s_range;
        s_upper = s_range->first;
        s_lower = s_range->second;
      } else {
        s_upper =
            boundary_cost_[boundary_index][svt_graph_point.index_t()].first;
        s_lower =
            boundary_cost_[boundary_index][svt_graph_point.index_t()].second;
      }
      // Make decision based on relative position of s, s_lower and s_upper.
      const auto decision = MakeStBoundaryDecision(s, s_lower, s_upper);
      // Record decision on this st_boundary without prior decision.
      st_boundary_decisions.emplace_back(st_boundary.id(), decision);
      // Synchronize the decisions of protective st-boundary & corresponding
      // original st-boundary.
      if (st_boundary_with_decision->raw_st_boundary()->is_protective()) {
        const auto& protected_st_boundary_id =
            st_boundary_with_decision->raw_st_boundary()
                ->protected_st_boundary_id();
        if (protected_st_boundary_id.has_value()) {
          if (const auto original_st_boundary_wd = FindOrNull(
                  original_st_boundary_wd_map_, *protected_st_boundary_id);
              nullptr != original_st_boundary_wd &&
              (*original_st_boundary_wd)->decision_type() ==
                  StBoundaryProto::UNKNOWN) {
            st_boundary_decisions.emplace_back(*protected_st_boundary_id,
                                               decision);
          }
        }
      }
    }
  }
  return st_boundary_decisions;
}

void DpSvtCost::GetStBoundaryCostAndDecisions(
    const SvtGraphPoint& prev_svt_graph_point,
    const SvtGraphPoint& svt_graph_point, double init_av_speed,
    double current_av_speed, double* st_boundary_cost,
    std::vector<SvtGraphPoint::StBoundaryDecision>* st_boundary_decisions,
    std::vector<SvtGraphPoint::StBoundaryBackupDecision>*
        st_boundary_backup_decisions) {
  QCHECK_NOTNULL(st_boundary_cost);
  QCHECK_NOTNULL(st_boundary_decisions);
  QCHECK_NOTNULL(st_boundary_backup_decisions);
  QCHECK(st_boundary_decisions->empty());

  const double prev_s = prev_svt_graph_point.point().s();
  const double prev_t = prev_svt_graph_point.point().t();
  const int t_index = svt_graph_point.index_t();

  double cost = 0.0;
  for (const auto* st_boundary_with_decision :
       *sorted_st_boundaries_with_decision_) {
    if (st_boundary_with_decision->decision_type() == StBoundaryProto::IGNORE) {
      // Skip computing cost for st_boundary that has a prior IGNORE decision.
      continue;
    }

    double s = svt_graph_point.point().s();
    double t = svt_graph_point.point().t();
    double comfortable_brake_s = comfortable_brake_s_[t_index];
    const auto& st_boundary = *st_boundary_with_decision->st_boundary();
    if (t < st_boundary.min_t() ||
        (prev_t >= st_boundary.min_t() && t > st_boundary.max_t())) {
      continue;
    }

    // If overlap period is too small that located between current and previous
    // point, recalculate decision making s and t.
    if (t >= st_boundary.max_t()) {
      s = Lerp(prev_s, s, LerpFactor(prev_t, t, st_boundary.max_t()));
      t = st_boundary.max_t();
      comfortable_brake_s =
          init_av_speed * t + 0.5 * kComfortableBrakeAcc * Sqr(t);
    }

    double s_upper = 0.0;
    double s_lower = 0.0;
    int boundary_index = boundary_map_[st_boundary.id()];
    {
      absl::MutexLock lock(&boundary_cost_mutex_);
      auto& boundary_bounds = boundary_cost_[boundary_index][t_index];
      if (boundary_bounds.first < 0.0) {
        const auto s_range = st_boundary.GetBoundarySRange(t);
        QCHECK(s_range.has_value());
        boundary_bounds = *s_range;
        s_upper = s_range->first;
        s_lower = s_range->second;
      } else {
        s_upper = boundary_bounds.first;
        s_lower = boundary_bounds.second;
      }
    }

    DecisionType decision = StBoundaryProto::UNKNOWN;
    bool decided_by_protective = false;
    std::optional<SvtGraphPoint::BackupDecisionInfo> backup_decision_info;
    if (st_boundary_with_decision->decision_type() !=
        StBoundaryProto::UNKNOWN) {
      // If st_boundary has a prior decision, use it.
      decision = st_boundary_with_decision->decision_type();
    } else {
      // st_boundary doesn't have a prior decision.
      const auto prev_point_decision =
          prev_svt_graph_point.GetStBoundaryDecision(st_boundary.id());
      if (prev_point_decision) {
        QCHECK_NE(*prev_point_decision, StBoundaryProto::UNKNOWN);
        // If prev point has decision on the st_boundary, keep it.
        decision = *prev_point_decision;
        backup_decision_info =
            prev_svt_graph_point.GetBackupStBoundaryDecision(st_boundary.id());
      } else {
        // Compute decision decisive s, we can make decision via decisive st
        // point (decisive_s, min_t) no matter whether speed profile
        // cross st boundary between lower points or first lower point and upper
        // point.
        for (const auto& st_boundary_decision : *st_boundary_decisions) {
          if (st_boundary_decision.first == st_boundary.id()) {
            // If the decision of this st-boundary is already made by its
            // protective st-boundary, use this decison directly.
            decision = st_boundary_decision.second;
            decided_by_protective = true;
            QCHECK_NE(decision, StBoundaryProto::UNKNOWN);
            QCHECK(ContainsKey(original_st_boundary_wd_map_, st_boundary.id()));
            break;
          }
        }
        if (!decided_by_protective) {
          const double decisive_s =
              Lerp(prev_s, s, LerpFactor(prev_t, t, st_boundary.min_t()));
          const double first_s_upper = st_boundary.upper_points().front().s();
          const double first_s_lower = st_boundary.lower_points().front().s();
          // Make decision based on relative position of s, s_lower and s_upper.
          decision =
              MakeStBoundaryDecision(decisive_s, first_s_lower, first_s_upper);
          if (IsUncertainDecision(svt_graph_point, st_boundary,
                                  original_st_boundary_wd_map_, decision,
                                  init_av_speed)) {
            backup_decision_info = SvtGraphPoint::BackupDecisionInfo{
                .backup_decision = GetBackupDecision(decision),
                .original_decision_cost = 0.0,
                .backup_decison_cost = 0.0};
          }
        }
      }
    }
    double st_boundary_cost = GetStBoundaryCost(
        decision, comfortable_brake_s, current_av_speed, st_boundary, s,
        s_lower, s_upper, t,
        st_boundary_with_decision->follow_standstill_distance(),
        follow_distance_rel_speed_plf_, *speed_finder_params_, *params_);
    if (backup_decision_info.has_value()) {
      const double backup_decision_cost = GetStBoundaryCost(
          backup_decision_info->backup_decision, comfortable_brake_s,
          current_av_speed, st_boundary, s, s_lower, s_upper, t,
          st_boundary_with_decision->follow_standstill_distance(),
          follow_distance_rel_speed_plf_, *speed_finder_params_, *params_);
      backup_decision_info->original_decision_cost += st_boundary_cost;
      backup_decision_info->backup_decison_cost += backup_decision_cost;
      if (t + unit_t_ > std::min(st_boundary.max_t(), total_t_)) {
        constexpr double kDecisionChangeCostRatio = 0.1;
        if (backup_decision_info->original_decision_cost *
                kDecisionChangeCostRatio >
            backup_decision_info->backup_decison_cost) {
          // If the cost of back-up decision is less than the 0.1 of cost of
          // original decision, then we change the decision of this st-boundary
          // to back-up decision and use the cost of back-up decision.
          decision = backup_decision_info->backup_decision;
          st_boundary_cost += backup_decision_info->backup_decison_cost -
                              backup_decision_info->original_decision_cost;
        }
      } else {
        st_boundary_backup_decisions->emplace_back(st_boundary.id(),
                                                   *backup_decision_info);
      }
    }
    if (st_boundary_with_decision->decision_type() ==
            StBoundaryProto::UNKNOWN &&
        !decided_by_protective) {
      // Record decision on this st_boundary without prior decision.
      st_boundary_decisions->emplace_back(st_boundary.id(), decision);
      // Synchronize the decisions of protective st-boundary & corresponding
      // original st-boundary.
      if (st_boundary_with_decision->raw_st_boundary()->is_protective()) {
        const auto& protected_st_boundary_id =
            st_boundary_with_decision->raw_st_boundary()
                ->protected_st_boundary_id();
        if (protected_st_boundary_id.has_value()) {
          if (const auto original_st_boundary_wd = FindOrNull(
                  original_st_boundary_wd_map_, *protected_st_boundary_id);
              nullptr != original_st_boundary_wd &&
              (*original_st_boundary_wd)->decision_type() ==
                  StBoundaryProto::UNKNOWN) {
            st_boundary_decisions->emplace_back(*protected_st_boundary_id,
                                                decision);
          }
        }
      }
    }
    cost += st_boundary_cost;
  }
  *st_boundary_cost = cost * unit_t_;
}

double DpSvtCost::GetSpatialPotentialCost(double s) const {
  return (total_s_ - s) * params_->spatial_potential_weight();
}

double DpSvtCost::GetSpeedLimitCost(double speed, double speed_limit) const {
  double cost = 0.0;
  const double det_speed =
      std::clamp(speed - speed_limit, -kMaxSpeedLimitDiff, kMaxSpeedLimitDiff);
  if (det_speed > 0.0) {
    cost = exceed_speed_unit_cost_ * det_speed * det_speed;
  } else if (det_speed < 0.0) {
    cost = low_speed_unit_cost_ * det_speed * det_speed;
  }
  return cost;
}

double DpSvtCost::GetReferenceSpeedCost(double speed,
                                        double cruise_speed) const {
  double cost = 0.0;

  if (FLAGS_enable_sampling_dp_reference_speed) {
    const double diff_speed = speed - cruise_speed;
    cost += params_->reference_speed_penalty() * params_->speed_weight() *
            diff_speed * diff_speed * unit_t_;
  }

  return cost;
}

double DpSvtCost::GetAccelCost(double accel, double speed) const {
  double cost = 0.0;

  const double accel_sq = accel * accel;
  const double accel_penalty = params_->accel_penalty();
  const double decel_penalty = params_->decel_penalty();

  if (accel > 0.0) {
    double accel_penalty_factor = 1.0;
    if (accel > params_->large_accel_thres()) {
      accel_penalty_factor = speed_large_accel_penalty_factor_plf_(speed);
    }
    cost = accel_penalty_factor * accel_penalty * accel_sq;
  } else {
    cost = decel_penalty * accel_sq;
  }

  return cost * unit_t_;
}

}  // namespace qcraft::planner
