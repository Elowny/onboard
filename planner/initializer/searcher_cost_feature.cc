#include "onboard/planner/initializer/searcher_cost_feature.h"

#include <array>
#include <cmath>
#include <optional>
#include <ostream>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
// If speed lower than kIgnoreLateralAccelSpeedLimit, do not add lateral
// acceleration cost.
constexpr double kIgnoreLateralAccelSpeedLimit = 2.0;  // m/s
constexpr double kLateralAccelHardLimit = 2.5;         // m/s^2
constexpr double kLateralAccelSoftLimit = 1.0;         // m/s^2
constexpr double kInvLateralAccelSoftLimit = 1.0 / kLateralAccelSoftLimit;
constexpr double kIgnoreLateralJerkSpeedLimit = 3.0;  // m/s.
constexpr double kLateralJerkLimit = 2.0;             // m/s^3
constexpr double kInvLateralJerkLimit = 1.0 / kLateralJerkLimit;

constexpr double kCurvatureRegularizer = 0.1;  // m^-1
constexpr double kInvCurvatureRegularizer = 1.0 / kCurvatureRegularizer;

constexpr double kRefSpeedRegularizer = 3.0;  // m/s
constexpr double kInvRefSpeedRegularizer = 1.0 / kRefSpeedRegularizer;
// For leading object.
constexpr std::array<double, 11> kInterpT = {0.0, 0.9, 1.9, 2.9, 3.9, 4.9,
                                             5.9, 6.9, 7.9, 8.9, 9.9};
constexpr int kGroupLen = 10;
constexpr int kMaxGroupSize = prediction::kPredictionPointNum / kGroupLen;

// Curvature cost based on state velocity.
const PiecewiseLinearFunction<double> kCurvatureVelocityPlf(
    {1.0, 2.0, 3.0, 4.0, 5.0}, {0.05, 0.1, 0.2, 0.2, 1.0});

enum TransformType {
  kL1Norm = 0,
  kL2Norm = 1,
};

template <TransformType type>
double Transform(double val) {
  switch (type) {
    case kL1Norm:
      return std::fabs(val);
    case kL2Norm:
      return Sqr(val);
  }
}

inline void SetZeroCost(absl::Span<double> cost) {
  for (auto& val : cost) val = 0.0;
}

std::array<double, 5> CalculatePathSlCostRatio(double s, double l,
                                               const PathSlBoundary& path_sl,
                                               double sdc_half_width) {
  constexpr double kEpsilon = 1e-8;
  const double ref_center_l = path_sl.QueryReferenceCenterL(s);
  const double ref_l_ratio =
      std::clamp((l - ref_center_l) * (1.0 / kDefaultHalfLaneWidth), -1.0, 1.0);

  const auto [l_target_right, l_target_left] = path_sl.QueryTargetBoundaryL(s);
  const double target_right_border = sdc_half_width + l_target_right;
  const double target_left_border = l_target_left - sdc_half_width;
  const double target_right_ratio = std::clamp(
      (ref_center_l - l) / (ref_center_l - target_right_border + kEpsilon), 0.0,
      1.0);
  const double target_left_ratio = std::clamp(
      (l - ref_center_l) / (target_left_border - ref_center_l + kEpsilon), 0.0,
      1.0);

  const auto [l_right, l_left] = path_sl.QueryBoundaryL(s);
  const double right_border = sdc_half_width + l_right;
  const double left_border = l_left - sdc_half_width;
  const double right_ratio = std::clamp(
      (ref_center_l - l) / (ref_center_l - right_border + kEpsilon), 0.0, 1.0);
  const double left_ratio = std::clamp(
      (l - ref_center_l) / (left_border - ref_center_l + kEpsilon), 0.0, 1.0);

  return {ref_l_ratio, target_right_ratio, target_left_ratio, right_ratio,
          left_ratio};
}

absl::StatusOr<double> FilterAndGetBoxMinS(const DrivePassage& passage,
                                           const Box2d& box) {
  ASSIGN_OR_RETURN(
      const auto frenet_box, passage.QueryFrenetBoxAt(box),
      absl::NotFoundError("Cannot project the object box onto drive passage."));
  ASSIGN_OR_RETURN(
      const auto passage_tangent,
      passage.QueryTangentAtS(frenet_box.center_s()),
      absl::NotFoundError("Cannot find passage tangent near the object box."));

  if (std::abs(NormalizeAngle(passage_tangent.FastAngle() - box.heading())) >
      M_PI_2) {
    return absl::NotFoundError("Oncoming object ignored for leading object.");
  }

  return frenet_box.s_min;
}

PiecewiseLinearFunction<double> GetTrajTimeAccumulatedMinSPlf(
    const DrivePassage& drive_passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<std::string>& leading_trajs) {
  // Get a time - accumulated_s correspondence of a group of object.
  std::vector<double> ty(kInterpT.size(),
                         std::numeric_limits<double>::infinity());
  for (const auto& traj_id : leading_trajs) {
    const auto states =
        QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id))->states();
    const auto box_min_s_or =
        FilterAndGetBoxMinS(drive_passage, states.front().box);
    if (box_min_s_or.ok()) {
      ty[0] = std::min(ty[0], *box_min_s_or);
    }

    for (int i = 1; i <= kMaxGroupSize; ++i) {
      const int state_idx = i * kGroupLen - 1;
      if (state_idx >= states.size()) break;

      ASSIGN_OR_CONTINUE(
          const auto box_min_s,
          FilterAndGetBoxMinS(drive_passage, states[state_idx].box));
      ty[i] = std::min(ty[i], box_min_s);
    }
  }
  return PiecewiseLinearFunction({kInterpT.begin(), kInterpT.end()},
                                 std::move(ty));
}

absl::StatusOr<double> FilterAndGetBoxMaxS(const DrivePassage& passage,
                                           const Box2d& box) {
  ASSIGN_OR_RETURN(
      const auto frenet_box, passage.QueryFrenetBoxAt(box),
      absl::NotFoundError("Cannot project the object box onto drive passage."));
  ASSIGN_OR_RETURN(
      const auto passage_tangent,
      passage.QueryTangentAtS(frenet_box.center_s()),
      absl::NotFoundError("Cannot find passage tangent near the object box."));

  if (std::abs(NormalizeAngle(passage_tangent.FastAngle() - box.heading())) >
      M_PI_2) {
    return absl::NotFoundError("Oncoming object ignored for leading object.");
  }

  return frenet_box.s_max;
}

double GetOvertakeLeadingTrajBehindMinS(
    double traj_horizon, const DrivePassage& drive_passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<std::vector<std::string>>& leading_groups,
    const int leading_group_idx) {
  // Get max s of leading trajectories behind, also min s to overtake leading
  // trajectories behind for ego
  double traj_behind_max_s = std::numeric_limits<double>::lowest();
  for (int i = 0; i < leading_group_idx; ++i) {
    for (const auto& traj_id : leading_groups[i]) {
      const auto states =
          QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id))->states();
      const int traj_end_idx =
          std::min(static_cast<int>(states.size()),
                   CeilToInt(traj_horizon / prediction::kPredictionTimeStep)) -
          1;
      ASSIGN_OR_CONTINUE(
          const auto box_max_s,
          FilterAndGetBoxMaxS(drive_passage, states[traj_end_idx].box));
      traj_behind_max_s = std::max(traj_behind_max_s, box_max_s);
    }
  }
  return traj_behind_max_s;
}

}  // namespace

// --------------- Acceleration Cost --------------------
void AccelerationFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                          absl::Span<double> cost) const {
  const auto get_normalize_a = [this](double a) {
    return a < 0.0 ? max_decel_constraint_ : max_accel_constraint_;
  };
  DCHECK_EQ(cost.size(), 1);
  const auto& states = edge_info.equal_interval_states;
  DCHECK_GT(states.size(), 1);
  cost[0] = 0.0;
  for (int i = 0; i < states.size(); ++i) {
    const double cur_cost = Transform<TransformType::kL2Norm>(
        states[i].a / get_normalize_a(states[i].a));
    const double factor = (i == 0 || i == states.size() - 1) ? 1.0 : 2.0;
    cost[0] += (factor * cur_cost);
  }
  // Trapezodial rule of integration = 0.5 * dt * (f(0) + 2f(1) + ... +
  // 2f(n-2)+f(n-1))
  const double dt = states[1].t - states[0].t;
  cost[0] = 0.5 * dt * cost[0];
}

//--------------- Lane Boundary Cost -----------------
void LaneBoundaryFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                          absl::Span<double> cost) const {
  DCHECK_EQ(cost.size(), 3);
  SetZeroCost(cost);
  const auto& states = edge_info.equal_interval_states;
  DCHECK_GT(states.size(), 1);
  for (int i = 0; i < states.size(); ++i) {
    const auto l2ratios = CalculatePathSlCostRatio(
        states[i].accumulated_s, states[i].l, *path_sl_, sdc_half_width_);
    const double ref_l_ratio = l2ratios[0];
    const double cur_cost0 = Transform<TransformType::kL2Norm>(ref_l_ratio);
    const double target_right_ratio = l2ratios[1];
    const double target_left_ratio = l2ratios[2];
    const double cur_cost1 =
        Transform<TransformType::kL2Norm>(target_right_ratio) +
        Transform<TransformType::kL2Norm>(target_left_ratio);
    const double right_ratio = l2ratios[3];
    const double left_ratio = l2ratios[4];
    const double cur_cost2 = Transform<TransformType::kL2Norm>(right_ratio) +
                             Transform<TransformType::kL2Norm>(left_ratio);
    const double factor = (i == 0 || i == states.size() - 1) ? 1.0 : 2.0;
    cost[0] += (factor * cur_cost0);
    cost[1] += (factor * cur_cost1);
    cost[2] += (factor * cur_cost2);
  }
  const double dt = states[1].t - states[0].t;
  for (int i = 0; i < 3; ++i) {
    cost[i] = 0.5 * dt * cost[i];
  }
}

//--------------- Curvature Cost -----------------
void CurvatureFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                       absl::Span<double> cost) const {
  DCHECK_EQ(cost.size(), 1);
  cost[0] = 0.0;
  const auto& states = edge_info.equal_interval_states;
  // Normal handling of curvature.
  for (int i = 0; i < states.size(); ++i) {
    const double relax_factor = kCurvatureVelocityPlf(states[i].v);
    const double cur_cost =
        Transform<TransformType::kL2Norm>((states[i].k - states[i].ref_k) *
                                          kInvCurvatureRegularizer) *
        relax_factor;
    const double factor = (i == 0 || i == states.size() - 1) ? 1.0 : 2.0;
    cost[0] += (factor * cur_cost);
  }
  // Trapezodial rule of integration = 0.5 * dt * (f(0) + 2f(1) + ... +
  // 2f(n-2)+f(n-1))
  const double dt = states[1].t - states[0].t;
  cost[0] = 0.5 * dt * cost[0];
}

//--------------- Lateral acceleration Cost -----------------
void LateralAccelerationFeatureCost::ComputeCost(
    const MotionEdgeInfo& edge_info, absl::Span<double> cost) const {
  DCHECK_EQ(cost.size(), 3);
  SetZeroCost(cost);
  const auto& states = edge_info.equal_interval_states;
  DCHECK_GT(states.size(), 1);
  const double duration = states.back().t - states.front().t;
  const double dt = states[1].t - states[0].t;

  // Compute lateral acceleration cost.
  for (int i = 0; i < states.size(); ++i) {
    if (states[i].v < kIgnoreLateralAccelSpeedLimit) continue;

    const double abs_lat_acc =
        std::abs((states[i].k - states[i].ref_k) * Sqr(states[i].v));
    if (abs_lat_acc > kLateralAccelHardLimit) {
      cost[0] = 1.0 * duration;
    }
    double cur_lat_cost = 0.0;
    if (abs_lat_acc > kLateralAccelSoftLimit) {
      cur_lat_cost = Transform<TransformType::kL2Norm>(
          std::max(0.0, abs_lat_acc * kInvLateralAccelSoftLimit - 1.0));
    }

    const double factor = (i == 0 || i == states.size() - 1) ? 1.0 : 2.0;
    cost[1] += (factor * cur_lat_cost);
  }
  cost[1] = 0.5 * dt * cost[1];

  // Compute lateral jerk cost(in sl coordinate, dddl in fact).
  if (!is_lane_change_) {
    for (int i = 0; i < states.size(); ++i) {
      const double dddl = states[i].dddl.has_value() ? (*states[i].dddl) : 0;
      const double jerk_ratio =
          states[i].v < kIgnoreLateralJerkSpeedLimit
              ? 0.0
              : std::clamp(dddl * kInvLateralJerkLimit, -1.0, 1.0);
      const double cur_cost = Transform<TransformType::kL2Norm>(jerk_ratio);

      const double factor = (i == 0 || i == states.size() - 1) ? 1.0 : 2.0;
      cost[2] += (factor * cur_cost);
    }
    cost[2] = 0.5 * dt * cost[2];
  }
}

//--------------- Stop Constraint Cost -----------------
void StopConstraintFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                            absl::Span<double> cost) const {
  DCHECK_EQ(cost.size(), 1);
  const auto& states = edge_info.equal_interval_states;
  DCHECK_GT(states.size(), 1);
  const double last_s = states.back().accumulated_s;
  cost[0] = std::max(0.0, last_s - nearest_stop_s_);
}

// -------------------- RefSpeedFeatureCost --------------------
void RefSpeedFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                      absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 3);
  SetZeroCost(cost);

  const auto& states = edge_info.equal_interval_states;
  DCHECK_GT(states.size(), 1);

  // This cost is time consuming, evaluate every kEvalStep point to reduce time
  // consumption.
  for (int i = 0; i < states.size(); ++i) {
    const double t = states[i].t + edge_info.start_t;
    const double accumulated_s = states[i].accumulated_s;
    // TODO(xiangjun): change to batch lookup.
    const auto [v_limit, v_ref] =
        ref_speed_table_->LookUpRefSpeed(t, accumulated_s);
    const double v = states[i].v;

    const double factor = (i == 0 || i == states.size() - 1) ? 1.0 : 2.0;
    // exceeds speed limit
    cost[0] += factor * Transform<TransformType::kL2Norm>(
                            v > v_limit ? v - v_limit : 0.0);
    // exceeds ref speed
    const double exceed_speed = std::max(v - v_ref, 0.0);
    cost[1] += factor * Transform<TransformType::kL2Norm>(std::clamp(
                            exceed_speed * kInvRefSpeedRegularizer, 0.0, 1.0));
    // below ref speed
    const double below_speed = std::max(v_ref - v, 0.0);
    cost[2] += factor * Transform<TransformType::kL2Norm>(std::clamp(
                            below_speed * kInvRefSpeedRegularizer, 0.0, 1.0));
  }
  const double dt = states[1].t - states[0].t;
  for (int i = 0; i < 3; ++i) {
    cost[i] = 0.5 * dt * cost[i];
  }
}

// -------------------- DynamicCollisionFeatureCost --------------------
DynamicCollisionFeatureCost::DynamicCollisionFeatureCost(
    const CollisionChecker* collision_checker)
    : FeatureCost("dynamic_collision"), cc_(CHECK_NOTNULL(collision_checker)) {}

void DynamicCollisionFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                              absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 1);
  CollisionInfo collision_info;
  const auto start_time = edge_info.start_t;
  cc_->CheckCollisionWithTrajectories(start_time,
                                      edge_info.const_interval_states,
                                      /*ignored_trajs=*/{}, &collision_info);
  if (collision_info.collision_objects.empty()) {
    cost[0] = 0.0;
  } else {
    cost[0] = 1.0;
  }
}

IgnoreTrajMap DynamicCollisionFeatureCost::ComputeInteractiveCost(
    const MotionEdgeInfo& edge_info, const IgnoreTrajMap& ignored_trajs,
    absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 1);
  CollisionInfo collision_info;
  const auto start_time = edge_info.start_t;
  cc_->CheckCollisionWithTrajectories(start_time,
                                      edge_info.const_interval_states,
                                      ignored_trajs, &collision_info);
  cost[0] = 0.0;
  auto new_ignored_trajs = ignored_trajs;
  for (const auto& collision_obj : collision_info.collision_objects) {
    if (collision_obj.collision_configuration.has_value()) {
      if (*collision_obj.collision_configuration !=
          CollisionConfiguration::BACK) {
        cost[0] = 1.0;
      } else {
        // Add ignorable trajectories into the hash set.
        new_ignored_trajs[collision_obj.traj->traj_id()] =
            CollisionConfigurationInfo{
                .time_idx = collision_obj.time,
                .collision_config = *collision_obj.collision_configuration,
            };
      }
    }
  }
  return new_ignored_trajs;
}

// ----------------------- LeadingObjectFeatureCost---------------------
LeadingObjectFeatureCost::LeadingObjectFeatureCost(
    double traj_horizon, const DrivePassage& drive_passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<std::vector<std::string>>& leading_groups,
    const int leading_group_idx, double ego_front_to_ra, double ego_back_to_ra)
    : FeatureCost("leading_object"),
      traj_horizon_(traj_horizon),
      leading_group_idx_(leading_group_idx),
      ego_front_to_ra_(ego_front_to_ra),
      ego_back_to_ra_(ego_back_to_ra) {
  max_s_t_ = GetTrajTimeAccumulatedMinSPlf(drive_passage, st_traj_mgr,
                                           leading_groups[leading_group_idx]);
  min_s_ =
      GetOvertakeLeadingTrajBehindMinS(traj_horizon, drive_passage, st_traj_mgr,
                                       leading_groups, leading_group_idx);
}

void LeadingObjectFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                           absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 1);
  cost[0] = 0.0;
  const auto& states = edge_info.equal_interval_states;
  for (int i = 0; i < states.size(); ++i) {
    const double t = states[i].t + edge_info.start_t;
    const double max_s = max_s_t_(t);
    if (states[i].accumulated_s + ego_front_to_ra_ +
            kInitializerMinFollowDistance >
        max_s) {
      cost[0] = 1.0;
      return;
    }
  }

  // return if following just in front of the ego
  if (leading_group_idx_ == 0) return;

  // cost for not overtaking leading group behind
  constexpr double kInitializerMinFollowedDistance = 1.0;
  const double end_time = edge_info.start_t + edge_info.motion_form->duration();
  if (end_time >= traj_horizon_) {
    const auto end_state =
        edge_info.motion_form->State(traj_horizon_ - edge_info.start_t);
    if (end_state.accumulated_s - ego_back_to_ra_ -
            kInitializerMinFollowedDistance <
        min_s_) {
      cost[0] = 1.0;
      return;
    }
  }
}

// ----------------------- AstarLeadingObjectFeatureCost---------------------
AstarLeadingObjectFeatureCost::AstarLeadingObjectFeatureCost(
    double traj_horizon, const DrivePassage& drive_passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<std::vector<std::string>>& leading_groups,
    const int leading_group_idx, const bool able_to_overtake_leading_behind,
    double ego_front_to_ra, double ego_back_to_ra)
    : FeatureCost("leading_object"),
      traj_horizon_(traj_horizon),
      leading_group_idx_(leading_group_idx),
      ego_front_to_ra_(ego_front_to_ra),
      ego_back_to_ra_(ego_back_to_ra),
      able_to_overtake_leading_behind_(able_to_overtake_leading_behind) {
  max_s_t_ = GetTrajTimeAccumulatedMinSPlf(drive_passage, st_traj_mgr,
                                           leading_groups[leading_group_idx]);
  min_s_ =
      GetOvertakeLeadingTrajBehindMinS(traj_horizon, drive_passage, st_traj_mgr,
                                       leading_groups, leading_group_idx);
}

void AstarLeadingObjectFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                                absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 1);
  cost[0] = 0.0;
  const auto& states = edge_info.equal_interval_states;
  for (int i = 0; i < states.size(); ++i) {
    const double t = states[i].t + edge_info.start_t;
    const double max_s = max_s_t_(t);
    if (states[i].accumulated_s + ego_front_to_ra_ +
            kInitializerMinFollowDistance >
        max_s) {
      cost[0] = 1.0;
      return;
    }
  }

  // return if following just in front of the ego
  if (leading_group_idx_ == 0 || !able_to_overtake_leading_behind_) return;

  // cost for not overtaking leading group behind
  constexpr double kInitializerMinFollowedDistance = 1.0;
  const double end_time = edge_info.start_t + edge_info.motion_form->duration();
  if (end_time >= traj_horizon_) {
    const auto end_state =
        edge_info.motion_form->State(traj_horizon_ - edge_info.start_t);
    if (end_state.accumulated_s - ego_back_to_ra_ -
            kInitializerMinFollowedDistance <
        min_s_) {
      cost[0] = 1.0;
      return;
    }
  }
}

// ----------------------- RefTrajectoryFeatureCost---------------------
void RefTrajectoryFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                           absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 1);
  cost[0] = 0.0;

  if (captain_net_output_->traj_points.empty()) return;

  const auto start_it =
      std::lower_bound(captain_net_output_->traj_points.begin(),
                       captain_net_output_->traj_points.end(),
                       edge_info.start_t, [](const auto& traj_pt, double time) {
                         return traj_pt.relative_time() < time;
                       });
  const double end_time = edge_info.start_t + edge_info.motion_form->duration();
  const auto end_it =
      std::upper_bound(captain_net_output_->traj_points.begin(),
                       captain_net_output_->traj_points.end(), end_time,
                       [](double time, const auto& traj_pt) {
                         return time < traj_pt.relative_time();
                       });
  if (start_it >= end_it) return;

  if (FLAGS_planner_capnet_ref_traj_use_mahalanobis_distance) {
    for (auto it = start_it; it < end_it; ++it) {
      const auto state =
          edge_info.motion_form->State(it->relative_time() - edge_info.start_t);
      auto index = it - captain_net_output_->traj_points.begin();
      cost[0] +=
          ref_traj_weight_[it - captain_net_output_->traj_points.begin()] *
          state.xy.MahalanobisDistanceTo(
              it->path_point().x(), it->path_point().y(),
              captain_net_output_->sdxs[index],
              captain_net_output_->sdys[index],
              captain_net_output_->covariances[index]);
    }
  } else {
    for (auto it = start_it; it < end_it; ++it) {
      const auto state =
          edge_info.motion_form->State(it->relative_time() - edge_info.start_t);
      cost[0] +=
          ref_traj_weight_[it - captain_net_output_->traj_points.begin()] *
          state.xy.DistanceTo(Vec2dFromApolloTrajectoryPointProto(*it));
    }
  }
  cost[0] /= reg_factor_;
}

// ----------------------- FinalProgressFeatureCost---------------------
void FinalProgressFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                           absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 2);
  cost[0] = cost[1] = 0.0;
  const double end_time = edge_info.start_t + edge_info.motion_form->duration();
  if (end_time >= traj_horizon_) {
    // Only calculate final progress cost when end time greater than 10.0s.
    // Longitudinal progress cost.
    const auto end_state =
        edge_info.motion_form->State(traj_horizon_ - edge_info.start_t);
    constexpr double kStandardSDiffRatio = 0.2;
    cost[0] = std::max((1.0 - kStandardSDiffRatio) * max_accumulated_s_ -
                           end_state.accumulated_s,
                       0.0) /
              (kStandardSDiffRatio * max_accumulated_s_);
    // Lateral progress cost.
    const double ref_center_l =
        path_sl_->QueryReferenceCenterL(end_state.accumulated_s);
    constexpr double kEpsilon = 0.1;  // m.
    if (edge_info.motion_form->geometry() != nullptr &&
        std::abs(edge_info.motion_form->geometry()->states().back().l -
                 ref_center_l) < kEpsilon) {
      // Destination geometry node on ref center, no lateral cost here.
      cost[1] = 0.0;
    } else {
      cost[1] =
          std::abs(end_state.l - ref_center_l) * (1.0 / kDefaultHalfLaneWidth);
    }
  }
}
}  // namespace qcraft::planner
