#include "onboard/prediction/post_process/svt_feature_cost.h"

// IWYU pragma: no_include <stddef.h>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/planner/speed/speed_point.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kZeroVelocityEpsilon = 1e-3;
constexpr double kMinTimeDuration = 1e-3;
constexpr int kEvaluateStep = 2;

constexpr double kRelativeSpeedRegularizer = 4.0;
const double kInvRelativeSpeedRegularizer = 1.0 / kRelativeSpeedRegularizer;
}  // namespace

// ------------------ Object Collision Feature ------------------
void SvtObjectCollisionFeatureCost::ComputeFeatureCost(
    const std::vector<SvtState>& states, absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 1);  // Stationary.
  // Stationary collision cost.
  cost[0] = 0.0;
  if (!stationary_objects_.empty()) {
    if (states.size() == 1) {
      const auto& state = states.front();
      double pass_ratio = (min_s_ - state.s - stationary_follow_distance_) *
                          inv_stationary_follow_distance_;
      pass_ratio = pass_ratio > 0.0 ? 0.0 : pass_ratio;
      double rel_v_regularized = state.v * kInvRelativeSpeedRegularizer;
      rel_v_regularized = rel_v_regularized > 0.0 ? rel_v_regularized : 0.0;
      cost[0] =
          Sqr(pass_ratio) * (1.0 + Sqr(rel_v_regularized)) * kMinTimeDuration;
    } else {
      for (size_t i = 0; i + 1 < states.size(); ++i) {
        const auto& state = states[i];
        const auto& next_state = states[i + 1];
        const double dt = next_state.t - state.t;
        double pass_ratio = (min_s_ - state.s - stationary_follow_distance_) *
                            inv_stationary_follow_distance_;
        pass_ratio = pass_ratio > 0.0 ? 0.0 : pass_ratio;
        double rel_v_regularized = state.v * kInvRelativeSpeedRegularizer;
        rel_v_regularized = rel_v_regularized > 0.0 ? rel_v_regularized : 0.0;
        cost[0] += Sqr(pass_ratio) * (1.0 + Sqr(rel_v_regularized)) * dt;
      }
    }
  }
}

// ------------------ Stopline Feature ------------------
void SvtStoplineFeatureCost::ComputeFeatureCost(
    const std::vector<SvtState>& states, absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 1);  // only tl red for now.
  cost[0] = 0.0;
  if (stoplines_.empty()) return;
  if (states.size() == 1 && (std::fabs(states[0].v) < kZeroVelocityEpsilon)) {
    // Only one state and it's stationary.
    cost[0] = stoplines_.front() < states[0].s ? kMinTimeDuration : 0.0;
  }
  for (size_t i = 1; i < states.size(); ++i) {
    const double dt = states[i].t - states[i - 1].t;
    cost[0] += stoplines_.front() < states[i].s ? dt : 0.0;
  }
}

// ------------------ Reference Speed Feature ------------------
void SvtReferenceSpeedFeatureCost::ComputeFeatureCost(
    const std::vector<SvtState>& states, absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 2);
  cost[0] = 0.0;
  cost[1] = 0.0;

  const int num_states = states.size();
  if (num_states == 1) {
    const auto speed_point_or = ref_speed_->EvaluateByS(
        std::min<double>(states.front().s, total_length_));
    const auto v_diff = (*speed_point_or).v() - states.front().v;
    cost[0] += kMinTimeDuration * Sqr(v_diff * inv_average_v_);
    return;
  }

  for (int i = 0; i < num_states; i += kEvaluateStep) {
    const auto& state = states[i];
    const int next_state_idx = std::min<int>(i + kEvaluateStep, num_states - 1);
    const auto dt = states[next_state_idx].t - state.t;
    const auto speed_point_or =
        ref_speed_->EvaluateByS(std::min<double>(state.s, total_length_));
    const auto v_diff = (*speed_point_or).v() - state.v;
    cost[0] += dt * Sqr(v_diff * inv_average_v_);
  }
}

}  // namespace prediction
}  // namespace qcraft
