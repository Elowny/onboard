#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SEGMENTED_SPEED_LIMIT_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SEGMENTED_SPEED_LIMIT_COST_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"

namespace qcraft {
namespace planner {

// TODO(Fang) use hysteresis in gain switches.
template <typename PROB>
class SegmentedSpeedLimitCost : public Cost<PROB> {
 public:
  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  using GType = typename PROB::GType;
  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;
  using DDGDxDxType = typename PROB::DDGDxDxType;
  using DDGDxDuType = typename PROB::DDGDxDuType;
  using DDGDuDxType = typename PROB::DDGDuDxType;
  using DDGDuDuType = typename PROB::DDGDuDuType;

  using DividedG = typename Cost<PROB>::DividedG;

  static constexpr double kNormalizedScale = 0.01;
  explicit SegmentedSpeedLimitCost(
      int horizon, const std::vector<Vec2d>& ref_points,
      std::vector<double> speed_limits,
      const std::vector<Vec2d>& free_ref_points,
      std::vector<double> free_speed_limits, int free_index,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "SegmentedSpeedLimitCost"),
      double scale = 1.0, double stop_speed_gain = kDefaultStopSpeedCostGain,
      double over_speed_gain = kDefaultOverSpeedCostGain,
      double under_speed_gain = kDefaultUnderSpeedCostGain,
      bool use_qtfm = false)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale),
        horizon_(horizon),
        speed_limits_(std::move(speed_limits)),
        free_speed_limits_(std::move(free_speed_limits)),
        free_index_(free_index),
        stop_speed_gain_(stop_speed_gain),
        over_speed_gain_(over_speed_gain),
        under_speed_gain_(under_speed_gain) {
    QCHECK_GT(horizon_, 0);
    QCHECK_GT(ref_points.size(), 1);
    QCHECK_EQ(ref_points.size(), speed_limits_.size() + 1);
    QCHECK_GT(free_ref_points.size(), 1);
    QCHECK_EQ(free_ref_points.size(), free_speed_limits_.size() + 1);
    if (use_qtfm) {
      ref_path_ = std::make_unique<QtfmEnhancedKdTreeFrenetFrame>(
          BuildQtfmEnhancedKdTreeFrenetFrame(ref_points,
                                             /*down_sample_raw_points=*/true)
              .value());
    } else {
      ref_path_ = std::make_unique<KdTreeFrenetFrame>(
          BuildKdTreeFrenetFrame(ref_points, /*down_sample_raw_points=*/true)
              .value());
    }
    // QTFM frenet frame is not covering speed limits beyond the drive passage
    // as there shouldn't be many querry outside drive passage.
    free_ref_path_ = std::make_unique<KdTreeFrenetFrame>(
        BuildKdTreeFrenetFrame(free_ref_points, /*down_sample_raw_points=*/true)
            .value());

    speed_diff_.resize(horizon_);
    gain_.resize(horizon_);
    speed_limit_factor_.resize(horizon_);
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    DCHECK_EQ(horizon, horizon_);
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon_; ++k) {
      res.AddSubG(/*idx=*/0, gain_[k] * Sqr(speed_diff_[k]));
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    return 0.5 * Cost<PROB>::scale() * gain_[k] * Sqr(speed_diff_[k]);
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    const double speed_diff_k = speed_diff_[k];
    const auto& speed_limit_factor_k = speed_limit_factor_[k];
    const double dgdv = Cost<PROB>::scale() * gain_[k] * speed_diff_k;
    (*dgdx)[PROB::kStateVIndex] += dgdv;
    (*dgdx).template segment<2>(PROB::kStateXIndex) +=
        dgdv * speed_limit_factor_k;
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    const double gain = Cost<PROB>::scale() * gain_[k];
    const auto& speed_limit_factor_k = speed_limit_factor_[k];
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateVIndex) += gain;
    const Vec2d ddgdvdxy = gain * speed_limit_factor_k;
    const Vec2d factor_multi =
        Vec2d(Sqr(speed_limit_factor_k.x()), Sqr(speed_limit_factor_k.y()));
    const Vec2d ddgdxydxy = gain * factor_multi;
    const double ddgdxdy =
        gain * speed_limit_factor_k.x() * speed_limit_factor_k.y();
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateXIndex) = ddgdvdxy[0];
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateYIndex) = ddgdvdxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateVIndex) = ddgdvdxy[0];
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateVIndex) = ddgdvdxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateXIndex) = ddgdxydxy[0];
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateYIndex) = ddgdxydxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateYIndex) = ddgdxdy;
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateXIndex) = ddgdxdy;
  }
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

  void Update(const StatesType& xs, const ControlsType& /*us*/,
              int horizon) override {
    DCHECK_EQ(horizon, horizon_);
    // Compensation for low forward force when speed limit is low, by increasing
    // the under-speed gain.
    constexpr double kEps = 1e-7;
    const std::vector<double>& s_knot = ref_path_->s_knots();
    for (int k = 0; k < horizon_ && k <= free_index_; ++k) {
      const Vec2d pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha = 0.0;
      ref_path_->XYToSL(pos, &sl, &normal, &index_pair, &alpha);
      const int next_index = std::min(
          index_pair.second, static_cast<int>(speed_limits_.size() - 1));
      const double speed_limit = Lerp(speed_limits_[index_pair.first],
                                      speed_limits_[next_index], alpha);
      const double speed_diff = PROB::v(xs, k) - speed_limit;
      speed_diff_[k] = PROB::v(xs, k) - speed_limit;
      const double length = s_knot[next_index] - s_knot[index_pair.first];
      if (length < kEps) {
        speed_limit_factor_[k] = Vec2d(0.0, 0.0);
      } else {
        speed_limit_factor_[k] =
            (speed_limits_[next_index] - speed_limits_[index_pair.first]) *
            normal.Perp() / length;
      }
      if (speed_limit == 0.0) {
        gain_[k] = stop_speed_gain_;
      } else {
        gain_[k] = speed_diff > 0.0 ? over_speed_gain_ : under_speed_gain_;
      }
    }
    const std::vector<double>& free_s_knot = free_ref_path_->s_knots();
    for (int k = free_index_ + 1; k < horizon_; ++k) {
      const Vec2d pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha = 0.0;
      free_ref_path_->XYToSL(pos, &sl, &normal, &index_pair, &alpha);
      const int next_index = std::min(
          index_pair.second, static_cast<int>(free_speed_limits_.size() - 1));
      const double speed_limit = Lerp(free_speed_limits_[index_pair.first],
                                      free_speed_limits_[next_index], alpha);
      const double speed_diff = PROB::v(xs, k) - speed_limit;
      speed_diff_[k] = PROB::v(xs, k) - speed_limit;
      const double length =
          free_s_knot[next_index] - free_s_knot[index_pair.first];
      if (length < kEps) {
        speed_limit_factor_[k] = Vec2d(0.0, 0.0);
      } else {
        speed_limit_factor_[k] = (free_speed_limits_[next_index] -
                                  free_speed_limits_[index_pair.first]) *
                                 normal.Perp() / length;
      }
      if (speed_limit == 0.0) {
        gain_[k] = stop_speed_gain_;
      } else {
        gain_[k] = speed_diff > 0.0 ? over_speed_gain_ : under_speed_gain_;
      }
    }
  }

 protected:
  static constexpr double kDefaultStopSpeedCostGain = 800.0;
  static constexpr double kDefaultOverSpeedCostGain = 100.0;
  static constexpr double kDefaultUnderSpeedCostGain = 1.0;

 private:
  int horizon_ = 0;
  std::vector<double> speed_limits_;
  std::vector<double> free_speed_limits_;
  int free_index_;  // State index beyond this value uses free speed limit.
  double stop_speed_gain_;
  double over_speed_gain_;
  double under_speed_gain_;
  std::unique_ptr<FrenetFrame> ref_path_;
  std::unique_ptr<FrenetFrame> free_ref_path_;

  // States.
  std::vector<double> speed_diff_;
  std::vector<double> gain_;
  std::vector<Vec2d> speed_limit_factor_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SEGMENTED_SPEED_LIMIT_COST_H_
