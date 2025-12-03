#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_REFERENCE_SINGLE_STATE_DEVIATION_COST_H_  // NOLINT
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_REFERENCE_SINGLE_STATE_DEVIATION_COST_H_  // NOLINT

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/strings/str_cat.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class ReferenceSingleStateDeviationCost : public Cost<PROB> {
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

  using CostType = typename Cost<PROB>::CostType;

  static constexpr double kNormalizedScale = 1.0;
  ReferenceSingleStateDeviationCost(
      StateType ref_state, int ref_index, std::vector<double> weights,
      std::vector<double> base_numbers,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "ReferenceSingleStateDeviationCost"),
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        ref_state_(std::move(ref_state)),
        ref_index_(ref_index),
        weights_(std::move(weights)),
        base_numbers_(std::move(base_numbers)) {
    QCHECK_EQ(ref_state_.size(), weights_.size());
    QCHECK_EQ(ref_state_.size(), base_numbers_.size());
    ks_.resize(PROB::kStateSize, 0.0);
    bs_.resize(PROB::kStateSize, 0.0);
    for (int i = 0; i < PROB::kStateSize; ++i) {
      const double base_number = base_numbers_[i];
      QCHECK_GE(base_number, 0.0);
      ks_[i] = 2.0 * base_number;
      bs_[i] = -Sqr(base_number);
    }
  }

  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& /*us*/,
                           int horizon) const override {
    DCHECK_LT(ref_index_, horizon);
    DividedG res(/*size=*/1);
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    StateType x_diff = PROB::GetStateAtStep(xs, ref_index_) - ref_state_;
    x_diff[PROB::kStateThetaIndex] =
        NormalizeAngle(x_diff[PROB::kStateThetaIndex]);
    for (int i = 0; i < PROB::kStateSize; ++i) {
      const double diff = x_diff[i];
      const double base_number = base_numbers_[i];
      const double k = ks_[i];
      const double b = bs_[i];
      if (std::abs(diff) < base_number) {
        res.AddSubG(/*idx=*/0, Sqr(x_diff[i]) * weights_[i]);
      } else if (diff > 0.0) {
        res.AddSubG(/*idx=*/0, (k * diff + b) * weights_[i]);
      } else {
        res.AddSubG(/*idx=*/0, (-k * diff + b) * weights_[i]);
      }
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // f1(x_diff) = 0.5 * w * x_diff^2         abs(x_diff) <= base;
  // f2(x_diff) = 0.5 * w * (k * x_diff + b)  x_diff > base;
  // f3(x_diff) = 0.5 * w * (-k * x_diff + b) x_diff < -base;
  // f1'(base) = f2'(base) -> k, b.
  double EvaluateG(int k, const StateType& x,
                   const ControlType& /*u*/) const override {
    if (k != ref_index_) {
      return 0.0;
    }
    StateType x_diff = x - ref_state_;
    x_diff[PROB::kStateThetaIndex] =
        NormalizeAngle(x_diff[PROB::kStateThetaIndex]);
    double g = 0.0;
    for (int i = 0; i < PROB::kStateSize; ++i) {
      const double diff = x_diff[i];
      const double base_number = base_numbers_[i];
      const double k = ks_[i];
      const double b = bs_[i];
      if (std::abs(diff) < base_number) {
        g += Sqr(x_diff[i]) * weights_[i];
      } else if (diff > 0.0) {
        g += (k * diff + b) * weights_[i];
      } else {
        g += (-k * diff + b) * weights_[i];
      }
    }
    g *= 0.5 * Cost<PROB>::scale();

    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& x, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    if (k != ref_index_) {
      return;
    }
    StateType x_diff = x - ref_state_;
    x_diff[PROB::kStateThetaIndex] =
        NormalizeAngle(x_diff[PROB::kStateThetaIndex]);
    for (int i = 0; i < PROB::kStateSize; ++i) {
      const double diff = x_diff[i];
      const double base_number = base_numbers_[i];
      const double k = ks_[i];
      if (std::abs(diff) < base_number) {
        (*dgdx)[i] += Cost<PROB>::scale() * x_diff[i] * weights_[i];
      } else if (diff > 0.0) {
        (*dgdx)[i] += 0.5 * Cost<PROB>::scale() * k * weights_[i];
      } else {
        (*dgdx)[i] -= 0.5 * Cost<PROB>::scale() * k * weights_[i];
      }
    }
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& x, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    if (k != ref_index_) {
      return;
    }
    StateType x_diff = x - ref_state_;
    x_diff[PROB::kStateThetaIndex] =
        NormalizeAngle(x_diff[PROB::kStateThetaIndex]);
    for (int i = 0; i < PROB::kStateSize; ++i) {
      const double diff = x_diff[i];
      const double base_number = base_numbers_[i];
      if (std::abs(diff) < base_number) {
        (*ddgdxdx)(i, i) += Cost<PROB>::scale() * weights_[i];
      } else {
      }
    }
  }
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

 private:
  StateType ref_state_;
  int ref_index_;
  std::vector<double> weights_;
  std::vector<double> base_numbers_;
  std::vector<double> ks_;
  std::vector<double> bs_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_REFERENCE_SINGLE_STATE_DEVIATION_COST_H_
