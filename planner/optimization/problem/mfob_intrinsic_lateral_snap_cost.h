#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_INSTRINSTIC_LATERAL_SNAP_COST_H_  // NOLINT
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_INSTRINSTIC_LATERAL_SNAP_COST_H_  // NOLINT

#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"

#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class MfobIntrinsicLateralSnapCost : public Cost<PROB> {
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
  explicit MfobIntrinsicLateralSnapCost(
      std::string name = "MfobIntrinsicLateralSnapCost", double scale = 1.0,
      CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type) {}

  // intrinstic_lateral_snap = v^2 * chi
  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& us,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon; ++k) {
      res.AddSubG(/*idx=*/0, Sqr(Sqr(PROB::v(xs, k)) * PROB::chi(us, k)));
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int /*k*/, const StateType& x,
                   const ControlType& u) const override {
    return 0.5 * Cost<PROB>::scale() *
           Sqr(Sqr(x[PROB::kStateVIndex]) * u[PROB::kControlChiIndex]);
  }

  // Gradients with Superposition.
  void AddDGDx(int /*k*/, const StateType& x, const ControlType& u,
               DGDxType* dgdx) const override {
    (*dgdx)[PROB::kStateVIndex] += Cost<PROB>::scale() * 2.0 *
                                   Cube(x[PROB::kStateVIndex]) *
                                   Sqr(u[PROB::kControlChiIndex]);
  }
  void AddDGDu(int /*k*/, const StateType& x, const ControlType& u,
               DGDuType* dgdu) const override {
    (*dgdu)[PROB::kControlChiIndex] += Cost<PROB>::scale() *
                                       Quar(x[PROB::kStateVIndex]) *
                                       u[PROB::kControlChiIndex];
  }

  // Hessians with Superposition.
  void AddDDGDxDx(int /*k*/, const StateType& x, const ControlType& u,
                  DDGDxDxType* ddgdxdx) const override {
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateVIndex) +=
        Cost<PROB>::scale() * 6.0 * Sqr(x[PROB::kStateVIndex]) *
        Sqr(u[PROB::kControlChiIndex]);
  }
  void AddDDGDuDx(int /*k*/, const StateType& x, const ControlType& u,
                  DDGDuDxType* ddgdudx) const override {
    (*ddgdudx)(PROB::kControlChiIndex, PROB::kStateVIndex) +=
        Cost<PROB>::scale() * 4.0 * Cube(x[PROB::kStateVIndex]) *
        u[PROB::kControlChiIndex];
  }
  void AddDDGDuDu(int /*k*/, const StateType& x, const ControlType& /*u*/,
                  DDGDuDuType* ddgdudu) const override {
    (*ddgdudu)(PROB::kControlChiIndex, PROB::kControlChiIndex) +=
        Cost<PROB>::scale() * Quar(x[PROB::kStateVIndex]);
  }
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_INSTRINSTIC_LATERAL_SNAP_COST_H_
