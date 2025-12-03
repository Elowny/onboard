#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_LATERAL_ACCELERATION_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_LATERAL_ACCELERATION_COST_H_

#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/strings/str_cat.h"

#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class LateralAccelerationCost : public Cost<PROB> {
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
  explicit LateralAccelerationCost(
      bool using_hessian_approximate,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "LateralAccelerationCost"),
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        using_hessian_approximate_(using_hessian_approximate) {}

  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& /*us*/,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon; ++k) {
      const double a_lat = Sqr(PROB::v(xs, k)) * PROB::kappa(xs, k);
      res.AddSubG(/*idx=*/0, Sqr(a_lat));
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int /*k*/, const StateType& x,
                   const ControlType& /*u*/) const override {
    const double a_lat = Sqr(x[PROB::kStateVIndex]) * x[PROB::kStateKappaIndex];
    return 0.5 * Cost<PROB>::scale() * Sqr(a_lat);
  }

  // Gradients with superposition.
  void AddDGDx(int /*k*/, const StateType& x, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    const double a_lat = Sqr(x[PROB::kStateVIndex]) * x[PROB::kStateKappaIndex];
    (*dgdx)[PROB::kStateVIndex] += 2.0 * Cost<PROB>::scale() * a_lat *
                                   x[PROB::kStateVIndex] *
                                   x[PROB::kStateKappaIndex];
    (*dgdx)[PROB::kStateKappaIndex] +=
        Cost<PROB>::scale() * a_lat * Sqr(x[PROB::kStateVIndex]);
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int /*k*/, const StateType& x, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    if (!using_hessian_approximate_) {
      const double a_lat =
          Sqr(x[PROB::kStateVIndex]) * x[PROB::kStateKappaIndex];
      (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateVIndex) +=
          6.0 * a_lat * x[PROB::kStateKappaIndex];
      (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateKappaIndex) +=
          4.0 * a_lat * x[PROB::kStateVIndex];
      (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateVIndex) +=
          4.0 * a_lat * x[PROB::kStateVIndex];
      (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateKappaIndex) +=
          Sqr(Sqr(x[PROB::kStateVIndex]));
    } else {
      DGDxType da_lat_dx = DGDxType::Zero();
      da_lat_dx(PROB::kStateVIndex) =
          2.0 * x[PROB::kStateVIndex] * x[PROB::kStateKappaIndex];
      da_lat_dx(PROB::kStateKappaIndex) = Sqr(x[PROB::kStateVIndex]);
      (*ddgdxdx) += Cost<PROB>::scale() * da_lat_dx.transpose() * da_lat_dx;
    }
  }
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

 private:
  bool using_hessian_approximate_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_LATERAL_ACCELERATION_COST_H_
