#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_LATERAL_JERK_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_LATERAL_JERK_COST_H_

#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"

#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class MfobLateralJerkCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 0.2;
  explicit MfobLateralJerkCost(
      bool using_hessian_approximate, std::string name = "MfobLateralJerkCost",
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        using_hessian_approximate_(using_hessian_approximate) {}

  // j_lat = 3 * v * a * kappa + v^2 * psi
  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& /*us*/,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon; ++k) {
      res.AddSubG(/*idx=*/0,
                  Sqr(EvaluateJLateral(PROB::GetStateAtStep(xs, k))));
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int /*k*/, const StateType& x,
                   const ControlType& /*u*/) const override {
    return 0.5 * Cost<PROB>::scale() * Sqr(EvaluateJLateral(x));
  }

  // Gradients with Superposition.
  void AddDGDx(int /*k*/, const StateType& x, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    const double j_lat = EvaluateJLateral(x);
    (*dgdx)[PROB::kStateVIndex] +=
        Cost<PROB>::scale() * j_lat *
        (3.0 * x[PROB::kStateAIndex] * x[PROB::kStateKappaIndex] +
         2.0 * x[PROB::kStateVIndex] * x[PROB::kStatePsiIndex]);
    (*dgdx)[PROB::kStateKappaIndex] += Cost<PROB>::scale() * j_lat * 3.0 *
                                       x[PROB::kStateVIndex] *
                                       x[PROB::kStateAIndex];
    (*dgdx)[PROB::kStateAIndex] += Cost<PROB>::scale() * j_lat * 3.0 *
                                   x[PROB::kStateVIndex] *
                                   x[PROB::kStateKappaIndex];
    (*dgdx)[PROB::kStatePsiIndex] +=
        Cost<PROB>::scale() * j_lat * Sqr(x[PROB::kStateVIndex]);
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with Superposition.
  void AddDDGDxDx(int /*k*/, const StateType& x, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    if (!using_hessian_approximate_) {
      const double j_lat = EvaluateJLateral(x);
      // dv = dj_lat/dv, the same follows.
      const double dv =
          3.0 * x[PROB::kStateAIndex] * x[PROB::kStateKappaIndex] +
          2.0 * x[PROB::kStateVIndex] * x[PROB::kStatePsiIndex];
      const double dkappa = 3.0 * x[PROB::kStateVIndex] * x[PROB::kStateAIndex];
      const double da = 3.0 * x[PROB::kStateVIndex] * x[PROB::kStateKappaIndex];
      const double dpsi = Sqr(x[PROB::kStateVIndex]);
      // dvdkappa = d^2(g)/(dv*dkappa), the same follows.
      const double dvdkappa =
          Cost<PROB>::scale() *
          (dv * dkappa + j_lat * 3.0 * x[PROB::kStateAIndex]);
      const double dvda = Cost<PROB>::scale() *
                          (dv * da + j_lat * 3.0 * x[PROB::kStateKappaIndex]);
      const double dvdpsi = Cost<PROB>::scale() *
                            (dv * dpsi + j_lat * 2.0 * x[PROB::kStateVIndex]);
      const double dkappada =
          Cost<PROB>::scale() *
          (dkappa * da + j_lat * 3.0 * x[PROB::kStateVIndex]);
      const double dkappadpsi = Cost<PROB>::scale() * dkappa * dpsi;
      const double dadpsi = Cost<PROB>::scale() * da * dpsi;

      (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateVIndex) +=
          Cost<PROB>::scale() *
          (dv * dv + j_lat * 2.0 * x[PROB::kStatePsiIndex]);
      (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateKappaIndex) += dvdkappa;
      (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateAIndex) += dvda;
      (*ddgdxdx)(PROB::kStateVIndex, PROB::kStatePsiIndex) += dvdpsi;

      (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateVIndex) += dvdkappa;
      (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateKappaIndex) +=
          Cost<PROB>::scale() * dkappa * dkappa;
      (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateAIndex) += dkappada;
      (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStatePsiIndex) += dkappadpsi;

      (*ddgdxdx)(PROB::kStateAIndex, PROB::kStateVIndex) += dvda;
      (*ddgdxdx)(PROB::kStateAIndex, PROB::kStateKappaIndex) += dkappada;
      (*ddgdxdx)(PROB::kStateAIndex, PROB::kStateAIndex) +=
          Cost<PROB>::scale() * da * da;
      (*ddgdxdx)(PROB::kStateAIndex, PROB::kStatePsiIndex) += dadpsi;

      (*ddgdxdx)(PROB::kStatePsiIndex, PROB::kStateVIndex) += dvdpsi;
      (*ddgdxdx)(PROB::kStatePsiIndex, PROB::kStateKappaIndex) += dkappadpsi;
      (*ddgdxdx)(PROB::kStatePsiIndex, PROB::kStateAIndex) += dadpsi;
      (*ddgdxdx)(PROB::kStatePsiIndex, PROB::kStatePsiIndex) +=
          Cost<PROB>::scale() * dpsi * dpsi;
    } else {
      DGDxType dj_lat_dx = DGDxType::Zero();
      dj_lat_dx[PROB::kStateVIndex] +=
          3.0 * x[PROB::kStateAIndex] * x[PROB::kStateKappaIndex] +
          2.0 * x[PROB::kStateVIndex] * x[PROB::kStatePsiIndex];
      dj_lat_dx[PROB::kStateKappaIndex] +=
          3.0 * x[PROB::kStateVIndex] * x[PROB::kStateAIndex];
      dj_lat_dx[PROB::kStateAIndex] +=
          3.0 * x[PROB::kStateVIndex] * x[PROB::kStateKappaIndex];
      dj_lat_dx[PROB::kStatePsiIndex] += Sqr(x[PROB::kStateVIndex]);
      (*ddgdxdx) += Cost<PROB>::scale() * dj_lat_dx.transpose() * dj_lat_dx;
    }
  }
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

 private:
  // Evaluate lateral jerk
  static double EvaluateJLateral(const StateType& x) {
    return 3.0 * x[PROB::kStateVIndex] * x[PROB::kStateAIndex] *
               x[PROB::kStateKappaIndex] +
           Sqr(x[PROB::kStateVIndex]) * x[PROB::kStatePsiIndex];
  }

 private:
  bool using_hessian_approximate_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_LATERAL_JERK_COST_H_
