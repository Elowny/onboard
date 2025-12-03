#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_CURVATURE_RATE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_CURVATURE_RATE_COST_H_

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"

#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class MfobCurvatureRateCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 300.0;
  explicit MfobCurvatureRateCost(
      double curvature_rate_buffer, std::string name = "MfobCurvatureRateCost",
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        curvature_rate_buffer_(curvature_rate_buffer) {}

  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& /*us*/,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon; ++k) {
      res.AddSubG(/*idx=*/0, Sqr(PROB::psi(xs, k)));
      if (std::fabs(PROB::psi(xs, k)) > curvature_rate_buffer_) {
        res.AddSubG(/*idx=*/0,
                    kMaxBufferGain * Sqr(PROB::psi(xs, k) -
                                         std::copysign(curvature_rate_buffer_,
                                                       PROB::psi(xs, k))));
      }
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int /*k*/, const StateType& x,
                   const ControlType& /*u*/) const override {
    double g = 0.5 * Cost<PROB>::scale() * Sqr(x[PROB::kStatePsiIndex]);
    if (std::fabs(x[PROB::kStatePsiIndex]) > curvature_rate_buffer_) {
      g += 0.5 * kMaxBufferGain * Cost<PROB>::scale() *
           Sqr(x[PROB::kStatePsiIndex] -
               std::copysign(curvature_rate_buffer_, x[PROB::kStatePsiIndex]));
    }
    return g;
  }

  // Gradients with Superposition.
  void AddDGDx(int /*k*/, const StateType& x, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    (*dgdx)[PROB::kStatePsiIndex] +=
        Cost<PROB>::scale() * x[PROB::kStatePsiIndex];
    if (std::fabs(x[PROB::kStatePsiIndex]) > curvature_rate_buffer_) {
      (*dgdx)[PROB::kStatePsiIndex] +=
          Cost<PROB>::scale() * kMaxBufferGain *
          (x[PROB::kStatePsiIndex] -
           std::copysign(curvature_rate_buffer_, x[PROB::kStatePsiIndex]));
    }
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with Superposition.
  void AddDDGDxDx(int /*k*/, const StateType& x, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    (*ddgdxdx)(PROB::kStatePsiIndex, PROB::kStatePsiIndex) +=
        Cost<PROB>::scale();
    if (std::fabs(x[PROB::kStatePsiIndex]) > curvature_rate_buffer_) {
      (*ddgdxdx)(PROB::kStatePsiIndex, PROB::kStatePsiIndex) +=
          Cost<PROB>::scale() * kMaxBufferGain;
    }
  }
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

 protected:
  double curvature_rate_buffer_;
  static constexpr double kMaxBufferGain = 15.0;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_CURVATURE_RATE_COST_H_
