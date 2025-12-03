#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_CURVATURE_RATE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_CURVATURE_RATE_COST_H_

#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class TobCurvatureRateCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 300.0;
  explicit TobCurvatureRateCost(double curvature_rate_buffer,
                                std::string name = "TobCurvatureRateCost",
                                double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale),
        curvature_rate_buffer_(curvature_rate_buffer) {}

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& us,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon; ++k) {
      const double psi = PROB::psi(us, k);
      res.AddSubG(/*idx=*/0, Sqr(psi));
      if (std::fabs(psi) > curvature_rate_buffer_) {
        res.AddSubG(
            /*idx=*/0,
            kMaxBufferGain * Sqr(psi - Sign(psi) * curvature_rate_buffer_));
      }
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int /*k*/, const StateType& /*x*/,
                   const ControlType& u) const override {
    double g = 0.5 * Cost<PROB>::scale() * Sqr(u[PROB::kControlPsiIndex]);
    if (std::fabs(u[PROB::kControlPsiIndex]) > curvature_rate_buffer_) {
      g += 0.5 * kMaxBufferGain * Cost<PROB>::scale() *
           Sqr(u[PROB::kControlPsiIndex] -
               Sign(u[PROB::kControlPsiIndex]) * curvature_rate_buffer_);
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* /*dgdx*/) const override {}
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& u,
               DGDuType* dgdu) const override {
    (*dgdu)[PROB::kControlPsiIndex] +=
        Cost<PROB>::scale() * u[PROB::kControlPsiIndex];
    if (std::fabs(u[PROB::kControlPsiIndex]) > curvature_rate_buffer_) {
      (*dgdu)[PROB::kControlPsiIndex] +=
          Cost<PROB>::scale() * kMaxBufferGain *
          (u[PROB::kControlPsiIndex] -
           std::copysign(curvature_rate_buffer_, u[PROB::kControlPsiIndex]));
    }
  }

  // Hessians with superposition.
  void AddDDGDxDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* /*ddgdxdx*/) const override {}
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& u,
                  DDGDuDuType* ddgdudu) const override {
    (*ddgdudu)(PROB::kControlPsiIndex, PROB::kControlPsiIndex) +=
        Cost<PROB>::scale();
    if (std::fabs(u[PROB::kControlPsiIndex]) > curvature_rate_buffer_) {
      (*ddgdudu)(PROB::kControlPsiIndex, PROB::kControlPsiIndex) +=
          Cost<PROB>::scale() * kMaxBufferGain;
    }
  }

 protected:
  double curvature_rate_buffer_;
  static constexpr double kMaxBufferGain = 15.0;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_CURVATURE_RATE_COST_H_
