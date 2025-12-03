#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_CURVATURE_RATE_RATE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_CURVATURE_RATE_RATE_COST_H_

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
class MfobCurvatureRateRateCost : public Cost<PROB> {
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
  explicit MfobCurvatureRateRateCost(
      double curvature_rate_rate_buffer,
      std::string name = "MfobCurvatureRateRateCost", double scale = 1.0,
      CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        curvature_rate_rate_buffer_(curvature_rate_rate_buffer) {}

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& us,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon; ++k) {
      res.AddSubG(/*idx=*/0, Sqr(PROB::chi(us, k)));
      if (std::fabs(PROB::chi(us, k)) > curvature_rate_rate_buffer_) {
        res.AddSubG(
            /*idx=*/0,
            kMaxBufferGain * Sqr(PROB::chi(us, k) -
                                 std::copysign(curvature_rate_rate_buffer_,
                                               PROB::chi(us, k))));
      }
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int /*k*/, const StateType& /*x*/,
                   const ControlType& u) const override {
    double g = 0.5 * Cost<PROB>::scale() * Sqr(u[PROB::kControlChiIndex]);
    if (std::fabs(u[PROB::kControlChiIndex]) > curvature_rate_rate_buffer_) {
      g += 0.5 * kMaxBufferGain * Cost<PROB>::scale() *
           Sqr(u[PROB::kControlChiIndex] -
               std::copysign(curvature_rate_rate_buffer_,
                             u[PROB::kControlChiIndex]));
    }
    return g;
  }

  // Gradients with Superposition.
  void AddDGDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* /*dgdx*/) const override {}
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& u,
               DGDuType* dgdu) const override {
    (*dgdu)[PROB::kControlChiIndex] +=
        Cost<PROB>::scale() * u[PROB::kControlChiIndex];
    if (std::fabs(u[PROB::kControlChiIndex]) > curvature_rate_rate_buffer_) {
      (*dgdu)[PROB::kControlChiIndex] +=
          Cost<PROB>::scale() * kMaxBufferGain *
          (u[PROB::kControlChiIndex] -
           std::copysign(curvature_rate_rate_buffer_,
                         u[PROB::kControlChiIndex]));
    }
  }

  // Hessians with Superposition.
  void AddDDGDxDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* /*ddgdxdx*/) const override {}
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& u,
                  DDGDuDuType* ddgdudu) const override {
    (*ddgdudu)(PROB::kControlChiIndex, PROB::kControlChiIndex) +=
        Cost<PROB>::scale();
    if (std::fabs(u[PROB::kControlChiIndex]) > curvature_rate_rate_buffer_) {
      (*ddgdudu)(PROB::kControlChiIndex, PROB::kControlChiIndex) +=
          Cost<PROB>::scale() * kMaxBufferGain;
    }
  }

 protected:
  double curvature_rate_rate_buffer_;
  static constexpr double kMaxBufferGain = 15.0;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_CURVATURE_RATE_COST_H_
