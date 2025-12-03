#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_INTRINSIC_JERK_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_INTRINSIC_JERK_COST_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"

#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class IntrinsicJerkCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 0.4;
  explicit IntrinsicJerkCost(
      double accel_jerk_buffer, double decel_jerk_buffer,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "IntrinsicJerkCost"),
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        accel_jerk_buffer_(accel_jerk_buffer),
        decel_jerk_buffer_(decel_jerk_buffer) {
    QCHECK_GT(accel_jerk_buffer_, decel_jerk_buffer_);
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& us,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon; ++k) {
      const double j = PROB::j(us, k);
      res.AddSubG(/*idx=*/0, Sqr(j));
      if (j > accel_jerk_buffer_)
        res.AddSubG(/*idx=*/0, kMaxBufferGain_ * Sqr(j - accel_jerk_buffer_));
      else if (j < decel_jerk_buffer_)
        res.AddSubG(/*idx=*/0, kMaxBufferGain_ * Sqr(j - decel_jerk_buffer_));
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int /*k*/, const StateType& /*x*/,
                   const ControlType& u) const override {
    double g = 0.5 * Cost<PROB>::scale() * Sqr(u[PROB::kControlJIndex]);
    if (u[PROB::kControlJIndex] > accel_jerk_buffer_) {
      g += 0.5 * kMaxBufferGain_ * Cost<PROB>::scale() *
           Sqr(u[PROB::kControlJIndex] - accel_jerk_buffer_);
    } else if (u[PROB::kControlJIndex] < decel_jerk_buffer_) {
      g += 0.5 * kMaxBufferGain_ * Cost<PROB>::scale() *
           Sqr(u[PROB::kControlJIndex] - decel_jerk_buffer_);
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* /*dgdx*/) const override {}
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& u,
               DGDuType* dgdu) const override {
    (*dgdu)[PROB::kControlJIndex] +=
        Cost<PROB>::scale() * u[PROB::kControlJIndex];
    if (u[PROB::kControlJIndex] > accel_jerk_buffer_)
      (*dgdu)[PROB::kControlJIndex] +=
          Cost<PROB>::scale() * kMaxBufferGain_ *
          (u[PROB::kControlJIndex] - accel_jerk_buffer_);
    else if (u[PROB::kControlJIndex] < decel_jerk_buffer_)
      (*dgdu)[PROB::kControlJIndex] +=
          Cost<PROB>::scale() * kMaxBufferGain_ *
          (u[PROB::kControlJIndex] - decel_jerk_buffer_);
  }

  // Hessians with superposition.
  void AddDDGDxDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* /*ddgdxdx*/) const override {}
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& u,
                  DDGDuDuType* ddgdudu) const override {
    (*ddgdudu)(PROB::kControlJIndex, PROB::kControlJIndex) +=
        Cost<PROB>::scale();
    if (u[PROB::kControlJIndex] > accel_jerk_buffer_)
      (*ddgdudu)(PROB::kControlJIndex, PROB::kControlJIndex) +=
          Cost<PROB>::scale() * kMaxBufferGain_;
    else if (u[PROB::kControlJIndex] < decel_jerk_buffer_)
      (*ddgdudu)(PROB::kControlJIndex, PROB::kControlJIndex) +=
          Cost<PROB>::scale() * kMaxBufferGain_;
  }

 protected:
  double accel_jerk_buffer_;
  double decel_jerk_buffer_;
  static constexpr double kMaxBufferGain_ = 100.0;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_INTRINSIC_JERK_COST_H_
