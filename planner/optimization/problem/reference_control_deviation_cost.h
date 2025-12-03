#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_REFERENCE_CONTROL_DEVIATION_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_REFERENCE_CONTROL_DEVIATION_COST_H_

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
#include "onboard/planner/optimization/problem/third_order_bicycle.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class ReferenceControlDeviationCost : public Cost<PROB> {
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
  ReferenceControlDeviationCost(
      ControlsType ref_us, std::vector<double> weights = {},
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "ReferenceControlDeviationCost"),
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        ref_us_(std::move(ref_us)),
        weights_(weights.empty() ? std::vector<double>(ref_us_.size(), 1.0)
                                 : std::move(weights)) {
    QCHECK_EQ(ref_us_.size(), weights_.size());
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& us,
                           int horizon) const override {
    DCHECK_EQ(us.rows(), ref_us_.rows());
    DividedG res(/*size=*/1);
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    for (int k = 0; k < horizon; k++) {
      const ControlType u_diff =
          PROB::GetControlAtStep(us, k) - PROB::GetControlAtStep(ref_us_, k);
      res.AddSubG(/*idx=*/0,
                  Sqr(u_diff[0]) * weights_[k * PROB::kControlSize + 0] +
                      Sqr(u_diff[1]) * weights_[k * PROB::kControlSize + 1]);
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& u) const override {
    const ControlType u_diff = u - PROB::GetControlAtStep(ref_us_, k);
    return 0.5 * Cost<PROB>::scale() *
           (Sqr(u_diff[0]) * weights_[k * PROB::kControlSize + 0] +
            Sqr(u_diff[1]) * weights_[k * PROB::kControlSize + 1]);
  }

  // Gradients with superposition.
  void AddDGDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* /*dgdx*/) const override {}
  void AddDGDu(int k, const StateType& /*x*/, const ControlType& u,
               DGDuType* dgdu) const override {
    const ControlType u_diff = u - PROB::GetControlAtStep(ref_us_, k);
    (*dgdu)[0] +=
        Cost<PROB>::scale() * u_diff[0] * weights_[k * PROB::kControlSize + 0];
    (*dgdu)[1] +=
        Cost<PROB>::scale() * u_diff[1] * weights_[k * PROB::kControlSize + 1];
  }

  // Hessians with superposition.
  void AddDDGDxDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* /*ddgdxdx*/) const override {}
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* ddgdudu) const override {
    (*ddgdudu)(0, 0) +=
        Cost<PROB>::scale() * weights_[k * PROB::kControlSize + 0];
    (*ddgdudu)(1, 1) +=
        Cost<PROB>::scale() * weights_[k * PROB::kControlSize + 1];
  }

 private:
  ControlsType ref_us_;
  std::vector<double> weights_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_REFERENCE_CONTROL_DEVIATION_COST_H_
