#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_LONGITUDINAL_JERK_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_LONGITUDINAL_JERK_COST_H_

#include <string>
#include <utility>

#include "absl/strings/str_cat.h"

#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class LongitudinalJerkCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 1.0;
  explicit LongitudinalJerkCost(
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "LongitudinalJerkCost"),
      double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale) {}

  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& us,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon; ++k) {
      const double v =
          EvaluateV(PROB::GetStateAtStep(xs, k), PROB::GetControlAtStep(us, k));
      res.AddSubG(/*idx=*/0, Sqr(v));
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // V = j_lon = j - v^3 * kappa^2
  // g.
  double EvaluateG(int /*k*/, const StateType& x,
                   const ControlType& u) const override {
    const double v = EvaluateV(x, u);
    return 0.5 * Cost<PROB>::scale() * Sqr(v);
  }

  // Gradients with superposition.
  void AddDGDx(int /*k*/, const StateType& x, const ControlType& u,
               DGDxType* dgdx) const override {
    const double v = EvaluateV(x, u);
    const DGDxType dvdx = EvaluateDVDx(x, u);
    (*dgdx)[PROB::kStateVIndex] +=
        Cost<PROB>::scale() * v * dvdx[PROB::kStateVIndex];
    (*dgdx)[PROB::kStateKappaIndex] +=
        Cost<PROB>::scale() * v * dvdx[PROB::kStateKappaIndex];
    (*dgdx)[PROB::kStateAIndex] +=
        Cost<PROB>::scale() * v * dvdx[PROB::kStateAIndex];
  }
  void AddDGDu(int /*k*/, const StateType& x, const ControlType& u,
               DGDuType* dgdu) const override {
    (*dgdu)[PROB::kControlJIndex] += Cost<PROB>::scale() * EvaluateV(x, u) *
                                     EvaluateDVDu(x, u)[PROB::kControlJIndex];
  }

  // Hessians with superposition.
  void AddDDGDxDx(int /*k*/, const StateType& x, const ControlType& u,
                  DDGDxDxType* ddgdxdx) const override {
    const double v = EvaluateV(x, u);
    const DGDxType dvdx = EvaluateDVDx(x, u);
    const DDGDxDxType ddvdxdx = EvaluateDDVDxDx(x, u);
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateVIndex) +=
        Cost<PROB>::scale() *
        (Sqr(dvdx[PROB::kStateVIndex]) +
         v * ddvdxdx(PROB::kStateVIndex, PROB::kStateVIndex));
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateKappaIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateVIndex] * dvdx[PROB::kStateKappaIndex] +
         v * ddvdxdx(PROB::kStateVIndex, PROB::kStateKappaIndex));
    (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateKappaIndex) +=
        Cost<PROB>::scale() *
        (Sqr(dvdx[PROB::kStateKappaIndex]) +
         v * ddvdxdx(PROB::kStateKappaIndex, PROB::kStateKappaIndex));
    (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateVIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateVIndex] * dvdx[PROB::kStateKappaIndex] +
         v * ddvdxdx(PROB::kStateVIndex, PROB::kStateKappaIndex));
  }
  void AddDDGDuDx(int /*k*/, const StateType& x, const ControlType& u,
                  DDGDuDxType* ddgdudx) const override {
    // const double v = EvaluateV(x, u);
    const DGDxType dvdx = EvaluateDVDx(x, u);
    const DGDuType dvdu = EvaluateDVDu(x, u);
    // const DDGDuDxType ddvdudx = EvaluateDDVDuDx(x, u);
    (*ddgdudx)(PROB::kControlJIndex, PROB::kStateVIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateVIndex] *
         dvdu[PROB::kControlJIndex] /* + v * ddvdudx(1, 3) = 0*/);
    (*ddgdudx)(PROB::kControlJIndex, PROB::kStateKappaIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateKappaIndex] *
         dvdu[PROB::kControlJIndex] /* + v * ddvdudx(1, 4) = 0*/);
  }
  void AddDDGDuDu(int /*k*/, const StateType& x, const ControlType& u,
                  DDGDuDuType* ddgdudu) const override {
    DGDuType dvdu = EvaluateDVDu(x, u);
    (*ddgdudu)(PROB::kControlJIndex, PROB::kControlJIndex) +=
        Cost<PROB>::scale() * Sqr(dvdu[PROB::kControlJIndex]);
  }

 private:
  // Evaluate longitudinal jerk
  double EvaluateV(const StateType& x, const ControlType& u) const {
    return u[PROB::kControlJIndex] -
           Cube(x[PROB::kStateVIndex]) * Sqr(x[PROB::kStateKappaIndex]);
  }
  // Evaluate gradients of longitudinal jerk
  DGDxType EvaluateDVDx(const StateType& x, const ControlType& /*u*/) const {
    DGDxType dvdx = DGDxType::Zero();
    dvdx[PROB::kStateVIndex] =
        -3.0 * Sqr(x[PROB::kStateVIndex]) * Sqr(x[PROB::kStateKappaIndex]);
    dvdx[PROB::kStateKappaIndex] =
        -2.0 * Cube(x[PROB::kStateVIndex]) * x[PROB::kStateKappaIndex];
    return dvdx;
  }
  DGDuType EvaluateDVDu(const StateType& /*x*/,
                        const ControlType& /*u*/) const {
    DGDuType dvdu = DGDuType::Zero();
    dvdu[PROB::kControlJIndex] = 1.0;
    return dvdu;
  }
  // Evaluate Hessians of longitudinal jerk
  DDGDxDxType EvaluateDDVDxDx(const StateType& x,
                              const ControlType& /*u*/) const {
    DDGDxDxType ddvdxdx = DDGDxDxType::Zero();
    ddvdxdx(PROB::kStateVIndex, PROB::kStateVIndex) =
        -6.0 * x[PROB::kStateVIndex] * Sqr(x[PROB::kStateKappaIndex]);
    ddvdxdx(PROB::kStateVIndex, PROB::kStateKappaIndex) =
        -6.0 * Sqr(x[PROB::kStateVIndex]) * x[PROB::kStateKappaIndex];
    ddvdxdx(PROB::kStateKappaIndex, PROB::kStateKappaIndex) =
        -2.0 * Cube(x[PROB::kStateVIndex]);
    ddvdxdx(PROB::kStateKappaIndex, PROB::kStateVIndex) =
        ddvdxdx(PROB::kStateVIndex, PROB::kStateKappaIndex);
    return ddvdxdx;
  }
  DDGDuDxType EvaluateDDVDuDx(const StateType& /*x*/,
                              const ControlType& /*u*/) const {
    DDGDuDxType ddvdudx = DDGDuDxType::Zero();
    return ddvdudx;
  }
  DDGDuDuType EvaluateDDVDuDu(const StateType& /*x*/,
                              const ControlType& /*u*/) const {
    DDGDuDuType ddvdudu = DDGDuDuType::Zero();
    return ddvdudu;
  }
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_LONGITUDINAL_JERK_COST_H_
