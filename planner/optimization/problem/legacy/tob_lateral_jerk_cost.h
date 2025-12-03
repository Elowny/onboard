#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_LATERAL_JERK_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_LATERAL_JERK_COST_H_

#include <string>
#include <utility>

#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class TobLateralJerkCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 0.2;
  explicit TobLateralJerkCost(std::string name = "TobLateralJerkCost",
                              double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale) {}

  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& us,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon; ++k) {
      const double v =
          EvaluateV(PROB::GetStateAtStep(xs, k), PROB::GetControlAtStep(us, k));
      res.AddSubG(
          /*idx=*/0, Sqr(v));
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

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
    (*dgdu)[PROB::kStateXIndex] += Cost<PROB>::scale() * EvaluateV(x, u) *
                                   EvaluateDVDu(x, u)[PROB::kStateXIndex];
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
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateAIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateVIndex] * dvdx[PROB::kStateAIndex] +
         v * ddvdxdx(PROB::kStateVIndex, PROB::kStateAIndex));
    (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateKappaIndex) +=
        Cost<PROB>::scale() *
        (Sqr(dvdx[PROB::kStateKappaIndex]) +
         v * ddvdxdx(PROB::kStateKappaIndex, PROB::kStateKappaIndex));
    (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateAIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateKappaIndex] * dvdx[PROB::kStateAIndex] +
         v * ddvdxdx(PROB::kStateKappaIndex, PROB::kStateAIndex));
    (*ddgdxdx)(PROB::kStateAIndex, PROB::kStateAIndex) +=
        Cost<PROB>::scale() *
        (Sqr(dvdx[PROB::kStateAIndex]) +
         v * ddvdxdx(PROB::kStateAIndex, PROB::kStateAIndex));

    (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateVIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateVIndex] * dvdx[PROB::kStateKappaIndex] +
         v * ddvdxdx(PROB::kStateVIndex, PROB::kStateKappaIndex));
    (*ddgdxdx)(PROB::kStateAIndex, PROB::kStateVIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateVIndex] * dvdx[PROB::kStateAIndex] +
         v * ddvdxdx(PROB::kStateVIndex, PROB::kStateAIndex));
    (*ddgdxdx)(PROB::kStateAIndex, PROB::kStateKappaIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateKappaIndex] * dvdx[PROB::kStateAIndex] +
         v * ddvdxdx(PROB::kStateKappaIndex, PROB::kStateAIndex));
  }
  void AddDDGDuDx(int /*k*/, const StateType& x, const ControlType& u,
                  DDGDuDxType* ddgdudx) const override {
    const double v = EvaluateV(x, u);
    const DGDxType dvdx = EvaluateDVDx(x, u);
    const DGDuType dvdu = EvaluateDVDu(x, u);
    const DDGDuDxType ddvdudx = EvaluateDDVDuDx(x, u);
    (*ddgdudx)(PROB::kControlPsiIndex, PROB::kStateVIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateVIndex] * dvdu[PROB::kControlPsiIndex] +
         v * ddvdudx(PROB::kControlPsiIndex, PROB::kStateVIndex));
    (*ddgdudx)(PROB::kControlPsiIndex, PROB::kStateKappaIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateKappaIndex] * dvdu[PROB::kControlPsiIndex] +
         v * ddvdudx(PROB::kControlPsiIndex, PROB::kStateKappaIndex));
    (*ddgdudx)(PROB::kControlPsiIndex, PROB::kStateAIndex) +=
        Cost<PROB>::scale() *
        (dvdx[PROB::kStateAIndex] * dvdu[PROB::kControlPsiIndex] +
         v * ddvdudx(PROB::kControlPsiIndex, PROB::kStateAIndex));
  }
  void AddDDGDuDu(int /*k*/, const StateType& x, const ControlType& u,
                  DDGDuDuType* ddgdudu) const override {
    DGDuType dvdu = EvaluateDVDu(x, u);
    (*ddgdudu)(PROB::kControlPsiIndex, PROB::kControlPsiIndex) +=
        Cost<PROB>::scale() * Sqr(dvdu[PROB::kControlPsiIndex]);
  }

 private:
  // Evaluate lateral jerk
  double EvaluateV(const StateType& x, const ControlType& u) const {
    return 3.0 * x[PROB::kStateVIndex] * x[PROB::kStateKappaIndex] *
               x[PROB::kStateAIndex] +
           Sqr(x[PROB::kStateVIndex]) * u[PROB::kControlPsiIndex];
  }
  // Evaluate gradients of lateral jerk
  DGDxType EvaluateDVDx(const StateType& x, const ControlType& u) const {
    DGDxType dvdx = DGDxType::Zero();
    dvdx[PROB::kStateVIndex] =
        3.0 * x[PROB::kStateKappaIndex] * x[PROB::kStateAIndex] +
        2.0 * x[PROB::kStateVIndex] * u[PROB::kControlPsiIndex];
    dvdx[PROB::kStateKappaIndex] =
        3.0 * x[PROB::kStateVIndex] * x[PROB::kStateAIndex];
    dvdx[PROB::kStateAIndex] =
        3.0 * x[PROB::kStateVIndex] * x[PROB::kStateKappaIndex];
    return dvdx;
  }
  DGDuType EvaluateDVDu(const StateType& x, const ControlType& /*u*/) const {
    DGDuType dvdu = DGDuType::Zero();
    dvdu[PROB::kControlPsiIndex] = Sqr(x[PROB::kStateVIndex]);
    return dvdu;
  }
  // Evaluate Hessians of lateral jerk
  DDGDxDxType EvaluateDDVDxDx(const StateType& x, const ControlType& u) const {
    DDGDxDxType ddvdxdx = DDGDxDxType::Zero();
    ddvdxdx(PROB::kStateVIndex, PROB::kStateVIndex) =
        2.0 * u[PROB::kControlPsiIndex];
    ddvdxdx(PROB::kStateVIndex, PROB::kStateKappaIndex) =
        3.0 * x[PROB::kStateAIndex];
    ddvdxdx(PROB::kStateVIndex, PROB::kStateAIndex) =
        3.0 * x[PROB::kStateKappaIndex];
    ddvdxdx(PROB::kStateKappaIndex, PROB::kStateAIndex) =
        3.0 * x[PROB::kStateVIndex];
    ddvdxdx(PROB::kStateKappaIndex, PROB::kStateVIndex) =
        ddvdxdx(PROB::kStateVIndex, PROB::kStateKappaIndex);
    ddvdxdx(PROB::kStateAIndex, PROB::kStateVIndex) =
        ddvdxdx(PROB::kStateVIndex, PROB::kStateAIndex);
    ddvdxdx(PROB::kStateAIndex, PROB::kStateKappaIndex) =
        ddvdxdx(PROB::kStateKappaIndex, PROB::kStateAIndex);
    return ddvdxdx;
  }
  DDGDuDxType EvaluateDDVDuDx(const StateType& x,
                              const ControlType& /*u*/) const {
    DDGDuDxType ddvdudx = DDGDuDxType::Zero();
    ddvdudx(PROB::kControlPsiIndex, PROB::kStateVIndex) =
        2.0 * x[PROB::kStateVIndex];
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

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_LATERAL_JERK_COST_H_
