#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_EVALUATION_TEST_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_EVALUATION_TEST_UTIL_H_

#include <utility>
#include <vector>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB, int STEP>
class CostEvaluationTest {
 public:
  static_assert(STEP > 0, "STEP must > 0.");
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

  static void SumForAllStepsTest(Cost<PROB>* cost) {
    StateType x = PROB::TestState();
    ControlType u = PROB::TestControl();
    StatesType xs(STEP * PROB::kStateSize);
    ControlsType us(STEP * PROB::kControlSize);

    for (int k = 0; k < STEP; ++k) {
      PROB::SetStateAtStep(x, k, &xs);
      PROB::SetControlAtStep(u, k, &us);
    }
    cost->Update(xs, us, STEP);

    double g_evaluate = 0.0;
    for (int k = 0; k < STEP; ++k) {
      g_evaluate += cost->EvaluateG(k, x, u);
    }
    const double g_sum = cost->SumGForAllSteps(xs, us, STEP).sum();
    constexpr double kEpsilon = 1e-9;
    EXPECT_NEAR(g_sum, g_evaluate, kEpsilon);
  }

  static void EvaluateWithDebugInfoTest(Cost<PROB>* cost) {
    StateType x = PROB::TestState();
    ControlType u = PROB::TestControl();
    StatesType xs(STEP * PROB::kStateSize);
    ControlsType us(STEP * PROB::kControlSize);

    for (int k = 0; k < STEP; ++k) {
      PROB::SetStateAtStep(x, k, &xs);
      PROB::SetControlAtStep(u, k, &us);
    }
    cost->Update(xs, us, STEP);

    const double g_evaluate = cost->EvaluateG(/*k=*/0, x, u);
    const double g_evaluate_with_debug_info = cost->EvaluateG(/*k=*/0, x, u);

    constexpr double kEpsilon = 1e-9;
    EXPECT_NEAR(g_evaluate_with_debug_info, g_evaluate, kEpsilon);
  }
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_EVALUATION_TEST_UTIL_H_
