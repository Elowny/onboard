#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_CONVERGENCE_TEST_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_CONVERGENCE_TEST_UTIL_H_

#include <utility>
#include <vector>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/math/convergence_order.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB, int STEP>
class CostConvergenceTest {
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

  static void UpdateCost(const StateType& x, const ControlType& u,
                         Cost<PROB>* cost) {
    StatesType xs(STEP * PROB::kStateSize);
    ControlsType us(STEP * PROB::kControlSize);
    for (int k = 0; k < STEP; ++k) {
      PROB::SetStateAtStep(x, k, &xs);
      PROB::SetControlAtStep(u, k, &us);
    }

    cost->Update(xs, us, STEP);
    cost->UpdateDerivatives(xs, us, STEP);
  }

  static void ExpectCostGradientResidualOrder(
      const std::vector<StateType>& states,
      const std::vector<ControlType>& controls,
      const std::vector<std::pair<StateType, ControlType>>& variations,
      Cost<PROB>* cost) {
    for (const StateType& x : states) {
      for (const ControlType& u : controls) {
        UpdateCost(x, u, cost);
        const GType g = cost->EvaluateG(0, x, u);
        LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                  << " g = " << g;
        const DGDxType& dgdx = cost->EvaluateDGDx(0, x, u);
        const DGDuType& dgdu = cost->EvaluateDGDu(0, x, u);
        LOG(INFO) << "dgdx = " << std::endl << dgdx;
        LOG(INFO) << "dgdu = " << std::endl << dgdu;

        for (const auto& v : variations) {
          const int order = AssessConvergenceOrder([&](double perturbation) {
            const StateType dx = v.first * perturbation;
            const ControlType du = v.second * perturbation;
            UpdateCost(x + dx, u + du, cost);
            const GType perturbed_g = cost->EvaluateG(0, x + dx, u + du);
            const GType dg_fd = perturbed_g - g;
            const GType dg_der = 0.0 + dgdx * dx + dgdu * du;
            VLOG(3) << "Variation assessment: perturbation = " << perturbation
                    << " dx = " << dx.transpose() << " du = " << du.transpose()
                    << " g = " << g << " perturbed_g = " << perturbed_g
                    << " dg_fd = " << dg_fd << " dg_der = " << dg_der;
            return std::abs(dg_fd - dg_der);
          });
          LOG(INFO) << "variation: " << v.first.transpose() << "; "
                    << v.second.transpose() << " order = " << order;
          if (order != -1) EXPECT_GE(order, 2);
        }
      }
    }
  }

  static void ExpectCostHessianResidualOrder(
      const std::vector<StateType>& states,
      const std::vector<ControlType>& controls,
      const std::vector<std::pair<StateType, ControlType>>& variations0,
      const std::vector<std::pair<StateType, ControlType>>& variations1,
      Cost<PROB>* cost) {
    for (const StateType& x : states) {
      for (const ControlType& u : controls) {
        UpdateCost(x, u, cost);
        const GType g = cost->EvaluateG(0, x, u);
        LOG(INFO) << "x = " << x.transpose() << " u = " << u.transpose()
                  << " g = " << g;
        const DDGDxDxType& ddgdxdx = cost->EvaluateDDGDxDx(0, x, u);
        const DDGDuDxType& ddgdudx = cost->EvaluateDDGDuDx(0, x, u);
        const DDGDuDuType& ddgdudu = cost->EvaluateDDGDuDu(0, x, u);
        LOG(INFO) << "ddgdxdx = " << std::endl << ddgdxdx;
        LOG(INFO) << "ddgdudx = " << std::endl << ddgdudx;
        LOG(INFO) << "ddgdudu = " << std::endl << ddgdudu;

        for (const auto& v0 : variations0) {
          for (const auto& v1 : variations1) {
            const int order = AssessConvergenceOrder([&](double perturbation) {
              const StateType dx0 = v0.first * perturbation;
              const ControlType du0 = v0.second * perturbation;
              const StateType dx1 = v1.first * perturbation;
              const ControlType du1 = v1.second * perturbation;
              UpdateCost(x + dx0, u + du0, cost);
              const GType gv0 = cost->EvaluateG(0, x + dx0, u + du0);
              UpdateCost(x + dx1, u + du1, cost);
              const GType gv1 = cost->EvaluateG(0, x + dx1, u + du1);
              UpdateCost(x + dx0 + dx1, u + du0 + du1, cost);
              const GType gv01 =
                  cost->EvaluateG(0, x + dx0 + dx1, u + du0 + du1);
              const GType dg_fd = g + gv01 - gv0 - gv1;
              const GType dg_der = 0.0 + dx0.transpose() * ddgdxdx * dx1 +
                                   du0.transpose() * ddgdudx * dx1 +
                                   du1.transpose() * ddgdudx * dx0 +
                                   du0.transpose() * ddgdudu * du1;
              VLOG(3) << "Variation assessment: perturbation = " << perturbation
                      << " dx0 = " << dx0.transpose()
                      << " du0 = " << du0.transpose()
                      << " dx1 = " << dx1.transpose()
                      << " du1 = " << du1.transpose() << " g = " << g
                      << " gv0 = " << gv0 << " gv1 = " << gv1
                      << " gv01 = " << gv01 << " dg_fd = " << dg_fd
                      << " dg_der = " << dg_der;
              return std::abs(dg_fd - dg_der);
            });
            LOG(INFO) << "variation0: " << v0.first.transpose() << "; "
                      << v0.second.transpose()
                      << " variation1: " << v1.first.transpose() << "; "
                      << v1.second.transpose() << " order = " << order;
            if (order != -1) EXPECT_GE(order, 3);
          }
        }
      }
    }
  }

  static std::vector<std::pair<StateType, ControlType>>
  GenerateStateVariationBasis() {
    std::vector<std::pair<StateType, ControlType>> variations;
    for (int i = 0; i < PROB::kStateSize; ++i) {
      StateType dx = StateType::Zero();
      dx[i] = 1.0;
      variations.emplace_back(dx, ControlType::Zero());
    }
    return variations;
  }

  static std::vector<std::pair<StateType, ControlType>>
  GenerateControlVariationBasis() {
    std::vector<std::pair<StateType, ControlType>> variations;
    for (int i = 0; i < PROB::kControlSize; ++i) {
      ControlType du = ControlType::Zero();
      du[i] = 1.0;
      variations.emplace_back(StateType::Zero(), du);
    }
    return variations;
  }

  static std::vector<std::pair<StateType, ControlType>>
  GenerateVariationBasis() {
    std::vector<std::pair<StateType, ControlType>> variations =
        GenerateStateVariationBasis();
    const auto control_variations = GenerateControlVariationBasis();
    variations.insert(variations.end(), control_variations.begin(),
                      control_variations.end());
    return variations;
  }
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_CONVERGENCE_TEST_UTIL_H_
