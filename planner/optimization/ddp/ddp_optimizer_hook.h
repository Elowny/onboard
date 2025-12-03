#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_HOOK_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_HOOK_H_

#include <string>
#include <vector>

#include "onboard/math/eigen.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class DdpOptimizerHook {
 public:
  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  struct OptimizerInspector {
    double cost;  // Updated before OnLineSearchIterationEnd().
    double js0;   // Updated in OnIterationEnd().
    // Stores the name and value of costs.
    std::vector<NamedCostEntry> named_costs;
  };

  explicit DdpOptimizerHook(int horizon) : horizon_(horizon) {
    QCHECK_GT(horizon_, 0);
  }

  virtual ~DdpOptimizerHook() {}
  virtual void OnSolveStart(const StatesType& /*xs*/,
                            const ControlsType& /*us*/) {}
  virtual void OnSolveEnd(const StatesType& /*xs*/, const ControlsType& /*us*/,
                          const OptimizerInspector& /*oi*/) {}
  virtual void OnIterationStart(int /*iter*/, const StatesType& /*xs*/,
                                const ControlsType& /*us*/,
                                const OptimizerInspector& /*oi*/) {}
  virtual void OnIterationEnd(int /*iter*/, const StatesType& /*xs*/,
                              const ControlsType& /*us*/,
                              const OptimizerInspector& /*oi*/) {}
  virtual void OnLineSearchIterationStart(const StatesType& /*xs*/,
                                          const ControlsType& /*us*/) {}
  virtual void OnLineSearchIterationEnd(int /*iter*/, double /*alpha*/,
                                        double /*cost*/) {}
  virtual void OnStepSizeAdjustmentIterationStart(const StatesType& /*xs*/,
                                                  const ControlsType& /*us*/) {}
  virtual void OnStepSizeAdjustmentIterationEnd(int /*iter*/,
                                                int /*k_stepsize*/,
                                                double /*cost*/) {}

  int horizon() const { return horizon_; }

 private:
  int horizon_ = 0;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_HOOK_H_
