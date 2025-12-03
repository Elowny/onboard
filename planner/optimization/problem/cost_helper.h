#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_HELPER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_HELPER_H_

#include <string>
#include <utility>
#include <vector>

#include "onboard/lite/check.h"

namespace qcraft {
namespace planner {

// Cost helper used to compute results could be shared by many costs.

template <typename PROB>
class CostHelper {
 public:
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  explicit CostHelper(int horizon, std::string name = "")
      : horizon_(horizon), name_(std::move(name)) {
    QCHECK_GT(horizon_, 0);
  }

  virtual ~CostHelper() {}

  const std::string& name() const { return name_; }

  // Update internal states.
  virtual void Update(const StatesType& /*xs*/, const ControlsType& /*us*/) {}

  int horizon() const { return horizon_; }

 private:
  int horizon_ = 0;
  std::string name_ = "";
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_HELPER_H_
