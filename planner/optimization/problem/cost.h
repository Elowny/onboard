#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_H_

#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "onboard/lite/logging.h"
#include "onboard/math/eigen.h"

namespace qcraft {
namespace planner {

// A small structure to record cost name and values.
struct NamedCostEntry {
  std::string name;
  double value;
  bool is_soft;
};

// General time-variable non-linear state-control cost for optimization-based
// motion planning, g(x, u).
//
// Implementation needs to provide the cost value, gradient and hessian
// evaluation for any given step k through the horizon. The implementation also
// needs to provide interfaces to update its internal states according to the
// current xs and us before its value, gradient or hessian is evaluated.
template <typename PROB>
class Cost {
 public:
  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;
  using FType = typename PROB::FType;
  using DFDxType = typename PROB::DFDxType;
  using DFDuType = typename PROB::DFDuType;
  using DDFDxDxType = typename PROB::DDFDxDxType;
  using DDFDxDuType = typename PROB::DDFDxDuType;
  using DDFDuDxType = typename PROB::DDFDuDxType;
  using DDFDuDuType = typename PROB::DDFDuDuType;
  using GType = typename PROB::GType;
  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;
  using DDGDxDxType = typename PROB::DDGDxDxType;
  using DDGDxDuType = typename PROB::DDGDxDuType;
  using DDGDuDxType = typename PROB::DDGDuDxType;
  using DDGDuDuType = typename PROB::DDGDuDuType;

  // If a cost will exists in any scenarios and in the same form, the type will
  // be MUST_HAVE. Otherwise, create a new enum label for it, or use UNKNOWN.
  enum class CostType {
    UNKNOWN = 0,                                  // NOLINT
    MUST_HAVE = 1,                                // NOLINT
    GROUP_IMMEDIATE_FUTURE = 2,                   // NOLINT
    GROUP_OBJECT = 3,                             // NOLINT
    SOLID_LINE_MSD_STATIC_BOUNDARY = 4,           // NOLINT
    CURB_MSD_STATIC_BOUNDARY_V2 = 5,              // NOLINT
    UTURN_RIGHT_CURB_MSD_STATIC_BOUNDARY_V2 = 6,  // NOLINT
  };

  explicit Cost(std::string name = "", double scale = 1.0,
                CostType cost_type = CostType::UNKNOWN)
      : name_(std::move(name)), scale_(scale), cost_type_(cost_type) {}
  virtual ~Cost() {}

  const std::string& name() const { return name_; }
  const CostType& cost_type() const { return cost_type_; }
  void set_cost_type(CostType cost_type) { cost_type_ = cost_type; }
  double scale() const { return scale_; }
  void set_scale(double scale) { scale_ = scale; }

  class DividedG {
   public:
    explicit DividedG(int size)
        : gs_(size, {.name = "", .value = 0.0}), sum_(0.0) {}

    double sum() const { return sum_; }
    const std::vector<NamedCostEntry>& gs() const { return gs_; }

    void SetSubName(int idx, std::string sub_name) {
      gs_[idx].name = std::move(sub_name);
    }

    void SetIsSoft(int idx, bool is_soft) { gs_[idx].is_soft = is_soft; }

    void AddSubG(int idx, double g) {
      gs_[idx].value += g;
      sum_ += g;
    }

    void Multi(double p) {
      for (auto& g : gs_) {
        g.value *= p;
      }
      sum_ *= p;
    }

    void VecDiv(const std::vector<double>& ds) {
      QCHECK_EQ(gs_.size(), ds.size());
      double new_sum = 0.0;
      for (int i = 0; i < gs_.size(); ++i) {
        if (ds[i] != 0.0) {
          gs_[i].value /= ds[i];
        }
        new_sum += gs_[i].value;
      }
      sum_ = new_sum;
    }

   private:
    std::vector<NamedCostEntry> gs_;
    double sum_;
  };

  virtual DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& us,
                                   int horizon) const = 0;

  // g.
  virtual double EvaluateG(int k, const StateType& x,
                           const ControlType& u) const = 0;

  // g.
  virtual DividedG EvaluateGWithDebugInfo(int k, const StateType& x,
                                          const ControlType& u,
                                          bool using_scale) const {
    DividedG res(/*size=*/1);
    res.SetSubName(/*idx=*/0, name());
    res.AddSubG(/*idx=*/0, EvaluateG(k, x, u));
    if (!using_scale) {
      if (scale() != 0.0) {
        res.Multi(1.0 / scale());
      }
    }
    return res;
  }

  // Gradients.
  virtual DGDxType EvaluateDGDx(int k, const StateType& x,
                                const ControlType& u) const {
    DGDxType dgdx = DGDxType::Zero();
    AddDGDx(k, x, u, &dgdx);
    return dgdx;
  }
  virtual DGDuType EvaluateDGDu(int k, const StateType& x,
                                const ControlType& u) const {
    DGDuType dgdu = DGDuType::Zero();
    AddDGDu(k, x, u, &dgdu);
    return dgdu;
  }

  // Hessians.
  virtual DDGDxDxType EvaluateDDGDxDx(int k, const StateType& x,
                                      const ControlType& u) const {
    DDGDxDxType ddgdxdx = DDGDxDxType::Zero();
    AddDDGDxDx(k, x, u, &ddgdxdx);
    return ddgdxdx;
  }
  virtual DDGDuDxType EvaluateDDGDuDx(int k, const StateType& x,
                                      const ControlType& u) const {
    DDGDuDxType ddgdudx = DDGDuDxType::Zero();
    AddDDGDuDx(k, x, u, &ddgdudx);
    return ddgdudx;
  }
  virtual DDGDuDuType EvaluateDDGDuDu(int k, const StateType& x,
                                      const ControlType& u) const {
    DDGDuDuType ddgdudu = DDGDuDuType::Zero();
    AddDDGDuDu(k, x, u, &ddgdudu);
    return ddgdudu;
  }

  // Gradients with superposition.
  virtual void AddDGDx(int k, const StateType& x, const ControlType& u,
                       DGDxType* dgdx) const = 0;
  virtual void AddDGDu(int k, const StateType& x, const ControlType& u,
                       DGDuType* dgdu) const = 0;

  // Hessians with superposition.
  virtual void AddDDGDxDx(int k, const StateType& x, const ControlType& u,
                          DDGDxDxType* ddgdxdx) const = 0;
  virtual void AddDDGDuDx(int k, const StateType& x, const ControlType& u,
                          DDGDuDxType* ddgdudx) const = 0;
  virtual void AddDDGDuDu(int k, const StateType& x, const ControlType& u,
                          DDGDuDuType* ddgdudu) const = 0;

  // Update common internal states for computing both cost value and
  // derivatives.
  virtual void Update(const StatesType& /*xs*/, const ControlsType& /*us*/,
                      int /*horizon*/) {}

  // Update internal states for computing derivatives only. Please make sure
  // call Update() first.
  virtual void UpdateDerivatives(const StatesType& /*xs*/,
                                 const ControlsType& /*us*/, int /*horizon*/) {}

 private:
  std::string name_ = "";
  double scale_ = 1.0;
  CostType cost_type_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_H_
