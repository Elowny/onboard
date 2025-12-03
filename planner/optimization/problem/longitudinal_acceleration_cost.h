#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_LONGITUDINAL_ACCELERATION_COST_H_  // NOLINT
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_LONGITUDINAL_ACCELERATION_COST_H_  // NOLINT

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

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

namespace qcraft {
namespace planner {

template <typename PROB>
class LongitudinalAccelerationCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 0.05;
  explicit LongitudinalAccelerationCost(
      double max_acceleration_buffer, double max_deceleration_buffer,
      std::vector<double> accel_cascade_buffers = {0.0},
      std::vector<double> accel_cascade_gains = {20.0},
      std::vector<double> decel_cascade_buffers = {0.0},
      std::vector<double> decel_cascade_gains = {1.0},
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "LongitudinalAccelerationCost"),
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        accel_cascade_buffers_(std::move(accel_cascade_buffers)),
        accel_cascade_gains_(std::move(accel_cascade_gains)),
        decel_cascade_buffers_(std::move(decel_cascade_buffers)),
        decel_cascade_gains_(std::move(decel_cascade_gains)),
        max_acceleration_buffer_(max_acceleration_buffer),
        max_deceleration_buffer_(max_deceleration_buffer) {
    // Sanity checks.
    QCHECK_GT(max_acceleration_buffer_, max_deceleration_buffer_);
    QCHECK_EQ(accel_cascade_buffers_[0], 0.0);
    QCHECK_EQ(decel_cascade_buffers_[0], 0.0);
    QCHECK_EQ(accel_cascade_buffers_.size(), accel_cascade_gains_.size());
    QCHECK_EQ(decel_cascade_buffers_.size(), decel_cascade_gains_.size());
  }

  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& /*us*/,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon; ++k) {
      const double a = PROB::a(xs, k);
      for (int i = 0; i < accel_cascade_buffers_.size(); ++i) {
        if (a > accel_cascade_buffers_[i]) {
          res.AddSubG(/*idx=*/0, accel_cascade_gains_[i] *
                                     Sqr(a - accel_cascade_buffers_[i]));
        }
      }
      for (int i = 0; i < decel_cascade_buffers_.size(); ++i) {
        if (a < decel_cascade_buffers_[i]) {
          res.AddSubG(/*idx=*/0, decel_cascade_gains_[i] *
                                     Sqr(a - decel_cascade_buffers_[i]));
        }
      }
      if (a > max_acceleration_buffer_)
        res.AddSubG(/*idx=*/0,
                    kMaxBufferGain * Sqr(a - max_acceleration_buffer_));
      else if (a < max_deceleration_buffer_)
        res.AddSubG(/*idx=*/0,
                    kMaxBufferGain * Sqr(a - max_deceleration_buffer_));
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int /*k*/, const StateType& x,
                   const ControlType& /*u*/) const override {
    double g = 0.0;
    for (int i = 0; i < accel_cascade_buffers_.size(); ++i) {
      if (x[PROB::kStateAIndex] > accel_cascade_buffers_[i]) {
        g += accel_cascade_gains_[i] *
             Sqr(x[PROB::kStateAIndex] - accel_cascade_buffers_[i]);
      }
    }
    for (int i = 0; i < decel_cascade_buffers_.size(); ++i) {
      if (x[PROB::kStateAIndex] < decel_cascade_buffers_[i]) {
        g += decel_cascade_gains_[i] *
             Sqr(x[PROB::kStateAIndex] - decel_cascade_buffers_[i]);
      }
    }
    if (x[PROB::kStateAIndex] > max_acceleration_buffer_) {
      g += kMaxBufferGain *
           Sqr(x[PROB::kStateAIndex] - max_acceleration_buffer_);
    } else if (x[PROB::kStateAIndex] < max_deceleration_buffer_) {
      g += kMaxBufferGain *
           Sqr(x[PROB::kStateAIndex] - max_deceleration_buffer_);
    }
    return g * 0.5 * Cost<PROB>::scale();
  }

  // Gradients with superposition.
  void AddDGDx(int /*k*/, const StateType& x, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    for (int i = 0; i < accel_cascade_buffers_.size(); ++i) {
      if (x[PROB::kStateAIndex] > accel_cascade_buffers_[i]) {
        (*dgdx)[PROB::kStateAIndex] +=
            Cost<PROB>::scale() * accel_cascade_gains_[i] *
            (x[PROB::kStateAIndex] - accel_cascade_buffers_[i]);
      }
    }
    for (int i = 0; i < decel_cascade_buffers_.size(); ++i) {
      if (x[PROB::kStateAIndex] < decel_cascade_buffers_[i]) {
        (*dgdx)[PROB::kStateAIndex] +=
            Cost<PROB>::scale() * decel_cascade_gains_[i] *
            (x[PROB::kStateAIndex] - decel_cascade_buffers_[i]);
      }
    }
    if (x[PROB::kStateAIndex] > max_acceleration_buffer_) {
      (*dgdx)[PROB::kStateAIndex] +=
          kMaxBufferGain * Cost<PROB>::scale() *
          (x[PROB::kStateAIndex] - max_acceleration_buffer_);
    } else if (x[PROB::kStateAIndex] < max_deceleration_buffer_) {
      (*dgdx)[PROB::kStateAIndex] +=
          kMaxBufferGain * Cost<PROB>::scale() *
          (x[PROB::kStateAIndex] - max_deceleration_buffer_);
    }
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int /*k*/, const StateType& x, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    for (int i = 0; i < accel_cascade_buffers_.size(); ++i) {
      if (x[PROB::kStateAIndex] > accel_cascade_buffers_[i]) {
        (*ddgdxdx)(PROB::kStateAIndex, PROB::kStateAIndex) +=
            Cost<PROB>::scale() * accel_cascade_gains_[i];
      }
    }
    for (int i = 0; i < decel_cascade_buffers_.size(); ++i) {
      if (x[PROB::kStateAIndex] < decel_cascade_buffers_[i]) {
        (*ddgdxdx)(PROB::kStateAIndex, PROB::kStateAIndex) +=
            Cost<PROB>::scale() * decel_cascade_gains_[i];
      }
    }
    if (x[PROB::kStateAIndex] > max_acceleration_buffer_) {
      (*ddgdxdx)(PROB::kStateAIndex, PROB::kStateAIndex) +=
          kMaxBufferGain * Cost<PROB>::scale();
    } else if (x[PROB::kStateAIndex] < max_deceleration_buffer_) {
      (*ddgdxdx)(PROB::kStateAIndex, PROB::kStateAIndex) +=
          kMaxBufferGain * Cost<PROB>::scale();
    }
  }
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

 private:
  std::vector<double> accel_cascade_buffers_;
  std::vector<double> accel_cascade_gains_;
  std::vector<double> decel_cascade_buffers_;
  std::vector<double> decel_cascade_gains_;
  double max_acceleration_buffer_;
  double max_deceleration_buffer_;
  static constexpr double kMaxBufferGain = 10000.0;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_LONGITUDINAL_ACCELERATION_COST_H_
