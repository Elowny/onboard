#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_END_HEADING_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_END_HEADING_COST_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/center_line_query_helper.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class EndHeadingCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 5.0;
  EndHeadingCost(int horizon, std::vector<double> ref_thetas,
                 const std::vector<Vec2d>& ref_points,
                 const CenterLineQueryHelper<PROB>* center_line_helper,
                 std::vector<double> gains,
                 std::string name = absl::StrCat(PROB::kProblemPrefix,
                                                 "EndHeadingCost"),
                 double scale = 1.0,
                 CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        horizon_(horizon),
        ref_thetas_(std::move(ref_thetas)),
        center_line_helper_(center_line_helper),
        ref_gains_(std::move(gains)),
        ref_thetas_size_(ref_thetas_.size()) {
    if (center_line_helper_ == nullptr) {
      QCHECK_GT(ref_points.size(), 1);
      QCHECK_LT(ref_thetas_.size(), ref_points.size());
      QCHECK_LT(ref_gains_.size(), ref_points.size());
      ref_path_ = std::make_unique<KdTreeFrenetFrame>(
          BuildKdTreeFrenetFrame(ref_points, /*down_sample_raw_points=*/false)
              .value());
      // Make sure frenet frame has same path points with ref_points, which
      // frenet frame deduplication is illegally.
      QCHECK_EQ(ref_points.size(), ref_path_->points().size());
    } else {
      QCHECK_LT(ref_thetas_.size(), center_line_helper_->points().size());
      QCHECK_LT(ref_gains_.size(), center_line_helper_->points().size());
    }

    QCHECK_GT(horizon_, 0);
    deviations_.resize(horizon_);
    factors_.resize(horizon_);
    gains_.resize(horizon_);
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    DCHECK_EQ(horizon, horizon_);
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon_ && k < effective_index_; ++k) {
      res.AddSubG(/*idx=*/0, gains_[k] * Sqr(deviations_[k]));
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    if (k >= effective_index_) return 0.0;
    return 0.5 * gains_[k] * Cost<PROB>::scale() * Sqr(deviations_[k]);
  }

  // Gradients with Superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    if (k >= effective_index_) return;
    const double gain = gains_[k] * Cost<PROB>::scale();
    (*dgdx)(PROB::kStateThetaIndex) += gain * deviations_[k];
    (*dgdx).template segment<2>(PROB::kStateXIndex) +=
        gain * deviations_[k] * factors_[k];
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with Superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    if (k >= effective_index_) return;
    const double gain = gains_[k] * Cost<PROB>::scale();
    (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateThetaIndex) += gain;
    const auto& factor_k = factors_[k];
    const Vec2d ddgdthetadxy = gain * factor_k;
    const Vec2d ddgdxydxy = gain * Vec2d(Sqr(factor_k.x()), Sqr(factor_k.y()));
    const double ddgdxdy = gain * factor_k.x() * factor_k.y();
    (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateXIndex) += ddgdthetadxy[0];
    (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateYIndex) += ddgdthetadxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateThetaIndex) += ddgdthetadxy[0];
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateThetaIndex) += ddgdthetadxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateXIndex) += ddgdxydxy[0];
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateYIndex) += ddgdxydxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateYIndex) += ddgdxdy;
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateXIndex) += ddgdxdy;
  }
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

  void Update(const StatesType& xs, const ControlsType& /*us*/,
              int horizon) override {
    DCHECK_EQ(horizon, horizon_);
    effective_index_ = horizon_;
    constexpr double kEps = 1e-7;
    if (center_line_helper_ == nullptr) {
      const std::vector<double>& s_knot = ref_path_->s_knots();
      for (int k = 0; k < horizon_; ++k) {
        const Vec2d pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
        FrenetCoordinate sl;
        Vec2d normal;
        std::pair<int, int> raw_index_pair;
        double alpha = 0.0;
        ref_path_->XYToSL(pos, &sl, &normal, &raw_index_pair, &alpha);
        if (raw_index_pair.second >= ref_thetas_size_) {
          effective_index_ = k;
          break;
        }
        int current_index = raw_index_pair.first;
        int next_index = std::clamp(raw_index_pair.second, 0,
                                    static_cast<int>(ref_thetas_.size() - 1));
        const auto curent_ref_theta = ref_thetas_[current_index];
        const auto next_ref_theta = ref_thetas_[next_index];
        const double ref_theta =
            LerpAngle(curent_ref_theta, next_ref_theta, alpha);
        const double arc_length = s_knot[next_index] - s_knot[current_index];
        if (arc_length < kEps) {
          factors_[k] = Vec2d(0.0, 0.0);
        } else {
          factors_[k] = NormalizeAngle(next_ref_theta - curent_ref_theta) *
                        normal.Perp() / arc_length;
        }
        deviations_[k] = NormalizeAngle(
            PROB::StateGetTheta(PROB::GetStateAtStep(xs, k)) - ref_theta);
        gains_[k] = ref_gains_[current_index] *
                    (k == horizon_ - 1 ? kEndStateGain : kPathGain);
      }
    } else {
      const auto& rac_index_pairs = center_line_helper_->index_pairs();
      const auto& alphas = center_line_helper_->alphas();
      const std::vector<double>& s_knot = center_line_helper_->s_knots();
      const auto& normals = center_line_helper_->normals();
      for (int k = 0; k < horizon_; ++k) {
        if (rac_index_pairs[k].second >= ref_thetas_size_) {
          effective_index_ = k;
          break;
        }
        const int current_index = rac_index_pairs[k].first;
        int next_index = std::clamp(rac_index_pairs[k].second, 0,
                                    static_cast<int>(ref_thetas_.size() - 1));
        const auto curent_ref_theta = ref_thetas_[current_index];
        const auto next_ref_theta = ref_thetas_[next_index];
        const double ref_theta =
            LerpAngle(curent_ref_theta, next_ref_theta, alphas[k]);
        const double arc_length = s_knot[next_index] - s_knot[current_index];
        if (arc_length < kEps) {
          factors_[k] = Vec2d(0.0, 0.0);
        } else {
          factors_[k] = NormalizeAngle(next_ref_theta - curent_ref_theta) *
                        normals[k].Perp() / arc_length;
        }
        deviations_[k] = NormalizeAngle(
            PROB::StateGetTheta(PROB::GetStateAtStep(xs, k)) - ref_theta);
        gains_[k] = ref_gains_[current_index] *
                    (k == horizon_ - 1 ? kEndStateGain : kPathGain);
      }
    }
  }

 protected:
  static constexpr double kPathGain = 2.0;
  static constexpr double kEndStateGain = 10.0;

 private:
  int horizon_ = 0;
  std::vector<double> ref_thetas_;
  std::unique_ptr<FrenetFrame> ref_path_;
  const CenterLineQueryHelper<PROB>* center_line_helper_;
  std::vector<double> ref_gains_;
  int ref_thetas_size_;

  // States.
  std::vector<double> deviations_;
  std::vector<Vec2d> factors_;
  std::vector<double> gains_;
  int effective_index_ = 0;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_END_HEADING_COST_H_
