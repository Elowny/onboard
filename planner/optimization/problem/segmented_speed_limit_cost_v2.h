#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SEGMENTED_SPEED_LIMIT_COST2_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SEGMENTED_SPEED_LIMIT_COST2_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
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

// TODO(Fang) use hysteresis in gain switches.
template <typename PROB>
class SegmentedSpeedLimitCostV2 : public Cost<PROB> {
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

  struct SpeedlimitInfoPoint {
    double alpha = 0.0;
    double speed_limit = 0.0;
  };

  // Speed limit info is bounded to ref path points.
  // Speed limit point with ref points:
  // •————•——————•————•————•————•————•——————•————•————•————•
  //   •-•  •-•-•                 •   •-•-•   •-•       •
  // Additional speed limit(upper line).
  class SpeedLimitInfo {
   public:
    SpeedLimitInfo(const std::vector<double>& speed_limits,
                   const std::vector<std::vector<SpeedlimitInfoPoint>>&
                       additional_speed_limits)
        : speed_limits_(speed_limits),
          additional_speed_limits_(additional_speed_limits),
          size_(speed_limits_.size()) {
      QCHECK_EQ(speed_limits_.size(), additional_speed_limits_.size() + 1);
      for (const auto& speed_limit_info : additional_speed_limits) {
        for (int i = 0; i < speed_limit_info.size(); ++i) {
          // TODO(runbing): Use check instead of qevent.
          if (speed_limit_info[i].alpha > 1.0) {
            QEVENT("runbing", "speed_limit_cost_additional_info_LE",
                   [&](QEvent* qevent) {
                     qevent->AddField("alpha", speed_limit_info[i].alpha);
                   });
          }
          if (speed_limit_info[i].alpha < 0.0) {
            QEVENT("runbing", "speed_limit_cost_additional_info_GE",
                   [&](QEvent* qevent) {
                     qevent->AddField("alpha", speed_limit_info[i].alpha);
                   });
          }
        }
        for (int i = 1; i < speed_limit_info.size(); ++i) {
          CHECK_LE(speed_limit_info[i - 1].alpha, speed_limit_info[i].alpha);
        }
      }
    }

    int size() { return size_; }

    std::pair<SpeedlimitInfoPoint, SpeedlimitInfoPoint> GetSpeedLimitInfoPair(
        int index, double alpha) {
      QCHECK_GE(index, 0);
      QCHECK_LE(index, size_ - 1);
      const auto& additional_speed_limit = additional_speed_limits_[index];
      if (alpha < 0.0) {
        return {{0.0, speed_limits_[index]}, {0.0, speed_limits_[index]}};
      } else if (alpha >= 1.0) {
        return {{1.0, speed_limits_[index + 1]},
                {1.0, speed_limits_[index + 1]}};
      } else {
        if (additional_speed_limit.empty()) {
          return {{0.0, speed_limits_[index]}, {1.0, speed_limits_[index + 1]}};
        } else {
          if (alpha < additional_speed_limit.front().alpha) {
            return {{0.0, speed_limits_[index]},
                    additional_speed_limit.front()};
          } else if (alpha >= additional_speed_limit.back().alpha) {
            return {additional_speed_limit.back(),
                    {1.0, speed_limits_[index + 1]}};
          } else {
            const auto max_iter = std::upper_bound(
                additional_speed_limit.begin(), additional_speed_limit.end(),
                alpha, [](double value, const SpeedlimitInfoPoint& pt) {
                  return value < pt.alpha;
                });
            const int index =
                std::distance(additional_speed_limit.begin(), max_iter);
            return {additional_speed_limit[index - 1],
                    additional_speed_limit[index]};
          }
        }
      }
    }

   private:
    std::vector<double> speed_limits_;
    std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits_;
    int size_;
  };

  static constexpr double kNormalizedScale = 0.01;
  explicit SegmentedSpeedLimitCostV2(
      int horizon, const std::vector<Vec2d>& ref_points,
      const CenterLineQueryHelper<PROB>* center_line_query_helper,
      SpeedLimitInfo speed_limits, SpeedLimitInfo free_speed_limits,
      int free_index,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "SegmentedSpeedLimitCostV2"),
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::MUST_HAVE,
      double stop_speed_gain = kDefaultStopSpeedCostGain,
      double over_speed_gain = kDefaultOverSpeedCostGain,
      double under_speed_gain = kDefaultUnderSpeedCostGain,
      bool use_qtfm = false)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        horizon_(horizon),
        center_line_query_helper_(center_line_query_helper),
        speed_limits_(std::move(speed_limits)),
        free_speed_limits_(std::move(free_speed_limits)),
        free_index_(free_index),
        stop_speed_gain_(stop_speed_gain),
        over_speed_gain_(over_speed_gain),
        under_speed_gain_(under_speed_gain) {
    QCHECK_GT(horizon_, 0);

    if (center_line_query_helper == nullptr) {
      QCHECK_GT(ref_points.size(), 1);
      QCHECK_EQ(ref_points.size(), speed_limits_.size());
      QCHECK_EQ(ref_points.size(), free_speed_limits_.size());
    } else {
      QCHECK_GT(center_line_query_helper->points().size(), 1);
      QCHECK_EQ(center_line_query_helper->points().size(),
                speed_limits_.size());
      QCHECK_EQ(center_line_query_helper->points().size(),
                free_speed_limits_.size());
    }

    if (center_line_query_helper == nullptr) {
      if (use_qtfm) {
        ref_path_ = std::make_unique<QtfmEnhancedKdTreeFrenetFrame>(
            BuildQtfmEnhancedKdTreeFrenetFrame(ref_points,
                                               /*down_sample_raw_points=*/true)
                .value());
      } else {
        ref_path_ = std::make_unique<KdTreeFrenetFrame>(
            BuildKdTreeFrenetFrame(ref_points, /*down_sample_raw_points=*/true)
                .value());
      }

      lengths_inv_.reserve(ref_points.size() - 1);
      for (int idx = 0; idx < ref_points.size() - 1; ++idx) {
        lengths_inv_.push_back(1.0 /
                               (ref_points[idx + 1] - ref_points[idx]).norm());
      }
    } else {
      const std::vector<Vec2d>& points = center_line_query_helper->points();
      lengths_inv_.reserve(points.size() - 1);
      for (int idx = 0; idx < points.size() - 1; ++idx) {
        lengths_inv_.push_back(1.0 / (points[idx + 1] - points[idx]).norm());
      }
    }

    speed_diff_.resize(horizon_);
    gain_.resize(horizon_);
    speed_limit_factor_.resize(horizon_);
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    DCHECK_EQ(horizon, horizon_);
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon_; ++k) {
      res.AddSubG(/*idx=*/0, gain_[k] * Sqr(speed_diff_[k]));
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    return 0.5 * Cost<PROB>::scale() * gain_[k] * Sqr(speed_diff_[k]);
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    const double speed_diff_k = speed_diff_[k];
    const auto& speed_limit_factor_k = speed_limit_factor_[k];
    const double dgdv = Cost<PROB>::scale() * gain_[k] * speed_diff_k;
    (*dgdx)[PROB::kStateVIndex] += dgdv;
    (*dgdx).template segment<2>(PROB::kStateXIndex) +=
        dgdv * speed_limit_factor_k;
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    const double gain = Cost<PROB>::scale() * gain_[k];
    const auto& speed_limit_factor_k = speed_limit_factor_[k];
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateVIndex) += gain;
    const Vec2d ddgdvdxy = gain * speed_limit_factor_k;
    const Vec2d factor_multi =
        Vec2d(Sqr(speed_limit_factor_k.x()), Sqr(speed_limit_factor_k.y()));
    const Vec2d ddgdxydxy = gain * factor_multi;
    const double ddgdxdy =
        gain * speed_limit_factor_k.x() * speed_limit_factor_k.y();
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateXIndex) += ddgdvdxy[0];
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateYIndex) += ddgdvdxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateVIndex) += ddgdvdxy[0];
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateVIndex) += ddgdvdxy[1];
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
    // Compensation for low forward force when speed limit is low, by
    // increasing the under-speed gain.
    DCHECK_EQ(horizon, horizon_);

    if (center_line_query_helper_ == nullptr) {
      for (int k = 0; k < horizon_; ++k) {
        const Vec2d pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
        FrenetCoordinate sl;
        Vec2d normal;
        int index;
        double alpha = 0.0;
        ref_path_->XYToSL(pos, &sl, &normal, &index, &alpha);
        if (k <= free_index_) {
          UpdateSpeedLimitInfo(
              PROB::v(xs, k), speed_limits_.GetSpeedLimitInfoPair(index, alpha),
              lengths_inv_[index], alpha, normal, &speed_diff_[k], &gain_[k],
              &speed_limit_factor_[k]);
        } else {
          UpdateSpeedLimitInfo(
              PROB::v(xs, k),
              free_speed_limits_.GetSpeedLimitInfoPair(index, alpha),
              lengths_inv_[index], alpha, normal, &speed_diff_[k], &gain_[k],
              &speed_limit_factor_[k]);
        }
      }
    } else {
      const auto& rac_index_pairs = center_line_query_helper_->index_pairs();
      const auto& rac_normals = center_line_query_helper_->normals();
      const auto& rac_alphas = center_line_query_helper_->alphas();
      for (int k = 0; k < horizon_; ++k) {
        const Vec2d& normal = rac_normals[k];
        const int index = rac_index_pairs[k].first;
        const double alpha = rac_alphas[k];
        if (k <= free_index_) {
          UpdateSpeedLimitInfo(
              PROB::v(xs, k), speed_limits_.GetSpeedLimitInfoPair(index, alpha),
              lengths_inv_[index], alpha, normal, &speed_diff_[k], &gain_[k],
              &speed_limit_factor_[k]);
        } else {
          UpdateSpeedLimitInfo(
              PROB::v(xs, k),
              free_speed_limits_.GetSpeedLimitInfoPair(index, alpha),
              lengths_inv_[index], alpha, normal, &speed_diff_[k], &gain_[k],
              &speed_limit_factor_[k]);
        }
      }
    }
  }

 private:
  void UpdateSpeedLimitInfo(
      double v,
      std::pair<SpeedlimitInfoPoint, SpeedlimitInfoPoint> speed_limit_pair,
      double segment_length_inv, double alpha, const Vec2d& normal,
      double* speed_diff, double* gain, Vec2d* speed_limit_factor) {
    const auto& speed_limit_info = speed_limit_pair.first;
    const auto& speed_limit_info_next = speed_limit_pair.second;
    const double length = speed_limit_info_next.alpha - speed_limit_info.alpha;
    constexpr double kEps = 1e-7;
    const double speed_alpha =
        length > kEps ? ((alpha - speed_limit_info.alpha) / length) : 0.0;
    const double speed_limit =
        Lerp(speed_limit_info.speed_limit, speed_limit_info_next.speed_limit,
             speed_alpha);

    const double speed_diff_tmp = v - speed_limit;
    *speed_diff = speed_diff_tmp;

    if (length < kEps) {
      *speed_limit_factor = Vec2d(0.0, 0.0);
    } else {
      *speed_limit_factor =
          (speed_limit_info_next.speed_limit - speed_limit_info.speed_limit) *
          normal.Perp() / length * segment_length_inv;
    }

    if (speed_limit == 0.0) {
      *gain = stop_speed_gain_;
    } else {
      *gain = speed_diff_tmp > 0.0 ? over_speed_gain_ : under_speed_gain_;
    }
  }

 protected:
  static constexpr double kDefaultStopSpeedCostGain = 800.0;
  static constexpr double kDefaultOverSpeedCostGain = 100.0;
  static constexpr double kDefaultUnderSpeedCostGain = 1.0;

 private:
  int horizon_ = 0;
  std::unique_ptr<FrenetFrame> ref_path_;
  const CenterLineQueryHelper<PROB>* center_line_query_helper_;
  std::vector<double> lengths_inv_;
  SpeedLimitInfo speed_limits_;
  SpeedLimitInfo free_speed_limits_;
  int free_index_;
  // State index beyond this value uses free speed limit.
  double stop_speed_gain_;
  double over_speed_gain_;
  double under_speed_gain_;

  // States.
  std::vector<double> speed_diff_;
  std::vector<double> gain_;
  std::vector<Vec2d> speed_limit_factor_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SEGMENTED_SPEED_LIMIT_COST2_H_
