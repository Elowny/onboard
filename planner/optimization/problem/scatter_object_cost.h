#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SCATTER_OBJECT_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SCATTER_OBJECT_COST_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class ScatterStaticObjectCost : public Cost<PROB> {
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

  using PenetrationJacobianType = Eigen::Matrix<double, 1, PROB::kStateSize>;
  using PenetrationHessianType =
      Eigen::Matrix<double, PROB::kStateSize, PROB::kStateSize>;

  static constexpr double kNormalizedScale = 1.0;
  ScatterStaticObjectCost(
      std::vector<Vec2d> points, std::vector<double> dist_to_rac,
      std::vector<double> angle_to_axis,
      std::vector<std::vector<std::vector<double>>> buffers,
      std::vector<double> gains, std::vector<std::string> sub_names,
      int num_objects, bool using_hessian_approximate,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "ScatterStaticObjectCost"),
      double scale = 1.0,
      CostType cost_type = Cost<PROB>::CostType::GROUP_OBJECT)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        num_objects_(num_objects),
        dist_to_rac_(std::move(dist_to_rac)),
        angle_to_axis_(std::move(angle_to_axis)),
        circle_size_(dist_to_rac_.size()),
        points_(std::move(points)),
        buffers_(std::move(buffers)),
        gains_(std::move(gains)),
        sub_names_(std::move(sub_names)),
        using_hessian_approximate_(using_hessian_approximate) {
    QCHECK_EQ(sub_names_.size(), gains_.size());
    QCHECK_EQ(dist_to_rac_.size(), angle_to_axis_.size());
    QCHECK_EQ(buffers_.size(), points_.size());
    for (const auto& circle_buffers : buffers_) {
      QCHECK_EQ(circle_buffers.size(), dist_to_rac_.size());
      for (const auto& segment_buffers : circle_buffers) {
        QCHECK_EQ(segment_buffers.size(), sub_names_.size());
      }
    }
    rotation_.reserve(circle_size_);
    for (const double angle : angle_to_axis_) {
      rotation_.push_back(Vec2d::FastUnitFromAngle(angle));
    }

    penetrations_.resize(num_objects_);
    points_ptr_.resize(num_objects_);
    penetration_jacobians_.resize(num_objects_);
    penetration_hessians_.resize(num_objects_);
    for (int k = 0; k < num_objects_; ++k) {
      penetrations_[k].resize(circle_size_);
      penetration_jacobians_[k].resize(circle_size_,
                                       PenetrationJacobianType::Zero());
      penetration_hessians_[k].resize(circle_size_,
                                      PenetrationHessianType::Zero());
      points_ptr_[k].resize(circle_size_, nullptr);
    }

    for (int i = 0, n = num_objects_; i < n; ++i) {
      for (int idx = 0; idx < circle_size_; ++idx) {
        auto& penetration = penetrations_[i][idx];
        penetration.reserve(gains_.size());
        for (int k = 0; k < gains_.size(); ++k) {
          penetration.push_back(std::numeric_limits<double>::infinity());
        }
      }
    }
  }

  const std::vector<std::vector<std::vector<double>>>& penetrations() const {
    return penetrations_;
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
      res.SetIsSoft(i, sub_names_[i] == "a");
    }
    horizon = std::min(horizon, num_objects_);
    for (int k = 0; k < horizon; ++k) {
      for (int idx = 0; idx < circle_size_; ++idx) {
        const auto penetrations_k = absl::MakeConstSpan(penetrations_[k][idx]);
        for (int i = 0; i < penetrations_k.size(); ++i) {
          if (penetrations_k[i] < 0.0) {
            res.AddSubG(i, gains_[i] * Sqr(penetrations_k[i]));
          }
        }
      }
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType& /*x*/,
                                  const ControlType& /*u*/,
                                  bool using_scale) const override {
    DCHECK_GE(k, 0);
    std::vector<double> gains(gains_.size());
    if (using_scale) {
      gains = gains_;
    } else {
      std::fill(gains.begin(), gains.end(), 1.0);
    }
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    if (k >= num_objects_) return res;
    for (int idx = 0; idx < circle_size_; ++idx) {
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k][idx]);
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          res.AddSubG(
              i, 0.5 * Cost<PROB>::scale() * gains[i] * Sqr(penetrations_k[i]));
        }
      }
    }
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return 0.0;
    double g = 0.0;
    for (int idx = 0; idx < circle_size_; ++idx) {
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k][idx]);
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          g += 0.5 * Cost<PROB>::scale() * gains_[i] * Sqr(penetrations_k[i]);
        }
      }
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return;
    for (int idx = 0; idx < circle_size_; ++idx) {
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k][idx]);
      const auto& penetration_jacobians_k = penetration_jacobians_[k][idx];
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          *dgdx += Cost<PROB>::scale() * gains_[i] * penetrations_k[i] *
                   penetration_jacobians_k;
        }
      }
    }
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return;
    for (int idx = 0; idx < circle_size_; ++idx) {
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k][idx]);
      const auto& penetration_jacobians_k = penetration_jacobians_[k][idx];
      const auto& penetration_hessians_k = penetration_hessians_[k][idx];
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          if (using_hessian_approximate_) {
            *ddgdxdx +=
                Cost<PROB>::scale() * gains_[i] *
                (penetration_jacobians_k.transpose() * penetration_jacobians_k);
          } else {
            *ddgdxdx +=
                Cost<PROB>::scale() * gains_[i] *
                (penetrations_k[i] * penetration_hessians_k +
                 penetration_jacobians_k.transpose() * penetration_jacobians_k);
          }
        }
      }
    }
  }
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

  void Update(const StatesType& xs, const ControlsType& /*us*/,
              int horizon) override {
    horizon = std::min(horizon, num_objects_);
    for (int k = 0; k < horizon; ++k) {
      const auto x0 = PROB::GetStateAtStep(xs, k);
      // Vehicle position, rear center.
      const Vec2d pos = PROB::pos(xs, k);
      const Vec2d rac_tangent =
          Vec2d::FastUnitFromAngleN12(PROB::StateGetTheta(x0));
      for (int idx = 0; idx < circle_size_; ++idx) {
        auto penetrations_k = absl::MakeSpan(penetrations_[k][idx]);
        const double dist_to_rac = dist_to_rac_[idx];
        const Vec2d tangent = rac_tangent.Rotate(rotation_[idx]);
        const double xr = pos.x() + dist_to_rac * tangent.x();
        const double yr = pos.y() + dist_to_rac * tangent.y();
        const Vec2d pt = {xr, yr};
        int nearest_index = 0;
        double nearest_square_dist = std::numeric_limits<double>::infinity();
        for (int idx = 0; idx < points_.size(); ++idx) {
          const Vec2d& point = points_[idx];
          const double square_dist = (pt - point).squaredNorm();
          if (square_dist < nearest_square_dist) {
            nearest_square_dist = square_dist;
            nearest_index = idx;
          }
        }
        points_ptr_[k][idx] = &points_[nearest_index];
        const std::vector<double>& buffers = buffers_[nearest_index][idx];
        const double dist = std::sqrt(nearest_square_dist);
        for (int i = 0; i < penetrations_k.size(); ++i) {
          penetrations_k[i] = dist - buffers[i];
        }
      }
    }
  }

  void UpdateDerivatives(const StatesType& xs, const ControlsType& /*us*/,
                         int horizon) override {
    constexpr double kEps = 1e-9;
    horizon = std::min(horizon, num_objects_);
    for (int k = 0; k < horizon; ++k) {
      const auto x0 = PROB::GetStateAtStep(xs, k);
      const Vec2d rac_tangent =
          Vec2d::FastUnitFromAngleN12(PROB::StateGetTheta(x0));
      for (int idx = 0; idx < circle_size_; ++idx) {
        auto& penetration_jacobians_k = penetration_jacobians_[k][idx];
        auto& penetration_hessians_k = penetration_hessians_[k][idx];

        penetration_jacobians_k.template segment<3>(PROB::kStateXIndex) =
            Eigen::Matrix<double, 3, 1>::Zero();
        penetration_hessians_k.template block<3, 3>(PROB::kStateXIndex,
                                                    PROB::kStateXIndex) =
            Eigen::Matrix<double, 3, 3>::Zero();

        const double dist_to_rac = dist_to_rac_[idx];

        const Vec2d tangent = rac_tangent.Rotate(rotation_[idx]);
        VLOG(3) << "Step " << k;
        VLOG(4) << "x0 = " << x0.transpose()
                << " tangent = " << tangent.transpose();
        // Vehicle position, rear center.
        const Vec2d pos = PROB::pos(xs, k);
        const double xr = pos.x() + dist_to_rac * tangent.x();
        const double yr = pos.y() + dist_to_rac * tangent.y();
        const Vec2d pt = {xr, yr};
        const Vec2d normal = tangent.Perp();

        const auto& pt2 = *points_ptr_[k][idx];
        const Vec2d x1 = pt - pt2;
        const double dist = x1.norm();
        const double dist_inv = dist < kEps ? 0.0 : 1.0 / dist;
        const double dist_inv_cube = Cube(dist_inv);
        penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
            x1 * dist_inv;
        penetration_jacobians_k(PROB::kStateThetaIndex) =
            x1.dot(normal * dist_to_rac) * dist_inv;
        penetration_hessians_k(PROB::kStateXIndex, PROB::kStateXIndex) =
            dist_inv - Sqr(x1.x()) * dist_inv_cube;
        penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex) =
            -x1.x() * x1.y() * dist_inv_cube;
        penetration_hessians_k(PROB::kStateXIndex, PROB::kStateThetaIndex) =
            normal.x() * dist_to_rac * dist_inv -
            dist_inv_cube * x1.x() * x1.dot(normal * dist_to_rac);

        penetration_hessians_k(PROB::kStateYIndex, PROB::kStateXIndex) =
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex);
        penetration_hessians_k(PROB::kStateYIndex, PROB::kStateYIndex) =
            dist_inv - Sqr(x1.y()) * dist_inv_cube;
        penetration_hessians_k(1, PROB::kStateThetaIndex) =
            normal.y() * dist_to_rac * dist_inv -
            dist_inv_cube * x1.y() * x1.dot(normal * dist_to_rac);

        penetration_hessians_k(PROB::kStateThetaIndex, PROB::kStateXIndex) =
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateThetaIndex);
        penetration_hessians_k(PROB::kStateThetaIndex, PROB::kStateYIndex) =
            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateThetaIndex);
        penetration_hessians_k(PROB::kStateThetaIndex, PROB::kStateThetaIndex) =
            (x1.dot(-tangent * dist_to_rac) +
             dist_to_rac * normal.dot(normal * dist_to_rac)) *
                dist_inv -
            dist_inv_cube * x1.dot(normal * dist_to_rac) *
                x1.dot(normal * dist_to_rac);
      }
    }
  }

 private:
  int num_objects_;

  // Distances from control points to RAC on vehicle longitudinal axis.
  std::vector<double> dist_to_rac_;
  std::vector<double> angle_to_axis_;
  std::vector<Vec2d> rotation_;
  int circle_size_;

  std::vector<Vec2d> points_;
  // Buffer to static object
  std::vector<std::vector<std::vector<double>>> buffers_;
  std::vector<double> gains_;
  std::vector<std::string> sub_names_;
  bool using_hessian_approximate_;

  // States.
  std::vector<std::vector<std::vector<double>>> penetrations_;
  std::vector<std::vector<const Vec2d*>> points_ptr_;
  std::vector<std::vector<PenetrationJacobianType>> penetration_jacobians_;
  std::vector<std::vector<PenetrationHessianType>> penetration_hessians_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SCATTER_OBJECT_COST_H_
