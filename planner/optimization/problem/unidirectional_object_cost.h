#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_UNIDIRECTIONAL_OBJECT_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_UNIDIRECTIONAL_OBJECT_COST_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class UnidirectionalObjectCost : public Cost<PROB> {
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

  struct Object {
    // Direction that the cost decrease in.
    Vec2d dir = Vec2d::UnitX();
    // Reference point.
    Vec2d ref = Vec2d::Zero();
    // Lateral extent (full extent is twice this much).
    double lateral_extent = 0.0;
    std::vector<double> buffers;
    // Individual object can have a variable gain on top of the uniform scale.
    std::vector<double> gains;
    bool enable = false;
  };

  static constexpr double kNormalizedScale = 1.0;
  explicit UnidirectionalObjectCost(
      std::vector<Object> objects, std::vector<double> dist_to_rac,
      std::vector<double> angle_to_axis, std::vector<std::string> sub_names,
      bool using_hessian_approximate,
      std::string name = "UnidirectionalObjectCost", double scale = 1.0,
      CostType cost_type = Cost<PROB>::CostType::GROUP_OBJECT)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        objects_(std::move(objects)),
        dist_to_rac_(std::move(dist_to_rac)),
        angle_to_axis_(std::move(angle_to_axis)),
        num_objects_(objects_.size()),
        circle_size_(dist_to_rac_.size()),
        sub_names_(std::move(sub_names)),
        using_hessian_approximate_(using_hessian_approximate) {
    QCHECK_EQ(dist_to_rac_.size(), angle_to_axis_.size());
    rotation_.reserve(circle_size_);
    for (const double angle : angle_to_axis_) {
      rotation_.push_back(Vec2d::FastUnitFromAngle(angle));
    }
    penetrations_.resize(num_objects_);
    penetration_jacobians_.resize(num_objects_);
    penetration_hessians_.resize(num_objects_);
    filtered_.resize(num_objects_);
    for (int k = 0; k < num_objects_; ++k) {
      penetration_jacobians_[k].resize(circle_size_,
                                       PenetrationJacobianType::Zero());
      penetration_hessians_[k].resize(circle_size_,
                                      PenetrationHessianType::Zero());
      filtered_[k].resize(circle_size_, 0);
      penetrations_[k].resize(circle_size_);
      const auto& ob = objects_[k];
      QCHECK_EQ(ob.buffers.size(), ob.gains.size());
      QCHECK_EQ(ob.buffers.size(), sub_names_.size());
      for (int i = 0; i < circle_size_; ++i) {
        penetrations_[k][i].resize(ob.buffers.size(),
                                   std::numeric_limits<double>::infinity());
      }
    }
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    DividedG res(sub_names_.size());
    horizon = std::min(horizon, num_objects_);
    for (int k = 0; k < horizon; ++k) {
      const auto& penetration_k = penetrations_[k];
      const auto& ob = objects_[k];
      for (int i = 0; i < circle_size_; ++i) {
        const auto& penetration_ki = penetration_k[i];
        for (int j = 0; j < penetration_ki.size(); ++j) {
          if (penetration_ki[j] < 0.0) {
            res.AddSubG(j, ob.gains[j] * Sqr(penetration_ki[j]));
          }
        }
      }
    }
    for (int j = 0; j < sub_names_.size(); ++j) {
      res.SetSubName(j, sub_names_[j] + Cost<PROB>::name());
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return 0.0;
    const auto& penetration_k = penetrations_[k];
    const auto& ob = objects_[k];
    double g = 0.0;
    for (int i = 0; i < circle_size_; ++i) {
      const auto& penetration_ki = penetration_k[i];
      for (int j = 0; j < penetration_ki.size(); ++j) {
        if (penetration_ki[j] < 0.0) {
          g += 0.5 * Cost<PROB>::scale() * ob.gains[j] * Sqr(penetration_ki[j]);
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
    const auto& penetration_k = penetrations_[k];
    const auto& penetration_jacobian_k = penetration_jacobians_[k];
    const auto& ob = objects_[k];
    for (int i = 0; i < circle_size_; ++i) {
      const auto& penetration_jacobian = penetration_jacobian_k[i];
      const auto& penetration_ki = penetration_k[i];
      for (int j = 0; j < penetration_ki.size(); ++j) {
        if (penetration_ki[j] < 0.0) {
          *dgdx += Cost<PROB>::scale() * ob.gains[j] * penetration_ki[j] *
                   penetration_jacobian;
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
    const auto& penetration_k = penetrations_[k];
    const auto& penetration_jacobian_k = penetration_jacobians_[k];
    const auto& penetration_hessian_k = penetration_hessians_[k];
    const auto& ob = objects_[k];

    for (int i = 0; i < circle_size_; ++i) {
      const auto& penetration_jacobian = penetration_jacobian_k[i];
      const auto& penetration_hessian = penetration_hessian_k[i];
      const auto& penetration_ki = penetration_k[i];
      for (int j = 0; j < penetration_ki.size(); ++j) {
        if (penetration_ki[j] < 0.0) {
          if (using_hessian_approximate_) {
            *ddgdxdx += Cost<PROB>::scale() * ob.gains[j] *
                        penetration_jacobian.transpose() * penetration_jacobian;
          } else {
            *ddgdxdx +=
                Cost<PROB>::scale() * ob.gains[j] *
                (penetration_ki[j] * penetration_hessian +
                 penetration_jacobian.transpose() * penetration_jacobian);
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
      const Object& ob = objects_[k];
      auto& filtered_k = filtered_[k];
      auto& penetration_k = penetrations_[k];
      const Vec2d av_pos = PROB::pos(xs, k);
      const Vec2d av_tangent_rac =
          Vec2d::FastUnitFromAngleN12(PROB::theta(xs, k));
      for (int i = 0; i < circle_size_; ++i) {
        filtered_k[i] = false;
        std::vector<double>& penetration_ki = penetration_k[i];
        for (int j = 0; j < penetration_ki.size(); ++j) {
          penetration_ki[j] = std::numeric_limits<double>::infinity();
        }
        if (!ob.enable) continue;
        const Vec2d av_tangent = av_tangent_rac.Rotate(rotation_[i]);
        const Vec2d x_fac = av_pos + av_tangent * dist_to_rac_[i];
        const Vec2d& dir = ob.dir;
        const double lat_offset = dir.CrossProd(x_fac - ob.ref);
        if (lat_offset > ob.lateral_extent || lat_offset < -ob.lateral_extent) {
          filtered_k[i] = true;
          continue;
        }
        const double dist = (x_fac - ob.ref).dot(dir);
        for (int j = 0; j < penetration_ki.size(); ++j) {
          penetration_ki[j] = dist - ob.buffers[j];
        }
      }
    }
  }

  void UpdateDerivatives(const StatesType& xs, const ControlsType& /*us*/,
                         int horizon) override {
    horizon = std::min(horizon, num_objects_);
    for (int k = 0; k < horizon; ++k) {
      auto& penetration_jacobian_k = penetration_jacobians_[k];
      auto& penetration_hessian_k = penetration_hessians_[k];
      const auto& filtered_k = filtered_[k];
      const Object& ob = objects_[k];
      const Vec2d av_tangent_rac =
          Vec2d::FastUnitFromAngleN12(PROB::theta(xs, k));
      for (int i = 0; i < circle_size_; ++i) {
        PenetrationJacobianType& penetration_jacobian_ki =
            penetration_jacobian_k[i];
        PenetrationHessianType& penetration_hessian_ki =
            penetration_hessian_k[i];
        penetration_jacobian_ki(PROB::kStateXIndex) = 0.0;
        penetration_jacobian_ki(PROB::kStateYIndex) = 0.0;
        penetration_jacobian_ki(PROB::kStateThetaIndex) = 0.0;
        penetration_hessian_ki(PROB::kStateThetaIndex, PROB::kStateThetaIndex) =
            0.0;
        if (!ob.enable) continue;
        if (filtered_k[i]) {
          continue;
        }
        const Vec2d av_tangent = av_tangent_rac.Rotate(rotation_[i]);
        const Vec2d av_normal = av_tangent.Perp();
        const Vec2d& dir = ob.dir;
        penetration_jacobian_ki.template segment<2>(PROB::kStateXIndex) =
            dir.transpose();
        penetration_jacobian_ki(PROB::kStateThetaIndex) =
            dir.dot(av_normal * dist_to_rac_[i]);
        penetration_hessian_ki(PROB::kStateThetaIndex, PROB::kStateThetaIndex) =
            dir.dot(-av_tangent * dist_to_rac_[i]);
      }
    }
  }

 private:
  std::vector<Object> objects_;
  std::vector<double> dist_to_rac_;
  std::vector<double> angle_to_axis_;
  std::vector<Vec2d> rotation_;
  int num_objects_;
  int circle_size_;
  std::vector<std::string> sub_names_;
  bool using_hessian_approximate_;

  // States.
  using PenetrationJacobianType = Eigen::Matrix<double, 1, PROB::kStateSize>;
  using PenetrationHessianType =
      Eigen::Matrix<double, PROB::kStateSize, PROB::kStateSize>;

  std::vector<std::vector<std::vector<double>>> penetrations_;
  std::vector<std::vector<char>> filtered_;
  std::vector<std::vector<PenetrationJacobianType>> penetration_jacobians_;
  std::vector<std::vector<PenetrationHessianType>> penetration_hessians_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_UNIDIRECTIONAL_OBJECT_COST_H_
