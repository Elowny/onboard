#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_PARTITIONED_STATIC_BOUNDARY_COST_H  // NOLINT
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_PARTITIONED_STATIC_BOUNDARY_COST_H  // NOLINT

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class PartitionedStaticBoundaryCost : public Cost<PROB> {
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

  enum class Type {
    kRac,  // Rear axis center.
    kFac,  // Front axis center.
    kCor,  // Corner.
  };

  PartitionedStaticBoundaryCost(
      int horizon, const VehicleGeometryParamsProto* vehicle_geometry_params,
      const Segment2d& boundary_segment, Type type, double dist_to_rac,
      double angle_to_axis, std::vector<std::string> sub_names,
      bool using_hessian_approximate,
      const std::vector<double>& cascade_buffers = {0.0},
      const std::vector<double>& cascade_gains = {1.0},
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "PartitionedStaticBoundaryCost"),
      double scale = 1.0)
      : Cost<PROB>(std::move(name), scale),
        horizon_(horizon),
        vehicle_geometry_params_(*CHECK_NOTNULL(vehicle_geometry_params)),
        boundary_segment_(boundary_segment),
        type_(type),
        dist_to_rac_(dist_to_rac),
        angle_to_axis_(angle_to_axis),
        cascade_buffers_(cascade_buffers),
        cascade_gains_(cascade_gains),
        sub_names_(std::move(sub_names)),
        using_hessian_approximate_(using_hessian_approximate) {
    constexpr double kEpsilon = 1.0e-10;
    QCHECK_GT(horizon_, 0);
    QCHECK_GT(boundary_segment_.length(), kEpsilon);
    QCHECK_EQ(cascade_buffers_.size(), cascade_gains_.size());
    QCHECK_EQ(cascade_buffers_.size(), sub_names_.size());

    // Init states.
    penetrations_.reserve(horizon_);
    for (int i = 0; i < horizon_; ++i) {
      penetrations_.push_back(std::vector<double>(
          cascade_buffers_.size(), std::numeric_limits<double>::infinity()));
    }
    penetration_jacobians_.resize(horizon_, PenetrationJacobianType::Zero());
    penetration_hessians_.resize(horizon_, PenetrationHessianType::Zero());
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    DCHECK_EQ(horizon, horizon_);
    DividedG res(sub_names_.size());
    for (int k = 0; k < horizon_; ++k) {
      const auto& penetrations_k = penetrations_[k];
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          res.AddSubG(i, cascade_gains_[i] * Sqr(penetrations_k[i]));
        }
      }
    }
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, Cost<PROB>::name() + sub_names_[i]);
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType& /*x*/,
                                  const ControlType& /*u*/,
                                  bool using_scale) const override {
    DCHECK_GE(k, 0);
    double scale;
    if (using_scale) {
      scale = Cost<PROB>::scale();
    } else {
      scale = 1.0;
    }
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, Cost<PROB>::name() + sub_names_[i]);
    }
    const auto& penetrations_k = penetrations_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        res.AddSubG(i,
                    0.5 * scale * cascade_gains_[i] * Sqr(penetrations_k[i]));
      }
    }
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    DCHECK_GE(k, 0);
    double g = 0.0;
    const auto& penetrations_k = penetrations_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        g += 0.5 * Cost<PROB>::scale() * cascade_gains_[i] *
             Sqr(penetrations_k[i]);
      }
    }

    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    DCHECK_GE(k, 0);

    const auto& penetrations_k = penetrations_[k];
    const auto& penetration_jacobians_k = penetration_jacobians_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        *dgdx += Cost<PROB>::scale() * cascade_gains_[i] * penetrations_k[i] *
                 penetration_jacobians_k;
      }
    }
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    DCHECK_GE(k, 0);

    const auto& penetrations_k = penetrations_[k];
    const auto& penetration_jacobians_k = penetration_jacobians_[k];
    const auto& penetration_hessians_k = penetration_hessians_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        if (using_hessian_approximate_) {
          *ddgdxdx +=
              Cost<PROB>::scale() * cascade_gains_[i] *
              (penetration_jacobians_k.transpose() * penetration_jacobians_k);
        } else {
          *ddgdxdx +=
              Cost<PROB>::scale() * cascade_gains_[i] *
              (penetrations_k[i] * penetration_hessians_k +
               penetration_jacobians_k.transpose() * penetration_jacobians_k);
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
    DCHECK_EQ(horizon_, horizon);
    for (int k = 0; k < horizon_; ++k) {
      auto& penetrations_k = penetrations_[k];
      auto& penetration_jacobians_k = penetration_jacobians_[k];
      auto& penetration_hessians_k = penetration_hessians_[k];
      penetration_jacobians_k.template segment<3>(PROB::kStateXIndex) =
          Eigen::Matrix<double, 3, 1>::Zero();
      penetration_hessians_k.template block<3, 3>(PROB::kStateXIndex,
                                                  PROB::kStateXIndex) =
          Eigen::Matrix<double, 3, 3>::Zero();

      switch (type_) {
        case Type::kRac: {
          const Vec2d x0 = PROB::pos(xs, k) - boundary_segment_.start();
          const double proj = x0.Dot(boundary_segment_.unit_direction());
          if (proj >= 0.0 && proj <= boundary_segment_.length()) {
            const double signed_dist =
                x0.CrossProd(boundary_segment_.unit_direction());
            for (int i = 0; i < cascade_buffers_.size(); ++i) {
              penetrations_k[i] = std::abs(signed_dist) - cascade_buffers_[i];
            }
            const Vec2d force_dir =
                (signed_dist < 0.0
                     ? boundary_segment_.unit_direction().Perp()
                     : -boundary_segment_.unit_direction().Perp());

            penetration_jacobians_k.template segment<2>(0) = force_dir;

          } else {
            const Vec2d x1 =
                (proj < 0.0 ? x0 : PROB::pos(xs, k) - boundary_segment_.end());
            const double dist = std::sqrt(x1.Sqr());
            const double dist_inv = 1.0 / dist;
            const double dist_inv_cube = Cube(dist_inv);
            for (int i = 0; i < cascade_buffers_.size(); ++i) {
              penetrations_k[i] = dist - cascade_buffers_[i];
            }

            penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
                x1 * dist_inv;
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateXIndex) =
                dist_inv - Sqr(x1.x()) * dist_inv_cube;
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex) =
                -x1.x() * x1.y() * dist_inv_cube;
            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateXIndex) =
                penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex);
            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateYIndex) =
                dist_inv - Sqr(x1.y()) * dist_inv_cube;
          }
        } break;
        case Type::kFac:
        case Type::kCor: {
          const Vec2d tangent =
              Vec2d::FastUnitFromAngleN12(PROB::theta(xs, k) + angle_to_axis_);
          const Vec2d normal = tangent.Perp();
          const Vec2d x0 = PROB::pos(xs, k) + dist_to_rac_ * tangent -
                           boundary_segment_.start();
          const double proj = x0.Dot(boundary_segment_.unit_direction());
          if (proj >= 0.0 && proj <= boundary_segment_.length()) {
            const double signed_dist =
                x0.CrossProd(boundary_segment_.unit_direction());
            for (int i = 0; i < cascade_buffers_.size(); ++i) {
              penetrations_k[i] = std::abs(signed_dist) - cascade_buffers_[i];
            }
            const Vec2d force_dir =
                (signed_dist < 0.0
                     ? boundary_segment_.unit_direction().Perp()
                     : -boundary_segment_.unit_direction().Perp());

            penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
                force_dir;
            penetration_jacobians_k(PROB::kStateThetaIndex) =
                force_dir.dot(normal * dist_to_rac_);
            penetration_hessians_k(PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
                force_dir.dot(-tangent * dist_to_rac_);

          } else {
            const Vec2d x1 =
                (proj < 0.0 ? x0
                            : PROB::pos(xs, k) + dist_to_rac_ * tangent -
                                  boundary_segment_.end());
            const double dist = std::sqrt(x1.Sqr());
            const double dist_inv = 1.0 / dist;
            const double dist_inv_cube = Cube(dist_inv);
            for (int i = 0; i < cascade_buffers_.size(); ++i) {
              penetrations_k[i] = dist - cascade_buffers_[i];
            }

            penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
                x1 * dist_inv;
            penetration_jacobians_k(PROB::kStateThetaIndex) =
                x1.dot(normal * dist_to_rac_) * dist_inv;

            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateXIndex) =
                dist_inv - Sqr(x1.x()) * dist_inv_cube;
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex) =
                -x1.x() * x1.y() * dist_inv_cube;
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateThetaIndex) =
                normal.x() * dist_to_rac_ * dist_inv -
                dist_inv_cube * x1.x() * x1.dot(normal * dist_to_rac_);

            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateXIndex) =
                penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex);
            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateYIndex) =
                dist_inv - Sqr(x1.y()) * dist_inv_cube;
            penetration_hessians_k(1, PROB::kStateThetaIndex) =
                normal.y() * dist_to_rac_ * dist_inv -
                dist_inv_cube * x1.y() * x1.dot(normal * dist_to_rac_);

            penetration_hessians_k(PROB::kStateThetaIndex, PROB::kStateXIndex) =
                penetration_hessians_k(PROB::kStateXIndex,
                                       PROB::kStateThetaIndex);
            penetration_hessians_k(PROB::kStateThetaIndex, PROB::kStateYIndex) =
                penetration_hessians_k(PROB::kStateYIndex,
                                       PROB::kStateThetaIndex);
            penetration_hessians_k(PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
                (x1.dot(-tangent * dist_to_rac_) +
                 dist_to_rac_ * normal.dot(normal * dist_to_rac_)) *
                    dist_inv -
                dist_inv_cube * x1.dot(normal * dist_to_rac_) *
                    x1.dot(normal * dist_to_rac_);
          }
        } break;
      }  // Switch case.
    }    // For loop.
  }

 private:
  int horizon_ = 0;
  VehicleGeometryParamsProto vehicle_geometry_params_;
  Segment2d boundary_segment_;
  Type type_;
  double dist_to_rac_;
  double angle_to_axis_;
  std::vector<double> cascade_buffers_;
  std::vector<double> cascade_gains_;
  std::vector<std::string> sub_names_;
  bool using_hessian_approximate_;

  // States.
  using PenetrationJacobianType = Eigen::Matrix<double, 1, PROB::kStateSize>;
  using PenetrationHessianType =
      Eigen::Matrix<double, PROB::kStateSize, PROB::kStateSize>;
  std::vector<std::vector<double>> penetrations_;
  std::vector<PenetrationJacobianType> penetration_jacobians_;
  std::vector<PenetrationHessianType> penetration_hessians_;
};
}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_PARTITIONED_STATIC_BOUNDARY_COST_H
