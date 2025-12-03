#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_GRADIENT_FIXED_PROJECTIVE_OBSTACLE_COST_H_  // NOLINT
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_GRADIENT_FIXED_PROJECTIVE_OBSTACLE_COST_H_  // NOLINT

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class MfobGradientFixedProjectiveObjectCost : public Cost<PROB> {
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

  struct Object {
    std::vector<Segment2d> lines;
    Vec2d dir;
    Vec2d ref;
    std::vector<double> buffers;
    double extent = 1.0;
    std::vector<double> gains;
    enum class Type {
      kRac,
      kFac,
    };
    Type type = Type::kRac;
    // If object is enabled
    bool enable = false;
  };

  static constexpr double kNormalizedScale = 1.0;
  MfobGradientFixedProjectiveObjectCost(
      int horizon, const VehicleGeometryParamsProto* vehicle_geometry_params,
      std::vector<Object> objects,
      std::string name = "MfobGradientFixedProjectiveObjectCost",
      double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale),
        horizon_(horizon),
        vehicle_geometry_params_(*CHECK_NOTNULL(vehicle_geometry_params)),
        objects_(std::move(objects)) {
    QCHECK_GT(horizon_, 0);
    const auto calc_sin_theta = [](const Vec2d& ref_unit_dir,
                                   const Vec2d& dir) -> double {
      constexpr double kEpsilon = 1e-10;
      const double sin_theta = ref_unit_dir.CrossProd(dir);
      if (sin_theta >= 0.0) {
        return std::fmax(kEpsilon, sin_theta);
      } else {
        return std::fmin(-kEpsilon, sin_theta);
      }
    };

    penetrations_.resize(horizon_);
    line_infos_.resize(horizon_);
    for (int i = 0; i < objects_.size(); ++i) {
      const auto& ob = objects_[i];
      QCHECK_EQ(ob.buffers.size(), ob.buffers.size());
      auto& line_infos_i = line_infos_[i];
      line_infos_i.reserve(ob.lines.size());
      for (int k = 0; k < ob.lines.size(); ++k) {
        const Vec2d& ref_unit_dir = ob.lines[k].unit_direction();
        line_infos_i.push_back(
            {.inner_product = ob.dir.CrossProd(ob.lines[k].end() - ob.ref),
             .sin_theta_inv = 1.0 / calc_sin_theta(ref_unit_dir, ob.dir)});
      }
      auto& penetrations_i = penetrations_[i];
      penetrations_i.reserve(ob.buffers.size());
      for (int k = 0; k < ob.buffers.size(); ++k) {
        penetrations_i.push_back(std::numeric_limits<double>::infinity());
      }
    }
    penetration_jacobians_.resize(horizon_, PenetrationJacobianType::Zero());
    penetration_hessians_.resize(horizon_, PenetrationHessianType::Zero());
  }

  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& us,
                           int horizon) const override {
    DividedG res(objects_[0].gains.size());
    DCHECK_EQ(horizon, horizon_);
    for (int k = 0; k < horizon_; ++k) {
      const auto& penetrations_k = penetrations_[k];
      const auto& ob_k = objects_[k];
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          res.AddSubG(/*idx=*/0, ob_k.gains[i] * Sqr(penetrations_k[i]));
        }
      }
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& x,
                   const ControlType& u) const override {
    DCHECK_GE(k, 0);

    double g = 0.0;
    const auto& ob_k = objects_[k];
    const auto& penetrations_k = penetrations_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        g += 0.5 * Cost<PROB>::scale() * ob_k.gains[i] * Sqr(penetrations_k[i]);
      }
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& x, const ControlType& u,
               DGDxType* dgdx) const override {
    DCHECK_GE(k, 0);
    DCHECK_LT(k, objects_.size());

    const auto& penetrations_k = penetrations_[k];
    const auto& ob_k = objects_[k];
    const auto& penetration_jacobians_k = penetration_jacobians_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        *dgdx += Cost<PROB>::scale() * ob_k.gains[i] * penetrations_k[i] *
                 penetration_jacobians_k;
      }
    }
  }
  void AddDGDu(int k, const StateType& x, const ControlType& u,
               DGDuType* dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& x, const ControlType& u,
                  DDGDxDxType* ddgdxdx) const override {
    DCHECK_GE(k, 0);
    DCHECK_LT(k, objects_.size());

    const auto& penetrations_k = penetrations_[k];
    const auto& ob_k = objects_[k];
    const auto& penetration_jacobians_k = penetration_jacobians_[k];
    const auto& penetration_hessians_k = penetration_hessians_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        *ddgdxdx +=
            Cost<PROB>::scale() * ob_k.gains[i] *
            (penetrations_k[i] * penetration_hessians_k +
             penetration_jacobians_k.transpose() * penetration_jacobians_k);
      }
    }
  }
  void AddDDGDuDx(int k, const StateType& x, const ControlType& u,
                  DDGDuDxType* ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType& x, const ControlType& u,
                  DDGDuDuType* ddgdudu) const override {}

  // TODO(runbing): Split derivatives computation from update and move into
  // UpdateDerivatives before using.
  void Update(const StatesType& xs, const ControlsType& us,
              int horizon) override {
    // TODO(runbing): Use value_only to reduce unnecessary updates.
    const double wheel_base = vehicle_geometry_params_.wheel_base();
    DCHECK_EQ(horizon, horizon_);

    for (int k = 0; k < horizon_; ++k) {
      auto& penetrations_k = penetrations_[k];
      auto& penetration_jacobians_k = penetration_jacobians_[k];
      auto& penetration_hessians_k = penetration_hessians_[k];

      const Object& ob = objects_[k];
      if (!ob.enable) continue;

      const auto x0 = PROB::GetStateAtStep(xs, k);

      Vec2d pos;
      Vec2d av_tangent;
      switch (ob.type) {
        case Object::Type::kRac:
          pos = PROB::pos(xs, k);
          break;
        case Object::Type::kFac:
          av_tangent = Vec2d::FastUnitFromAngleN12(PROB::StateGetTheta(x0));
          pos = PROB::pos(xs, k) + av_tangent * wheel_base;
          break;
      }

      const std::vector<Segment2d>& lines = ob.lines;
      const auto& line_infos = line_infos_[k];
      const auto& buffers = ob.buffers;
      const Vec2d& dir = ob.dir;

      const double projection_perp = dir.CrossProd(pos - ob.ref);
      // if (std::abs(projection_perp) > ob.extent) continue;

      int index = line_infos.size() - 1;
      for (int j = 0; j < index; ++j) {
        if (projection_perp <= line_infos[j].inner_product) {
          index = j;
          break;
        }
      }

      const Segment2d& line_ref = lines[index];
      const Vec2d& ref_unit_dir = line_ref.unit_direction();
      const double dist = ref_unit_dir.CrossProd(pos - line_ref.start()) *
                          line_infos[index].sin_theta_inv;

      for (int i = 0; i < penetrations_k.size(); ++i) {
        penetrations_k[i] = dist - buffers[i];
      }
      penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) = dir;

      switch (ob.type) {
        case Object::Type::kRac:
          break;
        case Object::Type::kFac: {
          const Vec2d av_normal = av_tangent.Perp();
          penetration_jacobians_k(PROB::kStateThetaIndex) =
              dir.dot(av_normal * wheel_base);
          penetration_hessians_k(PROB::kStateThetaIndex,
                                 PROB::kStateThetaIndex) =
              dir.dot(-av_tangent * wheel_base);
          break;
        }
      }

      VLOG(4) << "Step " << k;
      VLOG(4) << "x0 = " << x0.transpose();
      VLOG(4) << "  ob " << k << ": type = " << static_cast<int>(ob.type)
              << " jacobian = " << penetration_jacobians_[k];
      for (int i = 0; i < penetrations_k.size(); ++i) {
        VLOG(4) << "penetration(" << i << "):  " << penetrations_k[i];
      }
    }
  }

 private:
  int horizon_ = 0;
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::vector<Object> objects_;
  struct LineInfo {
    double inner_product;
    double sin_theta_inv;
  };
  std::vector<std::vector<LineInfo>> line_infos_;

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
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_GRADIENT_FIXED_PROJECTIVE_OBSTACLE_COST_H_
