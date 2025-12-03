#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_SPEED_LIMITING_OBSTACLE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_SPEED_LIMITING_OBSTACLE_COST_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class TobSpeedLimitingObjectCost : public Cost<PROB> {
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

  // The cost consists of two parts: penetration cost and speed cost.
  // penetration = (x - ref).dot(dir)
  // Penetration cost = 0.5 * scale * penetration^2,      if penetration > 0;
  //                    0,                                if penetration <= 0.
  // x could be the AV RAC, front left corner, or front right corner, depending
  // on type.
  // overspeed = (v - ref_v)
  // Speed cost = 0.5 * scale * penetration * overspeed,  if penetration > 0 and
  //                                                         overspeed > 0
  //              0,                                      otherwise
  //
  struct Object {
    // Direction that the cost increases in.
    Vec2d dir = Vec2d::UnitX();
    // Reference point.
    Vec2d ref = Vec2d::Zero();
    // Lateral extent (full extent is twice this much).
    double lateral_extent = 0.0;
    // Individual object can have a variable gain on top of the uniform scale.
    double gain = 1.0;

    enum class Type {
      kRac,
      kFrontLeft,
      kFrontRight,
    };
    Type type = Type::kRac;

    // Reference speed.
    double v_ref = 0.0;
    // Gain of the speed cost term.
    double speed_gain = 0.0;
    // If object is enabled
    bool enable = false;
  };

  static constexpr double kNormalizedScale = 1.0;
  explicit TobSpeedLimitingObjectCost(
      int horizon, const VehicleGeometryParamsProto* vehicle_geometry_params,
      std::vector<Object> objects,
      std::string name = "TobSpeedLimitingObjectCost", double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale),
        horizon_(horizon),
        vehicle_geometry_params_(*CHECK_NOTNULL(vehicle_geometry_params)),
        objects_(std::move(objects)) {
    QCHECK_GT(horizon_, 0);
    QCHECK_EQ(horizon_, objects_.size());
    penetrations_.resize(horizon_, -std::numeric_limits<double>::infinity());
    penetration_jacobians_.resize(horizon_, PenetrationJacobianType::Zero());
    penetration_hessians_.resize(horizon_, PenetrationHessianType::Zero());
    overspeeds_.resize(horizon_, 0.0);
  }

  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& us,
                           int horizon) const override {
    DCHECK_EQ(horizon, horizon_);
    DividedG res(/*size=*/1);
    for (int k = 0; k < horizon_; ++k) {
      if (penetrations_[k] > 0.0) {
        res.AddSubG(
            /*idx=*/0, 0.5 * Cost<PROB>::scale() * objects_[k].gain *
                           Sqr(penetrations_[k]));
        if (overspeeds_[k] > 0.0) {
          res.AddSubG(
              /*idx=*/0, 0.5 * Cost<PROB>::scale() * objects_[k].speed_gain *
                             penetrations_[k] * Sqr(overspeeds_[k]));
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
    double g = 0.0;
    if (penetrations_[k] > 0.0) {
      g += 0.5 * Cost<PROB>::scale() * objects_[k].gain * Sqr(penetrations_[k]);
      if (overspeeds_[k] > 0.0) {
        g += 0.5 * Cost<PROB>::scale() * objects_[k].speed_gain *
             penetrations_[k] * Sqr(overspeeds_[k]);
      }
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& x, const ControlType& u,
               DGDxType* dgdx) const override {
    if (penetrations_[k] > 0.0) {
      *dgdx += Cost<PROB>::scale() * objects_[k].gain * penetrations_[k] *
               penetration_jacobians_[k];
      if (overspeeds_[k] > 0.0) {
        (*dgdx)[PROB::kStateVIndex] += Cost<PROB>::scale() *
                                       objects_[k].speed_gain *
                                       penetrations_[k] * overspeeds_[k];
      }
    }
  }
  void AddDGDu(int k, const StateType& x, const ControlType& u,
               DGDuType* dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& x, const ControlType& u,
                  DDGDxDxType* ddgdxdx) const override {
    if (penetrations_[k] > 0.0) {
      *ddgdxdx +=
          Cost<PROB>::scale() * objects_[k].gain *
          (penetrations_[k] * penetration_hessians_[k] +
           penetration_jacobians_[k].transpose() * penetration_jacobians_[k]);
      if (overspeeds_[k] > 0.0) {
        (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateVIndex) +=
            Cost<PROB>::scale() * objects_[k].speed_gain * penetrations_[k];
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
    QCHECK_EQ(horizon_, horizon);
    const double rac2fb = vehicle_geometry_params_.front_edge_to_center();
    const double half_width = vehicle_geometry_params_.width() * 0.5;

    const auto in_lat_range = [](const Vec2d& p0, const Vec2d& p1,
                                 const Object& ob) {
      const double lat_offset0 = ob.dir.CrossProd(p0 - ob.ref);
      const double lat_offset1 = ob.dir.CrossProd(p1 - ob.ref);
      if ((lat_offset0 > ob.lateral_extent &&
           lat_offset1 > ob.lateral_extent) ||
          (lat_offset0 < -ob.lateral_extent &&
           lat_offset1 < -ob.lateral_extent)) {
        return false;
      }
      return true;
    };

    for (int k = 0; k < horizon_; ++k) {
      penetrations_[k] = -std::numeric_limits<double>::infinity();
      penetration_jacobians_[k](PROB::kStateXIndex) = 0.0;
      penetration_jacobians_[k](PROB::kStateYIndex) = 0.0;
      penetration_jacobians_[k](PROB::kStateThetaIndex) = 0.0;
      penetration_hessians_[k](PROB::kStateThetaIndex, PROB::kStateThetaIndex) =
          0.0;
      overspeeds_[k] = 0.0;

      const Object& ob = objects_[k];
      if (!ob.enable) continue;

      const auto x0 = PROB::GetStateMapAtStep(xs, k);
      const auto x1 =
          PROB::GetStateMapAtStep(xs, std::min(horizon_ - 1, k + 1));

      const Vec2d av_tangent =
          Vec2d::FastUnitFromAngleN12(PROB::StateGetTheta(x0));
      const Vec2d av_normal = av_tangent.Perp();
      VLOG(3) << "Step " << k;
      VLOG(4) << "x0 = " << x0.transpose() << " x1 = " << x1.transpose()
              << " av_tangent = " << av_tangent.transpose()
              << " av_normal = " << av_normal.transpose();

      overspeeds_[k] = PROB::StateGetV(x0) - ob.v_ref;

      switch (ob.type) {
        case Object::Type::kRac: {
          const Vec2d pos0 = PROB::StateGetPos(x0);
          const Vec2d pos1 = PROB::StateGetPos(x1);
          if (!in_lat_range(pos0, pos1, ob)) break;
          penetrations_[k] = (pos0 - ob.ref).dot(ob.dir);
          penetration_jacobians_[k].template segment<2>(PROB::kStateXIndex) =
              ob.dir.transpose();
          VLOG(4) << "  ob: type = " << static_cast<int>(ob.type)
                  << " pos0 = " << pos0.transpose()
                  << " pos1 = " << pos1.transpose()
                  << " penetration = " << penetrations_[k]
                  << " jacobian = " << penetration_jacobians_[k];
          break;
        }
        case Object::Type::kFrontLeft: {
          const Vec2d front_left0 = PROB::StateGetPos(x0) +
                                    av_tangent * rac2fb +
                                    av_normal * half_width;
          const Vec2d front_left1 = PROB::StateGetPos(x1) +
                                    av_tangent * rac2fb +
                                    av_normal * half_width;
          if (!in_lat_range(front_left0, front_left1, ob)) break;
          penetrations_[k] = (front_left0 - ob.ref).dot(ob.dir);
          penetration_jacobians_[k].template segment<2>(PROB::kStateXIndex) =
              ob.dir.transpose();
          penetration_jacobians_[k](PROB::kStateThetaIndex) =
              ob.dir.dot(av_normal * rac2fb - av_tangent * half_width);
          penetration_hessians_[k](PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
              ob.dir.dot(-av_tangent * rac2fb - av_normal * half_width);
          VLOG(4) << "  ob: type = " << static_cast<int>(ob.type)
                  << " front_left0 = " << front_left0.transpose()
                  << " front_left1 = " << front_left1.transpose()
                  << " penetration = " << penetrations_[k]
                  << " jacobian = " << penetration_jacobians_[k];
          break;
        }
        case Object::Type::kFrontRight: {
          const Vec2d front_right0 = PROB::StateGetPos(x0) +
                                     av_tangent * rac2fb -
                                     av_normal * half_width;
          const Vec2d front_right1 = PROB::StateGetPos(x1) +
                                     av_tangent * rac2fb -
                                     av_normal * half_width;
          if (!in_lat_range(front_right0, front_right1, ob)) break;
          penetrations_[k] = (front_right0 - ob.ref).dot(ob.dir);
          penetration_jacobians_[k].template segment<2>(PROB::kStateXIndex) =
              ob.dir.transpose();
          penetration_jacobians_[k](PROB::kStateThetaIndex) =
              ob.dir.dot(av_normal * rac2fb + av_tangent * half_width);
          penetration_hessians_[k](PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
              ob.dir.dot(-av_tangent * rac2fb + av_normal * half_width);
          VLOG(4) << "  ob: type = " << static_cast<int>(ob.type)
                  << " front_right0 = " << front_right0.transpose()
                  << " front_right1 = " << front_right1.transpose()
                  << " penetration = " << penetrations_[k]
                  << " jacobian = " << penetration_jacobians_[k];
          break;
        }
      }
    }
  }

 private:
  int horizon_ = 0;
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::vector<Object> objects_;

  // States.
  using PenetrationJacobianType = Eigen::Matrix<double, 1, PROB::kStateSize>;
  using PenetrationHessianType =
      Eigen::Matrix<double, PROB::kStateSize, PROB::kStateSize>;

  std::vector<double> penetrations_;
  std::vector<PenetrationJacobianType> penetration_jacobians_;
  std::vector<PenetrationHessianType> penetration_hessians_;

  std::vector<double> overspeeds_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_SPEED_LIMITING_OBSTACLE_COST_H_
