#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_S_LIMITING_OBSTACLE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_S_LIMITING_OBSTACLE_COST_H_

#include <algorithm>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class TobSLimitingObjectCost : public Cost<PROB> {
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

  // penetration =
  // (s + standoff - s_ref) if type = Backward;
  // (s_ref - s + standoff) if type = Forward.
  // Cost = 0.5 * scale * penetration^2,      if penetration > 0;
  //        0,                                if penetration <= 0.
  struct Object {
    // Reference point on reference path.
    std::string id;
    Vec2d ref = Vec2d::Zero();
    // Heading direction of reference point.
    Vec2d dir = Vec2d::Zero();
    // Reference s on reference path.
    double s_ref = 0.0;
    // Individual object can have a variable gain on top of the uniform scale.
    double gain = 1.0;
    // Max penetration before deactivating.
    double depth = 10.0;  // m.
    double standoff = 0.0;
    enum class Type {
      kForward,   // Push AV forwards
      kBackward,  // Push AV backwards
    };
    Type type = Type::kBackward;
    // If object is enabled
    bool enable = false;
  };

  static constexpr double kNormalizedScale = 1.0;
  TobSLimitingObjectCost(
      int horizon, const VehicleGeometryParamsProto* vehicle_geometry_params,
      std::vector<Object> objects, std::string name = "TobSLimitingObjectCost",
      double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale),
        horizon_(horizon),
        vehicle_geometry_params_(*CHECK_NOTNULL(vehicle_geometry_params)),
        objects_(std::move(objects)) {
    QCHECK_GT(horizon_, 0);
    QCHECK_EQ(objects_.size(), horizon_);
    standoffs_.resize(horizon_);
    for (int k = 0; k < horizon_; ++k) {
      standoffs_[k] = objects_[k].standoff;
    }

    penetrations_.resize(horizon_, -std::numeric_limits<double>::infinity());
    penetrations_dir_.resize(horizon_, 0.0);
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    DividedG res(/*size=*/1);
    DCHECK_EQ(horizon, horizon_);
    for (int k = 0; k < horizon_; ++k) {
      if (penetrations_[k] > 0.0) {
        res.AddSubG(
            /*idx=*/0, 0.5 * Cost<PROB>::scale() * objects_[k].gain *
                           Sqr(penetrations_[k]));
      }
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    double g = 0.0;
    if (penetrations_[k] > 0.0) {
      g += 0.5 * Cost<PROB>::scale() * objects_[k].gain * Sqr(penetrations_[k]);
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    if (penetrations_[k] > 0.0) {
      (*dgdx)[PROB::kStateSIndex] += Cost<PROB>::scale() * objects_[k].gain *
                                     penetrations_[k] * penetrations_dir_[k];
    }
  }
  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    if (penetrations_[k] > 0.0) {
      (*ddgdxdx)(PROB::kStateSIndex, PROB::kStateSIndex) +=
          Cost<PROB>::scale() * objects_[k].gain;
    }
  }
  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

  // TODO(runbing): Split derivatives computation from update and move into
  // UpdateDerivatives before using.
  void Update(const StatesType& xs, const ControlsType& /*us*/,
              int horizon) override {
    DCHECK_EQ(horizon, horizon_);
    const double rac2fb = vehicle_geometry_params_.front_edge_to_center();
    const double rac2rb = vehicle_geometry_params_.back_edge_to_center();
    for (int k = 0; k < horizon_; ++k) {
      const Object& ob = objects_[k];

      if (!ob.enable) continue;
      const auto x = PROB::GetStateMapAtStep(xs, k);
      VLOG(4) << "Step " << k;
      VLOG(4) << "x = " << x.transpose();

      switch (ob.type) {
        case Object::Type::kBackward: {
          const double s = PROB::StateGetS(x);
          penetrations_[k] = s + rac2fb + standoffs_[k] - ob.s_ref;
          penetrations_dir_[k] = 1.0;
          VLOG(4) << "At step " << k << " for Backward ob " << ob.id
                  << ": type = " << static_cast<int>(ob.type) << " s = " << s
                  << " rac2fb = " << rac2fb << " standoff = " << standoffs_[k]
                  << " ob.s_ref " << ob.s_ref
                  << " penetration = " << penetrations_[k];
          break;
        }
        case Object::Type::kForward: {
          const double s = PROB::StateGetS(x);
          penetrations_[k] = ob.s_ref - s + rac2rb + standoffs_[k];
          penetrations_dir_[k] = -1.0;
          VLOG(4) << "At step " << k << " for Forward ob " << ob.id
                  << ": type = " << static_cast<int>(ob.type) << " s = " << s
                  << " rac2fb = " << rac2fb << " standoff = " << standoffs_[k]
                  << " ob.s_ref " << ob.s_ref
                  << " penetration = " << penetrations_[k];
          break;
        }
        default:
          QLOG(FATAL) << "Unknown object type " << static_cast<int>(ob.type);
          break;
      }
      // Restrict penetration to object depth to prevent overreaction.
      penetrations_[k] = std::min(penetrations_[k], ob.depth);
    }
  }

 private:
  int horizon_ = 0;
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::vector<Object> objects_;

  // States.
  std::vector<double> standoffs_;
  std::vector<double> penetrations_;
  std::vector<double> penetrations_dir_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_S_LIMITING_OBSTACLE_COST_H_
