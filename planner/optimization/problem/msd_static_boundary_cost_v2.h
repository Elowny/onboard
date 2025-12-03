#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MSD_STATIC_BOUNDARY_COST_V2_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MSD_STATIC_BOUNDARY_COST_V2_H_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/center_line_query_helper.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/min_segment_distance_problem.h"

namespace qcraft {
namespace planner {

// Use 3 circles to represent ADV
// static boundary are storaged in a min_segment_distance_problem
// Will avoid ADV from hitting static boundary
template <typename PROB>
class MsdStaticBoundaryCostV2 : public Cost<PROB> {
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

  struct Layer {
    double gain_of_weight = 1.0;
    PiecewiseLinearFunction<double> buffer_at_speed;
  };

  static constexpr double kNormalizedScale = 10.0;
  // Offset = {forward shift, leftward shift} over rac.
  MsdStaticBoundaryCostV2(  // Curbs
      int horizon, MsdProblemWithBuffer curb_msd,
      const CenterLineQueryHelper<PROB>* center_line_helper,
      // Multiple layers of costs.
      std::vector<std::string> sub_names, std::vector<Layer> sub_layers,
      // Vehicle shape
      std::vector<Vec2d> circle_center_offsets,
      std::vector<double> circle_radiuses,
      // Others
      bool using_hessian_approximate,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "MsdStaticBoundaryCostV2"),
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::UNKNOWN)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        horizon_(horizon),
        curb_msd_(std::move(curb_msd)),
        center_line_helper_(center_line_helper),
        sub_names_(std::move(sub_names)),
        sub_layers_(std::move(sub_layers)),
        circle_center_offsets_(std::move(circle_center_offsets)),
        circle_radiuses_(std::move(circle_radiuses)),
        using_hessian_approximate_(using_hessian_approximate) {
    QCHECK_GT(horizon_, 0);

    const int num_layers = static_cast<int>(sub_layers_.size());
    QCHECK_GT(num_layers, 0);
    QCHECK_EQ(num_layers, sub_names_.size());

    const int num_circles = static_cast<int>(circle_center_offsets_.size());
    QCHECK_GT(num_circles, 0);
    QCHECK_EQ(num_circles, circle_radiuses_.size());

    workspaces_.resize(horizon_);
    for (int i = 0; i < horizon_; ++i) {
      workspaces_[i].circles.resize(num_circles);
      workspaces_[i].sub_layer_buffers.resize(num_layers);
      workspaces_[i].sub_layer_dbuffer_dv.resize(num_layers);
    }

    cost_result_.resize(horizon_);
    for (int i = 0; i < horizon_; ++i) {
      cost_result_[i].cost_at_layer.resize(num_layers);
    }
    gradient_result_.resize(horizon_);
  }

  // Sum up g over all steps.
  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    DCHECK_EQ(horizon_, horizon);
    DCHECK(cost_updated_);
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
      res.SetIsSoft(i, sub_names_[i] == SoftNameString);
    }
    for (int k = 0; k < horizon_; ++k) {
      for (int i = 0; i < sub_names_.size(); ++i) {
        res.AddSubG(i, cost_result_[k].cost_at_layer[i]);
      }
    }
    return res;
  }

  // For auto tunning.
  DividedG EvaluateGWithDebugInfo(int k, const StateType& /*x*/,
                                  const ControlType& /*u*/,
                                  bool using_scale) const override {
    DCHECK(cost_updated_);
    DCHECK_LT(k, horizon_);
    DCHECK_GE(k, 0);
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }

    for (int i = 0; i < sub_names_.size(); ++i) {
      if (using_scale) {
        res.AddSubG(i, cost_result_[k].cost_at_layer[i]);
      } else {
        res.AddSubG(i, cost_result_[k].cost_at_layer[i] /
                           sub_layers_[i].gain_of_weight);
      }
    }

    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    DCHECK(cost_updated_);
    DCHECK_LT(k, horizon_);
    DCHECK_GE(k, 0);
    double g = 0.0;
    for (int i = 0; i < sub_layers_.size(); ++i) {
      g += cost_result_[k].cost_at_layer[i];
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    DCHECK(gradient_updated_);
    DCHECK_LT(k, horizon_);
    DCHECK_GE(k, 0);
    if (!gradient_result_[k].is_trivial) {
      *dgdx += gradient_result_[k].dg_dx;
    }
  }

  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    DCHECK(gradient_updated_);
    DCHECK_LT(k, horizon_);
    DCHECK_GE(k, 0);
    if (!gradient_result_[k].is_trivial) {
      *ddgdxdx += gradient_result_[k].d2g_dx2;
    }
  }

  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

  void Update(const StatesType& xs, const ControlsType& /*us*/,
              int horizon) override {
    DCHECK_EQ(horizon_, horizon);
    MoveTo(xs);
    UpdateCost();
    UpdateDerivatives();
  }

  void UpdateDerivatives(const StatesType& /*xs*/, const ControlsType& /*us*/,
                         int horizon) override {
    DCHECK_EQ(horizon_, horizon);
    UpdateDerivatives();
  }

 private:
  struct CircleInfo {
    bool is_trivial = true;
    double msd = 0.0;
    double segment_gain = 1.0;
    Vec2d circle_center;
    // Nearest segment pointer.
    const Segment2d* seg_ptr = nullptr;
  };

  struct OneStepWorkspace {
    Vec2d vehicle_tangent;
    std::vector<CircleInfo> circles;
    std::vector<double> sub_layer_buffers;
    std::vector<double> sub_layer_dbuffer_dv;
  };

  struct OneStepGradientResult {
    bool is_trivial = true;
    DGDxType dg_dx;
    DDGDxDxType d2g_dx2;

    void SetZero() {
      is_trivial = true;
      dg_dx.setZero();
      d2g_dx2.setZero();
    }
  };

  struct OneStepCostResult {
    std::vector<double> cost_at_layer;
    void SetZero() {
      for (int i = 0; i < cost_at_layer.size(); ++i) {
        cost_at_layer[i] = 0.0;
      }
    }
  };

  // ====Proposed new public interfaces====
  // Change the state and control input
  // of the problem.
  void MoveTo(const StatesType& xs /*also add us*/) {
    cost_updated_ = false;
    gradient_updated_ = false;
    xs_ptr_ = &xs;
  }

  // Make sure cost result is ready
  // after calling this function.
  void UpdateCost() {
    DCHECK_NOTNULL(xs_ptr_);
    if (cost_updated_) {
      return;
    }
    if (center_line_helper_ == nullptr) {
      for (int k = 0; k < horizon_; ++k) {
        const StateType& state = PROB::GetStateAtStep(*xs_ptr_, k);
        CalculateCost(state, &cost_result_[k], &workspaces_[k]);
      }
    } else {
      const int last_real_point_index =
          center_line_helper_->last_real_point_index();
      const auto& indices = center_line_helper_->indices();
      for (int k = 0; k < horizon_; ++k) {
        if (indices[k] > last_real_point_index) {
          cost_result_[k].SetZero();
        } else {
          const StateType& state = PROB::GetStateAtStep(*xs_ptr_, k);
          CalculateCost(state, &cost_result_[k], &workspaces_[k]);
        }
      }
    }
    cost_updated_ = true;
  }

  // Make sure gradient result is ready
  // after calling this function.
  void UpdateDerivatives() {
    DCHECK_NOTNULL(xs_ptr_);
    if (gradient_updated_) {
      return;
    }
    QCHECK(cost_updated_);
    if (center_line_helper_ == nullptr) {
      for (int k = 0; k < horizon_; ++k) {
        const StateType& state = PROB::GetStateAtStep(*xs_ptr_, k);
        CalculateGradient(state, &gradient_result_[k], &workspaces_[k]);
      }
    } else {
      const int last_real_point_index =
          center_line_helper_->last_real_point_index();
      const auto& indices = center_line_helper_->indices();

      for (int k = 0; k < horizon_; ++k) {
        if (indices[k] > last_real_point_index) {
          gradient_result_[k].SetZero();
        } else {
          const StateType& state = PROB::GetStateAtStep(*xs_ptr_, k);
          CalculateGradient(state, &gradient_result_[k], &workspaces_[k]);
        }
      }
    }

    gradient_updated_ = true;
  }
  // ====Proposed new public interfaces====

  // Calculate one step cost.
  // workspace gets updated after calling this function.
  void CalculateCost(const StateType& state, OneStepCostResult* result,
                     OneStepWorkspace* workspace) const {
    DCHECK_NOTNULL(workspace);
    DCHECK_NOTNULL(result);
    DCHECK_EQ(result->cost_at_layer.size(), sub_layers_.size());
    DCHECK_EQ(workspace->circles.size(), circle_center_offsets_.size());
    DCHECK_EQ(workspace->sub_layer_buffers.size(), sub_layers_.size());

    const Vec2d pos = PROB::StateGetPos(state);
    const double speed = state[PROB::kStateVIndex];

    for (int i = 0; i < sub_layers_.size(); ++i) {
      workspace->sub_layer_buffers[i] =
          sub_layers_[i].buffer_at_speed.Evaluate(speed);
    }

    workspace->vehicle_tangent =
        Vec2d::FastUnitFromAngleN12(PROB::StateGetTheta(state));

    // Reset the result buffer to 0;
    for (int i = 0; i < sub_layers_.size(); ++i) {
      result->cost_at_layer[i] = 0.0;
    }

    // For each circle, update cost.
    for (int i = 0; i < circle_center_offsets_.size(); ++i) {
      const Vec2d& offset = circle_center_offsets_[i];
      Vec2d circle_center = pos + workspace->vehicle_tangent * offset.x() +
                            workspace->vehicle_tangent.Perp() * offset.y();
      const MsdProblemWithBuffer::SegmentType* seg_with_buffer_ptr =
          curb_msd_.GetNearestSegmentWithBuffer(circle_center);

      double msd = curb_msd_.cutoff_distance();
      double segment_gain = 1.0;
      if (seg_with_buffer_ptr != nullptr) {
        msd = seg_with_buffer_ptr->segment.DistanceTo(circle_center) -
              seg_with_buffer_ptr->buffer;
        segment_gain = seg_with_buffer_ptr->gain;
      }

      // For each layer, update cost.
      bool has_cost = false;
      for (int j = 0; j < sub_layers_.size(); ++j) {
        const double penetration =
            msd - workspace->sub_layer_buffers[j] - circle_radiuses_[i];
        if (penetration < 0.0) {
          has_cost = true;
          result->cost_at_layer[j] += 0.5 * Cost<PROB>::scale() *
                                      sub_layers_[j].gain_of_weight *
                                      segment_gain * Sqr(penetration);
        }
      }

      // Update circle cache.
      CircleInfo& circle = workspace->circles[i];
      circle.circle_center = std::move(circle_center);
      circle.msd = msd;
      circle.segment_gain = segment_gain;
      circle.seg_ptr = seg_with_buffer_ptr != nullptr
                           ? &seg_with_buffer_ptr->segment
                           : nullptr;
      circle.is_trivial = !has_cost;
    }
  }

  // Calculate one step gradient.
  // Assuming the workspace is updated by CalculateCost.
  void CalculateGradient(const StateType& state, OneStepGradientResult* result,
                         OneStepWorkspace* workspace) const {
    DCHECK_NOTNULL(result);
    DCHECK_NOTNULL(workspace);
    DCHECK_EQ(workspace->sub_layer_dbuffer_dv.size(), sub_layers_.size());

    const double speed = state[PROB::kStateVIndex];

    // dbuffer_dv
    for (int i = 0; i < sub_layers_.size(); ++i) {
      workspace->sub_layer_dbuffer_dv[i] =
          sub_layers_[i].buffer_at_speed.EvaluateSlope(speed);
    }

    result->SetZero();
    // For each cricle.
    for (int i = 0; i < circle_center_offsets_.size(); ++i) {
      const CircleInfo& circle_cache = workspace->circles[i];

      // Skip no cost circles.
      if (circle_cache.seg_ptr == nullptr || circle_cache.is_trivial) {
        continue;
      }

      // Get derivative of the circle center's msd.
      PenetrationHessianType msd_hessian = PenetrationHessianType::Zero();
      PenetrationJacobianType msd_jacobian = PenetrationJacobianType::Zero();
      {
        const Vec2d rotated_offset = circle_center_offsets_[i].Rotate(
            workspace->vehicle_tangent.x(), workspace->vehicle_tangent.y());
        const Vec2d rotated_offset_perp = rotated_offset.Perp();

        const double projection =
            circle_cache.seg_ptr->ProjectOntoUnit(circle_cache.circle_center);
        const double production =
            circle_cache.seg_ptr->ProductOntoUnit(circle_cache.circle_center);

        if (projection >= 0.0 && projection <= circle_cache.seg_ptr->length()) {
          Vec2d force_dir = circle_cache.seg_ptr->unit_direction().Perp();
          if (production < 0.0) {
            force_dir = -force_dir;
          }
          msd_jacobian.template segment<2>(PROB::kStateXIndex) = force_dir;
          msd_jacobian(PROB::kStateThetaIndex) =
              force_dir.dot(rotated_offset_perp);

          if (!using_hessian_approximate_) {
            // hessian
            msd_hessian(PROB::kStateThetaIndex, PROB::kStateThetaIndex) =
                force_dir.dot(-rotated_offset);
          }
        } else {
          const auto& nearest_point_on_seg = projection < 0.0
                                                 ? circle_cache.seg_ptr->start()
                                                 : circle_cache.seg_ptr->end();
          const Vec2d x1 = circle_cache.circle_center - nearest_point_on_seg;
          // When x1 is short, clamp the items coverging to inf.
          // So there would be a small region of gradient disappearance.
          const double dist = std::max(x1.Length(), 1e-5);
          const double dist_inv = 1.0 / dist;
          const double dist_inv_cube = Cube(dist_inv);
          msd_jacobian.template segment<2>(PROB::kStateXIndex) = x1 * dist_inv;
          msd_jacobian(PROB::kStateThetaIndex) =
              x1.dot(rotated_offset_perp) * dist_inv;

          // hessian.
          if (!using_hessian_approximate_) {
            msd_hessian(PROB::kStateXIndex, PROB::kStateXIndex) =
                dist_inv - Sqr(x1.x()) * dist_inv_cube;
            msd_hessian(PROB::kStateXIndex, PROB::kStateYIndex) =
                -x1.x() * x1.y() * dist_inv_cube;
            msd_hessian(PROB::kStateXIndex, PROB::kStateThetaIndex) =
                rotated_offset_perp.x() * dist_inv -
                dist_inv_cube * x1.x() * x1.dot(rotated_offset_perp);

            msd_hessian(PROB::kStateYIndex, PROB::kStateXIndex) =
                msd_hessian(PROB::kStateXIndex, PROB::kStateYIndex);
            msd_hessian(PROB::kStateYIndex, PROB::kStateYIndex) =
                dist_inv - Sqr(x1.y()) * dist_inv_cube;
            msd_hessian(PROB::kStateYIndex, PROB::kStateThetaIndex) =
                rotated_offset_perp.y() * dist_inv -
                dist_inv_cube * x1.y() * x1.dot(rotated_offset_perp);

            msd_hessian(PROB::kStateThetaIndex, PROB::kStateXIndex) =
                msd_hessian(PROB::kStateXIndex, PROB::kStateThetaIndex);
            msd_hessian(PROB::kStateThetaIndex, PROB::kStateYIndex) =
                msd_hessian(PROB::kStateYIndex, PROB::kStateThetaIndex);
            msd_hessian(PROB::kStateThetaIndex, PROB::kStateThetaIndex) =
                (x1.dot(-rotated_offset) + rotated_offset_perp.Sqr()) *
                    dist_inv -
                dist_inv_cube * Sqr(x1.dot(rotated_offset_perp));
          }
        }
      }

      // For each layer, add derivatives.
      for (int j = 0; j < sub_layers_.size(); ++j) {
        const double penatration = circle_cache.msd -
                                   workspace->sub_layer_buffers[j] -
                                   circle_radiuses_[i];
        if (penatration < 0.0) {
          // Use the same space of msd_jacobian to represent penetration
          // jacobian. So as to avoid scarse matrix addition.
          // ====Push====
          const double old_entry = msd_jacobian(PROB::kStateVIndex);
          msd_jacobian(PROB::kStateVIndex) -=
              workspace->sub_layer_dbuffer_dv[j];

          const PenetrationJacobianType& penetration_jacobian = msd_jacobian;
          const PenetrationHessianType& penetration_hessian = msd_hessian;

          // ddgdxdx.
          if (using_hessian_approximate_) {
            result->d2g_dx2 +=
                Cost<PROB>::scale() * sub_layers_[j].gain_of_weight *
                circle_cache.segment_gain *
                (penetration_jacobian.transpose() * penetration_jacobian);
          } else {
            result->d2g_dx2 +=
                Cost<PROB>::scale() * sub_layers_[j].gain_of_weight *
                circle_cache.segment_gain *
                (penatration * penetration_hessian +
                 penetration_jacobian.transpose() * penetration_jacobian);
          }

          // dgdx.
          result->dg_dx += Cost<PROB>::scale() * sub_layers_[j].gain_of_weight *
                           circle_cache.segment_gain * penatration *
                           penetration_jacobian;
          result->is_trivial = false;

          // Use the same space to represent penetration jacobian.
          // So as to avoid scarse matrix addition.
          // ====Pop====
          msd_jacobian(PROB::kStateVIndex) = old_entry;
        }
      }
    }
  }

  int horizon_ = 0;
  MsdProblemWithBuffer curb_msd_;
  const CenterLineQueryHelper<PROB>* center_line_helper_;

  // Multiple layers of costs.
  std::vector<std::string> sub_names_;
  std::vector<Layer> sub_layers_;

  // Vehicle shape info
  std::vector<Vec2d> circle_center_offsets_;
  std::vector<double> circle_radiuses_;

  bool using_hessian_approximate_ = false;

  // When called UpdateCost()
  // In addition to the fact that cost is computed,
  // certain useful inter-mediate variables are cached in workspace.
  // If the user want to UpdateDerivatives() soon after knowing the
  // cost, those cache would be useful.

  const StatesType* xs_ptr_ = nullptr;
  bool cost_updated_ = false;
  bool gradient_updated_ = false;
  std::vector<OneStepWorkspace> workspaces_;

  std::vector<OneStepCostResult> cost_result_;
  std::vector<OneStepGradientResult> gradient_result_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MSD_STATIC_BOUNDARY_COST_V2_H_
