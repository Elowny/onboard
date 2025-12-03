#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <limits.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <numeric>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"  // IWYU pragma: keep
#include "Eigen/Eigenvalues"
#include "Eigen/LU"  // IWYU pragma: keep
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/base/macros.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/ml/optimizer_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/optimization/ddp/ddp_cost_manager_hook.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer_hook.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft::planner {
template <typename PROB>

class DdpOptimizer;
}  // namespace qcraft::planner

DECLARE_int32(planner_dopt_canvas_level);

namespace qcraft {
namespace planner {

#define DDPVLOG(verboselevel) \
  VLOG_IF(verboselevel, ((verbosity_ >= verboselevel)))

// Type PROB is the DDP problem definition.
// A DDP problem is a tuple (f, g^n) where f is the (non-linear) system dynamics
// and g^n is the (time-variable) cost, both of which functions of x and u.
template <typename PROB>
class DdpOptimizer {
 public:
  // Configuration for solve action.
  struct SolveConfig {
    int max_iteration = INT_MAX;
    bool forward = true;
    bool enable_iteration_failure_postprocess = true;
    bool enable_qtrace = false;
    bool enable_qevent = false;

    static constexpr SolveConfig Default() {
      return SolveConfig{
          .max_iteration = INT_MAX,
          .forward = true,
          .enable_iteration_failure_postprocess = true,
          .enable_qtrace = false,
          .enable_qevent = false,
      };
    }

    static constexpr SolveConfig Onboard() {
      return SolveConfig{
          .max_iteration = INT_MAX,
          .forward = true,
          .enable_iteration_failure_postprocess = true,
          .enable_qtrace = true,
          .enable_qevent = true,
      };
    }
  };

  static constexpr int kStateSize = PROB::kStateSize;
  static constexpr int kControlSize = PROB::kControlSize;

  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  // problem will not be owned.
  // But member of problem will be changed when calling
  // const member functions of DdpOptimizer.
  DdpOptimizer(const PROB* problem, int horizon, std::string owner,
               int verbosity, DdpOptimizerParamsProto params,
               std::string dopt_tag);

  DdpOptimizer(const PROB* problem, int horizon, std::string owner,
               int verbosity, DdpOptimizerParamsProto params)
      : DdpOptimizer(problem, horizon, owner, verbosity, params,
                     /*dopt_tag=*/"0") {}

  DdpOptimizer(const PROB* problem, int horizon, std::string owner,
               int verbosity)
      : DdpOptimizer(problem, horizon, owner, verbosity,
                     /*params=*/CreateDefaultParams()) {}

  DdpOptimizer(const PROB* problem, int horizon, std::string owner)
      : DdpOptimizer(problem, horizon, owner, /*verbosity=*/0) {}

  DdpOptimizer(const PROB* problem, int horizon)
      : DdpOptimizer(problem, horizon, /*owner=*/"") {}

  using HookType = typename qcraft::planner::DdpOptimizerHook<PROB>;

  // Not owned.
  void AddHook(HookType* hook) { hooks_.push_back(hook); }

  // Optimize the given init_trajectory, making its cost smaller.
  // Pre-requisite:
  // init_trajectory.size() must greater equal to horizon_.
  absl::StatusOr<std::vector<TrajectoryPoint>> Solve(
      const std::vector<TrajectoryPoint>& init_trajectory,
      const SolveConfig& config = SolveConfig::Default()) const;

  // Evaluate the cost of the given trajectory.
  // Pre-requisite:
  // init_trajectory.size() must greater equal to horizon_.
  double EvaluateCostForTrajectory(
      const std::vector<TrajectoryPoint>& trajectory) const {
    QCHECK_GE(trajectory.size(), horizon_);
    StateType x0;
    ControlsType us(horizon_ * kControlSize);
    StatesType xs(horizon_ * kStateSize);
    FitTrajectoryPointToSolverStates(trajectory, &x0, &us, &xs);

    for (auto* hook : hooks_) {
      hook->OnSolveStart(xs, us);
    }

    return EvaluateCost(xs, us);
  }

  // This function will compute the accumulative discounted(based on gamma) cost
  // for different cost type, and store them separately in
  // AccumulatedDiscountedCostsProto.
  AccumulatedDiscountedCostsProto EvaluateEachDiscountedAccumulativeCost(
      const StatesType& xs, const ControlsType& us, int horizon_clamp,
      double gamma) const;

  int problem_costs_size() const { return problem_->costs().size(); }

 protected:
  using FType = typename PROB::FType;
  using DFDxType = typename PROB::DFDxType;
  using DFDuType = typename PROB::DFDuType;
  using DDFDxDxType = typename PROB::DDFDxDxType;
  using DDFDxDuType = typename PROB::DDFDxDuType;
  using DDFDuDxType = typename PROB::DDFDuDxType;
  using DDFDuDuType = typename PROB::DDFDuDuType;
  using GType = typename PROB::GType;
  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;
  using DDGDxDxType = typename PROB::DDGDxDxType;
  using DDGDxDuType = typename PROB::DDGDxDuType;
  using DDGDuDxType = typename PROB::DDGDuDxType;
  using DDGDuDuType = typename PROB::DDGDuDuType;
  using ClampInfo = typename PROB::ClampInfo;
  using JType = double;
  using DJDxType = Eigen::Matrix<double, 1, kStateSize>;
  using DDJDxDxType = Eigen::Matrix<double, kStateSize, kStateSize>;

  // Returns the optimal total cost, i.e. Js[0].
  double SolveLinearDdp(const PROB& problem, const StatesType& xs,
                        const ControlsType& us, StatesType* dxs,
                        ControlsType* dus, ControlsType* dus_open,
                        std::vector<DDGDuDxType>* dus_close_gain,
                        int* k_ne) const;

  void LineSearchAndStepsizeAdjustment(
      const StateType& x0, const StatesType& xs, const ControlsType& us,
      const ControlsType& dus_open,
      const std::vector<DDGDuDxType>& dus_close_gain, int iteration,
      double line_search_init_cost,
      const std::vector<double>& line_search_alphas, int k_n_e,
      StatesType* curr_xs, ControlsType* curr_us, double* curr_cost,
      int* alpha_idx, int* line_search_count,
      typename HookType::OptimizerInspector* oi) const;

  static bool IsDdJdxdxSemiPositive(const DDJDxDxType& ddJdxdx);  // NOLINT
  template <int k, int j>
  static void ForLoop1(StateType* d_vec, const DDJDxDxType& l, double v) {
    if constexpr (k < j) {
      v -= Sqr(l(j, k)) * (*d_vec)(k);
      ForLoop1<k + 1, j>(d_vec, l, v);
    } else {
      (*d_vec)(j) = v;
    }
  }

  template <int i, int j, int end>
  static void ForLoop3(DDJDxDxType* l) {
    if constexpr (i < end) {
      (*l)(i, j) = 0.0;
      ForLoop3<i + 1, j, end>(l);
    }
  }

  template <int j, int end>
  static bool LoopForIsDdJdxdxSemiPositive(const DDJDxDxType& dd,
                                           StateType* d_vec, DDJDxDxType* l) {
    if constexpr (j < end) {
      constexpr double kEps = 1e-9;
      double v = dd(j, j);
      ForLoop1<0, j>(d_vec, *l, v);
      if ((*d_vec)(j) < -kEps) {
        return false;
      }
      if ((*d_vec)(j) < kEps) {
        ForLoop3<j + 1, j, end>(l);
      } else {
        double d_j_recip = 1.0 / (*d_vec)(j);
        for (int i = j + 1; i < end; ++i) {
          double l_ij = dd(i, j);
          for (int k = 0; k < j; ++k) {
            l_ij -= (*l)(i, k) * (*l)(j, k) * (*d_vec)(k);
          }
          l_ij *= d_j_recip;
          (*l)(i, j) = l_ij;
        }
      }
      return LoopForIsDdJdxdxSemiPositive<j + 1, end>(dd, d_vec, l);
    } else {
      return true;
    }
  }
  template <int i, int end, class HESSG>
  static void ForLoopGradJDot(const Eigen::Matrix<double, 1, kStateSize>& g,
                              const std::array<HESSG, kStateSize>& h,
                              HESSG* v) {
    if constexpr (i < end) {
      *v += g[i] * h[i];
      ForLoopGradJDot<i + 1, end, HESSG>(g, h, v);
    }
  }

  template <typename HESSG>
  static HESSG GradJDot(const Eigen::Matrix<double, 1, kStateSize>& grad_j,
                        const std::array<HESSG, kStateSize>& hess_f,
                        bool enable_dynamic_2nd_derivatives) {
    if (enable_dynamic_2nd_derivatives) return HESSG::Zero();
    HESSG hess_g = grad_j[0] * hess_f[0];
    ForLoopGradJDot<1, kStateSize, HESSG>(grad_j, hess_f, &hess_g);
    return hess_g;
  }

  double EvaluateCost(const StatesType& xs, const ControlsType& us,
                      std::vector<NamedCostEntry>* named_costs) const;
  double EvaluateCost(const StatesType& xs, const ControlsType& us) const {
    return EvaluateCost(xs, us, /*named_costs=*/nullptr);
  }

  void FitTrajectoryPointToSolverStates(std::vector<TrajectoryPoint> trajectory,
                                        StateType* x0, ControlsType* us,
                                        StatesType* xs) const;

  std::vector<TrajectoryPoint> GenerateTrajectoryPointsFromSolverStates(
      const StatesType& xs, const ControlsType& us, bool s_increasing) const;

  double LineSearchAndEvaluateCost(
      int iteration, const StatesType& tentative_xs,
      const ControlsType& tentative_us, double alpha,
      typename HookType::OptimizerInspector* oi) const;

  double StepSizeAdjustmentAndEvaluateCost(
      int iteration, const StatesType& xs, const ControlsType& us,
      int k_stepsize, typename HookType::OptimizerInspector* oi) const;

  ControlsType OptimizeInitialControl(const StatesType& init_xs,
                                      const ControlsType& init_us);

  DdpOptimizerParamsProto CreateDefaultParams() {
    DdpOptimizerParamsProto params;
    return params;
  }

  void MaybeDrawDdpIterCanvas(int iteration, StatesType xs,
                              ControlsType us) const;

 private:
  const PROB* problem_;
  int horizon_ = 0;
  std::string owner_;
  int verbosity_ = 0;
  DdpOptimizerParamsProto params_;
  std::string dopt_tag_;  // Delete after DoptPlanner is deprecated.
  std::unique_ptr<DdpCostManagerHook<PROB>> cost_manager_hook_;

  std::vector<HookType*> hooks_;
  std::vector<TrajectoryPoint> init_points_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementations.
template <typename PROB>
DdpOptimizer<PROB>::DdpOptimizer(const PROB* problem, int horizon,
                                 std::string owner, int verbosity,
                                 DdpOptimizerParamsProto params,
                                 std::string dopt_tag)
    : problem_(CHECK_NOTNULL(problem)),
      horizon_(horizon),
      owner_(std::move(owner)),
      verbosity_(verbosity),
      params_(std::move(params)),
      dopt_tag_(std::move(dopt_tag)) {
  QCHECK_GT(horizon_, 0);
  cost_manager_hook_ = std::make_unique<DdpCostManagerHook<PROB>>(horizon_);
  for (const auto& helper : problem_->cost_helpers()) {
    cost_manager_hook_->AddCostHelper(helper.get());
  }
  for (const auto& cost : problem_->costs()) {
    cost_manager_hook_->AddCost(cost.get());
  }
  AddHook(cost_manager_hook_.get());
}

// According to https://en.wikipedia.org/wiki/Cholesky_decomposition, LDLT
// decomposition: Indefinite matrices have an LDLT decomposition with negative
// entries in D.
template <typename PROB>
bool DdpOptimizer<PROB>::IsDdJdxdxSemiPositive(
    const DDJDxDxType& ddJdxdx) {           // NOLINT
  StateType D_vec = StateType::Zero();      // NOLINT
  DDJDxDxType L = DDJDxDxType::Identity();  // NOLINT
  return LoopForIsDdJdxdxSemiPositive<0, kStateSize>(ddJdxdx, &D_vec, &L);
}

template <typename PROB>
double DdpOptimizer<PROB>::SolveLinearDdp(
    const PROB& problem, const StatesType& xs, const ControlsType& us,
    StatesType* dxs, ControlsType* dus, ControlsType* dus_open,
    std::vector<DDGDuDxType>* dus_close_gain, int* k_ne) const {
  const StateType x0 = PROB::GetStateAtStep(xs, 0);

  std::vector<JType> Js(horizon_ + 1, {0});         // NOLINT
  std::vector<DJDxType> dJdxs(horizon_ + 1);        // NOLINT
  std::vector<DDJDxDxType> ddJdxdxs(horizon_ + 1);  // NOLINT

  std::vector<ControlType> dus_open_tmp(horizon_);
  std::vector<DDGDuDxType> dus_close_gain_tmp(horizon_);

  // Last step has no J_{k+1}.
  Js[horizon_] = 0.0;
  dJdxs[horizon_] = DJDxType::Zero();
  ddJdxdxs[horizon_] = DDJDxDxType::Zero();

  // Step_size adjustment method Ne
  std::optional<int> k_n_e;
  constexpr double kEps = 1e-8;

  typename PROB::FDerivatives fds;
  typename PROB::GDerivatives gds;
  for (int k = horizon_ - 1; k >= 0; --k) {
    problem.EvaluateFDerivatives(k, PROB::GetStateAtStep(xs, k),
                                 PROB::GetControlAtStep(us, k), &fds);
    problem.AddGDerivatives(k, PROB::GetStateAtStep(xs, k),
                            PROB::GetControlAtStep(us, k), &gds);
    const DFDxType& dfdx = fds.dfdx;
    const DFDuType& dfdu = fds.dfdu;
    const DDFDxDxType& ddfdxdx = fds.ddfdxdx;
    const DDFDuDxType& ddfdudx = fds.ddfdudx;
    const DDFDuDuType& ddfdudu = fds.ddfdudu;

    const GType& g = gds.value;
    const DGDxType& dgdx = gds.dgdx;
    const DGDuType& dgdu = gds.dgdu;
    const DDGDxDxType& ddgdxdx = gds.ddgdxdx;
    const DDGDuDxType& ddgdudx = gds.ddgdudx;
    const DDGDuDuType& ddgdudu = gds.ddgdudu;

    const JType& J = Js[k + 1];                    // NOLINT
    const DJDxType& dJdx = dJdxs[k + 1];           // NOLINT
    const DDJDxDxType& ddJdxdx = ddJdxdxs[k + 1];  // NOLINT

    const DDGDuDuType A =  // NOLINT
        ddgdudu +
        GradJDot(dJdx, ddfdudu, problem.enable_dynamic_2nd_derivatives()) +
        dfdu.transpose() * ddJdxdx * dfdu;
    const DDGDuDxType b_lin =
        ddgdudx +
        GradJDot(dJdx, ddfdudx, problem.enable_dynamic_2nd_derivatives()) +
        dfdu.transpose() * ddJdxdx * dfdx;

    double min_eigen_value = 0.0;
    if (!IsDdJdxdxSemiPositive(ddJdxdx)) {
      Eigen::EigenSolver<DDJDxDxType> es(ddJdxdx);
      min_eigen_value = es.pseudoEigenvalueMatrix().minCoeff();
    }
    const DDGDuDuType A_tilde =                         // NOLINT
        A - dfdu.transpose() * min_eigen_value * dfdu;  // NOLINT
    const DDGDuDuType Ainv_tilde = A_tilde.inverse();   // NOLINT
    const DGDuType b_base_tilde = dgdu + dJdx * dfdu;
    const DDGDuDxType b_lin_tilde =
        b_lin - dfdu.transpose() * min_eigen_value * dfdx;
    const GType c_base_tilde = g + J;

    auto& du_open = dus_open_tmp[k];
    auto& du_close_gain = dus_close_gain_tmp[k];
    du_open = -Ainv_tilde * b_base_tilde.transpose();
    du_close_gain = -Ainv_tilde * b_lin_tilde;
    const auto du_open_transpose = du_open.transpose();
    const auto du_close_gain_transpose = du_close_gain.transpose();

    const double dJ_expected =  // NOLINT
        0.5 * du_open_transpose * A * du_open + (b_base_tilde * du_open)(0);
    if (dJ_expected < -kEps && !k_n_e.has_value()) k_n_e = k + 1;

    Js[k] = c_base_tilde + dJ_expected;
    dJdxs[k] = dgdx + dJdx * dfdx + du_open_transpose * A * du_close_gain +
               b_base_tilde * du_close_gain + du_open_transpose * b_lin;
    ddJdxdxs[k] =
        ddgdxdx +
        GradJDot(dJdx, ddfdxdx, problem.enable_dynamic_2nd_derivatives()) +
        dfdx.transpose() * ddJdxdx * dfdx +
        du_close_gain_transpose * A * du_close_gain +
        du_close_gain_transpose * b_lin + b_lin.transpose() * du_close_gain;

    // Symmetrize J hessian. If we don't manually do this, numerical error
    // may accumulate over backward steps and may come to dominate A in the
    // late steps (early time steps in the forward direction), making A^-1
    // significantly inaccurate.
    ddJdxdxs[k] = (ddJdxdxs[k] + ddJdxdxs[k].transpose()) * 0.5;
  }

  // Forward pass: evaluate u_k^* and x_k^*.
  *dxs = StatesType::Zero(horizon_ * kStateSize);
  *dus = ControlsType::Zero(horizon_ * kControlSize);

  const int k_stepsize_upper = k_n_e.has_value() ? *k_n_e : 0;
  int k_stepsize = -k_stepsize_upper;
  constexpr double kDuLimit = 1e4;
  do {
    k_stepsize = (k_stepsize + k_stepsize_upper) >> 1;
    StateType x = x0;
    for (int k = 0; k < horizon_; ++k) {
      const StateType dx = x - PROB::GetStateAtStep(xs, k);
      PROB::SetStateAtStep(dx, k, dxs);
      if (k < k_stepsize) {
        x = PROB::GetStateAtStep(xs, k + 1);
        (*dus_close_gain)[k] = DDGDuDxType::Zero();
        PROB::SetControlAtStep(ControlType::Zero(), k, dus);
        PROB::SetControlAtStep(ControlType::Zero(), k, dus_open);
      } else {
        const ControlType du = dus_open_tmp[k] + dus_close_gain_tmp[k] * dx;
        const ControlType u = PROB::GetControlAtStep(us, k) + du;
        x = problem.EvaluateF(k, x, u);
        (*dus_close_gain)[k] = dus_close_gain_tmp[k];
        PROB::SetControlAtStep(du, k, dus);
        PROB::SetControlAtStep(dus_open_tmp[k], k, dus_open);
      }
    }
  } while ((dus->maxCoeff() > kDuLimit || dus->minCoeff() < -kDuLimit) &&
           k_stepsize < (k_stepsize_upper - 1));

  *k_ne = k_stepsize_upper;

  // Record QEvent when trigger step-size adjustment.
  if (k_stepsize > 0 && k_n_e.has_value()) {
    if (owner_ == "trajectory_optimizer") {
      QEVENT_EVERY_N_SECONDS(
          "runbing", owner_ + "_step_size_adjustment", 0.2,
          [&](QEvent* qevent) { qevent->AddField("k_stepsize", k_stepsize); });
    }
  }

  // Record Qevent if unreasonable du appeared.
  constexpr double kDusNormCheckLimit = 200.0;
  if (dus->norm() > kDusNormCheckLimit && owner_ == "trajectory_optimizer") {
    QEVENT_EVERY_N_SECONDS(
        "runbing", "traj_opt_unreasonable_du", 0.2,
        [&](QEvent* qevent) { qevent->AddField("dus", dus->norm()); });
  }

  return Js[0];
}

template <typename PROB>
void DdpOptimizer<PROB>::LineSearchAndStepsizeAdjustment(
    const StateType& x0, const StatesType& xs, const ControlsType& us,
    const ControlsType& dus_open,
    const std::vector<DDGDuDxType>& dus_close_gain, int iteration,
    double line_search_init_cost, const std::vector<double>& line_search_alphas,
    int k_n_e, StatesType* curr_xs, ControlsType* curr_us, double* curr_cost,
    int* alpha_idx, int* line_search_count,
    typename HookType::OptimizerInspector* oi) const {
  const double min_acceptable_cost_drop = params_.convergence_tolerance_dcost();

  // Setting params_.line_search_min_alpha() to 1.0 or higher will disable
  // the line search.
  // Set curr_alpha to first line_search_alphas front.
  double curr_alpha = line_search_alphas.front();
  int alpha_count = line_search_alphas.size();

  if (params_.line_search_to_min()) {
    double next_alpha = curr_alpha;
    StatesType next_xs = *curr_xs;
    ControlsType next_us = *curr_us;
    double next_cost = *curr_cost;
    // Line search is terminated if one of the following conditions is met:
    // 1. Current step cost drop > min_acceptable_cost_drop and next step
    // cost > current step cost.
    // 2. Next step alpha_idx >= alpha_count.
    while (!(*curr_cost < line_search_init_cost - min_acceptable_cost_drop &&
             next_cost > *curr_cost) &&
           *alpha_idx < alpha_count) {
      curr_alpha = next_alpha;
      *curr_xs = next_xs;
      *curr_us = next_us;
      *curr_cost = next_cost;
      next_alpha = line_search_alphas[*alpha_idx];

      const ControlsType processed_dus_open = dus_open * next_alpha;
      StateType x = x0;
      for (int k = 0; k < horizon_; ++k) {
        PROB::SetStateAtStep(x, k, &next_xs);
        const StateType dx = x - PROB::GetStateAtStep(xs, k);
        const ControlType du = PROB::GetControlAtStep(processed_dus_open, k) +
                               dus_close_gain[k] * dx;
        const ControlType u = PROB::GetControlAtStep(us, k) + du;
        PROB::SetControlAtStep(u, k, &next_us);
        x = problem_->EvaluateF(k, x, u);
      }

      next_cost = LineSearchAndEvaluateCost(iteration, next_xs, next_us,
                                            next_alpha, oi);
      DDPVLOG(2) << "Line search: next_alpha: " << next_alpha
                 << " next_cost: " << next_cost << " curr_alpha: " << curr_alpha
                 << " curr_cost: " << *curr_cost;
      ++*(alpha_idx);
      ++(*line_search_count);
    }

    if (next_cost < *curr_cost) {
      curr_alpha = next_alpha;
      *curr_xs = next_xs;
      *curr_us = next_us;
      *curr_cost = next_cost;
    }
  } else if (*curr_cost > line_search_init_cost - min_acceptable_cost_drop) {
    // Line search is terminated if one of the following conditions is
    // met:
    // 1. Current step cost drop > min_acceptable_cost_drop.
    // 2. Next step alpha_idx >= alpha_count.
    do {
      curr_alpha = line_search_alphas[*alpha_idx];
      const ControlsType processed_dus_open = dus_open * curr_alpha;
      StateType x = x0;
      for (int k = 0; k < horizon_; ++k) {
        PROB::SetStateAtStep(x, k, curr_xs);
        const StateType dx = x - PROB::GetStateAtStep(xs, k);
        const ControlType du = PROB::GetControlAtStep(processed_dus_open, k) +
                               dus_close_gain[k] * dx;
        const ControlType u = PROB::GetControlAtStep(us, k) + du;
        PROB::SetControlAtStep(u, k, curr_us);
        x = problem_->EvaluateF(k, x, u);
      }
      *curr_cost = LineSearchAndEvaluateCost(iteration, *curr_xs, *curr_us,
                                             curr_alpha, oi);
      DDPVLOG(2) << "Line search: curr_alpha: " << curr_alpha
                 << " curr_cost: " << *curr_cost
                 << " drop: " << line_search_init_cost - *curr_cost;
      ++(*alpha_idx);
      ++(*line_search_count);
    } while ((*curr_cost > line_search_init_cost - min_acceptable_cost_drop) &&
             (*alpha_idx) < alpha_count);
  }

  // Try step size adjustment method if line search failed.
  if (*curr_cost > line_search_init_cost - min_acceptable_cost_drop) {
    int k_stepsize = -k_n_e;
    DDPVLOG(2) << "Step_size adjustment: init_cost:" << line_search_init_cost
               << "cost at full step: " << *curr_cost << " k_n_e: " << k_n_e;
    do {
      *curr_xs = xs;
      *curr_us = us;
      k_stepsize = (k_stepsize + k_n_e) >> 1;
      StateType x = PROB::GetStateAtStep(*curr_xs, k_stepsize);
      for (int k = k_stepsize; k < horizon_; ++k) {
        PROB::SetStateAtStep(x, k, curr_xs);
        const StateType dx = x - PROB::GetStateAtStep(xs, k);
        const ControlType du =
            PROB::GetControlAtStep(dus_open, k) + dus_close_gain[k] * dx;
        const ControlType u = PROB::GetControlAtStep(us, k) + du;
        PROB::SetControlAtStep(u, k, curr_us);
        x = problem_->EvaluateF(k, x, u);
      }

      *curr_cost = StepSizeAdjustmentAndEvaluateCost(iteration, *curr_xs,
                                                     *curr_us, k_stepsize, oi);
      DDPVLOG(2) << "Step_size adjustment: k_stepsize: " << k_stepsize
                 << " curr_cost: " << *curr_cost;
    } while ((*curr_cost > line_search_init_cost - min_acceptable_cost_drop) &&
             k_stepsize < (k_n_e - 1));
  }
}

template <typename PROB>
void DdpOptimizer<PROB>::MaybeDrawDdpIterCanvas(int iteration, StatesType xs,
                                                ControlsType us) const {
  if (FLAGS_planner_dopt_canvas_level >= 3) {
    // pc for per-cost channels; pi for per-iteration channels.
    std::vector<vis::Canvas*> canvas_cost_grad_pc_x;
    std::vector<vis::Canvas*> canvas_cost_grad_pc_u;
    std::vector<vis::Canvas*> canvas_cost_grad_pi_x;
    std::vector<vis::Canvas*> canvas_cost_grad_pi_u;
    for (const auto& cost : problem_->costs()) {
      canvas_cost_grad_pc_x.push_back(
          &vis::vantage::GetCanvasClient()->GetCanvas(
              absl::StrFormat("dopt/%s/grad/%s/iter_%03d/x", dopt_tag_,
                              cost->name(), iteration)));
      canvas_cost_grad_pc_u.push_back(
          &vis::vantage::GetCanvasClient()->GetCanvas(
              absl::StrFormat("dopt/%s/grad/%s/iter_%03d/u", dopt_tag_,
                              cost->name(), iteration)));
      canvas_cost_grad_pi_x.push_back(
          &vis::vantage::GetCanvasClient()->GetCanvas(
              absl::StrFormat("dopt/%s/grad_iters/iter_%03d/%s/x", dopt_tag_,
                              iteration, cost->name())));
      canvas_cost_grad_pi_u.push_back(
          &vis::vantage::GetCanvasClient()->GetCanvas(
              absl::StrFormat("dopt/%s/grad_iters/iter_%03d/%s/u", dopt_tag_,
                              iteration, cost->name())));
    }
    vis::Canvas* canvas_total_cost_grad_pc_x =
        &vis::vantage::GetCanvasClient()->GetCanvas(absl::StrFormat(
            "dopt/%s/grad/total/iter_%03d/x", dopt_tag_, iteration));
    vis::Canvas* canvas_total_cost_grad_pc_u =
        &vis::vantage::GetCanvasClient()->GetCanvas(absl::StrFormat(
            "dopt/%s/grad/total/iter_%03d/u", dopt_tag_, iteration));
    vis::Canvas* canvas_total_cost_grad_pi_x =
        &vis::vantage::GetCanvasClient()->GetCanvas(absl::StrFormat(
            "dopt/%s/grad_iters/iter_%03d/total/x", dopt_tag_, iteration));
    vis::Canvas* canvas_total_cost_grad_pi_u =
        &vis::vantage::GetCanvasClient()->GetCanvas(absl::StrFormat(
            "dopt/%s/grad_iters/iter_%03d/total/u", dopt_tag_, iteration));
    for (int k = 0; k < horizon_; ++k) {
      const typename PROB::StateType x = PROB::GetStateAtStep(xs, k);
      const typename PROB::ControlType u = PROB::GetControlAtStep(us, k);
      const Vec2d pos = PROB::StateGetPos(x);
      const double z =
          k * problem_->dt() * kSpaceTimeVisualizationDefaultTimeScale;

      Vec2d total_grad_state_xy = Vec2d::Zero();
      Vec2d total_grad_control_xy = Vec2d::Zero();
      for (int j = 0; j < problem_->costs().size(); ++j) {
        const auto& cost = problem_->costs()[j];
        const typename PROB::DGDxType grad_x = cost->EvaluateDGDx(k, x, u);
        const typename PROB::DGDuType grad_u = cost->EvaluateDGDu(k, x, u);
        //////////////////////////////////////////////////////////////////////
        // Code specific to SOB and TOB problems.
        const Vec2d grad_state_xy(grad_x[0], grad_x[1]);
        const Vec2d grad_control_xy(grad_u[0], grad_u[1]);
        //////////////////////////////////////////////////////////////////////
        if (FLAGS_planner_dopt_canvas_level >= 4) {
          canvas_cost_grad_pc_x[j]->SetGroundZero(1);
          canvas_cost_grad_pc_x[j]->DrawLine(Vec3d(pos, z),
                                             Vec3d(pos + grad_state_xy, z),
                                             vis::Color(0.8, 0.6, 0.2));
          canvas_cost_grad_pc_u[j]->SetGroundZero(1);
          canvas_cost_grad_pc_u[j]->DrawLine(Vec3d(pos, z),
                                             Vec3d(pos + grad_control_xy, z),
                                             vis::Color(0.2, 0.6, 0.8));
          canvas_cost_grad_pi_x[j]->SetGroundZero(1);
          canvas_cost_grad_pi_x[j]->DrawLine(Vec3d(pos, z),
                                             Vec3d(pos + grad_state_xy, z),
                                             vis::Color(0.8, 0.6, 0.2));
          canvas_cost_grad_pi_u[j]->SetGroundZero(1);
          canvas_cost_grad_pi_u[j]->DrawLine(Vec3d(pos, z),
                                             Vec3d(pos + grad_control_xy, z),
                                             vis::Color(0.2, 0.6, 0.8));
        }
        total_grad_state_xy += grad_state_xy;
        total_grad_control_xy += grad_control_xy;
      }
      canvas_total_cost_grad_pc_x->SetGroundZero(1);
      canvas_total_cost_grad_pc_x->DrawLine(Vec3d(pos, z),
                                            Vec3d(pos + total_grad_state_xy, z),
                                            vis::Color(0.9, 0.7, 0.3));
      canvas_total_cost_grad_pc_u->SetGroundZero(1);
      canvas_total_cost_grad_pc_u->DrawLine(
          Vec3d(pos, z), Vec3d(pos + total_grad_control_xy, z),
          vis::Color(0.3, 0.7, 0.9));
      canvas_total_cost_grad_pi_x->SetGroundZero(1);
      canvas_total_cost_grad_pi_x->DrawLine(Vec3d(pos, z),
                                            Vec3d(pos + total_grad_state_xy, z),
                                            vis::Color(0.9, 0.7, 0.3));
      canvas_total_cost_grad_pi_u->SetGroundZero(1);
      canvas_total_cost_grad_pi_u->DrawLine(
          Vec3d(pos, z), Vec3d(pos + total_grad_control_xy, z),
          vis::Color(0.3, 0.7, 0.9));
    }
  }
}

template <typename PROB>
absl::StatusOr<std::vector<TrajectoryPoint>>
DdpOptimizer<PROB>::Solve(  // NOLINT
    const std::vector<TrajectoryPoint>& init_trajectory,
    const SolveConfig& config) const {
  QCHECK_GE(init_trajectory.size(), horizon_);

  SCOPED_QTRACE_IF_ARG1("DdpOptimizer::Solve", "num_costs",
                        problem_->costs().size(), config.enable_qtrace);

  const int xs_length = horizon_ * kStateSize;
  const int us_length = horizon_ * kControlSize;

  // Fit init trajectory into x0, xs, us.
  StateType x0;
  ControlsType init_us(us_length);
  StatesType init_xs(xs_length);
  FitTrajectoryPointToSolverStates(init_trajectory, &x0, &init_us, &init_xs);

  // Solver starts.
  DDPVLOG(1) << "Solve starts";
  typename HookType::OptimizerInspector oi;
  StatesType xs = init_xs;
  ControlsType us = init_us;
  for (auto* hook : hooks_) {
    hook->OnSolveStart(xs, us);
  }

  double total_cost = 0.0;
  {
    const double init_cost = EvaluateCost(init_xs, init_us, &oi.named_costs);
    if (VLOG_IS_ON(2)) {
      DDPVLOG(2) << "Total cost: " << init_cost;
      for (const auto& [cost_name, cost, is_soft] : oi.named_costs) {
        DDPVLOG(2) << "  [" << cost_name << "] cost: " << cost
                   << ", is_soft: " << is_soft;
      }
    }
    DDPVLOG(1) << "Initial cost: " << init_cost;
    total_cost = init_cost;
    oi.cost = init_cost;
  }
  // Set line search alphas variables.
  // Slphas vector.
  std::vector<double> line_search_alphas;
  constexpr double kLineSearchAlphaMultiplier = 0.5;
  // Line search must start from alpha = 1.0 to make sure ine search loop will
  // Try full step du update.
  line_search_alphas.push_back(1.0);
  while (line_search_alphas.back() > params_.line_search_min_alpha()) {
    line_search_alphas.push_back(line_search_alphas.back() *
                                 kLineSearchAlphaMultiplier);
  }
  // Start alpha index in alpha vector, set to 1 because full step will be tried
  // when init.
  int alpha_idx = 1;
  int line_search_count = 0;

  int iteration;
  const int max_iters = std::min(params_.max_iters(), config.max_iteration);
  for (iteration = 0; iteration < max_iters; ++iteration) {
    SCOPED_QTRACE_IF_ARG1("ddp_iter", "iter", iteration, config.enable_qtrace);

    DDPVLOG(2) << "Iteration " << iteration << " starts";

    for (auto* hook : hooks_) {
      hook->OnIterationStart(iteration, xs, us, oi);
    }

    MaybeDrawDdpIterCanvas(iteration, xs, us);

    int k_n_e = horizon_;
    StatesType dxs = StatesType::Zero(xs_length);
    ControlsType dus = ControlsType::Zero(us_length);
    ControlsType dus_open = ControlsType::Zero(us_length);
    std::vector<DDGDuDxType> dus_close_gain(horizon_, DDGDuDxType::Zero());
    const double Js0 =  // NOLINT
        SolveLinearDdp(*problem_, xs, us, &dxs, &dus, &dus_open,
                       &dus_close_gain, &k_n_e);

    if (!problem_->CheckDu(dus, owner_) ||
        !problem_->CheckDu(dus_open, owner_)) {
      return absl::InternalError(owner_ +
                                 " ddp optimizer has unreasonable du.");
    }

    const bool dx_converged =
        dxs.squaredNorm() < Sqr(params_.convergence_tolerance_dx());
    const bool du_converged =
        dus.squaredNorm() < Sqr(params_.convergence_tolerance_du());

    double dcost = 0.0;
    if (!dx_converged && !du_converged) {
      // Line search on du. The control flow below might appear weird, but it is
      // organized so that the calls to RollOutControl() and EvaluateCost() are
      // minimized.

      // Setting params_.line_search_min_alpha() to 1.0 or higher will disable
      // the line search.
      // Set curr_alpha to first line_search_alphas front.
      StatesType curr_xs = xs + dxs;
      ControlsType curr_us = us + dus;
      double curr_cost = LineSearchAndEvaluateCost(
          iteration, curr_xs, curr_us, line_search_alphas.front(), &oi);

      DDPVLOG(2) << "Line search: init cost: " << total_cost
                 << "; Js[0]: " << Js0 << " cost at full step: " << curr_cost
                 << " du step norm: " << dus.norm();

      LineSearchAndStepsizeAdjustment(x0, xs, us, dus_open, dus_close_gain,
                                      iteration, total_cost, line_search_alphas,
                                      k_n_e, &curr_xs, &curr_us, &curr_cost,
                                      &alpha_idx, &line_search_count, &oi);

      if (params_.line_search_min_alpha() >= 1.0) {
        DDPVLOG(2) << "Line search disabled. curr_cost = " << curr_cost
                   << "; old cost = " << total_cost;
        dcost = curr_cost - total_cost;
        total_cost = curr_cost;
        xs = curr_xs;
        us = curr_us;
      } else if (curr_cost >
                 total_cost - params_.convergence_tolerance_dcost()) {
        // Line search terminated due to alpha limit.
        DDPVLOG(2) << "Line search failed. This iteration is rejected.";
        // Final update if not enable line search to min, for print final cost.
        if (!params_.line_search_to_min()) {
          for (auto* hook : hooks_) {
            hook->OnLineSearchIterationStart(xs, us);
          }
        }
      } else {
        // Acceptable new cost found. Exit line search.
        DDPVLOG(2) << "Acceptable cost drop found by line search: alpha = "
                   << line_search_alphas[alpha_idx] << " cost = " << curr_cost
                   << " cost drop = " << total_cost - curr_cost << " from "
                   << total_cost;
        dcost = curr_cost - total_cost;
        total_cost = curr_cost;
        xs = curr_xs;
        us = curr_us;

        // Compute next line search start alpha.
        // Line search in next iteration should not start from first alpha, as
        // We think step length may similar to current step length.
        constexpr double kLineSearchDecayFactor = 2.0 / 3.0;
        const int alpha_idx_offset = -1;
        alpha_idx =
            params_.enable_adaptive_alpha()
                ? std::max(1, static_cast<int>(
                                  floor(alpha_idx * kLineSearchDecayFactor) +
                                  alpha_idx_offset))
                : 1;
      }
      // If enable line search to min, need final update to make sure
      // information in cost is coincided to total cost.
      if (params_.line_search_to_min()) {
        for (auto* hook : hooks_) {
          hook->OnLineSearchIterationStart(xs, us);
        }
      }

      oi.cost = total_cost;
      // End of line search.
    }

    total_cost = EvaluateCost(xs, us, &oi.named_costs);
    oi.cost = total_cost;
    oi.js0 = Js0;
    if (VLOG_IS_ON(2)) {
      DDPVLOG(2) << "Total cost: " << total_cost;
      for (const auto& [cost_name, cost, is_soft] : oi.named_costs) {
        DDPVLOG(2) << "  [" << cost_name << "] cost: " << cost
                   << ", is_soft: " << is_soft;
      }
    }

    for (auto* hook : hooks_) {
      hook->OnIterationEnd(iteration, xs, us, oi);
    }

    if (du_converged) {
      DDPVLOG(1) << "Terminating due to du convergence.";
      break;
    }
    if (dx_converged) {
      DDPVLOG(1) << "Terminating due to dx convergence.";
      break;
    }
    if (std::abs(dcost) < params_.convergence_tolerance_dcost()) {
      DDPVLOG(1) << "Terminating due to cost convergence.";
      constexpr double kDropFailedJs0Cost = 2.0;
      if ((total_cost - Js0) > kDropFailedJs0Cost &&
          owner_ == "trajectory_optimizer") {
        if (config.enable_qevent) {
          QEVENT("runbing", "traj_opt_ddp_drop_failed", [&](QEvent* qevent) {
            qevent->AddField("cost", total_cost)
                .AddField("Js0", Js0)
                .AddField("iteration", iteration);
          });
        }

        QLOG(WARNING) << "DDP optimizer drop failed in(" << owner_ << "), Js0("
                      << Js0 << "), (total_cost(" << total_cost << ").";
      }
      break;
    }
    // End of iteration.
  }

  // Warnings
  {
    if (iteration < params_.max_iters()) {
      DDPVLOG(1) << "DDP optimizer finished in " << iteration
                 << " iterations, line search count: " << line_search_count
                 << ".";
    } else {
      QLOG(WARNING) << owner_ << " DDP optimizer did not finish in "
                    << iteration << " iterations";
      if (config.enable_qevent) {
        if (owner_ == "trajectory_optimizer") {
          QEVENT("runbing", "traj_opt_max_iteration",
                 [&](QEvent* qevent) { qevent->AddField("iter", iteration); });
        } else if (owner_ == "freespace_local_smoother") {
          QEVENT("fengzhuang", "fs_local_smoother_max_iteration",
                 [&](QEvent* qevent) { qevent->AddField("iter", iteration); });
        }
      }
    }
    constexpr int kLineSearchQeventRecordLimit = 50;
    if (line_search_count >= kLineSearchQeventRecordLimit) {
      if (owner_ == "trajectory_optimizer" && config.enable_qevent) {
        QEVENT_EVERY_N_SECONDS("runbing", "traj_opt_line_search_count_large",
                               0.2, [&](QEvent* qevent) {
                                 qevent->AddField("line_search_count",
                                                  line_search_count);
                               });
      }
    }
  }

  DDPVLOG(1) << "Final cost: " << total_cost;

  // Only post process longitudinal control and state.
  // If ddp optimizer drop failed at first iteration, refuse to postprocess
  // trajectory.
  if (iteration > 0 || config.enable_iteration_failure_postprocess) {
    StateType x = x0;
    for (int k = 0; k < horizon_; ++k) {
      PROB::SetStateAtStep(x, k, &xs);
      ControlType u;
      ClampInfo state_clamp_info, control_clamp_info;
      u = problem_->PostProcessLonU(
          PROB::GetControlAtStep(us, k), x,
          k != horizon_ - 1
              ? PROB::GetStateAtStep(xs, k + 1)
              : problem_->EvaluateF(k, x, PROB::GetControlAtStep(us, k)),
          &control_clamp_info, config.forward);
      PROB::SetControlAtStep(u, k, &us);
      const auto x_origin = problem_->EvaluateF(k, x, u);
      x = problem_->PostProcessLonX(x_origin, u, &state_clamp_info);
    }
  }

  // Solver end hook.
  for (auto* hook : hooks_) {
    hook->OnSolveEnd(xs, us, oi);
  }

  // Build trajectory points from xs us.
  std::vector<TrajectoryPoint> res =
      GenerateTrajectoryPointsFromSolverStates(xs, us, config.forward);

  return res;
}

template <typename PROB>
void DdpOptimizer<PROB>::FitTrajectoryPointToSolverStates(
    std::vector<TrajectoryPoint> trajectory, StateType* x0, ControlsType* us,
    StatesType* xs) const {
  QCHECK_NOTNULL(x0);
  QCHECK_NOTNULL(us);
  QCHECK_NOTNULL(xs);

  // Preprocess trajectory.
  {
    QCHECK_GE(trajectory.size(), horizon_) << "Invalid trajectory size.";
    for (int k = 0; k < horizon_; ++k) {
      trajectory[k].set_t(k * problem_->dt());
    }

    // Make sure each consecutive points' theta diff
    //  aren't jumping 2*pi.
    trajectory.front().set_theta(
        trajectory[1].theta() +
        NormalizeAngle(trajectory.front().theta() - trajectory[1].theta()));
    for (int i = 2; i < trajectory.size(); ++i) {
      const double prev_theta = trajectory[i - 1].theta();
      const double angle_diff_with_prev_pt =
          NormalizeAngle(trajectory[i].theta() - prev_theta);
      trajectory[i].set_theta(prev_theta + angle_diff_with_prev_pt);
    }
  }

  // Generate initial controls and states.
  // We do not rollout initial states but use fitted initial states.
  *x0 = problem_->FitInitialState(trajectory);
  *us = problem_->FitControl(trajectory, *x0);
  *xs = problem_->FitState(trajectory);
}

template <typename PROB>
std::vector<TrajectoryPoint>
DdpOptimizer<PROB>::GenerateTrajectoryPointsFromSolverStates(
    const StatesType& xs, const ControlsType& us, bool s_increasing) const {
  // Build trajectory points from xs us.
  std::vector<TrajectoryPoint> res;
  res.resize(horizon_);
  for (int k = 0; k < horizon_; ++k) {
    TrajectoryPoint& point = res[k];
    problem_->ExtractTrajectoryPoint(k, PROB::GetStateAtStep(xs, k),
                                     PROB::GetControlAtStep(us, k), &point);
  }

  // Re-fill s of all trajectory points.
  if (problem_->enable_post_process()) {
    res.front().set_s(0.0);
    for (int i = 1; i < horizon_; ++i) {
      const double d = (res[i].pos() - res[i - 1].pos()).norm();
      if (s_increasing) {
        res[i].set_s(res[i - 1].s() + d);
      } else {
        res[i].set_s(res[i - 1].s() - d);
      }
    }
  }

  return res;
}

template <typename PROB>
double DdpOptimizer<PROB>::EvaluateCost(
    const StatesType& xs, const ControlsType& us,
    std::vector<NamedCostEntry>* named_costs) const {
  std::vector<double> costs(problem_->costs().size(), 0.0);

  if (named_costs != nullptr) named_costs->clear();

  for (int i = 0; i < problem_->costs().size(); ++i) {
    const auto divided_g =
        problem_->costs()[i]->SumGForAllSteps(xs, us, horizon_);
    costs[i] += divided_g.sum();
    if (named_costs != nullptr) {
      for (const auto& cost : divided_g.gs()) {
        named_costs->push_back(cost);
      }
    }
  }
  const double total_cost = std::accumulate(costs.begin(), costs.end(), 0.0);
  if (UNLIKELY(VLOG_IS_ON(4))) {
    DDPVLOG(4) << "Total cost: " << total_cost;
    for (const auto& [cost_name, cost, is_soft] : *named_costs) {
      DDPVLOG(4) << "  [" << cost_name << "] cost: " << cost
                 << ", is_soft: " << is_soft;
    }
  }
  return total_cost;
}

template <typename PROB>
double DdpOptimizer<PROB>::LineSearchAndEvaluateCost(
    int iteration, const StatesType& tentative_xs,
    const ControlsType& tentative_us, double alpha,
    typename HookType::OptimizerInspector* oi) const {
  for (auto* hook : hooks_) {
    hook->OnLineSearchIterationStart(tentative_xs, tentative_us);
  }
  const double cost = EvaluateCost(tentative_xs, tentative_us);
  oi->cost = cost;
  for (auto* hook : hooks_) {
    hook->OnLineSearchIterationEnd(iteration, alpha, cost);
  }
  return cost;
}

template <typename PROB>
double DdpOptimizer<PROB>::StepSizeAdjustmentAndEvaluateCost(
    int iteration, const StatesType& xs, const ControlsType& us, int k_stepsize,
    typename HookType::OptimizerInspector* oi) const {
  DDPVLOG(4) << "Evaluating cost for step-size adjustment iteration "
             << iteration << " with step " << k_stepsize;
  for (auto* hook : hooks_) {
    hook->OnStepSizeAdjustmentIterationStart(xs, us);
  }
  const double cost = EvaluateCost(xs, us);
  oi->cost = cost;
  for (auto* hook : hooks_) {
    hook->OnStepSizeAdjustmentIterationEnd(iteration, k_stepsize, cost);
  }
  return cost;
}

template <typename PROB>
AccumulatedDiscountedCostsProto
DdpOptimizer<PROB>::EvaluateEachDiscountedAccumulativeCost(
    const StatesType& xs, const ControlsType& us, int horizon_clamp,
    double gamma) const {
  horizon_clamp = std::min(horizon_clamp, horizon_);
  const int soft_feature_idx = 0;
  const int hard_feature_idx = 1;
  const std::string object_feature_name = "ObjectCost";
  const std::string curb_msd_feature_name = "MsdStaticBoundaryCostV2_curb";
  const std::string sold_line_msd_feature_name =
      "SolidLineMsdStaticBoundaryCost";
  double solid_line_msd_feature_cost_value = 0.0;
  std::vector<std::pair<std::string, double>> curb_msd_cost;
  curb_msd_cost.resize(/*object_cost_buffet_size=*/2, {"", 0.0});
  std::vector<std::pair<std::string, double>> object_cost;
  object_cost.resize(/*object_cost_buffet_size=*/2, {"", 0.0});

  AccumulatedDiscountedCostsProto costs_proto;

  for (int i = 0; i < problem_->costs().size(); ++i) {
    double gamma_k = 1.0;
    std::vector<NamedCostEntry> divide_cost;
    for (int k = 0; k < horizon_clamp; ++k) {
      const auto cost_k = problem_->costs()[i]->EvaluateGWithDebugInfo(
          k, PROB::GetStateAtStep(xs, k), PROB::GetControlAtStep(us, k),
          /*using_scale=*/false);
      const auto& gs = cost_k.gs();
      const int gs_size = gs.size();
      if (divide_cost.size() < gs_size) {
        const int old_size = divide_cost.size();
        divide_cost.resize(gs_size);
        for (int idx = old_size; idx < gs_size; ++idx) {
          divide_cost[idx].name = gs[idx].name;
          divide_cost[idx].is_soft = gs[idx].is_soft;
          divide_cost[idx].value = 0.0;
        }
      }
      for (int idx = 0; idx < gs_size; ++idx) {
        divide_cost[idx].value += gamma_k * gs[idx].value;
      }
      gamma_k *= gamma;
    }

    const auto& cost_type = problem_->costs()[i]->cost_type();
    if (cost_type == Cost<PROB>::CostType::UNKNOWN) {
      QLOG(ERROR) << problem_->costs()[i]->name()
                  << " is an unknown type cost.";
    }
    if (cost_type == Cost<PROB>::CostType::GROUP_OBJECT) {
      for (int idx = 0; idx < divide_cost.size(); ++idx) {
        if (divide_cost[idx].is_soft) {
          object_cost[soft_feature_idx].second += divide_cost[idx].value;
        } else {
          object_cost[hard_feature_idx].second += divide_cost[idx].value;
        }
      }
      continue;
    }
    // BANDIT(jingqiao): SolidLineMsdStaticBoundaryCost doesn't exist all the
    // time, so need to fake it with 0.0 value if it doesn't exist.
    if (cost_type == Cost<PROB>::CostType::SOLID_LINE_MSD_STATIC_BOUNDARY) {
      QCHECK_EQ(divide_cost.size(), 1);
      QCHECK_EQ(divide_cost[0].name, sold_line_msd_feature_name);

      solid_line_msd_feature_cost_value = divide_cost[0].value;
      continue;
    }

    // BANDIT(jingqiao): CURB_MSD_STATIC_BOUNDARY_V2 and
    // UTURN_RIGHT_CURB_MSD_STATIC_BOUNDARY_V2 share the same cost weight so
    // they need to be merged together by their corresponding gain.
    if (cost_type == Cost<PROB>::CostType::CURB_MSD_STATIC_BOUNDARY_V2) {
      for (int idx = 0; idx < divide_cost.size(); ++idx) {
        if (divide_cost[idx].is_soft) {
          curb_msd_cost[soft_feature_idx].second +=
              kCurbGain * divide_cost[idx].value;
        } else {
          curb_msd_cost[hard_feature_idx].second +=
              kCurbGain * divide_cost[idx].value;
        }
      }
      continue;
    }
    if (cost_type ==
        Cost<PROB>::CostType::UTURN_RIGHT_CURB_MSD_STATIC_BOUNDARY_V2) {
      for (int idx = 0; idx < divide_cost.size(); ++idx) {
        if (divide_cost[idx].is_soft) {
          curb_msd_cost[soft_feature_idx].second +=
              kUTurnCurbGain * divide_cost[idx].value;
        } else {
          curb_msd_cost[hard_feature_idx].second +=
              kUTurnCurbGain * divide_cost[idx].value;
        }
      }
      continue;
    }

    for (int idx = 0; idx < divide_cost.size(); ++idx) {
      FeatureCostProto* feature_cost = costs_proto.add_feature_costs();
      feature_cost->set_feature(divide_cost[idx].name);
      feature_cost->set_cost(divide_cost[idx].value);
    }
  }
  for (int idx = 0; idx < object_cost.size(); ++idx) {
    FeatureCostProto* object_feature_cost = costs_proto.add_feature_costs();
    if (idx == soft_feature_idx) {
      object_feature_cost->set_feature(SoftNameString + object_feature_name);
    } else if (idx == hard_feature_idx) {
      object_feature_cost->set_feature(HardNameString + object_feature_name);
    }
    object_feature_cost->set_cost(object_cost[idx].second);
  }
  // Add Soft and Hard CurbMsdStaticBoundaryCost at the end to preserve the
  // order of cost.
  for (int idx = 0; idx < curb_msd_cost.size(); ++idx) {
    FeatureCostProto* curb_msd_feature_cost = costs_proto.add_feature_costs();
    if (idx == soft_feature_idx) {
      curb_msd_feature_cost->set_feature(SoftNameString +
                                         curb_msd_feature_name);
    } else if (idx == hard_feature_idx) {
      curb_msd_feature_cost->set_feature(HardNameString +
                                         curb_msd_feature_name);
    }
    curb_msd_feature_cost->set_cost(curb_msd_cost[idx].second);
  }
  // Add SolidLineMsdStaticBoundaryCost at the end to preserve the order of
  // cost.
  FeatureCostProto* solid_line_msd_feature_cost =
      costs_proto.add_feature_costs();
  solid_line_msd_feature_cost->set_feature(sold_line_msd_feature_name);
  solid_line_msd_feature_cost->set_cost(solid_line_msd_feature_cost_value);

  if (UNLIKELY(VLOG_IS_ON(2))) {
    DDPVLOG(2) << "Discounted Accumulative Cost: ";
    for (int i = 0; i < costs_proto.feature_costs_size(); ++i) {
      DDPVLOG(2) << "  [" << costs_proto.feature_costs(i).feature()
                 << "] cost: " << costs_proto.feature_costs(i).cost();
    }
  }
  return costs_proto;
}

#undef DDPVLOG
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_H_
