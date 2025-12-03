#ifndef ONBOARD_PLANNER_SCHEDULER_REFERENCE_LINE_QP_SMOOTHER_H_
#define ONBOARD_PLANNER_SCHEDULER_REFERENCE_LINE_QP_SMOOTHER_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <type_traits>
#include <utility>
#include <vector>

#include "Eigen/SparseCore"
#include "osqp/osqp.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/eigen.h"
#include "onboard/math/qp/osqp_solver.h"
#include "onboard/math/vec.h"
#include "onboard/planner/scheduler/proto/reference_line_qp_smoother.pb.h"

namespace qcraft::planner {

class FrenetFrameQpAgent;

// The class tries to smooth a reference line described in frenet frame,
// using quadratic programming (QP).
// To be exact, a vector of bases with position and tangent must be provided
// to define a frenet frame. the first and last 3 lateral offsets of the
// reference line must be given to make sure the cuvature derivative cost is
// effective at both ends. Also, the class allows user to specify the
// range of the lateral offsets as the constraint of the QP problem.
// The effect of the smoothing action is decided by param.
// Please go to the .proto file for detailed explanation.
// Reference manual is here:
// https://docs.google.com/presentation/d/1owcki8anavZPJL2zDtKkkm-LlM9zYoFZrfaC_FCYYGU/edit?usp=sharing
// Author: huaiyuan@
class ReferenceLineQpSmoother {
 public:
  static constexpr double kEpsilon = 1e-9;
  static constexpr int kNumPreOptimizationPoints = 3;
  static constexpr int kNumPostOptimizationPoints = 3;

  struct Input {
    // The position and tangent of each base.
    // The following 2 vectors must have the same size.
    std::vector<Vec2d> base_poses;
    std::vector<Vec2d> base_tangents;

    // Fixed lateral offsets at base id 0,1,2.
    std::array<double, kNumPreOptimizationPoints> pre_fixed_lateral_offsets;

    // Fixed lateral offsets at base id max_bid-3,max_bid-2,max_bid-1.
    // where max_bid == base_poses.size()
    std::array<double, kNumPostOptimizationPoints> post_fixed_lateral_offsets;

    // l_upper_bound[i] is the upper bound of lateral offset
    // at base id i+3.
    // l_upper_bound.size() == base_poses.size() - 6.
    std::vector<double> l_upper_bound;
    // l_lower_bound[i] is the lower bound of lateral offset
    // at base id i+3.
    // l_lower_bound.size() == base_poses.size() - 6.
    std::vector<double> l_lower_bound;
  };

  ReferenceLineQpSmoother(const ReferenceLineQpSmootherParamProto* param,
                          Input input);

  // Run qp for the optimized result.
  // std::nullopt when QP fails.
  std::optional<std::vector<double>> Solve() const;

 private:
  using RowVec2d = Eigen::Matrix<double, 1, 2>;
  using RowVec3d = Eigen::Matrix<double, 1, 3>;
  using RowVec4d = Eigen::Matrix<double, 1, 4>;
  using RowVec5d = Eigen::Matrix<double, 1, 5>;
  using ColVec2d = Eigen::Matrix<double, 2, 1>;
  using ColVec3d = Eigen::Matrix<double, 3, 1>;
  using ColVec4d = Eigen::Matrix<double, 4, 1>;

  using Triplets = std::vector<Eigen::Triplet<double>>;

  void InitializeFactors();

  void AddDeltaCost(FrenetFrameQpAgent* agent) const;

  void AddKappaCost(FrenetFrameQpAgent* agent) const;

  void AddLambdaCost(FrenetFrameQpAgent* agent) const;

  void AddLateralOffsetConstraint(FrenetFrameQpAgent* agent) const;

  void AddLengthCost(FrenetFrameQpAgent* agent) const;

  double GetFixedLateralOffset(int optimization_id) const;

  // Not owned.
  const ReferenceLineQpSmootherParamProto* param_ = nullptr;
  Input input_;

  int center_line_size_ = 0;
  int n_dim_ = 0;
  int optimization_begin_ = 0;
  int optimization_end_ = 0;
  std::unique_ptr<OSQPSettings> solver_setting_;
  std::vector<double> x01_lengths_;
  // delta_01 = delta_factor * [1,l0,l1]'
  std::vector<RowVec3d> delta_01_factors_;
  // kappa_1 = kappa_factor * [1,l0,l1,l2]'
  std::vector<RowVec4d> kappa_1_factors_;
  // lambda_12 = lambda_factor * [1,l0,l1,l2,l3]'
  std::vector<RowVec5d> lambda_12_factors_;
  // length_01 = length_factor * [1,l0,l1]'
  std::vector<RowVec3d> length_01_factors_;
};

// FrenetFrameQpAgent builds a QP problem (in OsqpSolver)
// from quadratic cost formulas in the form of
// a small number of optimization variables.
class FrenetFrameQpAgent {
 public:
  static constexpr double kEpsilon = 1e-9;
  FrenetFrameQpAgent(int n_dim, const std::function<double(int)>* get_fixed_l)
      : n_dim_(n_dim), get_fixed_l_(QCHECK_NOTNULL(get_fixed_l)) {
    QCHECK_GT(n_dim, 0);
  }

  // Suppose one have a cost that can be expressed by a continuous range of
  // lateral offsets: l_local = [l[l0_id], l[l0_id+1], ... l[l0_id + LDim -
  // 1]] sub_cost = 0.5 * l_local * local_A * l_local' + local_l * local_b.
  // This function adds up the cost to A,b, according to the optimization
  // range settings.

  // l0_id can be out of [0, n_dim_), in such case, the agent will treat those
  // l's out of range as fixed and get their value using "get_fixed_l".
  template <int LDim>
  void AddLocalCost(const Eigen::Matrix<double, LDim, LDim>& local_A,
                    const Eigen::Matrix<double, LDim, 1>& local_b, int l0_id);

  // Suppose one have a constraint that can be expressed by a continuous range
  // of lateral offsets: l_local = [l[l0_id], l[l0_id+1], ... l[l0_id + LDim -
  // 1]], local_H * l_local' >= local_h.
  // This function adds up the constraint to H,h, according to the optimization
  // range settings.

  // l0_id can be out of [0, n_dim_), in such case, the agent will treat those
  // l's out of range as fixed and get their value using "get_fixed_l".
  template <int LDim, int NCons>
  void AddLocalConstraint(const Eigen::Matrix<double, NCons, LDim>& local_H,
                          const Eigen::Matrix<double, NCons, 1>& local_h,
                          int l0_id);

  // Build the Osqp solver.
  qcraft::OsqpSolver BuildOsqpSolver() const {
    SMatXd A(n_dim_, n_dim_);
    SMatXd b(n_dim_, 1);
    SMatXd H(number_of_constraints_, n_dim_);
    SMatXd h(number_of_constraints_, 1);

    A.setFromTriplets(A_triplets_.begin(), A_triplets_.end());
    b.setFromTriplets(b_triplets_.begin(), b_triplets_.end());
    H.setFromTriplets(H_triplets_.begin(), H_triplets_.end());
    h.setFromTriplets(h_triplets_.begin(), h_triplets_.end());

    return qcraft::OsqpSolver(std::move(A), std::move(b), SMatXd{0, n_dim_},
                              SMatXd{0, 1}, std::move(H), std::move(h));
  }

 private:
  using Triplets = std::vector<Eigen::Triplet<double>>;

  int n_dim_ = 0;

  // Not owned.
  // Querry fixed l's with id < 0 and >= n_dim_.
  const std::function<double(int)>* get_fixed_l_ = nullptr;

  Triplets A_triplets_;
  Triplets b_triplets_;
  Triplets H_triplets_;
  Triplets h_triplets_;
  int number_of_constraints_ = 0;
};

template <int LDim>
void FrenetFrameQpAgent::AddLocalCost(
    const Eigen::Matrix<double, LDim, LDim>& local_A,
    const Eigen::Matrix<double, LDim, 1>& local_b, int l0_id) {
  static_assert(LDim > 0, "");

  const int local_opt_begin = std::max(0, l0_id);
  const int local_opt_end = std::min(n_dim_, l0_id + LDim);
  const int pre_constant_num = local_opt_begin - l0_id;
  const int post_constant_num = l0_id + LDim - local_opt_end;
  const int opt_num = local_opt_end - local_opt_begin;

  if (opt_num < 1) {
    return;
  }

  // Optimizable sub A.
  // Only need upper triangular part for later osqp-0.6.0
  for (int j = local_opt_begin; j < local_opt_end; ++j) {
    for (int i = local_opt_begin; i <= j; ++i) {
      const double entry = local_A(i - l0_id, j - l0_id);
      if (std::abs(entry) > kEpsilon) {
        A_triplets_.emplace_back(i, j, entry);
      }
    }
  }

  // Optimizable sub b
  for (int i = local_opt_begin; i < local_opt_end; ++i) {
    const double entry = local_b(i - l0_id);
    if (std::abs(entry) > kEpsilon) {
      b_triplets_.emplace_back(i, 0, entry);
    }
  }

  // Additional items generated by cross items of optimizable and
  // non-optimizable variables in A.
  Eigen::MatrixXd extra_b = Eigen::MatrixXd::Zero(1, opt_num);

  if (pre_constant_num > 0) {
    Eigen::MatrixXd pre_constants(1, pre_constant_num);
    for (int i = 0; i < pre_constant_num; ++i) {
      pre_constants(0, i) = (*get_fixed_l_)(l0_id + i);
    }
    extra_b += 0.5 * pre_constants *
               (local_A.block(0, pre_constant_num, pre_constant_num, opt_num) +
                local_A.block(pre_constant_num, 0, opt_num, pre_constant_num)
                    .transpose());
  }

  if (post_constant_num > 0) {
    Eigen::MatrixXd post_constants(1, post_constant_num);
    for (int i = 0; i < post_constant_num; ++i) {
      post_constants(0, i) = (*get_fixed_l_)(local_opt_end + i);
    }
    extra_b += 0.5 * post_constants *
               (local_A.block(pre_constant_num + opt_num, pre_constant_num,
                              post_constant_num, opt_num) +
                local_A
                    .block(pre_constant_num, pre_constant_num + opt_num,
                           opt_num, post_constant_num)
                    .transpose());
  }

  for (int i = local_opt_begin; i < local_opt_end; ++i) {
    const double entry = extra_b(i - local_opt_begin);
    if (std::abs(entry) > kEpsilon) {
      b_triplets_.emplace_back(i, 0, entry);
    }
  }
}

template <int LDim, int NCons>
void FrenetFrameQpAgent::AddLocalConstraint(
    const Eigen::Matrix<double, NCons, LDim>& local_H,
    const Eigen::Matrix<double, NCons, 1>& local_h, int l0_id) {
  static_assert(NCons > 0, "");
  static_assert(LDim > 0, "");

  const int local_opt_begin = std::max(0, l0_id);
  const int local_opt_end = std::min(n_dim_, l0_id + LDim);
  const int pre_constant_num = local_opt_begin - l0_id;
  const int post_constant_num = l0_id + LDim - local_opt_end;
  const int opt_num = local_opt_end - local_opt_begin;

  if (opt_num < 1) {
    return;
  }

  Eigen::Matrix<double, NCons, 1> constants = local_h;

  if (pre_constant_num > 0) {
    Eigen::MatrixXd pre_constants(pre_constant_num, 1);
    for (int i = 0; i < pre_constant_num; ++i) {
      pre_constants(i, 0) = (*get_fixed_l_)(l0_id + i);
    }
    constants -= local_H.block(0, 0, NCons, pre_constant_num) * pre_constants;
  }

  if (post_constant_num > 0) {
    Eigen::MatrixXd post_constants(post_constant_num, 1);
    for (int i = 0; i < post_constant_num; ++i) {
      post_constants(i, 0) = (*get_fixed_l_)(local_opt_end + i);
    }
    constants -=
        local_H.block(0, pre_constant_num + opt_num, NCons, post_constant_num) *
        post_constants;
  }

  for (int i = 0; i < NCons; ++i) {
    const int H_row_id = number_of_constraints_;
    for (int j = local_opt_begin; j < local_opt_end; ++j) {
      H_triplets_.emplace_back(H_row_id, j, local_H(i, j - l0_id));
    }
    h_triplets_.emplace_back(H_row_id, 0, constants(i));
    number_of_constraints_ += 1;
  }
}

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_REFERENCE_LINE_QP_SMOOTHER_H_
