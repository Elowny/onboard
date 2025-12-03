
#include "onboard/planner/scheduler/reference_line_qp_smoother.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include "absl/status/status.h"

#include "onboard/lite/logging.h"
#include "onboard/math/util.h"

namespace qcraft::planner {

ReferenceLineQpSmoother::ReferenceLineQpSmoother(
    const ReferenceLineQpSmootherParamProto* param, Input input)
    : param_(QCHECK_NOTNULL(param)), input_(std::move(input)) {
  center_line_size_ = static_cast<int>(input_.base_poses.size());
  n_dim_ = center_line_size_ - kNumPreOptimizationPoints -
           kNumPostOptimizationPoints;
  optimization_begin_ = kNumPreOptimizationPoints;
  optimization_end_ = center_line_size_ - kNumPostOptimizationPoints;

  QCHECK_GE(n_dim_, 1) << "Too little points to be optimized";
  QCHECK_EQ(center_line_size_, input_.base_tangents.size());

  QCHECK_EQ(n_dim_, input_.l_upper_bound.size());
  QCHECK_EQ(n_dim_, input_.l_lower_bound.size());

  QCHECK_GE(param_->lateral_offset_weight(), 0.0);
  QCHECK_GE(param_->delta_weight(), 0.0);
  QCHECK_GE(param_->kappa_weight(), 0.0);
  QCHECK_GE(param_->lambda_weight(), 0.0);
  QCHECK_GE(param_->length_weight(), 0.0);
  QCHECK_GT(param_->base_kappa_gain(), 0.0);
  QCHECK_GE(param_->base_length_gain(), 0.0);

  for (int i = 0; i < n_dim_; ++i) {
    QCHECK_LE(input_.l_lower_bound[i], input_.l_upper_bound[i])
        << " index: " << i << " l_lower_bound: " << input_.l_lower_bound[i]
        << " l_upper_bound: " << input_.l_upper_bound[i];
  }

  // Initialize solver settings.
  solver_setting_ = std::make_unique<OSQPSettings>();
  osqp_set_default_settings(solver_setting_.get());
  solver_setting_->alpha = 1.6;
  solver_setting_->eps_abs = 1e-7;
  solver_setting_->eps_rel = 1e-7;
  solver_setting_->verbose = false;

  InitializeFactors();
}

void ReferenceLineQpSmoother::InitializeFactors() {
  // Norms and lengths of each baseline segments
  std::vector<Vec2d> t01s;

  x01_lengths_.reserve(center_line_size_ - 1);
  t01s.reserve(center_line_size_ - 1);

  for (int i = 0; i < center_line_size_ - 1; ++i) {
    const Vec2d x01 = input_.base_poses[i + 1] - input_.base_poses[i];
    const double x01_length = x01.norm();
    QCHECK_GE(x01_length, kEpsilon);
    const Vec2d t01 = x01.Perp() / x01_length;

    x01_lengths_.push_back(x01_length);
    t01s.push_back(t01);
  }

  // Norms of each base pose.
  std::vector<Vec2d> n0s;
  n0s.reserve(center_line_size_);

  for (int i = 0; i < center_line_size_; ++i) {
    n0s.push_back(input_.base_tangents[i].Perp());
  }

  // Neighbor segments angle differences of each base.
  std::vector<double> ad1s;
  ad1s.reserve(center_line_size_ - 2);

  for (int i = 0; i < center_line_size_ - 2; ++i) {
    ad1s.push_back(NormalizeAngle(t01s[i + 1].Angle() - t01s[i].Angle()));
  }

  // delta factors.
  delta_01_factors_.resize(center_line_size_ - 1);
  for (int i = 0; i < center_line_size_ - 1; ++i) {
    // delta factor
    delta_01_factors_[i] << 0, -n0s[i].Dot(t01s[i]) / x01_lengths_[i],
        n0s[i + 1].Dot(t01s[i]) / x01_lengths_[i];
  }

  // length factors
  length_01_factors_.resize(center_line_size_ - 1);
  for (int i = 0; i < center_line_size_ - 1; ++i) {
    // length factor
    length_01_factors_[i] << x01_lengths_[i] * param_->base_length_gain(),
        -n0s[i].CrossProd(t01s[i]), n0s[i + 1].CrossProd(t01s[i]);
  }

  // kappa factors.
  kappa_1_factors_.resize(center_line_size_ - 2);
  for (int i = 0; i < center_line_size_ - 2; ++i) {
    const Vec2d x02 = input_.base_poses[i + 2] - input_.base_poses[i];
    const double x02_length = x02.norm();
    QCHECK_GT(x02_length, kEpsilon);
    // kappa factor
    const double b02_inv = 2. / x02_length;
    double kappa_1_0 = -delta_01_factors_[i][1] * b02_inv;
    const double kappa_1_1 =
        (delta_01_factors_[i + 1][1] - delta_01_factors_[i][2]) * b02_inv;
    double kappa_1_2 = delta_01_factors_[i + 1][2] * b02_inv;
    const double kappa_1_constant =
        ad1s[i] * b02_inv * param_->base_kappa_gain();

    // When l0, l2 changes the arc length may get affected.
    const double x02_inv_2 = 0.25 * b02_inv * b02_inv;
    kappa_1_0 += x02.Dot(n0s[i]) * x02_inv_2 * kappa_1_constant;
    kappa_1_2 -= x02.Dot(n0s[i + 2]) * x02_inv_2 * kappa_1_constant;

    kappa_1_factors_[i] << kappa_1_constant, kappa_1_0, kappa_1_1, kappa_1_2;
  }

  // lambda factors.
  lambda_12_factors_.resize(center_line_size_ - 3);
  for (int i = 0; i < center_line_size_ - 3; ++i) {
    const double x12_inv = 1. / x01_lengths_[i + 1];
    const double lambda_12_0 = -kappa_1_factors_[i][1] * x12_inv;
    const double lambda_12_1 =
        (kappa_1_factors_[i + 1][1] - kappa_1_factors_[i][2]) * x12_inv;
    const double lambda_12_2 =
        (kappa_1_factors_[i + 1][2] - kappa_1_factors_[i][3]) * x12_inv;
    const double lambda_12_3 = kappa_1_factors_[i + 1][3] * x12_inv;
    const double lambda_12_constant =
        (kappa_1_factors_[i + 1][0] - kappa_1_factors_[i][0]) * x12_inv;

    // May also need to consider the changes upon arc length.
    lambda_12_factors_[i] << lambda_12_constant, lambda_12_0, lambda_12_1,
        lambda_12_2, lambda_12_3;
  }
}

namespace {

// Supportive function for class ReferenceLineQpSmoother.
// For each variable (for e.g. kappa_1 = kappa_factor * [1,l0,l1,l2]'),
// its square cost can be written as:
// weight * kappa_1' * kappa_1
// = weight * [1,l0,l1,l2] * kappa_factor' * kappa_factor * [1,l0,l1,l2]'
// = 0.5 * [l0,l1,l2] * local_A * [l0,l1,l2]' + local_b * [l0,l1,l2]'.
// The following function simply gets local_A from the factor of the variable.
template <int LDim>
Eigen::Matrix<double, LDim, LDim> GetSquareCostLocalAFromFactor(
    const Eigen::Matrix<double, 1, LDim + 1>& factor, double weight) {
  static_assert(LDim > 1, "");
  const Eigen::Matrix<double, 1, LDim> coeff = factor.block(0, 1, 1, LDim);

  return 2 * weight * coeff.transpose() * coeff;
}

// Supportive function for class ReferenceLineQpSmoother.
// For each variable (for e.g. kappa_1 = kappa_factor * [1,l0,l1,l2]'),
// its square cost can be written as:
// weight * kappa_1' * kappa_1
// = weight * [1,l0,l1,l2] * kappa_factor' * kappa_factor * [1,l0,l1,l2]'
// = 0.5 * [l0,l1,l2] * local_A * [l0,l1,l2]' + local_b * [l0,l1,l2]'.
// The following function simply gets local_b from the factor of the variable.
template <int LDim>
Eigen::Matrix<double, LDim, 1> GetSquareCostLocalBFromFactor(
    const Eigen::Matrix<double, 1, LDim + 1>& factor, double weight) {
  static_assert(LDim > 1, "");
  const Eigen::Matrix<double, 1, LDim> coeff = factor.block(0, 1, 1, LDim);

  return 2 * weight * factor(0) * coeff.transpose();
}

}  // namespace

void ReferenceLineQpSmoother::AddDeltaCost(FrenetFrameQpAgent* agent) const {
  QCHECK_NOTNULL(agent);
  for (int i = optimization_begin_ - 1; i < optimization_end_; ++i) {
    const double sample_length = x01_lengths_[i];
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d local_A = GetSquareCostLocalAFromFactor<2>(
        delta_01_factors_[i], param_->delta_weight() * sample_length);
    const ColVec2d local_b = GetSquareCostLocalBFromFactor<2>(
        delta_01_factors_[i], param_->delta_weight() * sample_length);
    agent->AddLocalCost(local_A, local_b, i - optimization_begin_);
  }
}

void ReferenceLineQpSmoother::AddKappaCost(FrenetFrameQpAgent* agent) const {
  QCHECK_NOTNULL(agent);
  for (int i = optimization_begin_ - 2; i < optimization_end_; ++i) {
    const double sample_length = 0.5 * (x01_lengths_[i + 1] + x01_lengths_[i]);
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat3d local_A = GetSquareCostLocalAFromFactor<3>(
        kappa_1_factors_[i], param_->kappa_weight() * sample_length);
    const ColVec3d local_b = GetSquareCostLocalBFromFactor<3>(
        kappa_1_factors_[i], param_->kappa_weight() * sample_length);
    agent->AddLocalCost(local_A, local_b, i - optimization_begin_);
  }
}

void ReferenceLineQpSmoother::AddLambdaCost(FrenetFrameQpAgent* agent) const {
  QCHECK_NOTNULL(agent);
  for (int i = optimization_begin_ - 3; i < optimization_end_; ++i) {
    const double sample_length = x01_lengths_[i + 1];
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat4d local_A = GetSquareCostLocalAFromFactor<4>(
        lambda_12_factors_[i], param_->lambda_weight() * sample_length);
    const ColVec4d local_b = GetSquareCostLocalBFromFactor<4>(
        lambda_12_factors_[i], param_->lambda_weight() * sample_length);
    agent->AddLocalCost(local_A, local_b, i - optimization_begin_);
  }
}

void ReferenceLineQpSmoother::AddLengthCost(FrenetFrameQpAgent* agent) const {
  QCHECK_NOTNULL(agent);
  for (int i = optimization_begin_ - 1; i < optimization_end_; ++i) {
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d local_A = GetSquareCostLocalAFromFactor<2>(
        length_01_factors_[i], param_->length_weight());
    const ColVec2d local_b = GetSquareCostLocalBFromFactor<2>(
        length_01_factors_[i], param_->length_weight());
    agent->AddLocalCost(local_A, local_b, i - optimization_begin_);
  }
}

void ReferenceLineQpSmoother::AddLateralOffsetConstraint(
    FrenetFrameQpAgent* agent) const {
  QCHECK_NOTNULL(agent);
  for (int i = optimization_begin_; i < optimization_end_; ++i) {
    // NOLINTNEXTLINE(readability-identifier-naming)
    ColVec2d local_H;
    local_H << 1, -1;
    ColVec2d local_h;
    local_h << input_.l_lower_bound[i - optimization_begin_],
        -input_.l_upper_bound[i - optimization_begin_];
    agent->AddLocalConstraint(local_H, local_h, i - optimization_begin_);
  }
}

double ReferenceLineQpSmoother::GetFixedLateralOffset(
    int optimization_id) const {
  int base_id = optimization_id + kNumPreOptimizationPoints;
  QCHECK_GE(base_id, 0);
  QCHECK_LT(base_id, center_line_size_);
  if (base_id < optimization_begin_) {
    return input_.pre_fixed_lateral_offsets[base_id];
  } else if (base_id >= optimization_end_) {
    return input_.post_fixed_lateral_offsets[base_id - optimization_end_];
  } else {
    QLOG(FATAL) << "ReferenceLineQpSmoother: FrenetFrameQpAgent is querrying "
                   "optimizable range, this should never be triggered."
                << " base_id:" << base_id
                << " optimization_begin_:" << optimization_begin_
                << " optimization_end_:" << optimization_end_;
    return 0.0;
  }
}

std::optional<std::vector<double>> ReferenceLineQpSmoother::Solve() const {
  // Create qp agent for frenet frame.
  const std::function<double(int)> get_fixed_lateral_offset =
      std::bind(&ReferenceLineQpSmoother::GetFixedLateralOffset, this,
                std::placeholders::_1);
  FrenetFrameQpAgent agent(n_dim_, &get_fixed_lateral_offset);

  // Add delta cost. w * (delta01 ** 2) * |x01|
  AddDeltaCost(&agent);

  // Add kappa cost. w * (kappa1 ** 2) * 0.5 * (|x01| + |x12|)
  AddKappaCost(&agent);

  // Add lambda cost. w  * (lambda ** 2) * |x12|
  AddLambdaCost(&agent);

  // lateral offset boundaries.
  AddLateralOffsetConstraint(&agent);

  // length cost. w * (length01 ** 2).
  AddLengthCost(&agent);

  // Build and run solver.
  qcraft::OsqpSolver solver = agent.BuildOsqpSolver();
  const auto status = solver.Solve(*solver_setting_);
  if (!status.ok()) {
    return std::nullopt;
  }

  // Output result.
  const VecXd& solution = solver.x();
  QCHECK_EQ(solution.rows(), n_dim_);
  std::vector<double> output;
  output.reserve(n_dim_);
  for (int i = 0; i < n_dim_; ++i) {
    output.push_back(solution(i, 0));
  }
  return output;
}

}  // namespace qcraft::planner
