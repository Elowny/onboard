#include "onboard/control/lateral_postprocess/mrac_control.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "Eigen/Cholesky"
#include "boost/move/utility_core.hpp"
#include "glog/logging.h"

#include "onboard/control/control_defs.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/filters/digital_filter_coefficients.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace control {
MracControl::MracControl(const MracConfProto& config, double steer_delay) {
  config_ = config;
  QCHECK(config_.has_state_weight() && config_.has_input_weight() &&
         config_.has_ke_weight() && config_.has_speed_limit() &&
         config_.has_kappa_threshold() && config_.has_latacc_threshold())
      << "Config proto is empty!";

  InitConfigMatrix();
  QCHECK(IsLyapunovStability()) << "System is not lyapunov stability!";
  Reset();

  // Filter for kappa feedback control
  constexpr double kFeedbackCutoffFreq = 3.0;  // (Hz)
  std::vector<double> denominators, numerators;
  LpfCoefficients(kControlInterval, kFeedbackCutoffFreq, &denominators,
                  &numerators);
  kappa_feedback_filter_ =
      std::make_unique<DigitalFilter>(denominators, numerators);

  // Consider steer delay.
  num_steer_delay_ = FloorToInt(steer_delay * kControlFrequency) + 1;
  kappa_target_cache_.resize(num_steer_delay_);
}

double MracControl::Compute(const MracInput& mrac_input,
                            MracDebugProto* mrac_debug) {
  if (!mrac_input.is_automode) {
    Reset();
    return mrac_input.kappa_target;
  }

  CHECK_NOTNULL(mrac_debug);

  // Ignore slow speed av kappa, av_kappa = 0.0
  double av_kappa = mrac_input.av_kappa;
  if (std::fabs(mrac_input.speed) < 0.01) av_kappa = mrac_input.kappa_target;

  // Mrac compute : kappa_target_cache
  if (is_first_run_) {
    is_first_run_ = false;
    kappa_target_cache_.assign(num_steer_delay_, mrac_input.kappa_target);
  }
  kappa_target_cache_.push_back(mrac_input.kappa_target);
  const double kappa_target_delay = kappa_target_cache_.front();

  // Mrac compute : kappa_forward + kappa_feedback
  double kappa_forward = mrac_input.kappa_target;
  double kappa_feedback = 0.0;
  double kappa_error = 0.0;
  if (std::fabs(mrac_input.speed) > config_.speed_limit()) {
    kappa_error =
        ComputeErrorMatrix(kappa_target_delay, av_kappa, mrac_input.speed);
    if (std::fabs(av_kappa) >= config_.kappa_threshold()) {
      ComputeSlopeMatrix(mrac_input.kappa_target);
      kappa_forward = kr_gain_matrix_(0, 0) * mrac_input.kappa_target +
                      kx_gain_matrix_(0, 0) * state_ref_matrix_(0, 0);
    }
    ComputeBiasMatrix(kappa_target_delay, mrac_input.speed);
    kappa_feedback = kappa_feedback_filter_->Filter(ke_gain_matrix_(0, 0) *
                                                    error_matrix_(0, 0));
  }
  double kappa_output = kappa_forward + kappa_feedback;
  kappa_output =
      std::clamp(kappa_output, mrac_input.kappa_lower, mrac_input.kappa_upper);

  mrac_debug_.set_state_gain(kx_gain_matrix_(0, 0));
  mrac_debug_.set_input_gain(kr_gain_matrix_(0, 0));
  mrac_debug_.set_kappa_input(mrac_input.kappa_target);
  mrac_debug_.set_av_kappa(av_kappa);
  mrac_debug_.set_kappa_target_delay(kappa_target_delay);
  mrac_debug_.set_kappa_error(kappa_error);
  mrac_debug_.set_kappa_forward(kappa_forward);
  mrac_debug_.set_kappa_feedback(kappa_feedback);
  mrac_debug_.set_kappa_cmd(kappa_output);
  *mrac_debug = mrac_debug_;
  return kappa_output;
}

void MracControl::Reset() {
  state_cur_matrix_.setZero(state_num_, 1);
  state_ref_matrix_.setZero(state_num_, 1);
  error_matrix_.setZero(state_num_, 1);

  kx_gain_matrix_.setZero(1, state_num_);
  kr_gain_matrix_.setOnes(1, input_num_);
  kappa_target_cache_.clear();
  is_first_run_ = true;
}

void MracControl::InitConfigMatrix() {
  constexpr double kCutoffFrequency = 20.0;  // (rad)
  constexpr double kDampingRatio = 0.707;    // (-)

  // StateMatrix
  a_matrix_ = Eigen::MatrixXd::Zero(state_num_, state_num_);
  a_matrix_(0, 1) = 1.0;
  a_matrix_(1, 0) = -Sqr(kCutoffFrequency);
  a_matrix_(1, 1) = -2 * kDampingRatio * kCutoffFrequency;

  // ReferenceMatrix
  am_matrix_ = Eigen::MatrixXd::Identity(state_num_, state_num_) +
               a_matrix_ * kControlInterval;
  bm_matrix_ = Eigen::MatrixXd::Zero(state_num_, input_num_);
  bm_matrix_(1, 0) = Sqr(kCutoffFrequency) * kControlInterval;

  // GainMatrix
  gamma_x_matrix_ = config_.state_weight() *
                    Eigen::MatrixXd::Identity(input_num_, input_num_);
  gamma_r_matrix_ = config_.input_weight() *
                    Eigen::MatrixXd::Identity(input_num_, input_num_);
  ke_gain_matrix_ = Eigen::MatrixXd::Zero(input_num_, state_num_);

  state_ref_matrix_ = Eigen::MatrixXd::Zero(state_num_, 1);
  state_cur_matrix_ = Eigen::MatrixXd::Zero(state_num_, 1);
  error_matrix_ = Eigen::MatrixXd::Zero(state_num_, 1);

  p_matrix_ = Eigen::MatrixXd::Zero(state_num_, state_num_);
  q_matrix_ = Eigen::MatrixXd::Zero(state_num_, state_num_);

  p_matrix_(0, 0) = 435.0;
  p_matrix_(0, 1) = 1.0;
  p_matrix_(1, 0) = 1.0;
  p_matrix_(1, 1) = 1.0;

  q_matrix_ = -p_matrix_ * a_matrix_ - a_matrix_.transpose() * p_matrix_;
}

bool MracControl::IsLyapunovStability() {
  // check matrix Q is or not symmetric and positive mratix
  Eigen::LLT<Eigen::MatrixXd> llt_matrix_q(q_matrix_);
  return (q_matrix_.isApprox(q_matrix_.transpose()) &&
          llt_matrix_q.info() != Eigen::NumericalIssue);
}

double MracControl::ComputeErrorMatrix(double kappa_target_delay,
                                       double av_kappa, double speed) {
  constexpr double kMaxKappaError = 0.02;  // (1/m)
  state_ref_matrix_ =
      am_matrix_ * state_ref_matrix_ + bm_matrix_ * kappa_target_delay;
  state_cur_matrix_(0, 0) = av_kappa;
  state_cur_matrix_(1, 0) = 0.0;
  error_matrix_ = state_ref_matrix_ - state_cur_matrix_;

  const PiecewiseLinearFunction<double, double>
      k_speed_ratio_plf(/* speed(m/s) = */
                        {0.0, 5.0, 10.0, 20.0, 30.0},
                        /* ratio(-) =  */ {1.0, 1.0, 0.6, 0.15, 0.05});
  const double max_error =
      kMaxKappaError * k_speed_ratio_plf.Evaluate(std::fabs(speed));
  const double kappa_error =
      std::clamp(error_matrix_(0, 0), -max_error, max_error);
  error_matrix_(0, 0) = kappa_error;
  error_matrix_(1, 0) = 0.0;
  return kappa_error;
}

void MracControl::ComputeSlopeMatrix(double kappa_target) {
  constexpr double kMaxDeltaGain = 0.005;  // ((1/m)/s)
  constexpr double kMinKr = -0.9;          // (-)
  constexpr double kMaxKr = 1.1;           // (-)
  constexpr double kMinKx = -0.1;          // (-)
  constexpr double kMaxKx = 0.1;           // (-)
  Eigen::MatrixXd delta_x_matrix(1, state_num_), delta_r_matrix(1, input_num_);

  delta_x_matrix = kControlInterval * gamma_x_matrix_ * bm_matrix_.transpose() *
                   p_matrix_ * error_matrix_ * state_cur_matrix_.transpose();
  delta_r_matrix = kControlInterval * gamma_r_matrix_ * bm_matrix_.transpose() *
                   p_matrix_ * error_matrix_ * kappa_target;
  const double max_delta_kx_step = kMaxDeltaGain * kControlInterval;
  const double max_delta_kr_step = kMaxDeltaGain * kControlInterval;
  delta_x_matrix(0, 0) =
      std::clamp(delta_x_matrix(0, 0), -max_delta_kx_step, max_delta_kx_step);
  delta_r_matrix(0, 0) =
      std::clamp(delta_r_matrix(0, 0), -max_delta_kr_step, max_delta_kr_step);

  kx_gain_matrix_ += delta_x_matrix;
  kx_gain_matrix_(0, 0) = std::clamp(kx_gain_matrix_(0, 0), kMinKx, kMaxKx);

  kr_gain_matrix_ += delta_r_matrix;
  kr_gain_matrix_(0, 0) = std::clamp(kr_gain_matrix_(0, 0), kMinKr, kMaxKr);

  mrac_debug_.set_state_rate_gain(delta_x_matrix(0, 0) * kControlFrequency);
  mrac_debug_.set_input_delta_gain(delta_r_matrix(0, 0) * kControlFrequency);
}

void MracControl::ComputeBiasMatrix(double kappa_target_delay, double speed) {
  const PiecewiseLinearFunction<double, double>
      k_lat_acc_ratio_plf(/* lat_acc(m/s2) = */
                          {0.0, config_.latacc_threshold()},
                          /* ratio(-) =  */ {0.0, 1.0});
  const double lat_acc = std::fabs(kappa_target_delay * speed * speed);
  ke_gain_matrix_(0, 0) =
      k_lat_acc_ratio_plf.Evaluate(lat_acc) * config_.ke_weight();
}

}  // namespace control
}  // namespace qcraft
