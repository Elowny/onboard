#include "onboard/control/controllers/lon_mpc_controller.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <ostream>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/predict_vehicle_pose.h"
#include "onboard/control/math/mpc_solver.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace control {

using Matrix = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

// TODO(yangyu): remove the dependency of vehicle_drive_params_ after separating
// LonPostProcess from lon mpc controller.
LonMpcController::LonMpcController(
    const ControllerConf* control_conf,
    const VehicleDriveParamsProto* vehicle_drive_params)
    : control_conf_(QCHECK_NOTNULL(control_conf)),
      vehicle_drive_params_(QCHECK_NOTNULL(vehicle_drive_params)) {
  ts_ = control_conf_->ts_pkmpc_controller_conf().ts();
  QCHECK_GT(ts_, 0.0) << "[LonMpcController] Invalid mpc time step " << ts_;

  // t-control matrix initialization
  t_initial_state_ = VecXd::Zero(kTControlStateNum);

  t_matrix_a_ = Matrix::Zero(kTControlStateNum, kTControlStateNum);
  t_matrix_ad_ = t_matrix_a_;
  t_matrix_ad_t_.assign(kTControlHorizon, t_matrix_ad_);
  t_matrix_b_ = Matrix::Zero(kTControlStateNum, 1);
  t_matrix_bd_ = t_matrix_b_;
  t_matrix_c_ = Matrix::Zero(kTControlStateNum, 1);
  t_matrix_cd_ = t_matrix_c_;

  t_matrix_q_ = Matrix::Zero(kTControlStateNum, kTControlStateNum);
  t_matrix_r_ = Matrix::Zero(1, 1);
  t_matrix_r_updated_ = t_matrix_r_;
  t_matrix_n_ = Matrix::Zero(kTControlStateNum, kTControlStateNum);
  t_matrix_n_updated_ = t_matrix_n_;

  t_control_state_reference_.assign(kTControlHorizon,
                                    VecXd::Zero(kTControlStateNum));
  t_control_input_reference_.assign(kTControlHorizon,
                                    VecXd::Zero(kTControlInputNum));
  t_control_input_lower_.assign(kTControlHorizon,
                                VecXd::Zero(kTControlInputNum));
  t_control_input_upper_.assign(kTControlHorizon,
                                VecXd::Zero(kTControlInputNum));
  t_control_state_lower_.assign(kTControlHorizon,
                                VecXd::Zero(kTControlStateNum));
  t_control_state_upper_.assign(kTControlHorizon,
                                VecXd::Zero(kTControlStateNum));

  t_control_input_constraint_enable_ =
      Matrix::Zero(kTControlInputNum, kTControlInputNum);
  t_control_state_constraint_enable_ =
      Matrix::Zero(kTControlStateNum, kTControlStateNum);

  // Load configuration parameter
  const int t_matrix_q_size =
      control_conf_->ts_pkmpc_controller_conf().t_matrix_q_size();
  const int t_matrix_r_size =
      control_conf_->ts_pkmpc_controller_conf().t_matrix_r_size();
  for (int i = 0; i < kTControlStateNum; ++i) {
    t_matrix_q_(i, i) = control_conf_->ts_pkmpc_controller_conf().t_matrix_q(i);
    t_matrix_n_(i, i) = control_conf_->ts_pkmpc_controller_conf().t_matrix_n(i);
  }
  for (int i = 0; i < t_matrix_r_size; ++i) {
    t_matrix_r_(i, i) = control_conf_->ts_pkmpc_controller_conf().t_matrix_r(i);
  }
  QCHECK(kTControlStateNum >= t_matrix_q_size)
      << "Lon mpc controller error: matrix_q size in parameter file is larger "
         "than control state size";

  LoadGainScheduler(
      control_conf_->ts_pkmpc_controller_conf().t_control_gain_scheduler(),
      &t_control_gain_scheduler_plf_);
  VLOG(1) << "Lon mpc control gain scheduler loaded.";
}

void LonMpcController::LoadGainScheduler(
    const GainScheduler& gain_scheduler,
    std::optional<PiecewiseLinearFunction<double>>* gain_scheduler_plf) {
  // TODO(shijun): replace it with PiecewiseLinearFunctionFromProto.
  std::vector<double> gain_scheduler_speed_vec;
  std::vector<double> gain_scheduler_ratio_vec;
  gain_scheduler_speed_vec.reserve(gain_scheduler.scheduler().size());
  gain_scheduler_ratio_vec.reserve(gain_scheduler.scheduler().size());
  for (const auto& scheduler : gain_scheduler.scheduler()) {
    gain_scheduler_speed_vec.push_back(scheduler.speed());
    gain_scheduler_ratio_vec.push_back(scheduler.ratio());
  }
  gain_scheduler_plf->emplace(gain_scheduler_speed_vec,
                              gain_scheduler_ratio_vec);
}

absl::Status LonMpcController::ComputeControlCommand(
    const TrajectoryInterface& trajectory_interface,
    const VehPose& predicted_pose_after_delay, ControlCommand* cmd,
    ControllerDebugProto* controller_debug_proto,
    LonControllerOutputProto* lon_controller_output) {
  QCHECK(cmd != nullptr);
  QCHECK(controller_debug_proto != nullptr);
  QCHECK(lon_controller_output != nullptr);
  SCOPED_QTRACE("LonMpcController");

  auto* debug = cmd->mutable_debug()->mutable_simple_mpc_debug();

  FindControlReference(
      predicted_pose_after_delay.x, predicted_pose_after_delay.y,
      predicted_pose_after_delay.timestamp, trajectory_interface, ts_,
      &t_control_state_reference_, &t_control_input_reference_, debug);

  UpdateInitialState(predicted_pose_after_delay.v, *debug, &t_initial_state_);

  UpdateTransitionMatrix(ts_, &t_matrix_a_, &t_matrix_ad_, &t_matrix_b_,
                         &t_matrix_bd_, &t_matrix_ad_t_);

  if (control_conf_->enable_gain_scheduler() &&
      t_control_gain_scheduler_plf_.has_value()) {
    const double scheduler_gain = t_control_gain_scheduler_plf_->Evaluate(
        std::fabs(predicted_pose_after_delay.v));
    QCHECK_GT(scheduler_gain, 0.0) << "Lon mpc control loads negative gain "
                                      "schedular at the speed of "
                                   << predicted_pose_after_delay.v;
    t_matrix_q_updated_ = std::vector<Eigen::MatrixXd>(
        kTControlHorizon, t_matrix_q_ * scheduler_gain);
    t_matrix_r_updated_ = t_matrix_r_;
    t_matrix_n_updated_ = t_matrix_n_ * scheduler_gain;
  } else {
    t_matrix_q_updated_ =
        std::vector<Eigen::MatrixXd>(kTControlHorizon, t_matrix_q_);
    t_matrix_r_updated_ = t_matrix_r_;
    t_matrix_n_updated_ = t_matrix_n_;
  }

  ConstraintSetup(control_conf_->max_deceleration_cmd(),
                  control_conf_->max_acceleration_cmd(),
                  &t_control_input_constraint_enable_, &t_control_input_lower_,
                  &t_control_input_upper_);

  // Longitudinal second-order bicycle kinematic mpc.
  controller_debug_proto->set_active_lon_controller("lon_sob_mpc");

  double acceleration_cmd;
  std::vector<VecXd> mpc_result_final;
  const auto mpc_result = SolveLinearMPC(
      t_matrix_ad_t_, std::vector<Matrix>(kTControlHorizon, t_matrix_bd_),
      std::vector<VecXd>(kTControlHorizon, t_matrix_cd_), t_matrix_q_updated_,
      t_matrix_r_updated_, t_matrix_n_updated_,
      t_control_input_constraint_enable_, t_control_input_lower_,
      t_control_input_upper_, t_control_state_constraint_enable_,
      t_control_state_lower_, t_control_state_upper_, t_initial_state_,
      t_control_state_reference_, t_control_input_reference_);
  if (!mpc_result.ok()) {
    QEVENT("shijun", "mpc_solver_fails", [&](QEvent* qevent) {
      qevent->AddField("controller_name", "lon_sob_mpc");
    });
    return absl::InternalError(
        absl::StrCat("MPC solver failed, detailed reason: ",
                     mpc_result.status().ToString()));
  } else {
    const auto a_planner = t_control_input_reference_.front()(0);
    constexpr double kHardBrakeAccThreshold = -1.5;  // m/s^2.
    constexpr double kPlannerAccSlackFactor = 1.2;
    const double a_ref =
        std::min(kHardBrakeAccThreshold, kPlannerAccSlackFactor * a_planner);
    const PiecewiseLinearFunction<double, double> overshoot_cost_acc_ref_plf(
        std::vector<double>{1.1 * a_ref, a_ref}, std::vector<double>{10, 1});
    const double init_mpc_a = mpc_result.value().front()(0);
    // Fallback, the purpose is to decrease control overshoot when planner is
    // hard braking.
    if (init_mpc_a < a_ref) {
      controller_debug_proto->mutable_mpc_debug_proto()->set_is_fallback(true);
      auto t_matrix_r_wrt_overshoot_cost = t_matrix_r_updated_;
      t_matrix_r_wrt_overshoot_cost *= overshoot_cost_acc_ref_plf(init_mpc_a);
      const auto mpc_result_fallback = SolveLinearMPC(
          t_matrix_ad_t_, std::vector<Matrix>(kTControlHorizon, t_matrix_bd_),
          std::vector<VecXd>(kTControlHorizon, t_matrix_cd_),
          t_matrix_q_updated_, t_matrix_r_wrt_overshoot_cost,
          t_matrix_n_updated_, t_control_input_constraint_enable_,
          t_control_input_lower_, t_control_input_upper_,
          t_control_state_constraint_enable_, t_control_state_lower_,
          t_control_state_upper_, t_initial_state_, t_control_state_reference_,
          t_control_input_reference_);
      if (!mpc_result_fallback.ok()) {
        QEVENT("shijun", "fallback_mpc_solver_fails", [&](QEvent* qevent) {
          qevent->AddField("controller_name", "lon_sob_mpc");
        });
        return absl::InternalError(
            absl::StrCat("Fallback MPC solver failed, detailed reason: ",
                         mpc_result.status().ToString()));
      } else {
        mpc_result_final = mpc_result_fallback.value();
      }
    } else {
      mpc_result_final = mpc_result.value();
    }
  }
  acceleration_cmd = mpc_result_final.front()(0);
  cmd->set_acceleration(acceleration_cmd);
  for (const auto& acc : mpc_result_final) {
    controller_debug_proto->mutable_mpc_debug_proto()->add_t_control_mpc_result(
        acc(0));
    lon_controller_output->add_t_control_acc_vec(acc(0));
  }

  cmd->set_aeb_triggered(trajectory_interface.aeb_triggered());

  return absl::OkStatus();
}

void LonMpcController::Reset(const VehicleStateProto& vehicle_state) {
  previous_speed_cmd_ = vehicle_state.linear_velocity();
}

void LonMpcController::FindControlReference(
    double x, double y, const double vehicle_state_time,
    const TrajectoryInterface& trajectory_interface, double mpc_period,
    std::vector<Eigen::VectorXd>* t_control_state_reference,
    std::vector<Eigen::VectorXd>* t_control_input_reference,
    SimpleMPCDebug* debug) {
  double target_relative_time =
      vehicle_state_time - trajectory_interface.GetPlannerStartTime();

  // i = 0: collect information for s-control use.
  // i = 1 to N: calculate t-control state reference information.
  // i = 0 to (N-1): calculate t-control input reference information.
  for (int i = 0; i < kTControlHorizon + 1; ++i) {
    const ApolloTrajectoryPointProto ref_traj_point =
        trajectory_interface.QueryTrajPointByRelativeTime(target_relative_time);
    if (i == 0) {
      // Collect information for s-control use.
      const Vec2d vs_pos(x, y);
      const auto closest_traj_point =
          trajectory_interface.QueryNearestTrajPointByXY(vs_pos);
      debug->set_av_s_projected_on_traj(closest_traj_point.path_point().s());
      debug->set_speed_reference(ref_traj_point.v());
      debug->set_acceleration_reference(ref_traj_point.a());
      debug->set_jerk_reference(ref_traj_point.j());
    } else {
      // MPC state reference step is from 1 to N.
      (*t_control_state_reference)[i - 1](0) = ref_traj_point.path_point().s();
      (*t_control_state_reference)[i - 1](1) = ref_traj_point.v();
    }
    if (i < kTControlHorizon) {
      // MPC input reference step is from 0 to N-1.
      (*t_control_input_reference)[i](0) = ref_traj_point.a();
    }

    target_relative_time += mpc_period;
  }
}

void LonMpcController::UpdateInitialState(double av_speed,
                                          const SimpleMPCDebug& debug,
                                          Eigen::VectorXd* t_initial_state) {
  (*t_initial_state)(0) = debug.av_s_projected_on_traj();
  (*t_initial_state)(1) = av_speed;
}

void LonMpcController::UpdateTransitionMatrix(
    double mpc_period, Eigen::MatrixXd* t_matrix_a,
    Eigen::MatrixXd* t_matrix_ad, Eigen::MatrixXd* t_matrix_b,
    Eigen::MatrixXd* t_matrix_bd, std::vector<Eigen::MatrixXd>* t_matrix_ad_t) {
  *t_matrix_a << 0.0, 1.0, 0.0, 0.0;
  *t_matrix_b << 0.0, 1.0;

  const Eigen::MatrixXd t_matrix_i =
      Eigen::MatrixXd::Identity(t_matrix_a->rows(), t_matrix_a->rows());

  *t_matrix_ad = t_matrix_i + (*t_matrix_a) * mpc_period;
  *t_matrix_bd = (*t_matrix_b) * mpc_period;

  for (int i = 0; i < kTControlHorizon; ++i) {
    (*t_matrix_ad_t)[i] = *t_matrix_ad;
  }
}

void LonMpcController::ConstraintSetup(
    double max_deceleration, double max_acceleration,
    Eigen::MatrixXd* t_control_input_constraint_enable,
    std::vector<Eigen::VectorXd>* t_control_input_lower,
    std::vector<Eigen::VectorXd>* t_control_input_upper) {
  *t_control_input_constraint_enable << 1.0;
  for (int i = 0; i < kTControlHorizon; ++i) {
    (*t_control_input_lower)[i](0) = max_deceleration;
    (*t_control_input_upper)[i](0) = max_acceleration;
  }
}

}  // namespace control
}  // namespace qcraft
