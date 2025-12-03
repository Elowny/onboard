#include "onboard/control/controllers/lon_tob_mpc_controller.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/control/controllers/predict_vehicle_pose.h"
#include "onboard/control/math/mpc_solver.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace control {

namespace {

// Max and min mpc acceleration command to avoid integral saturation.
// The setting should be over all vehicles boundaries of max_acceleration_cmd
// and max_deceleration_cmd;
constexpr double kAccelCmdMax = 2.0;   // m/s2;
constexpr double kAccelCmdMin = -6.0;  // m/s2;

// TODO(shijun): make jerk limit plf with speed and add acceleration limit to
// control state.
void ConstraintSetup(Eigen::MatrixXd* t_control_input_constraint_enable,
                     std::vector<Eigen::VectorXd>* t_control_input_lower,
                     std::vector<Eigen::VectorXd>* t_control_input_upper) {
  *t_control_input_constraint_enable << 1.0;
  constexpr double kJerkUpperLimit = 3.0;   // m/s^3.
  constexpr double kJerkLowerLimit = -5.0;  // m/s^3.
  for (int i = 0; i < kTControlHorizon; ++i) {
    (*t_control_input_lower)[i](0) = kJerkLowerLimit;
    (*t_control_input_upper)[i](0) = kJerkUpperLimit;
  }
}

}  // namespace

using Matrix = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

// TODO(yangyu): remove the dependency of vehicle_drive_params_ after separating
// LonPostProcess from lon mpc controller.
LonTobMpcController::LonTobMpcController(
    const ControllerConf* control_conf,
    const VehicleDriveParamsProto* vehicle_drive_params)
    : control_conf_(QCHECK_NOTNULL(control_conf)),
      vehicle_drive_params_(QCHECK_NOTNULL(vehicle_drive_params)) {
  ts_ = control_conf_->lon_ts_pkmpc_controller_conf().ts();
  QCHECK_GT(ts_, 0.0) << "[LonTobMpcController] Invalid mpc time step " << ts_;

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
      control_conf_->lon_ts_pkmpc_controller_conf().matrix_q_size();
  const int t_matrix_r_size =
      control_conf_->lon_ts_pkmpc_controller_conf().matrix_r_size();
  for (int i = 0; i < kTControlStateNum; ++i) {
    t_matrix_q_(i, i) =
        control_conf_->lon_ts_pkmpc_controller_conf().matrix_q(i);
    t_matrix_n_(i, i) =
        control_conf_->lon_ts_pkmpc_controller_conf().matrix_n(i);
  }
  for (int i = 0; i < t_matrix_r_size; ++i) {
    t_matrix_r_(i, i) =
        control_conf_->lon_ts_pkmpc_controller_conf().matrix_r(i);
  }
  QCHECK(kTControlStateNum >= t_matrix_q_size)
      << "Lon mpc controller error: matrix_q size in parameter file is larger "
         "than control state size";

  LoadGainScheduler(
      control_conf_->lon_ts_pkmpc_controller_conf().control_gain_scheduler(),
      &t_control_gain_scheduler_plf_);
  VLOG(1) << "Lon mpc control gain scheduler loaded.";
}

void LonTobMpcController::LoadGainScheduler(
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

absl::Status LonTobMpcController::ComputeControlCommand(
    const TrajectoryInterface& trajectory_interface,
    const VehPose& predicted_pose_after_delay, ControlCommand* cmd,
    ControllerDebugProto* controller_debug_proto,
    LonControllerOutputProto* lon_controller_output) {
  QCHECK(cmd != nullptr);
  QCHECK(controller_debug_proto != nullptr);
  QCHECK(lon_controller_output != nullptr);
  SCOPED_QTRACE("LonTobMpcController");

  auto* debug = cmd->mutable_debug()->mutable_simple_mpc_debug();

  FindControlReference(
      predicted_pose_after_delay.x, predicted_pose_after_delay.y,
      predicted_pose_after_delay.timestamp, trajectory_interface, ts_,
      &t_control_state_reference_, &t_control_input_reference_, debug);

  UpdateInitialState(predicted_pose_after_delay, *debug, &t_initial_state_);

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

  ConstraintSetup(&t_control_input_constraint_enable_, &t_control_input_lower_,
                  &t_control_input_upper_);

  // Longitudinal third-order bicycle kinematic mpc.
  controller_debug_proto->set_active_lon_controller("lon_tob_mpc");

  double acc_cmd;
  const auto mpc_result = SolveLinearMPC(
      t_matrix_ad_t_, std::vector<Matrix>(kTControlHorizon, t_matrix_bd_),
      std::vector<VecXd>(kTControlHorizon, t_matrix_cd_), t_matrix_q_updated_,
      t_matrix_r_updated_, t_matrix_n_updated_,
      t_control_input_constraint_enable_, t_control_input_lower_,
      t_control_input_upper_, t_control_state_constraint_enable_,
      t_control_state_lower_, t_control_state_upper_, t_initial_state_,
      t_control_state_reference_, t_control_input_reference_);
  if (!mpc_result.ok()) {
    QEVENT("zhichao", "mpc_solver_fails", [&](QEvent* qevent) {
      qevent->AddField("controller_name", "lon_tob_mpc");
    });
    return absl::InternalError(
        absl::StrCat("MPC solver failed, detailed reason: ",
                     mpc_result.status().ToString()));
  } else {
    const double jerk_cmd = mpc_result.value()[0](0);
    acc_cmd = std::clamp(previous_acc_cmd_ + jerk_cmd * kControlInterval,
                         kAccelCmdMin, kAccelCmdMax);
    cmd->set_acceleration(acc_cmd);
    previous_acc_cmd_ = acc_cmd;
    debug->set_jerk_cmd(jerk_cmd);
    for (const auto& res : mpc_result.value()) {
      controller_debug_proto->mutable_mpc_debug_proto()
          ->add_t_control_mpc_result(res(0));
    }
    double acc_cmd_in_mpc_horizon = acc_cmd;
    lon_controller_output->add_t_control_acc_vec(acc_cmd_in_mpc_horizon);
    for (int i = 1; i < mpc_result.value().size(); ++i) {
      acc_cmd_in_mpc_horizon += mpc_result.value()[i](0) * ts_;
      lon_controller_output->add_t_control_acc_vec(acc_cmd_in_mpc_horizon);
    }
  }

  UpdateMpcDebugInfo(t_control_state_reference_, t_control_input_reference_,
                     t_initial_state_, t_matrix_ad_, t_matrix_bd_, t_matrix_cd_,
                     t_matrix_q_updated_, t_matrix_r_updated_,
                     t_matrix_n_updated_, *mpc_result, trajectory_interface,
                     controller_debug_proto);
  cmd->set_aeb_triggered(trajectory_interface.aeb_triggered());

  return absl::OkStatus();
}

void LonTobMpcController::Reset(const VehicleStateProto& vehicle_state) {
  previous_acc_cmd_ = vehicle_state.linear_acceleration();
}

void LonTobMpcController::FindControlReference(
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
      (*t_control_state_reference)[i - 1](2) =
          ref_traj_point.a() * FLAGS_reference_acc_slack_factor;
    }
    if (i < kTControlHorizon) {
      // MPC input reference step is from 0 to N-1.
      (*t_control_input_reference)[i](0) =
          ref_traj_point.j() * FLAGS_reference_jerk_slack_factor;
    }

    target_relative_time += mpc_period;
  }
}

void LonTobMpcController::UpdateInitialState(
    const VehPose& predicted_pose_after_delay, const SimpleMPCDebug& debug,
    Eigen::VectorXd* t_initial_state) {
  (*t_initial_state)(0) = debug.av_s_projected_on_traj();
  (*t_initial_state)(1) = predicted_pose_after_delay.v;
  (*t_initial_state)(2) = predicted_pose_after_delay.acc;
}

void LonTobMpcController::UpdateTransitionMatrix(
    double mpc_period, Eigen::MatrixXd* t_matrix_a,
    Eigen::MatrixXd* t_matrix_ad, Eigen::MatrixXd* t_matrix_b,
    Eigen::MatrixXd* t_matrix_bd, std::vector<Eigen::MatrixXd>* t_matrix_ad_t) {
  *t_matrix_a << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  *t_matrix_b << 0.0, 0.0, 1.0;

  const Eigen::MatrixXd t_matrix_i =
      Eigen::MatrixXd::Identity(t_matrix_a->rows(), t_matrix_a->rows());

  *t_matrix_ad = t_matrix_i + (*t_matrix_a) * mpc_period;
  *t_matrix_bd = (*t_matrix_b) * mpc_period;

  for (int i = 0; i < kTControlHorizon; ++i) {
    (*t_matrix_ad_t)[i] = *t_matrix_ad;
  }
}

void LonTobMpcController::UpdateMpcDebugInfo(
    const std::vector<Eigen::VectorXd>& state_reference,
    const std::vector<Eigen::VectorXd>& input_reference,
    const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& matrix_ad,
    const Eigen::MatrixXd& matrix_bd, const Eigen::MatrixXd& matrix_cd,
    const std::vector<Eigen::MatrixXd>& matrix_q_updated,
    const Eigen::MatrixXd& matrix_r_updated,
    const Eigen::MatrixXd& matrix_n_updated,
    const std::vector<Eigen::VectorXd>& control_output,
    const TrajectoryInterface& trajectory_interface,
    ControllerDebugProto* controller_debug_proto) const {
  VecXd current_control_state = initial_state;
  VecXd next_control_state;
  double s_cost = 0.0;
  double v_cost = 0.0;
  double a_cost = 0.0;
  double jerk_cost = 0.0;
  auto* mpc_debug = controller_debug_proto->mutable_mpc_debug_proto();
  for (int i = 0; i < kTControlHorizon; ++i) {
    // Based on MPC output, update current_control_state and next_control_state.
    next_control_state = matrix_ad * current_control_state +
                         matrix_bd * control_output[i] + matrix_cd;
    current_control_state = next_control_state;

    // Collect mpc reference and predicted points for visualization.
    const auto predicted_pt =
        trajectory_interface.QueryTrajPointBasedOnPathS(next_control_state(0));
    auto* predicted_pt_proto = mpc_debug->add_lon_mpc_predicted_pt();
    predicted_pt_proto->set_x(predicted_pt.path_point().x());
    predicted_pt_proto->set_y(predicted_pt.path_point().y());

    const auto reference_pt =
        trajectory_interface.QueryTrajPointBasedOnPathS(state_reference[i](0));
    auto* reference_pt_proto = mpc_debug->add_lon_mpc_reference_pt();
    reference_pt_proto->set_x(reference_pt.path_point().x());
    reference_pt_proto->set_y(reference_pt.path_point().y());

    // Collect longitudinal mpc costs info.
    double s_cost_per_cycle =
        matrix_q_updated[i](0, 0) *
        Sqr(state_reference[i](0) - next_control_state(0));
    double v_cost_per_cycle =
        matrix_q_updated[i](1, 1) *
        Sqr(state_reference[i](1) - next_control_state(1));
    double a_cost_per_cycle =
        matrix_q_updated[i](2, 2) *
        Sqr(state_reference[i](2) - next_control_state(2));
    if (i == kTControlHorizon - 1) {
      s_cost_per_cycle *= Sqr(matrix_n_updated(0, 0));
      v_cost_per_cycle *= Sqr(matrix_n_updated(1, 1));
      a_cost_per_cycle *= Sqr(matrix_n_updated(2, 2));
    }
    s_cost += s_cost_per_cycle;
    v_cost += v_cost_per_cycle;
    a_cost += a_cost_per_cycle;
    jerk_cost += matrix_r_updated(0, 0) *
                 Sqr(control_output[i](0) - input_reference[i](0));
  }

  for (const auto& [name, cost] : {std::pair{"s", s_cost},
                                   {"v", v_cost},
                                   {"a", a_cost},
                                   {"jerk", jerk_cost}}) {
    auto* costs = controller_debug_proto->mutable_mpc_debug_proto()
                      ->mutable_lon_mpc_costs()
                      ->add_costs();
    costs->set_name(name);
    costs->set_cost(cost);
  }
}

}  // namespace control
}  // namespace qcraft
