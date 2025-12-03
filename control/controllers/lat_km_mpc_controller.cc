#include "onboard/control/controllers/lat_km_mpc_controller.h"

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
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/controller_common.h"
#include "onboard/control/controllers/model/state_space.h"
#include "onboard/control/controllers/model/tob_tv_kinematic_model.h"
#include "onboard/control/math/mpc_solver.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace control {

namespace {

Eigen::VectorXd SquareVecXd(const Eigen::VectorXd& vec) {
  Eigen::VectorXd result = Eigen::VectorXd::Zero(vec.size());
  for (int i = 0; i < vec.size(); ++i) {
    result[i] = Sqr(vec[i]);
  }

  return result;
}

MpcConstraint BuildTimeVaryingMpcConstraint(int state_size, int input_size,
                                            double kappa_rate_lower,
                                            double kappa_rate_upper) {
  MpcConstraint constraint;
  constraint.Init(state_size, input_size);
  constraint.input_enable(0) = 1.0;
  constraint.input_lower(0) = kappa_rate_lower;
  constraint.input_upper(0) = kappa_rate_upper;

  return constraint;
}

std::vector<VecXd> FindStateReference(
    const Vec2d& xy, double yaw,
    const TrajectoryInterface& trajectory_interface,
    const std::vector<double>& lon_control_s_vec, SimpleMPCDebug* debug,
    ControllerDebugProto* controller_debug_proto) {
  // Step 1: find the closest trajectory point from trajectory and past
  // points.
  const auto closest_trajectory_point =
      trajectory_interface.QueryNearestTrajPointByXY(xy);
  controller_debug_proto->set_lat_ref_relative_time(
      closest_trajectory_point.relative_time());
  const auto& closest_path_point = closest_trajectory_point.path_point();
  // Render the closest_path_point in vantage
  Vec2dProto* closest_path_point_for_render =
      controller_debug_proto->mutable_mpc_debug_proto()
          ->add_mpc_reference_traj_point();
  closest_path_point_for_render->set_x(closest_path_point.x());
  closest_path_point_for_render->set_y(closest_path_point.y());
  const double current_s = closest_path_point.s();

  std::vector<VecXd> state_ref_vector;
  state_ref_vector.reserve(kSControlHorizon);

  for (int i = 0; i < kSControlHorizon; ++i) {
    const double t_control_s = current_s + lon_control_s_vec[i];
    const auto reference_traj_point =
        trajectory_interface.QueryTrajPointBasedOnPathS(t_control_s);
    const auto& reference_path_point = reference_traj_point.path_point();

    if (i == 0) {
      // Find the planner kappa kControlInterval before current relative time.
      const double relative_time = reference_traj_point.relative_time();
      const auto reference_path_point_prev_cycle =
          trajectory_interface
              .QueryTrajPointByRelativeTime(relative_time - kControlInterval)
              .path_point();
      controller_debug_proto->mutable_mpc_debug_proto()
          ->add_s_control_kappa_ref(reference_path_point_prev_cycle.kappa());
    }
    controller_debug_proto->mutable_mpc_debug_proto()->add_s_control_kappa_ref(
        reference_path_point.kappa());

    VecXd state_ref(kKinematicModelStateSize);
    state_ref << reference_path_point.x(), reference_path_point.y(),
        NormalizeAngle(reference_path_point.theta() - yaw),
        reference_path_point.kappa();
    state_ref_vector.push_back(state_ref);
  }
  const double reference_kappa = state_ref_vector[0](3);
  debug->set_kappa_feedforward(reference_kappa);

  return state_ref_vector;
}

void DebugMpcResult(const MpcReference& mpc_reference, const VecXd& init_state,
                    const MpcCost& mpc_cost,
                    const TimeVaryingDiscreteStateSpace& tvd_state_space,
                    const std::vector<VecXd>& mpc_output,
                    ControllerDebugProto* controller_debug_proto) {
  controller_debug_proto->set_active_lat_controller("lat_km_mpc");
  auto* mpc_debug = controller_debug_proto->mutable_mpc_debug_proto();

  auto* predicted_point = mpc_debug->add_mpc_predicted_traj_point();
  predicted_point->set_x(init_state(0));
  predicted_point->set_y(init_state(1));

  VecXd mpc_state_cost_evaluation = VecXd::Zero(tvd_state_space.StateSize());
  VecXd mpc_input_cost_evaluation = VecXd::Zero(tvd_state_space.InputSize());

  const std::vector<Eigen::VectorXd> predicted_state =
      EvaluateTvdStateSpace(init_state, tvd_state_space, mpc_output);

  for (int i = 0; i < kSControlHorizon; ++i) {
    // Collect MPC reference position.
    const Vec2d reference_xy(mpc_reference.state_reference[i](0),
                             mpc_reference.state_reference[i](1));
    reference_xy.ToProto(mpc_debug->add_mpc_reference_traj_point());

    const Vec2d predicted_xy(predicted_state[i](0), predicted_state[i](1));
    predicted_xy.ToProto(mpc_debug->add_mpc_predicted_traj_point());

    const Matrix state_cost_coef_matrix =
        i + 1 == kSControlHorizon
            ? mpc_cost.N.transpose() * mpc_cost.Q[i] * mpc_cost.N
            : mpc_cost.Q[i];
    const Eigen::VectorXd state_deviation =
        mpc_reference.state_reference[i] - predicted_state[i];
    mpc_state_cost_evaluation +=
        state_cost_coef_matrix * SquareVecXd(state_deviation);

    const Eigen::VectorXd input_deviation =
        mpc_reference.input_reference[i] - mpc_output[i];
    mpc_input_cost_evaluation += mpc_cost.R * SquareVecXd(input_deviation);
  }

  for (const auto& [name, cost] :
       {std::pair{"state_xy",
                  mpc_state_cost_evaluation(0) + mpc_state_cost_evaluation(1)},
        {"state_heading", mpc_state_cost_evaluation(2)},
        {"state_kappa", mpc_state_cost_evaluation(3)},
        {"control_psi", mpc_input_cost_evaluation(0)}}) {
    auto* costs = controller_debug_proto->mutable_mpc_debug_proto()
                      ->mutable_lateral_mpc_costs()
                      ->add_costs();
    costs->set_name(name);
    costs->set_cost(cost);
  }
}

PiecewiseLinearFunction<double> LoadGainScheduler(
    const GainScheduler& gain_scheduler) {
  // TODO(shijun): replace it with PiecewiseLinearFunctionFromProto.
  std::vector<double> gain_scheduler_speed_vec;
  std::vector<double> gain_scheduler_ratio_vec;
  gain_scheduler_speed_vec.reserve(gain_scheduler.scheduler().size());
  gain_scheduler_ratio_vec.reserve(gain_scheduler.scheduler().size());
  for (const auto& scheduler : gain_scheduler.scheduler()) {
    gain_scheduler_speed_vec.push_back(scheduler.speed());
    gain_scheduler_ratio_vec.push_back(scheduler.ratio());
  }
  return PiecewiseLinearFunction<double>(gain_scheduler_speed_vec,
                                         gain_scheduler_ratio_vec);
}

std::vector<double> GrabRefYaw(double yaw, const MpcReference& mpc_reference) {
  std::vector<double> ref_yaw_vector;
  ref_yaw_vector.reserve(kSControlHorizon);
  for (int i = 0; i < kSControlHorizon; ++i) {
    ref_yaw_vector.push_back(
        NormalizeAngle(mpc_reference.state_reference[i](2) + yaw));
  }
  return ref_yaw_vector;
}

void ModifyMpcCost(const std::vector<double>& ref_yaw_vector, MpcCost* cost) {
  for (int i = 0; i < kSControlHorizon; ++i) {
    const double lat_err_weight = cost->Q[i](0, 0);
    const double sin_ref_yaw = fast_math::Sin(ref_yaw_vector[i]);
    const double cos_ref_yaw = fast_math::Cos(ref_yaw_vector[i]);
    cost->Q[i](0, 0) = lat_err_weight * Sqr(sin_ref_yaw);
    cost->Q[i](0, 1) = -lat_err_weight * sin_ref_yaw * cos_ref_yaw;
    cost->Q[i](1, 0) = -lat_err_weight * sin_ref_yaw * cos_ref_yaw;
    cost->Q[i](1, 1) = lat_err_weight * Sqr(cos_ref_yaw);
  }
}

}  // namespace

LatKmMpcController::LatKmMpcController(
    const ControllerConf* control_conf,
    const SteeringConverter* steering_converter)
    : control_conf_(QCHECK_NOTNULL(control_conf)),
      steering_converter_(QCHECK_NOTNULL(steering_converter)) {
  QCHECK(control_conf_->has_ts_pkmpc_controller_conf() &&
         control_conf_->ts_pkmpc_controller_conf().has_s_matrix_psi() &&
         control_conf_->ts_pkmpc_controller_conf().has_s_matrix_n_kappa())
      << "[LatKmMpcController] lacks the controller weights.";
  QCHECK_GT(control_conf_->ts_pkmpc_controller_conf().ts(), 0.0)
      << "[LatKmMpcController] Invalid control time step "
      << control_conf_->ts_pkmpc_controller_conf().ts();

  // Load configuration parameter
  // Lateral state = [x, y, theta, kappa]^T;
  Eigen::MatrixXd matrix_q =
      Matrix::Zero(kKinematicModelStateSize, kKinematicModelStateSize);
  Eigen::MatrixXd matrix_r =
      Matrix::Zero(kKinematicModelInputSize, kKinematicModelInputSize);
  Eigen::MatrixXd matrix_n =
      Matrix::Zero(kKinematicModelStateSize, kKinematicModelStateSize);
  matrix_q(0, 0) = control_conf_->ts_pkmpc_controller_conf().s_matrix_xy();
  matrix_q(1, 1) = matrix_q(0, 0);
  matrix_q(2, 2) = control_conf_->ts_pkmpc_controller_conf().s_matrix_yaw();
  matrix_q(3, 3) = control_conf_->ts_pkmpc_controller_conf().s_matrix_kappa();
  matrix_r(0, 0) = control_conf_->ts_pkmpc_controller_conf().s_matrix_psi();
  matrix_n(0, 0) = control_conf_->ts_pkmpc_controller_conf().s_matrix_n_xy();
  matrix_n(1, 1) = matrix_n(0, 0);
  matrix_n(2, 2) = control_conf_->ts_pkmpc_controller_conf().s_matrix_n_yaw();
  matrix_n(3, 3) = control_conf_->ts_pkmpc_controller_conf().s_matrix_n_kappa();

  mpc_cost_conf_ = MpcCost(kSControlHorizon, matrix_q, matrix_r, matrix_n);

  s_control_gain_scheduler_plf_ = LoadGainScheduler(
      control_conf_->ts_pkmpc_controller_conf().s_control_gain_scheduler());
}

void LatKmMpcController::Reset(const VehicleStateProto& vehicle_state) {
  // Set control kappa cmd memory the same as chassis steering position.
  previous_kappa_cmd_ = steering_converter_.value()->SteerPctToKappa(
      vehicle_state.chassis_steering_percentage());
}

absl::Status LatKmMpcController::ComputeControlCommand(
    const VehicleStateProto& vehicle_state,
    const TrajectoryInterface& trajectory_interface,
    const SteeringProtectionResult& steering_protection_result,
    const VehPose& predicted_pose_after_delay,
    const LonControllerOutputProto& lon_controller_output, ControlCommand* cmd,
    ControllerDebugProto* controller_debug_proto) {
  QCHECK(cmd != nullptr);
  QCHECK(controller_debug_proto != nullptr);
  SCOPED_QTRACE("LatKmMpcController");

  const double ts = control_conf_->ts_pkmpc_controller_conf().ts();
  SimpleMPCDebug* debug = cmd->mutable_debug()->mutable_simple_mpc_debug();

  const bool is_full_stop = IsFullStop(
      trajectory_interface.accumulate_s(), vehicle_state.linear_velocity(),
      trajectory_interface.GetIsLowSpeedFreespace());
  // Is stationary indicates av is standstill and don't have moving intention.
  const bool is_stationary =
      IsStandstill(vehicle_state.linear_velocity()) && is_full_stop;

  const auto t_control_speed_vec = CalcSControlHorizonSpeedSequence(
      is_stationary, ts, vehicle_state.linear_velocity(),
      absl::MakeSpan(lon_controller_output.t_control_acc_vec()),
      vehicle_state.gear());
  const auto t_control_s_vec =
      ComputeStepLengthFromTControl(t_control_speed_vec, ts);
  UpdateStepLengthToProto(t_control_s_vec,
                          controller_debug_proto->mutable_mpc_debug_proto());

  // Set the init state;
  VecXd init_state(kKinematicModelStateSize);
  init_state << predicted_pose_after_delay.x, predicted_pose_after_delay.y, 0.0,
      previous_kappa_cmd_;

  // Calculate mpc reference.
  const double vehicle_heading_controller =
      control_conf_->enable_yaw_consider_slip()
          ? predicted_pose_after_delay.moving_direction
          : predicted_pose_after_delay.heading;
  MpcReference mpc_reference;
  mpc_reference.Init(kSControlHorizon, kKinematicModelStateSize,
                     kKinematicModelInputSize);
  mpc_reference.state_reference = FindStateReference(
      {predicted_pose_after_delay.x, predicted_pose_after_delay.y},
      vehicle_heading_controller, trajectory_interface, t_control_s_vec, debug,
      controller_debug_proto);

  // Set the mpc constraint.
  const MpcConstraint constraint = BuildTimeVaryingMpcConstraint(
      kKinematicModelStateSize, kKinematicModelInputSize,
      steering_protection_result.kappa_rate_lower(),
      steering_protection_result.kappa_rate_upper());

  const std::vector<double> ref_yaw_vector =
      GrabRefYaw(vehicle_heading_controller, mpc_reference);
  const TimeVaryingDiscreteStateSpace tvd_state_space =
      TobTvKinematicModelUpdate(
          ts,
          KinematicModelInput{
              .yaw = vehicle_heading_controller,
              .speed = t_control_speed_vec,
              .ref_yaw = ref_yaw_vector,
              .kappa_decay_ratio =
                  control_conf_->veh_dynamic_model_conf().kappa_decay_ratio()});
  // Update mpc costs based on current speed.
  MpcCost mpc_cost_update = mpc_cost_conf_;
  if (control_conf_->enable_gain_scheduler() &&
      s_control_gain_scheduler_plf_.has_value()) {
    const double v = vehicle_state.linear_velocity();
    double scheduler_gain =
        s_control_gain_scheduler_plf_->Evaluate(std::fabs(v));
    QCHECK_GT(scheduler_gain, 0.0) << "Lateral mpc control loads negative gain "
                                      "schedular at the speed of "
                                   << v;

    // Restrict state weight to lower than 1.0 in very low speed to avoid
    // radical steering caused by localization disturbance. And this strategy is
    // only used for noa which is not sensitive for control precision in low
    // speed.
    if (!trajectory_interface.GetIsLowSpeedFreespace()) {
      scheduler_gain = std::min(scheduler_gain, 1.0);
      const PiecewiseLinearFunction<double, double>
          planner_ref_kappa_weigth_plf(std::vector<double>{0.2, 1.0, 1.39, 2.0},
                                       std::vector<double>{0.0, 8.0, 8.0, 0.0});
      const double kappa_weight = planner_ref_kappa_weigth_plf(v);
      for (auto& q : mpc_cost_update.Q) {
        q(3, 3) = kappa_weight;
      }
    }
    for (auto& q : mpc_cost_update.Q) {
      q *= scheduler_gain;
    }
    // TODO(shijun): remove it, removing it will cause mpc solver failed:
    // OSQP_SOLVED_INACCURATE, validate it with scenario name:
    // 20220518_144157_Q2301_1801.62_CONTROL.
    mpc_cost_update.N *= scheduler_gain;
  }

  // Modify mpc cost on the distance part to punish lateral deviation.
  ModifyMpcCost(ref_yaw_vector, &mpc_cost_update);

  const auto mpc_result = SolveLinearMPCofTimeVaryingSystem(
      init_state, tvd_state_space, mpc_cost_update, constraint, mpc_reference);
  if (!mpc_result.ok()) {
    QEVENT("zhichao", "mpc_solver_fails", [&](QEvent* qevent) {
      qevent->AddField("controller_name", "lat_tok_mpc");
    });
    return absl::InternalError(
        absl::StrCat("MPC solver failed, detailed reason: ",
                     mpc_result.status().ToString()));
  }

  const double s_kappa_rate = mpc_result.value()[0](0);
  cmd->set_steer_speed_target(steering_converter_.value()->KappaRateToSteerRate(
      s_kappa_rate, previous_kappa_cmd_));
  for (const auto& res : mpc_result.value()) {
    controller_debug_proto->mutable_mpc_debug_proto()->add_s_control_mpc_result(
        res(0));
  }

  // TODO(shijun): move the curvature constraint into MPC state constraint.
  const double kappa_limit =
      std::min(steering_protection_result.kappa_limit_wrt_geometry(),
               steering_protection_result.kappa_limit_wrt_lat_a());
  const double kappa_cmd =
      std::clamp(previous_kappa_cmd_ + s_kappa_rate * kControlInterval,
                 -kappa_limit, kappa_limit);
  previous_kappa_cmd_ = kappa_cmd;
  cmd->set_curvature(kappa_cmd);

  // Prepare controller debug proto.
  DebugMpcResult(mpc_reference, init_state, mpc_cost_update, tvd_state_space,
                 mpc_result.value(), controller_debug_proto);

  return absl::OkStatus();
}

}  // namespace control
}  // namespace qcraft
