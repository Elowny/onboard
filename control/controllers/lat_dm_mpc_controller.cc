#include "onboard/control/controllers/lat_dm_mpc_controller.h"

#include <algorithm>
#include <initializer_list>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/controller_common.h"
#include "onboard/control/controllers/model/single_track_dynamic_model.h"
#include "onboard/control/controllers/model/state_space.h"
#include "onboard/control/math/mpc_solver.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/control/proto/dynamic_model_conf.pb.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace control {

DEFINE_double(control_dm_cg_ratio, 0.35,
              "the CoG relative position on wheel base.");
DEFINE_double(control_dm_c_af, 40000.0, "N/rad, front tire corning stiffness.");
DEFINE_double(control_dm_c_ar, 40000.0, "N/rad, rear tire corning stiffness.");
DEFINE_double(control_dm_mass, 2200.0, "kg, weight on front axis.");
DEFINE_double(control_dm_mpc_gain_xy, 0.1, ", xy weight.");
DEFINE_double(control_dm_mpc_gain_yaw, 1.0, ", yaw weight.");
DEFINE_double(control_dm_mpc_gain_dpsi, 1.0, ", psi rate weight.");
DEFINE_double(control_dm_mpc_gain_n, 2.0, ", terminal weight.");
DEFINE_bool(control_dm_mpc_enable_ref_input, false,
            "enable front wheel omega reference.");
DEFINE_double(control_dm_ts, 0.1, "time step.");

namespace {

constexpr int kSControlStateNum = 6;
constexpr int kSControlInputNum = 1;

Eigen::VectorXd SquareVecXd(const Eigen::VectorXd& vec) {
  Eigen::VectorXd result = Eigen::VectorXd::Zero(vec.size());
  for (int i = 0; i < vec.size(); ++i) {
    result[i] = Sqr(vec[i]);
  }

  return result;
}

// TODO(zhichao): replace the conf setting with vehicle params.
DynamicModelConfProto BuildDynamicModelConf() {
  DynamicModelConfProto conf;
  conf.set_mass(FLAGS_control_dm_mass);
  conf.mutable_geo_params()->set_cg_ratio(FLAGS_control_dm_cg_ratio);
  conf.mutable_tire_params()->set_c_af(FLAGS_control_dm_c_af);
  conf.mutable_tire_params()->set_c_ar(FLAGS_control_dm_c_ar);
  return conf;
}

MpcConstraint BuildTimeVaryingMpcConstraint(double front_wheel_steer_speed_ll,
                                            double front_wheel_steer_speed_ul) {
  QCHECK_LT(front_wheel_steer_speed_ll, 0.0);
  QCHECK_GT(front_wheel_steer_speed_ul, 0.0);

  MpcConstraint constraint;
  constraint.Init(kSControlStateNum, kSControlInputNum);
  constraint.input_enable(0) = 1.0;
  constraint.input_lower(0) = front_wheel_steer_speed_ll;
  constraint.input_upper(0) = front_wheel_steer_speed_ul;

  return constraint;
}

MpcReference FindStateReference(const VehPose& veh_pose,
                                const TrajectoryInterface& trajectory_interface,
                                const SteeringConverter& steering_converter,
                                const std::vector<double>& lon_control_s_vec,
                                SimpleMPCDebug* debug,
                                ControllerDebugProto* controller_debug_proto) {
  MpcReference mpc_reference;
  mpc_reference.Init(kSControlHorizon, kSControlStateNum, kSControlInputNum);
  // Step 1: find the closest trajectory point from trajectory and past
  // points.
  const auto closest_trajectory_point =
      trajectory_interface.QueryNearestTrajPointByXY({veh_pose.x, veh_pose.y});
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

  for (int i = 0; i < kSControlHorizon; ++i) {
    const double t_control_s = current_s + lon_control_s_vec[i];
    const auto reference_traj_point =
        trajectory_interface.QueryTrajPointBasedOnPathS(t_control_s);
    const auto& reference_path_point = reference_traj_point.path_point();
    const double ref_kappa = reference_path_point.kappa();
    const double ref_v = reference_traj_point.v();
    const double ref_kappa_rate = reference_path_point.lambda() * ref_v;
    const double ref_front_wheel_omega =
        steering_converter.KappaRateToFrontWheelOmega(ref_kappa_rate,
                                                      ref_kappa);
    if (i == 0) debug->set_kappa_feedforward(ref_kappa);

    VecXd state_ref(kSControlStateNum);
    state_ref << reference_path_point.x(), reference_path_point.y(),
        NormalizeAngle(reference_path_point.theta() - veh_pose.heading),
        /*omega*/ ref_kappa * ref_v, /*delta u*/ 0.0,
        steering_converter.KappaToFrontWheelAngle(ref_kappa);
    mpc_reference.state_reference[i] = state_ref;
    if (FLAGS_control_dm_mpc_enable_ref_input) {
      mpc_reference.input_reference[i] << ref_front_wheel_omega;
    }
  }

  return mpc_reference;
}

void DebugMpcResult(const MpcReference& mpc_reference, const VecXd& init_state,
                    const MpcCost& mpc_cost,
                    const TimeVaryingDiscreteStateSpace& tvd_state_space,
                    const std::vector<VecXd>& mpc_output,
                    ControllerDebugProto* controller_debug_proto) {
  controller_debug_proto->set_active_lat_controller("lat_dm_mpc");
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
        {"state_omega", mpc_state_cost_evaluation(3)},
        {"state_psi", mpc_state_cost_evaluation(5)},
        {"control_psi_rate", mpc_input_cost_evaluation(0)}}) {
    auto* costs = controller_debug_proto->mutable_mpc_debug_proto()
                      ->mutable_lateral_mpc_costs()
                      ->add_costs();
    costs->set_name(name);
    costs->set_cost(cost);
  }
}

}  // namespace

LatDmMpcController::LatDmMpcController(
    const VehicleGeometryParamsProto* vehicle_geometry_params,
    const ControllerConf* control_conf,
    const SteeringConverter* steering_converter)
    : geo_params_(QCHECK_NOTNULL(vehicle_geometry_params)),
      control_conf_(QCHECK_NOTNULL(control_conf)),
      steering_converter_(QCHECK_NOTNULL(steering_converter)) {
  QCHECK_GT(control_conf_->ts_pkmpc_controller_conf().ts(), 0.0)
      << "[LatDmMpcController] Invalid control time step "
      << control_conf_->ts_pkmpc_controller_conf().ts();

  // Load configuration parameter
  Eigen::MatrixXd matrix_q = Matrix::Zero(kSControlStateNum, kSControlStateNum);
  Eigen::MatrixXd matrix_r = Matrix::Zero(1, 1);
  Eigen::MatrixXd matrix_n = Matrix::Zero(kSControlStateNum, kSControlStateNum);
  matrix_q(0, 0) = FLAGS_control_dm_mpc_gain_xy;    // x.
  matrix_q(1, 1) = FLAGS_control_dm_mpc_gain_xy;    // y.
  matrix_q(2, 2) = FLAGS_control_dm_mpc_gain_yaw;   // theta.
  matrix_q(3, 3) = 0.0;                             // omega.
  matrix_q(5, 5) = 0.0;                             // psi.
  matrix_r(0, 0) = FLAGS_control_dm_mpc_gain_dpsi;  // psi rate.
  matrix_n(0, 0) = FLAGS_control_dm_mpc_gain_n;     // x.
  matrix_n(1, 1) = FLAGS_control_dm_mpc_gain_n;     // y.
  matrix_n(2, 2) = FLAGS_control_dm_mpc_gain_n;     // theta.
  matrix_n(3, 3) = FLAGS_control_dm_mpc_gain_n;     // omega.
  matrix_n(5, 5) = FLAGS_control_dm_mpc_gain_n;     // psi.

  mpc_cost_conf_ = MpcCost(kSControlHorizon, matrix_q, matrix_r, matrix_n);
}

void LatDmMpcController::Reset(const VehicleStateProto& vehicle_state) {
  previous_front_wheel_angle_cmd_ =
      steering_converter_.value()->SteerPctToFrontWheelAngle(
          vehicle_state.chassis_steering_percentage());
}

absl::Status LatDmMpcController::ComputeControlCommand(
    const VehicleStateProto& vehicle_state,
    const TrajectoryInterface& trajectory_interface,
    const SteeringProtectionResult& steering_protection_result,
    const VehPose& predicted_pose_after_delay,
    const LonControllerOutputProto& lon_controller_output, ControlCommand* cmd,
    ControllerDebugProto* controller_debug_proto) {
  QCHECK(cmd != nullptr);
  QCHECK(controller_debug_proto != nullptr);
  SCOPED_QTRACE("LatDmMpcController");

  SimpleMPCDebug* debug = cmd->mutable_debug()->mutable_simple_mpc_debug();

  const bool is_full_stop = IsFullStop(
      trajectory_interface.accumulate_s(), vehicle_state.linear_velocity(),
      trajectory_interface.GetIsLowSpeedFreespace());
  // Is stationary indicates av is standstill and don't have moving intention.
  const bool is_stationary =
      IsStandstill(vehicle_state.linear_velocity()) && is_full_stop;

  const std::vector<double> t_control_speed_vec =
      CalcSControlHorizonSpeedSequence(
          is_stationary, FLAGS_control_dm_ts, predicted_pose_after_delay.v,
          absl::MakeSpan(lon_controller_output.t_control_acc_vec()),
          vehicle_state.gear());
  const std::vector<double> t_control_s_vec =
      ComputeStepLengthFromTControl(t_control_speed_vec, FLAGS_control_dm_ts);
  UpdateStepLengthToProto(t_control_s_vec,
                          controller_debug_proto->mutable_mpc_debug_proto());

  // Set the init state in vehicle coordinate;
  // TODO(zhichao) the ref angular_velocity, lateral_velocity, and
  // front_wheel_angle should come from predicted_pose_after_delay.
  VecXd init_state(kSControlStateNum);
  init_state << predicted_pose_after_delay.x, predicted_pose_after_delay.y,
      /*delta_theta*/ 0.0, vehicle_state.angular_velocity(),
      vehicle_state.lateral_velocity(),
      steering_converter_.value()->KappaToFrontWheelAngle(
          predicted_pose_after_delay.kappa);

  // Calculate mpc reference.
  MpcReference mpc_reference =
      FindStateReference(predicted_pose_after_delay, trajectory_interface,
                         *steering_converter_.value(), t_control_s_vec, debug,
                         controller_debug_proto);

  // Set the mpc constraint.
  const double front_wheel_steer_speed_ll =
      steering_converter_.value()->KappaRateToFrontWheelOmega(
          steering_protection_result.kappa_rate_lower(), vehicle_state.kappa());
  const double front_wheel_steer_speed_ul =
      steering_converter_.value()->KappaRateToFrontWheelOmega(
          steering_protection_result.kappa_rate_upper(), vehicle_state.kappa());
  const MpcConstraint constraint = BuildTimeVaryingMpcConstraint(
      front_wheel_steer_speed_ll, front_wheel_steer_speed_ul);

  DynamicModelMeasurement dm_measurement{
      .psi = steering_converter_.value()->SteerPctToFrontWheelAngle(
          vehicle_state.chassis_steering_percentage()),
      .yaw = predicted_pose_after_delay.heading,
      .roll = vehicle_state.roll(),
      .u_0 = vehicle_state.lateral_velocity(),
      .omega = vehicle_state.angular_velocity(),
      .v = t_control_speed_vec};

  TimeVaryingDiscreteStateSpace tvd_state_space = BuildSingleTrackDynamicModel(
      FLAGS_control_dm_ts, geo_params_->wheel_base(), geo_params_->length(),
      dm_measurement, BuildDynamicModelConf());

  const auto mpc_result = SolveLinearMPCofTimeVaryingSystem(
      init_state, tvd_state_space, mpc_cost_conf_, constraint, mpc_reference);

  if (!mpc_result.ok()) {
    QEVENT("zhichao", "mpc_solver_fails", [&](QEvent* qevent) {
      qevent->AddField("controller_name", "lat_dm_mpc");
    });
    return absl::InternalError(
        absl::StrCat("MPC solver failed, detailed reason: ",
                     mpc_result.status().ToString()));
  }

  // TODO(zhichao): wrap dynamic model debug.
  auto* dm_debug = controller_debug_proto->mutable_dm_debug_proto();
  dm_debug->set_mpc_solved(true);

  for (int i = 0; i < mpc_result.value().size(); ++i) {
    dm_debug->add_front_wheel_rate_vector(mpc_result.value()[i](0));
  }

  DebugMpcResult(mpc_reference, init_state, mpc_cost_conf_, tvd_state_space,
                 mpc_result.value(), controller_debug_proto);

  // Prepare control cmd.
  const double kappa_limit =
      std::min(steering_protection_result.kappa_limit_wrt_geometry(),
               steering_protection_result.kappa_limit_wrt_lat_a());
  const double front_wheel_angle_rate_cmd = mpc_result.value()[0](0);
  cmd->set_steer_speed_target(
      steering_converter_.value()->FrontWheelOmegaToSteerRate(
          front_wheel_angle_rate_cmd));

  const double front_wheel_angle_cmd =
      previous_front_wheel_angle_cmd_ +
      front_wheel_angle_rate_cmd * kControlInterval;
  previous_front_wheel_angle_cmd_ = front_wheel_angle_cmd;

  const double kappa_cmd =
      std::clamp(steering_converter_.value()->FrontWheelAngleToKappa(
                     front_wheel_angle_cmd),
                 -kappa_limit, kappa_limit);

  cmd->set_curvature(kappa_cmd);

  return absl::OkStatus();
}

}  // namespace control
}  // namespace qcraft
