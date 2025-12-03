#ifndef ONBOARD_CONTROL_CONTROLLERS_LON_MPC_CONTROLLER_H_
#define ONBOARD_CONTROL_CONTROLLERS_LON_MPC_CONTROLLER_H_

#include <optional>
#include <vector>

#include "absl/status/status.h"

#include "onboard/control/controllers/predict_vehicle_pose.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

// MPC-Based longitudinal controller to compute acceleration or brake/throttle.
// Third-order kinematic model, 3rd-order: integral-s, s, v, a.
class LonMpcController {
 public:
  LonMpcController(const ControllerConf* control_conf,
                   const VehicleDriveParamsProto* vehicle_drive_params);

  absl::Status ComputeControlCommand(
      const TrajectoryInterface& trajectory_interface,
      const VehPose& predicted_pose_after_delay, ControlCommand* cmd,
      ControllerDebugProto* controller_debug_proto,
      LonControllerOutputProto* lon_controller_output);

  void Reset(const VehicleStateProto& vehicle_state);

 private:
  void FindControlReference(
      double x, double y, const double vehicle_state_time,
      const TrajectoryInterface& trajectory_interface, double mpc_period,
      std::vector<Eigen::VectorXd>* t_control_state_reference,
      std::vector<Eigen::VectorXd>* t_control_input_reference,
      SimpleMPCDebug* debug);

  void UpdateInitialState(double av_speed, const SimpleMPCDebug& debug,
                          Eigen::VectorXd* t_initial_state);

  void UpdateTransitionMatrix(double mpc_period, Eigen::MatrixXd* t_matrix_a,
                              Eigen::MatrixXd* t_matrix_ad,
                              Eigen::MatrixXd* t_matrix_b,
                              Eigen::MatrixXd* t_matrix_bd,
                              std::vector<Eigen::MatrixXd>* t_matrix_ad_t);

  void ConstraintSetup(double max_deceleration, double max_acceleration,
                       Eigen::MatrixXd* t_control_input_constraint_enable,
                       std::vector<Eigen::VectorXd>* t_control_input_lower,
                       std::vector<Eigen::VectorXd>* t_control_input_upper);

  void LoadGainScheduler(
      const GainScheduler& gain_scheduler,
      std::optional<PiecewiseLinearFunction<double>>* gain_scheduler_plf);

  // Mpc time step
  double ts_;

  std::optional<PiecewiseLinearFunction<double>> t_control_gain_scheduler_plf_;

  const ControllerConf* control_conf_ = nullptr;
  const VehicleDriveParamsProto* vehicle_drive_params_ = nullptr;

  static constexpr int kTControlStateNum = 2;
  static constexpr int kTControlInputNum = 1;

  // State matrix.
  Eigen::VectorXd t_initial_state_;

  // Vehicle state matrix.
  std::vector<Eigen::MatrixXd> t_matrix_ad_t_;
  Eigen::MatrixXd t_matrix_a_;
  // Vehicle state matrix (discrete-time).
  Eigen::MatrixXd t_matrix_ad_;
  // Vehicle control matrix.
  Eigen::MatrixXd t_matrix_b_;
  // Vehicle control matrix (discrete-time).
  Eigen::MatrixXd t_matrix_bd_;
  // Vehicle offset state.
  Eigen::MatrixXd t_matrix_c_;
  // Vehicle offset state (discrete-time).
  Eigen::MatrixXd t_matrix_cd_;

  // Control authority weighting matrix.
  Eigen::MatrixXd t_matrix_r_;
  // Updated control authority weighting matrix.
  Eigen::MatrixXd t_matrix_r_updated_;
  // State weighting matrix.
  Eigen::MatrixXd t_matrix_q_;
  // Updated state weighting matrix.
  std::vector<Eigen::MatrixXd> t_matrix_q_updated_;
  // Terminal state weighting matrix.
  Eigen::MatrixXd t_matrix_n_;
  // Updated terminal state weighting matrix.
  Eigen::MatrixXd t_matrix_n_updated_;

  // Time control state: [s, v]
  std::vector<Eigen::VectorXd> t_control_state_reference_;
  std::vector<Eigen::VectorXd> t_control_input_reference_;
  Eigen::MatrixXd t_control_input_constraint_enable_;
  std::vector<Eigen::VectorXd> t_control_input_lower_;
  std::vector<Eigen::VectorXd> t_control_input_upper_;
  Eigen::MatrixXd t_control_state_constraint_enable_;
  std::vector<Eigen::VectorXd> t_control_state_lower_;
  std::vector<Eigen::VectorXd> t_control_state_upper_;

  double previous_speed_cmd_ = 0.0;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROLLERS_LON_MPC_CONTROLLER_H_
