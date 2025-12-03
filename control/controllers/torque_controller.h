#ifndef ONBOARD_CONTROL_CONTROLLERS_TORQUE_CONTROLLER_H_
#define ONBOARD_CONTROL_CONTROLLERS_TORQUE_CONTROLLER_H_

#include <memory>
#include <optional>
#include <string>

#include "onboard/control/controllers/pid_controller.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/math/filters/digital_filter.h"
#include "onboard/math/interpolation_2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft::control {

// Compute the target steering torque based on the feedback control from
// steering angle and steering speed difference.
// Refer to: https://qcraft.feishu.cn/docx/doxcnzwVqqY9lC6RTuXSA1pThad Input
// needs: steer target, steer speed target from control; steer feedback, steer
// speed feedback from canbus;

struct TorqueControllerInput {
  bool is_auto_steer = false;
  bool is_lka = false;
  double steer_angle_target_past = 0.0;
  const VehicleStateProto* vehicle_state = nullptr;
  ControlCommand* control_cmd = nullptr;
  const SteeringConverter* steering_converter = nullptr;
};

struct RoadCondition {
  bool is_up_and_down = false;
  int up_and_down_counter = false;
};

struct TransitionCondition {
  bool is_transition = false;
  int transition_counter = false;
};

class TorqueController {
 public:
  explicit TorqueController(const ControllerSteerTorqueProto& torque_proto);
  explicit TorqueController(const std::string& proto_path);
  double ComputeSteerTorqueTarget(const TorqueControllerInput& input,
                                  ControllerDebugProto* control_debug);
  void Reset();

 private:
  void InitConfig();

  void LoadSteerAngleConfigPLF();
  void LoadSteerSpeedConfigPLF();
  void LoadSteerTorqueMaxPLF();
  void LoadSteerTorqueSpeedMaxPLF();
  void LoadSteerSpeedErrorLimitPLF();
  void LoadSteerAngleErrorLimitPLF();
  void LoadSteerTorqueInterpolation2D();

  PIDConfig ComputeAnglePIDConfig(double speed);
  PIDConfig ComputeAngleSpeedPIDConfig(double speed);
  double ComputeSteerTorqueForward(double speed, double steer_angle,
                                   double steer_speed);
  void ResetStaticTorqueIntegral(double speed);

  PIDControl pid_angle_controller_;
  PIDControl pid_angle_spd_controller_;
  ControllerSteerTorqueProto steer_torque_proto_;
  bool is_vehicle_stop_last_ = false;
  double torque_requested_last_ = 0.0;
  RoadCondition road_condition_;
  TransitionCondition transition_condition_;

  std::unique_ptr<PiecewiseLinearFunction<double>> max_torque_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> max_torque_speed_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> max_steer_speed_error_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> max_steer_angle_error_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> angle_p_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> angle_i_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> angle_d_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> angle_min_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> angle_max_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> angle_max_integral_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> speed_p_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> speed_i_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> speed_d_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> speed_min_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> speed_max_plf_;
  std::unique_ptr<PiecewiseLinearFunction<double>> speed_max_integral_plf_;

  std::optional<Interpolation2D> angle_interpolation2d_opt_ = std::nullopt;
  std::optional<Interpolation2D> speed_interpolation2d_opt_ = std::nullopt;

  DigitalFilter steer_speed_error_filter_;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROLLERS_TORQUE_CONTROLLER_H_
