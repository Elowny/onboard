#ifndef ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_MODULE_H_
#define ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_MODULE_H_

#include <memory>
#include <optional>

#include "onboard/control/closed_loop_acc/speed_mode_manager.h"
#include "onboard/control/longitudinal_postprocess/calibration_manager.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/math/filters/mean_filter.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::control {

// Acceleration interface define.
// https://qcraft.feishu.cn/docs/doccnc5INPjOm80l1kPCapeteNg#1Jdjfc

struct LonPostProcessInput {
  bool is_onboard_mode = true;
  bool is_auto_mode;
  Chassis::GearPosition gear_cmd;
  Chassis::GearPosition gear_fb;
  bool low_speed_freespace = false;
  double acc_target = 0.0;
  double acc_feedback = 0.0;
  double acc_planner = 0.0;
  double speed_feedback = 0.0;
  double speed_planner = 0.0;
  double pitch_pose = 0.0;
  double steer_wheel_angle = 0.0;
  double kappa_cmd = 0.0;
  double kappa_rate_cmd = 0.0;
  double steer_speed_target = 0.0;
  double trajectory_accumulate_s = 0.0;

  LonPostProcessInput() = default;

  LonPostProcessInput(bool is_onboard_mode, bool low_speed_freespace,
                      double kappa_rate, const ControlCommand& cmd,
                      const VehicleStateProto& vehicle_state,
                      const TrajectoryInterface& trajectory_interface);
};

class LonPostProcess {
 public:
  LonPostProcess(const ControllerConf* control_config,
                 const VehicleDriveParamsProto* vehicle_drive_params);

  /**
   * @description: Longitudinal post process
   * @param acc_target {acceleration_cmd, from mpc controller}
   * @param acc_feedback {acceleration_feedback, from pose}
   * @param acc_planner {acceleration_planner, from planner}
   * @param speed_feedback {speed_feedback, from pose}
   * @param speed_planner {speed_planner, from planner}
   * @param pitch_pose {pitch, from pose, pitch < 0 ,when climb slope}
   * @param steer_wheel_angle {steer_wheel_angle, from chassis}
   * @param is_standstill {is_standstill, from controller}
   * @param gear_fb {gear_position, from chassis}
   * @param low_speed_freespace {low_speed_freespace, from planner}
   * @param autonomy_state {autonomy_state, autodrive is or not}
   */

  void Process(const LonPostProcessInput& input, ControlCommand* control_cmd,
               ControllerDebugProto* control_debug);

 private:
  // Add deceleration filter to avoid hard brake.
  double SmoothHardBrake(double acc_target);

  // Compute acceleration offset caused by slope.
  double GetAccCompensationBySlope(double sin_slope);

  // Compute acceleration when fullstop.
  std::optional<double> GenerateAccForFullStop(
      const ControllerConf& controller_conf, Chassis::GearPosition gear_fb,
      double acc_offset, double acc_smoothed, bool is_full_stop) const;

  double CalcAccForCalibration(std::optional<double> acc_full_stop_opt,
                               Chassis::GearPosition gear_fb,
                               double acc_smoothed, double acc_offset) const;

  // Speed mode and closed acc function.
  double ComputeAccCalibrationBySpeedMode(bool is_auto_mode, bool is_full_stop,
                                          Chassis::GearPosition gear_fb,
                                          double speed_feedback,
                                          double acc_calibration,
                                          double acc_offset,
                                          double acc_target_climb);

  // Update standstill state.
  bool UpdateStandStillState(double speed_feedback, bool is_standstill);

  // Longitudinal control calibration.
  double ConvertAcc2ExecutorPercentage(
      bool is_auto_mode, Chassis::GearPosition gear_fb, double speed_feedback,
      double acc_feedback, double steer_wheel_angle, double acc_calibration,
      bool is_full_stop, bool is_gear_shifting);

  double ComputeLongitudinalJerk(bool is_full_stop, double acc_feedback,
                                 double acc_target, double delay);

  // Config
  const ControllerConf* control_config_ = nullptr;
  const VehicleDriveParamsProto* vehicle_params_config_ = nullptr;
  ControllerDebugProto control_debug_;
  ControlCommand control_cmd_;

  double previous_acc_smoothed_ = 0.0;
  double previous_acc_calibration_ = 0.0;

  MeanFilter sin_slope_mean_filter_;
  std::unique_ptr<ClosedLoopAcc> speed_mode_acc_closed_loop_;
  std::unique_ptr<CalibrationManager> calibration_manager_;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_MODULE_H_
