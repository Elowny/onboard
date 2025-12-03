#ifndef ONBOARD_CONTROL_PARAMETER_IDENTIFICATION_PARAMETER_IDENTIFICATION_H_
#define ONBOARD_CONTROL_PARAMETER_IDENTIFICATION_PARAMETER_IDENTIFICATION_H_

// IWYU pragma: no_include <ostream>

#include <optional>
#include <vector>

#include "boost/circular_buffer.hpp"

#include "onboard/control/control_cache_manager.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/math/filters/exponential_smoothing.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/proto/piecewise_linear_function.pb.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft::control {

struct SteerDelay {
  double steer_delay = 0.0;  // Current steer delay time.
  double canbus_steer_delay = 0.0;
  bool has_kappa_delay_gain_wrt_speed = false;

  PiecewiseLinearFunction<double> steer_delay_plf;
  PiecewiseLinearFunction<double> kappa_delay_gain_wrt_speed_plf;

  void InitConf(const ControllerConf& control_conf) {
    // TODO(Yangyu): convert steer_straight_th and steer_turn_th to ratio or
    // percentage to ease normalizing the setting.
    const double steer_straight_th =
        control_conf.steer_deadzone_adaptor_conf().steer_straight_th();
    const double steer_delay_time_straight =
        control_conf.steer_deadzone_adaptor_conf().steer_straight_delay_time();
    const double steer_turn_th =
        control_conf.steer_deadzone_adaptor_conf().steer_turn_th();
    const double steer_delay_time_turn = control_conf.steer_delay_time();
    has_kappa_delay_gain_wrt_speed =
        control_conf.has_kappa_delay_gain_wrt_speed() &&
        !control_conf.kappa_delay_gain_wrt_speed().x().empty();

    if (has_kappa_delay_gain_wrt_speed) {
      kappa_delay_gain_wrt_speed_plf = PiecewiseLinearFunctionFromProto(
          control_conf.kappa_delay_gain_wrt_speed());
    }

    steer_delay_plf = PiecewiseLinearFunction(
        std::vector<double>{steer_straight_th, steer_turn_th},
        std::vector<double>{steer_delay_time_straight, steer_delay_time_turn});

    canbus_steer_delay =
        control_conf.bias_estimation_conf().steer_status_delay_time();
  }
};

struct ParameterIdentificationInput {
  double steer_cmd;
  double steer_pose;
  double steer_feedback;
  double speed_measurement;
  double lat_acc;
  double lat_error;
  double heading_err;
  bool is_auto;
  const ControlCacheManager* control_cache_mgr;
};

class ParameterIdentificator {
 public:
  ParameterIdentificator(const ControllerConf& control_conf,
                         const SteeringConverter* steering_converter,
                         double init_steer_bias);

  void Process(const VehicleStateProto& vehicle_state,
               ControlCommand* control_command, double last_kappa_cmd);

  // Design doc: https://qcraft.feishu.cn/docs/doccn4isLXzD37Xdwc8QsyqY13b
  void EstimateLatBias(const ParameterIdentificationInput& input,
                       BiasEstimationDebug* bias_estimation_debug);

  double SteerDelayTime() const { return steer_delay_.steer_delay; }

  double SteerBiasOutput() const { return steer_bias_output_; }

  double HeadingBias() const { return heading_bias_; }

 private:
  double CalculateSteerDelay(const SteeringConverter& steering_converter,
                             const VehicleStateProto& vehicle_state,
                             double last_kappa_cmd);

  struct SteerBiasIdentificationInputData {
    double front_wheel_angle;  // unit: rad
    double kappa;
  };

  // Construct input data from pose, chassis and autonomy state, wrt time
  // delay between steering action and av pose
  std::optional<SteerBiasIdentificationInputData> AssembleInputData(
      const VehicleStateProto& vehicle_state);

  double CalculateSteerBias(const SteerBiasIdentificationInputData&
                                steer_bias_identification_input_data);

 private:
  const SteeringConverter* steering_converter_;
  std::optional<ExponentialSmoothing> steering_bias_smoother_;

  int prev_valid_result_num_ = 0;
  double steer_bias_output_ = 0.0;

  double calib_steering_ = 0.0;  // front wheel steer angle
  double heading_bias_ = 0.0;

  double weight_update_steer_ = 0.0;
  double weight_update_heading_ = 0.0;

  BiasEstimationConf bias_estimation_conf_;
  SteerDelay steer_delay_;

  // Time delay between chassis steering action and av pose.
  static constexpr int kSteeringBiasDelayCacheSize = 11;  // time range = 0.1s
  boost::circular_buffer<double> pose_curvature_cache_{
      kSteeringBiasDelayCacheSize};
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_PARAMETER_IDENTIFICATION_PARAMETER_IDENTIFICATION_H_
