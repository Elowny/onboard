#include "onboard/control/lateral_postprocess/compensate_curvature_gain.h"

#include <algorithm>

#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
namespace qcraft::control {
namespace {

// TODO(yangyu): add a design doc link.
double DynamicModelKappaGain(double speed, const VehDynamicModelConf& conf) {
  const double mass =
      conf.mass_fl() + conf.mass_fr() + conf.mass_rl() + conf.mass_rr();
  const double cf = conf.c_fl() + conf.c_fr();
  const double cr = conf.c_rl() + conf.c_rr();
  const double wheelbase = conf.wheelbase_f() + conf.wheelbase_r();
  QCHECK_GT(wheelbase * cf * cr, 0.0);

  const double sliding_factor =
      mass * (cf * conf.wheelbase_f() - cr * conf.wheelbase_r()) /
      (Sqr(wheelbase) * cf * cr);

  return std::clamp(1 - sliding_factor * Sqr(speed), /*lower limit*/ 0.99,
                    /*higher limit*/ 2.5);
}

double LatAccKappaGain(double speed, double lat_accel,
                       const LatAccGainConf& conf) {
  const PiecewiseLinearFunction<double>& lat_acc_steer_plf =
      PiecewiseLinearFunctionFromProto(conf.lat_acc_steer_plf());

  return speed > conf.velocity_threshold() ? lat_acc_steer_plf(lat_accel) : 1.0;
}

}  // namespace

double ComputeKappaGain(const KappaGainInput& input,
                        const ControllerConf& control_conf,
                        SteerCalibrationDebugProto* debug) {
  double kappa_gain = 1.0;
  // Kappa gain priority:
  // DynamicModelKappaGain > LatAccKappaGain > SpeedKappaGain;
  if (control_conf.has_veh_dynamic_model_conf() &&
      control_conf.veh_dynamic_model_conf()
          .enable_dynamic_model_compensation()) {
    kappa_gain = DynamicModelKappaGain(input.speed,
                                       control_conf.veh_dynamic_model_conf());
    debug->set_dynamic_gain(kappa_gain);
    return kappa_gain;
  }

  if (control_conf.has_lat_acc_gain_conf() &&
      control_conf.lat_acc_gain_conf().enable_lat_acc_gain()) {
    kappa_gain = LatAccKappaGain(input.speed, input.lat_accel,
                                 control_conf.lat_acc_gain_conf());
    debug->set_sliding_gain(kappa_gain);
    return kappa_gain;
  }
  return kappa_gain;
}

}  // namespace qcraft::control
