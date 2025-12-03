#ifndef ONBOARD_CONTROL_LATERAL_POSTPROCESS_COMPENSATE_CURVATURE_GAIN_H_
#define ONBOARD_CONTROL_LATERAL_POSTPROCESS_COMPENSATE_CURVATURE_GAIN_H_

#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft::control {

struct KappaGainInput {
  double mpc_kappa = 0.0;
  double speed = 0.0;
  double lat_accel = 0.0;
};

double ComputeKappaGain(const KappaGainInput& input,
                        const ControllerConf& control_conf,
                        SteerCalibrationDebugProto* debug);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_LATERAL_POSTPROCESS_COMPENSATE_CURVATURE_GAIN_H_
