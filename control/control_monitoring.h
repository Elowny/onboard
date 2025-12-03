#ifndef ONBOARD_CONTROL_CONTROL_MONITORING_H_
#define ONBOARD_CONTROL_CONTROL_MONITORING_H_

#include "onboard/control/control_cache_manager.h"
#include "onboard/control/controllers/predict_vehicle_pose.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::control {

inline constexpr double kMoveOffMonitoringTime = 3.0;  // s.

void QCounterPose(const PoseProto& pose_proto);

void QCounterControlError(const VehicleStateProto& vehicle_state,
                          const ControlError& control_error);

void QEventControlCache(double speed,
                        const ControlCacheManager& control_cache_manager);

void QEventDiscomfortable(const ControlCacheManager& control_cache_manager,
                          const SteeringConverter& steering_converter,
                          const PoseProto& pose);

void QEventHardBrake(double control_cmd_accel, double traj_min_accel,
                     double traj_ref_accel,
                     const ControlCacheManager& control_cache_manager);

void QEventTrackingError(double steer_ref_pct, double speed_ref,
                         const ControlCacheManager& control_cache_manager);

void QEventLatVehPosDiff(const VehPose& vehpos_km, const VehPose& vehpos_dm);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROL_MONITORING_H_
