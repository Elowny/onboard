#ifndef ONBOARD_CONTROL_CONTROLLERS_CONTROLLER_COMMON_H_
#define ONBOARD_CONTROL_CONTROLLERS_CONTROLLER_COMMON_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/control/control_cache_manager.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/math/vec.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace control {

struct LatControlError {
  double lateral_error = 0.0;
  double heading_error = 0.0;
};

struct LonControlError {
  double station_error = 0.0;
  double speed_error = 0.0;
  double acceleration_error = 0.0;
};

LatControlError CalculateLatControlError(const Vec2d& av_xy, double av_yaw,
                                         const PathPoint& ref_point);

LonControlError CalculateLonControlError(
    const Vec2d& av_xy, double av_speed, double acc_error,
    const ApolloTrajectoryPointProto& ref_point);

ControlError CalculateControlError(
    bool enable_yaw_consider_slip, const VehicleStateProto& vehicle_state,
    const TrajectoryInterface& lon_controller_trajectory_interface,
    const TrajectoryInterface& lat_controller_trajectory_interface,
    double lat_error_canbus, double acc_target_past, double steer_target_pct);

std::vector<double> CalcSControlHorizonSpeedSequence(
    bool is_stationary, double ts, double speed_at_beginning,
    absl::Span<const double> t_control_acc, Chassis::GearPosition chassis_gear);

ControlError CalculateLateralPredictionError(
    const VehicleStateProto& vehicle_state,
    const ControlCacheManager& control_cache_mgr, double steer_delay);

// Estimation step lengths from t control estimated speed sequence.
std::vector<double> ComputeStepLengthFromTControl(
    const std::vector<double>& speed_vec, double mpc_period);

void UpdateStepLengthToProto(const std::vector<double>& t_control_s_vec,
                             ControllerDebugProto::MPCDebugProto* mpc_debug);

// TODO(shijun): move body accessories control functions to more proper files.
void LightControl(const TrajectoryProto& trajectory,
                  ControlCommand* control_cmd);
void DoorControl(const TrajectoryProto& trajectory,
                 ControlCommand* control_cmd);

absl::StatusOr<Chassis::GearPosition> GenerateGearCmd(
    Chassis::GearPosition gear_fb, Chassis::GearPosition gear_target,
    double av_speed);

// If or not entering control full stop conditions.
bool IsFullStop(double trajectory_accumulate_s, double av_speed,
                bool is_freesapce);

bool IsStandstill(double av_speed);

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROLLERS_CONTROLLER_COMMON_H_
