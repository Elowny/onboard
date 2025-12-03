#ifndef ONBOARD_CONTROL_CONTROL_CACHE_MANAGER_H_
#define ONBOARD_CONTROL_CONTROL_CACHE_MANAGER_H_

// IWYU pragma: no_include <algorithm>

#include <optional>
#include <string>
#include <vector>

#include "boost/circular_buffer.hpp"

#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

static constexpr int kCacheSize = 500;  // 10 s.

class ControlCacheManager {
 public:
  ControlCacheManager();

  struct ControlStateData {
    bool is_auto_mode = false;
    bool is_auto_steer = false;
    bool is_auto_speed = false;
    bool is_full_stop = false;
    bool is_standstill = false;

    double kappa_cmd = 0.0;          // control_command.curvature();
    double planner_kappa = 0.0;      // path_point.().kappa() from trajectory.
    double steer_pct_control = 0.0;  // control_command.steering_target();
    double steer_pct_chassis = 0.0;  // chassis.steering_percentage();
    double control_acceleration = 0.0;  // control_command.acceleration();
    double steer_speed_target = 0.0;    // control_command.steer_speed_target()
    double kappa_pose = 0.0;
    double speed_pose = 0.0;
    double lateral_acceleration_pose = 0.0;
    ControlError control_error;

    // Refer to the doc of Post-processing of longitudinal control -
    // acceleration signal definitions".
    double acc_target = 0.0;
    double acc_planner = 0.0;
    double acc_pose = 0.0;
    SpeedMode speed_mode;

    // Lateral error due to canbus noise
    double lat_error_canbus = 0.0;
    VehPoseProto veh_predicted_pose;
  };

  struct DecModeStat {
    int count = 0;
    std::vector<double> brake_duration;  // s.
    std::vector<double> brake_interval;  // s.
    std::vector<double> avg_accl_cmd;    // m/s^2.

    void CountOneBrake(double duration, double interval, double sum_accl);
    std::string DebugString() const;
    bool operator==(const DecModeStat& compared) const;
  };

  struct OptionalArgs {
    std::optional<double> delay_time = std::nullopt;
    const ControllerDebugProto* control_debug = nullptr;
    const TrajectoryInterface* trajectory_interface = nullptr;
    const SteeringConverter* steering_converter = nullptr;
    const VehPoseProto* veh_predicted_pose = nullptr;
  };

  void UpdateCacheData(const ControlCommand& control_cmd,
                       const VehicleStateProto& vehicle_state,
                       OptionalArgs optional_args);

  // Query particular raw control data from cache.
  double QueryKappaCmd(int steps) const;
  double QueryPlannerKappa(int steps) const;
  double QuerySteerPctChassis(int steps) const;
  double QuerySteerPctControl(int steps) const;
  double QuerySteerSpeedTarget(int steps) const;
  double QueryAccTarget(int steps) const;
  double QueryAccPose(int steps) const;
  double QuerySpeedPose(int steps) const;
  ControlError QueryControlError(int steps) const;
  std::vector<double> QueryKappaCmdVector(int recent_steps) const;
  std::vector<double> QuerySteerSpeedTargetVector(int recent_steps) const;
  std::vector<double> QueryAccTargetVector(int recent_steps) const;
  std::vector<double> QueryAccPoseVector(int recent_steps) const;
  // Query control data based on the statistics method.
  double QueryMaxControlAcc(int steps) const;
  double QueryMinControlAcc(int steps) const;
  double QueryMaxKappaCmd(int steps) const;
  double QueryMinKappaCmd(int steps) const;
  double QueryMinAccPlanner(int steps) const;
  const VehPoseProto& QueryPredictPose(int steps) const;
  double IntegrateLatErrorCanbus(int steps) const;
  double QueryMinAbsLatError(int steps) const;
  double QueryMinAbsLatACC(int steps) const;
  double QueryMinAbsLonError(int steps) const;
  double QueryMinControlSteerPct(int steps) const;
  double QueryMinChassisSteerPct(int steps) const;
  double QueryMeanKappaPose(int steps) const;

  // Check whether it is always under auto_mode in recent steps.
  bool IsInAlwaysAutoMode(int steps) const;
  bool IsInAlwaysSteerMode(int steps) const;
  bool IsInAlwaysSpeedMode(int steps) const;

  // Whether vehicle is intented to move in n control steps ago.
  bool IsVehicleIntentedToMove(int steps) const;

  // Return speed mode switching times in the range of control cache.
  DecModeStat CountDecMode() const;

  // Calculate how many steps has the vehicle already moved.
  int CalculateStepsMoveOff() const;

 private:
  boost::circular_buffer<ControlStateData> control_state_cache_{kCacheSize};
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROL_cache_MANAGER_H_
