#ifndef ONBOARD_CONTROL_CONTROLLERS_PREDICT_VEHICLE_POSE_H_
#define ONBOARD_CONTROL_CONTROLLERS_PREDICT_VEHICLE_POSE_H_

#include <string>
#include <vector>

#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

struct VehPose {
  double timestamp = 0.0;
  double x = 0.0;
  double y = 0.0;
  double v = 0.0;
  double heading = 0.0;
  double acc = 0.0;
  double kappa = 0.0;
  double angular_velocity = 0.0;
  double lateral_velocity = 0.0;
  double front_wheel_steering_angle = 0.0;
  double moving_direction = 0.0;
  const VehicleGeometryParamsProto* geo_param = nullptr;

  VehPose() = default;
  VehPose(const VehicleGeometryParamsProto* vehicle_geometry_params,
          const VehicleStateProto& vehicle_state);
  VehPose(double acc_target_last,
          const VehicleGeometryParamsProto* vehicle_geometry_params,
          const VehicleStateProto& vehicle_state);

  void AdvancedByKinematicModel(double ts, double acc_cmd, double kappa_cmd);

  void AdvancedByDynamicModel(double ts, double acc_cmd,
                              double steering_speed_target,
                              const SteeringConverter& steering_converter);

  void ToProto(VehPoseProto* proto) const;

  VehPose operator-(const VehPose& comp_vehpose) const;

  std::string DebugString() const;
};

VehPose PredictControlInitPoseByKM(const VehPose& init_pose,
                                   const SteeringConverter& steering_converter,
                                   const std::vector<double>& kappa_target,
                                   const std::vector<double>& acc_target);

VehPose PredictControlInitPoseByDM(
    const VehPose& init_pose, const SteeringConverter& steering_converter,
    const std::vector<double>& steer_speed_target,
    const std::vector<double>& acc_target);

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROLLERS_PREDICT_VEHICLE_POSE_H_
