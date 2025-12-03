#ifndef ONBOARD_CONTROL_OPENLOOP_CONTROL_OPENLOOP_CONTROL_H_
#define ONBOARD_CONTROL_OPENLOOP_CONTROL_OPENLOOP_CONTROL_H_

#include <string>
#include <vector>

#include "gflags/gflags.h"

#include "onboard/control/openloop_control/proto/openloop_cmd.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/proto/control_cmd.pb.h"

DECLARE_string(control_openloop_path);

namespace qcraft {
namespace control {

namespace utils {
std::vector<std::vector<double>> ReadFromTxt(const std::string& file);
}  // namespace utils

class OpenloopControl {
 public:
  OpenloopControl(double wheel_base, double steer_ratio,
                  double max_steer_angle);
  void CreateCommand(ControlCommand* cmd);
  void SetOpenloopCmd(const OpenloopCmd& openloop_cmd);
  void Reset();

 private:
  OpenloopCmd openloop_cmd_;
  std::vector<double> aperture_vec_;
  std::vector<double> acc_vec_;
  std::vector<int> num_aperture_vec_;
  std::vector<int> num_acc_vec_;
  int num_seq_ = 0;
  int steer_seq_ = 0;
  double steer_angle_last_ = 0.0;
  std::vector<double> steer_angle_datas_;
  SteeringConverter steer_converter_;

  void LoadParameters(const std::string& config_path);
  void ProduceThrottleBrake(ControlCommand* cmd);
  void ProduceAcceleration(ControlCommand* cmd);
  void ProduceSteerSin(ControlCommand* cmd);
  void ProduceSteerFollow(ControlCommand* cmd);
  void ProduceGear(ControlCommand* cmd);
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_OPENLOOP_CONTROL_OPENLOOP_CONTROL_H_
