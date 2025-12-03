#include "onboard/control/openloop_control/openloop_control.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <fstream>

#include "onboard/control/control_defs.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/utils/file_util.h"

DEFINE_string(control_openloop_path,
              "onboard/control/openloop_control/testdata/openloop.pb.txt",
              "config path.");

namespace qcraft {
namespace control {
namespace utils {

std::vector<std::vector<double>> ReadFromTxt(const std::string& file) {
  std::vector<std::vector<double>> datas;
  std::ifstream infile;
  std::string str;

  infile.open(file.c_str());
  QCHECK(!infile.fail()) << "Open file is failed !";
  while (getline(infile, str)) {
    for (int i = 0; i < str.size(); ++i) {
      if (str[i] == ',') str[i] = ' ';
    }
    std::istringstream input(str);
    std::vector<double> row_datas;
    double data_tmp;
    while (input >> data_tmp) row_datas.emplace_back(data_tmp);
    datas.emplace_back(row_datas);
  }
  QCHECK(datas.size() > 2) << "Import file is empty !";
  infile.close();
  return datas;
}

}  // namespace utils

OpenloopControl::OpenloopControl(double wheel_base, double steer_ratio,
                                 double max_steer_angle)
    : steer_converter_(wheel_base, steer_ratio, max_steer_angle) {
  LoadParameters(FLAGS_control_openloop_path);
  SetOpenloopCmd(openloop_cmd_);
}

void OpenloopControl::SetOpenloopCmd(const OpenloopCmd& openloop_cmd) {
  openloop_cmd_ = openloop_cmd;
  QCHECK(openloop_cmd_.throttle_brake_cmd_size() > 0)
      << "[OpenLoop_Control] lacks aperture or times!";
  QCHECK(openloop_cmd_.acceleration_cmd_size() > 0)
      << "[OpenLoop_Control] lacks acceleration or times!";

  switch (openloop_cmd_.testtype()) {
    case OpenloopCmd::STEERZERO:
      break;
    case OpenloopCmd::STEERSIN:
      QCHECK(openloop_cmd_.has_steer_sin())
          << "[OpenLoop_Control] lacks steer sin or times!";
      break;
    case OpenloopCmd::STEERFOLLOW:
      QCHECK(openloop_cmd_.has_steer_follow())
          << "[OpenLoop_Control] lacks steer step or times!";
      break;
    default:
      break;
  }

  for (int i = 0; i < openloop_cmd_.throttle_brake_cmd_size(); ++i) {
    const int duration_num =
        FloorToInt(openloop_cmd_.throttle_brake_cmd().Get(i).duration_time() /
                   kControlInterval);
    num_aperture_vec_.push_back(duration_num);
    for (int j = 0; j < duration_num; ++j) {
      aperture_vec_.push_back(
          openloop_cmd_.throttle_brake_cmd().Get(i).aperture());
    }
  }

  for (int i = 0; i < openloop_cmd_.acceleration_cmd_size(); ++i) {
    const int duration_num =
        FloorToInt(openloop_cmd_.acceleration_cmd().Get(i).duration_time() /
                   kControlInterval);
    num_acc_vec_.push_back(duration_num);
    for (int j = 0; j < duration_num; ++j) {
      acc_vec_.push_back(
          openloop_cmd_.acceleration_cmd().Get(i).acceleration());
    }
  }

  if (openloop_cmd_.testtype() == OpenloopCmd::STEERFOLLOW) {
    const auto datas =
        utils::ReadFromTxt(openloop_cmd_.steer_follow().data_name());

    for (int i = 1; i < datas.size(); ++i) {
      steer_angle_datas_.push_back(datas[i][1]);
    }
  }
}

void OpenloopControl::Reset() {
  num_seq_ = 0;
  steer_seq_ = 0;
  steer_angle_last_ = 0.0;
}

void OpenloopControl::CreateCommand(ControlCommand* cmd) {
  ProduceGear(cmd);
  ProduceThrottleBrake(cmd);
  ProduceAcceleration(cmd);

  switch (openloop_cmd_.testtype()) {
    case OpenloopCmd::STEERZERO:
      cmd->set_steering_target(0.0);
      cmd->set_steer_angle_bias(0.0);
      cmd->set_curvature(0.0);
      cmd->set_steer_speed_target(0.0);
      break;
    case OpenloopCmd::STEERSIN:
      ProduceSteerSin(cmd);
      break;
    case OpenloopCmd::STEERFOLLOW:
      ProduceSteerFollow(cmd);
      break;
    default:
      break;
  }
  num_seq_++;
}

void OpenloopControl::LoadParameters(const std::string& config_path) {
  // Load proto config;
  QLOG(INFO) << "config_path: " << config_path;
  QCHECK(file_util::TextFileToProto(config_path, &openloop_cmd_))
      << "[OpenLoop_Control] failed to parse proto file!";
}

void OpenloopControl::ProduceGear(ControlCommand* cmd) {
  if (openloop_cmd_.gear_cmd() == -1) {
    cmd->set_gear_location(Chassis::GEAR_REVERSE);
  } else {
    cmd->set_gear_location(Chassis::GEAR_DRIVE);
  }
}

void OpenloopControl::ProduceThrottleBrake(ControlCommand* cmd) {
  const int index_max = aperture_vec_.size() - 1;
  const int num = num_seq_ > index_max ? index_max : num_seq_;
  double aperture = aperture_vec_[num];
  if (aperture >= 0.0) {
    cmd->set_throttle(aperture);
    cmd->set_brake(0.0);
  } else {
    cmd->set_throttle(0.0);
    cmd->set_brake(fabs(aperture));
  }
}

void OpenloopControl::ProduceAcceleration(ControlCommand* cmd) {
  const int index_max = acc_vec_.size() - 1;
  const int num = num_seq_ > index_max ? index_max : num_seq_;
  cmd->set_acceleration(acc_vec_[num]);
}

void OpenloopControl::ProduceSteerSin(ControlCommand* cmd) {
  const int start_num =
      FloorToInt(openloop_cmd_.steer_sin().start_time() * kControlFrequency);
  if (num_seq_ < start_num) {
    cmd->set_steering_target(0.0);
    cmd->set_steer_angle_bias(0.0);
    cmd->set_curvature(0.0);
    cmd->set_steer_speed_target(0.0);
    return;
  }
  const double sample_frequency =
      kControlInterval / openloop_cmd_.steer_sin().sin_period() * 2.0 * M_PI;
  const double cmd_steer_angle = openloop_cmd_.steer_sin().steer_angle() *
                                 std::sin(steer_seq_ * sample_frequency);

  const double cmd_kappa = steer_converter_.SteerAngleToKappa(cmd_steer_angle);
  const double cmd_steer_percentage =
      steer_converter_.KappaToSteerPct(cmd_kappa);
  const double cmd_steer_speed =
      (cmd_steer_angle - steer_angle_last_) * kControlFrequency;
  steer_angle_last_ = cmd_steer_angle;

  cmd->set_steering_target(cmd_steer_percentage);
  cmd->set_steer_angle_bias(0.0);
  cmd->set_curvature(cmd_kappa);
  cmd->set_steer_speed_target(cmd_steer_speed);
  steer_seq_++;
}

void OpenloopControl::ProduceSteerFollow(ControlCommand* cmd) {
  const int start_num =
      FloorToInt(openloop_cmd_.steer_follow().start_time() * kControlFrequency);
  if (num_seq_ < start_num) {
    cmd->set_steering_target(0.0);
    cmd->set_steer_angle_bias(0.0);
    cmd->set_curvature(0.0);
    cmd->set_steer_speed_target(0.0);
    return;
  }
  const int index_max = steer_angle_datas_.size() - 1;
  const int num = steer_seq_ > index_max ? index_max : steer_seq_;
  const double cmd_steer_angle =
      steer_angle_datas_[num] * openloop_cmd_.steer_follow().steer_scale();
  const double cmd_kappa = steer_converter_.SteerAngleToKappa(cmd_steer_angle);
  const double cmd_steer_percentage =
      steer_converter_.KappaToSteerPct(cmd_kappa);
  const double cmd_steer_speed =
      (cmd_steer_angle - steer_angle_last_) * kControlFrequency;
  steer_angle_last_ = cmd_steer_angle;

  cmd->set_steering_target(cmd_steer_percentage);
  cmd->set_steer_angle_bias(0.0);
  cmd->set_curvature(cmd_kappa);
  cmd->set_steer_speed_target(cmd_steer_speed);
  steer_seq_++;
}

}  // namespace control
}  // namespace qcraft
