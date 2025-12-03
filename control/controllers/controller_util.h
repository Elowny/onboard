#ifndef ONBOARD_CONTROL_CONTROLLER_CONTROLLER_UTIL_H_
#define ONBOARD_CONTROL_CONTROLLER_CONTROLLER_UTIL_H_

#include <vector>

#include "boost/circular_buffer.hpp"

namespace qcraft::control {

inline bool IsCmdBackToZero(const std::vector<double>& cmd) {
  for (size_t i = 0; i + 1 < cmd.size(); ++i) {
    const double cmd_current = cmd[i];
    const double cmd_next = cmd[i + 1];
    const double delta_cmd = cmd_next - cmd_current;
    if (cmd_current * delta_cmd > 0 || cmd_current * cmd_next < 0) {
      return false;
    }
  }
  return true;
}

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROLLER_CONTROLLER_UTIL_H_
