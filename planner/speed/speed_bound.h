#ifndef ONBOARD_PLANNER_SPEED_SPEED_BOUND_H_
#define ONBOARD_PLANNER_SPEED_SPEED_BOUND_H_

#include <map>
#include <string>
#include <vector>

#include "onboard/planner/speed/proto/speed_finder.pb.h"

namespace qcraft::planner {

struct SpeedBoundWithInfo {
  double bound = 0.0;
  std::string info;
};
using SpeedBoundMapType =
    std::map<SpeedLimitTypeProto::Type, std::vector<SpeedBoundWithInfo>>;

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_BOUND_H_
