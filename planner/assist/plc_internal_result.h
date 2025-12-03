#ifndef ONBOARD_PLANNER_ASSIST_RLC_INTERNAL_RESULT_H_
#define ONBOARD_PLANNER_ASSIST_RLC_INTERNAL_RESULT_H_

#include <memory>
#include <string>
#include <vector>

#include "onboard/maps/lane_path.h"
#include "onboard/planner/assist/proto/plc_result.pb.h"
#include "onboard/proto/autonomy_state.pb.h"

namespace qcraft::planner {

struct PlcInternalResult {
  PlcInternalStatus status = PlcInternalStatus::OK;

  std::vector<std::string> unsafe_object_ids;
  std::optional<bool> left_solid_boundary = std::nullopt;

  mapping::LanePath preferred_lane_path;
  DriverAction::LaneChangeCommand lane_change_command =
      DriverAction::LC_CMD_NONE;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ASSIST_RLC_INTERNAL_RESULT_H_
