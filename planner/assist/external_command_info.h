#ifndef ONBOARD_PLANNER_ASSIST_EXTERNAL_COMMAND_INFO_H_
#define ONBOARD_PLANNER_ASSIST_EXTERNAL_COMMAND_INFO_H_

#include <deque>
#include <optional>
#include <queue>
#include <string>

#include "absl/time/time.h"

#include "common/proto/qacc.pb.h"
#include "common/proto/qalc.pb.h"
#include "common/proto/qlcc.pb.h"

#include "onboard/planner/assist/proto/external_command_status.pb.h"
#include "onboard/planner/router/proto/route_external_command.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/remote_assist_common.pb.h"

namespace qcraft::planner {
struct ExternalCommandOutput {
  std::optional<double> cruising_speed_limit = std::nullopt;
  std::optional<double> current_lane_width = std::nullopt;
  std::optional<double> current_lane_length = std::nullopt;
  std::optional<double> current_lane_average_curvature_radius = std::nullopt;
  std::optional<double> distance_to_toll = std::nullopt;
  std::optional<double> distance_to_traffic_light = std::nullopt;
  std::optional<bool> is_av_overlap_boundary = std::nullopt;
  std::optional<bool> is_valid_both_side_boundary = std::nullopt;
  std::optional<bool> lane_path_lost = std::nullopt;
  std::optional<bool> is_av_in_emergency_lane = std::nullopt;
  bool is_route_lane_change_fail = false;

  ExternalRouterCommand planner_to_router_command = ExternalRouterCommand::NONE;

  std::optional<std::string> apa_parking_spot_id = std::nullopt;
  bool is_acc_engage_only = false;

  void ToProto(PlannerExternalCommandStatusProto* proto) const;
  void FromProto(const PlannerExternalCommandStatusProto& proto);
  bool operator==(const ExternalCommandOutput& other) const;
};

// Inherited from TeleopState
struct ExternalCommandStatus {
  bool enable_pull_over = false;
  // Traffic light stopping enable status.
  bool enable_traffic_light_stopping = true;
  // To be removed.
  bool enable_lc_objects = true;

  bool override_left_blinker_on = false;
  bool override_right_blinker_on = false;
  bool override_emergency_blinker_on = false;

  bool enable_stop_polyline_stopping = false;

  // Door override. If override_door_open has value, the door is forced
  // open/close according to override_door_open. Door opening controlled by
  // planner has not yet been implemented.
  std::optional<bool> override_door_open = std::nullopt;

  std::optional<double> brake_to_stop = std::nullopt;

  QLCCState lcc_state = QLCCState::LCC_OFF;
  QALCState alc_state = QALCState::ALC_OFF;
  QACCState acc_state = QACCState::ACC_OFF;

  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;

  // NOTE(jiayu): Receive lcc/noa speed limit from HMI.
  std::optional<double> lcc_cruising_speed_limit = std::nullopt;
  std::optional<double> noa_cruising_speed_limit = std::nullopt;
  std::optional<bool> noa_need_lane_change_confirmation = std::nullopt;
  std::optional<bool> alc_confirmation = std::nullopt;

  QRunEvent::LccFollowingDistanceLevel following_distance_level =
      QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM;

  DriverAction::LaneChangeCommand lane_change_command =
      DriverAction::LC_CMD_NONE;
  std::optional<absl::Time> plc_prepare_start_time = std::nullopt;

  std::optional<std::string> apa_parking_spot_id = std::nullopt;

  // For output only, should be cleared before each iteration.
  ExternalCommandOutput output;

  PlannerExternalCommandStatusProto ToProto() const;
  void FromProto(const PlannerExternalCommandStatusProto& proto);

  bool operator==(const ExternalCommandStatus& other) const;
};

struct ExternalCommandQueue {
  std::deque<DriverAction> pending_driver_actions;
  std::queue<LaneChangeRequestProto> pending_lane_change_requests;
  std::queue<OutOfBlockedRoadRequestProto> pending_out_of_blocked_road_requests;
  // TODO(weijun): Add Alc commands.

  void Clear() {
    pending_driver_actions = {};
    pending_lane_change_requests = {};
    pending_out_of_blocked_road_requests = {};
  }
};

struct ExternalCommandInfo {
  ExternalCommandStatus status;
  ExternalCommandQueue queue;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ASSIST_EXTERNAL_COMMAND_INFO_H_
