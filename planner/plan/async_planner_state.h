#ifndef ONBOARD_PLANNER_PLAN_ASYNC_PLANNER_STATE_H_
#define ONBOARD_PLANNER_PLAN_ASYNC_PLANNER_STATE_H_

#include <memory>
#include <string>
#include <vector>

#include "onboard/async/future.h"
#include "onboard/planner/assist/plc_internal_result.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/plan/path_bounded_est_planner_output.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/async_planner_state.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"

namespace qcraft {
namespace planner {

struct AsyncMultiTaskEstOutput {
  PlannerStatus est_status;
  EstPlannerOutput est_output;
  EstPlannerDebug est_debug;
  vis::vantage::ChartDataBundleProto chart_data_bundle;
  std::vector<PathPoint> st_path_points_global_including_past;

  std::optional<RouteTargetInfo> route_target_info = std::nullopt;

  std::optional<PlcInternalResult> plc_result = std::nullopt;
  std::optional<NudgeOjbectInfo> nudge_object_info;

  std::shared_ptr<const PlannerSemanticMapManager> low_freq_psmm = nullptr;
  std::shared_ptr<const DrivingMapTopo> driving_map_topo = nullptr;

  SelectorState selector_state;
  std::optional<SelectorOutput> selector_output = std::nullopt;

  // Alcc only.
  QALCState alc_state;
  LaneChangeDirection lc_direction;
  mapping::LanePath origin_lane_path;
  mapping::LanePath target_lane_path;
  int64_t online_map_id = kInvalidOnlineMapId;

  // Both cruise and alcc task will assign a value to
  // 'st_path_points_including_past'. Especially when task downgrade from cruise
  // to alcc, 'st_path_points_including_past' will exhibit continuity.
  std::vector<PathPoint> st_path_points_including_past;
};

// The state lives across iteration for async path planner.
struct AsyncPlannerState {
  int counter = kAsyncCounterInitVal;

  // For async task transition.
  bool task_transition = false;
  std::optional<int> secondary_counter = std::nullopt;

  // For paddle lane change.
  DriverAction::LaneChangeCommand pending_lane_change_command =
      DriverAction::LC_CMD_NONE;
  // For auto lane change user confirmation.
  std::optional<bool> pending_alc_confirmation = std::nullopt;

  bool wait_path_switch_route = false;
  bool wait_speed_switch_route = false;

  Future<PlannerStatus> future_multi_task_est_status;
  std::shared_ptr<PathBoundedEstPlannerOutput> future_multi_task_est_result;

  std::shared_ptr<AsyncMultiTaskEstOutput> latest_multi_task_est_result;

  // For update low est result in high freq.
  SmoothedReferenceLineResultMap low_freq_smooth_result_map;
  TrafficLightInfoMap low_freq_tl_info_map;

  bool operator==(const AsyncPlannerState& other) const {
    return counter == other.counter &&
           task_transition == other.task_transition &&
           secondary_counter == other.secondary_counter &&
           pending_lane_change_command == other.pending_lane_change_command &&
           pending_alc_confirmation == other.pending_alc_confirmation &&
           wait_path_switch_route == other.wait_path_switch_route &&
           wait_speed_switch_route == other.wait_speed_switch_route;
  }
  bool operator!=(const AsyncPlannerState& other) const {
    return !(*this == other);
  }

  void FromProto(const AsyncPlannerStateProto& proto) {
    counter = proto.counter();
    task_transition = proto.task_transition();
    secondary_counter = proto.has_secondary_counter()
                            ? std::optional<int>(proto.secondary_counter())
                            : std::nullopt;
    pending_alc_confirmation =
        proto.has_pending_alc_confirmation()
            ? std::optional<bool>(proto.pending_alc_confirmation())
            : std::nullopt;
    wait_path_switch_route = proto.has_wait_path_switch_route()
                                 ? proto.wait_path_switch_route()
                                 : false;
    wait_speed_switch_route = proto.has_wait_speed_switch_route()
                                  ? proto.wait_speed_switch_route()
                                  : false;
  }
  void ToProto(AsyncPlannerStateProto* proto) const {
    proto->set_counter(counter);
    proto->set_task_transition(task_transition);
    if (secondary_counter.has_value()) {
      proto->set_secondary_counter(*secondary_counter);
    }
    if (pending_alc_confirmation.has_value()) {
      proto->set_pending_alc_confirmation(*pending_alc_confirmation);
    }
    proto->set_pending_lane_change_command(pending_lane_change_command);
    proto->set_wait_path_switch_route(wait_path_switch_route);
    proto->set_wait_speed_switch_route(wait_speed_switch_route);
  }
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_ASYNC_PLANNER_STATE_H_
