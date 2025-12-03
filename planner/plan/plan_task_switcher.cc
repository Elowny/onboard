#include "onboard/planner/plan/plan_task_switcher.h"

#include <limits>
#include <optional>
#include <ostream>

#include "absl/time/time.h"

#include "common/proto/qacc.pb.h"
#include "common/proto/qalc.pb.h"
#include "common/proto/qlcc.pb.h"

#include "onboard/async/future.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_path.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/assist/tja_state.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/plan/async_planner_util.h"
#include "onboard/planner/plan/plan_task_helper.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_state_util.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::planner {

namespace {

SwitchTaskResult UpdateDriverlessPlan(
    bool rerouted, const RouteManagerOutput& route_output,
    const PlannerSemanticMapManager& psmm,
    const std::deque<PlanTask>& current_task_queue) {
  if (rerouted) {
    return SwitchTaskResult{
        .switched = false,
        .new_task_queue =
            CreatePlanTasksQueueFromRoutingResult(route_output, psmm)};
  }

  return SwitchTaskResult{.switched = false,
                          .new_task_queue = current_task_queue};
}

SwitchTaskResult DynamicSwitchPlanTask(
    AssistStateProto::AssistDriveSystemState assist_state,
    const std::deque<PlanTask>& current_task_queue, bool rerouted) {
  switch (assist_state) {
    case AssistStateProto::ASSIST_OFF:
    case AssistStateProto::ASSIST_NOT_READY:
    case AssistStateProto::ASSIST_LCC_READY:
    case AssistStateProto::ASSIST_LCC_ACTIVE:
      return SwitchTaskResult{
          .switched = current_task_queue.empty() ||
                      current_task_queue.front().type() != ALCC_PLAN,
          .new_task_queue = {PlanTask(ALCC_PLAN)}};

    case AssistStateProto::ASSIST_ACC_READY:
    case AssistStateProto::ASSIST_ACC_ACTIVE:
      return SwitchTaskResult{
          .switched = current_task_queue.empty() ||
                      current_task_queue.front().type() != ACC_PLAN,
          .new_task_queue = {PlanTask(ACC_PLAN)}};

    case AssistStateProto::ASSIST_NOA_READY:
    case AssistStateProto::ASSIST_NOA_ACTIVE: {
      const bool switched = (current_task_queue.empty() ||
                             current_task_queue.front().type() == ACC_PLAN ||
                             current_task_queue.front().type() == ALCC_PLAN);

      return SwitchTaskResult{
          .switched = switched,
          .new_task_queue =
              (switched || rerouted)
                  ? std::deque<PlanTask>({PlanTask(ON_ROAD_CRUISE_PLAN)})
                  : current_task_queue};
    }
    case AssistStateProto::ASSIST_APA_ACTIVE: {
      return SwitchTaskResult{
          .switched = (current_task_queue.empty() ||
                       current_task_queue.front().type() != APA_PLAN),
          .new_task_queue = {PlanTask(APA_PLAN)}};
    }
  }
}

SwitchTaskResult SwitchAssistDrivePlan(
    AssistStateProto::AssistDriveSystemState assist_state,
    const std::deque<PlanTask>& current_task_queue,
    const RouteManagerOutput& route_output, bool rerouted, int task_init_type) {
  switch (task_init_type) {
    case 0: {  // Run NOA only.
      if (route_output.update_id != kInvalidRouteUpdateId &&
          HasValidRouteResults(route_output)) {
        return SwitchTaskResult{
            .switched = false,
            .new_task_queue =
                rerouted ? std::deque<PlanTask>({PlanTask(ON_ROAD_CRUISE_PLAN)})
                         : current_task_queue};

      } else {
        return SwitchTaskResult{.switched = false, .new_task_queue = {}};
      }
    }
    case 1:  // Dynamic switch plan task by assist state.
      return DynamicSwitchPlanTask(assist_state, current_task_queue, rerouted);
    case 2:  // Run ACC only.
      return SwitchTaskResult{.switched = false,
                              .new_task_queue = {PlanTask(ACC_PLAN)}};
    case 3:  // Run LCC only.
      return SwitchTaskResult{.switched = false,
                              .new_task_queue = {PlanTask(ALCC_PLAN)}};
    case 4:  // Mapless NOA
      return SwitchTaskResult{.switched = false,
                              .new_task_queue = {PlanTask(MAPLESS_NOA)}};
    case 5:  // Run L2 APA (including parking and parking out).
      return SwitchTaskResult{.switched = false,
                              .new_task_queue = {PlanTask(APA_PLAN)}};
    default: {
      QLOG_EVERY_N_SEC(ERROR, 3)
          << "Input task_init_type " << task_init_type
          << " is invalid, will enter dynamic switch. Please confirm config.";
      return DynamicSwitchPlanTask(assist_state, current_task_queue, rerouted);
    }
  }
}

bool IsNoaPlanTask(PlanTaskType task) {
  switch (task) {
    case ON_ROAD_CRUISE_PLAN:
    case OFF_ROAD_PLAN:
    case UTURN_PLAN:
    case BLOCKED_PLAN:
      return true;
    case ALCC_PLAN:
    case ACC_PLAN:
    case MAPLESS_NOA:
    case APA_PLAN:
      return false;
  }
}

void FakeAlccOnlyLowFreqResults(AsyncMultiTaskEstOutput* low_freq_result) {
  if (low_freq_result == nullptr) return;

  low_freq_result->alc_state = ALC_STANDBY_ENABLE;
  low_freq_result->lc_direction = LCD_NONE;
  low_freq_result->origin_lane_path =
      !low_freq_result->est_output.scheduler_output.drive_passage.empty()
          ? low_freq_result->est_output.scheduler_output.drive_passage
                .lane_path()
          : mapping::LanePath();
  low_freq_result->target_lane_path = low_freq_result->origin_lane_path;
}

void UpdateTransitionStates(AsyncPlannerState* async_planner_state) {
  if (async_planner_state->secondary_counter.has_value()) {
    async_planner_state->counter = *async_planner_state->secondary_counter;
  }

  async_planner_state->task_transition = false;
  async_planner_state->secondary_counter = std::nullopt;
  if (async_planner_state->future_multi_task_est_status.IsValid()) {
    async_planner_state->task_transition = true;
  } else {
    async_planner_state->secondary_counter = kAsyncCounterInitVal;
  }
}

void FillAlccRelatedAssistState(const AsyncMultiTaskEstOutput* low_freq_result,
                                AssistPlanStateProto* assist_plan_state) {
  if (low_freq_result == nullptr) return;

  low_freq_result->origin_lane_path.ToProto(
      assist_plan_state->mutable_origin_lane_path());
  low_freq_result->target_lane_path.ToProto(
      assist_plan_state->mutable_target_lane_path());
  assist_plan_state->set_alc_state(low_freq_result->alc_state);
  assist_plan_state->set_lc_direction(low_freq_result->lc_direction);
}

}  // namespace

SwitchTaskResult SwitchPlanTask(
    int run_mode, int task_init_type,
    const std::deque<PlanTask>& current_task_queue,
    const std::shared_ptr<PlannerSemanticMapManager>& psmm,
    AssistStateProto::AssistDriveSystemState assist_state,
    const RouteManagerOutput& route_output, bool rerouted) {
  switch (run_mode) {
    case 0: {  // L4
      return UpdateDriverlessPlan(rerouted, route_output, *psmm,
                                  current_task_queue);
    }
    case 1: {
      return SwitchAssistDrivePlan(assist_state, current_task_queue,
                                   route_output, rerouted, task_init_type);
    }  // assist drive
    default: {
      QLOG_EVERY_N_SEC(ERROR, 3)
          << "Input run mode " << run_mode
          << " is invalid, will enter assist drive. Please confirm config.";
      return SwitchAssistDrivePlan(assist_state, current_task_queue,
                                   route_output, rerouted, task_init_type);
    }
  }
}

void UpdatePlannerStateOnTaskSwitch(PlanTaskType prev_task,
                                    PlanTaskType new_task,
                                    int cruise_async_low_freq_cycle_iterations,
                                    int alcc_async_low_freq_cycle_iterations,
                                    PlannerState* planner_state) {
  planner_state->async_planner_state.pending_lane_change_command =
      DriverAction::LC_CMD_NONE;
  planner_state->async_planner_state.pending_alc_confirmation = std::nullopt;
  planner_state->selector_state.Reset();
  planner_state->tja_state.Reset();

  // NOTE: Delete after aeb planner is used by all tasks.
  planner_state->previously_triggered_aeb = false;

  if (prev_task == ALCC_PLAN) {
    planner_state->online_map_drift_buffer.clear();
  }
  if (new_task == ACC_PLAN) {
    planner_state->async_planner_state = AsyncPlannerState();
    planner_state->prev_target_lane_path.Clear();
    planner_state->lane_change_state.Clear();
    planner_state->prev_low_freq_psmm = nullptr;
  }

  // Function upgrade.
  if (prev_task == ACC_PLAN && new_task == ALCC_PLAN) {
    planner_state->async_planner_state.secondary_counter = kAsyncCounterInitVal;
    return;
  }

  if (prev_task == ACC_PLAN && IsNoaPlanTask(new_task)) {
    planner_state->async_planner_state.secondary_counter = kAsyncCounterInitVal;
    return;
  }

  if (prev_task == ALCC_PLAN && IsNoaPlanTask(new_task)) {
    if (IsPlannerAsync(cruise_async_low_freq_cycle_iterations)) {
      UpdateTransitionStates(&planner_state->async_planner_state);
    } else {
      planner_state->async_planner_state.secondary_counter =
          kAsyncCounterInitVal;
    }
    return;
  }

  // Function downgrade.
  if (IsNoaPlanTask(prev_task)) {
    planner_state->prev_route_sections.Clear();
    planner_state->preferred_lane_path.Clear();
    planner_state->prev_lane_path_before_lc.Clear();
    planner_state->prev_length_along_route = std::numeric_limits<double>::max();
    planner_state->prev_max_reach_length = std::numeric_limits<double>::max();

    if (new_task == ALCC_PLAN) {
      if (IsPlannerAsync(alcc_async_low_freq_cycle_iterations)) {
        FakeAlccOnlyLowFreqResults(planner_state->async_planner_state
                                       .latest_multi_task_est_result.get());
        UpdateTransitionStates(&planner_state->async_planner_state);
      } else {
        planner_state->async_planner_state = AsyncPlannerState();
      }
      FillAlccRelatedAssistState(
          planner_state->async_planner_state.latest_multi_task_est_result.get(),
          &planner_state->assist_plan_state);
      return;
    }
  }
}

void ResetAssistPlanStateByTaskType(PlanTaskType type,
                                    AssistPlanStateProto* assist_plan_state,
                                    ExternalCommandStatus* ext_cmd_status) {
  ext_cmd_status->lane_change_command = DriverAction::LC_CMD_NONE;
  ext_cmd_status->plc_prepare_start_time = std::nullopt;
  switch (type) {
    case ON_ROAD_CRUISE_PLAN:
    case OFF_ROAD_PLAN:
    case UTURN_PLAN:
    case BLOCKED_PLAN: {
      ResetAlccAssistPlanState(assist_plan_state);
      ext_cmd_status->lcc_state = QLCCState::LCC_OFF;
      ext_cmd_status->acc_state = QACCState::ACC_OFF;
      return;
    }
    case ALCC_PLAN: {
      ResetAlccAssistPlanState(assist_plan_state);
      ext_cmd_status->acc_state = QACCState::ACC_OFF;
      return;
    }
    case ACC_PLAN: {
      ResetAccAssistPlanState(assist_plan_state);
      ext_cmd_status->lcc_state = QLCCState::LCC_OFF;
      return;
    }
    case MAPLESS_NOA:
    case APA_PLAN: {
      // TODO(weijun): reset
      return;
    }
  }
}

}  // namespace qcraft::planner
