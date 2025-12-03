#include "onboard/planner/plan/plan_task_switcher.h"

#include <initializer_list>

#include "absl/container/flat_hash_map.h"

#include "gtest/gtest.h"

#include "common/proto/qacc.pb.h"
#include "common/proto/qalc.pb.h"
#include "common/proto/qlcc.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::planner {

namespace {

// NOLINTNEXTLINE(readability-function-size)
TEST(SwitchPlanTask, SwitchAssistDrivePlanTest) {
  const auto psmm = CreateDojoTestPSMMSharedPtr();

  // Dynamic switch test, ALCC_PLAN test.
  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/1, std::deque<PlanTask>(), psmm,
        AssistStateProto::ASSIST_OFF, RouteManagerOutput(), /*rerouted=*/false);

    EXPECT_TRUE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ALCC_PLAN);
  }

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/1, {PlanTask(ALCC_PLAN)}, psmm,
        AssistStateProto::ASSIST_NOT_READY, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ALCC_PLAN);
  }

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/1, {PlanTask(ACC_PLAN)}, psmm,
        AssistStateProto::ASSIST_ACC_READY, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ACC_PLAN);
  }

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/1, {PlanTask(ALCC_PLAN)}, psmm,
        AssistStateProto::ASSIST_LCC_READY, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ALCC_PLAN);
  }

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/1, {PlanTask(ON_ROAD_CRUISE_PLAN)},
        psmm, AssistStateProto::ASSIST_NOA_READY, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ON_ROAD_CRUISE_PLAN);
  }

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/1, {PlanTask(ACC_PLAN)}, psmm,
        AssistStateProto::ASSIST_LCC_ACTIVE, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_TRUE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ALCC_PLAN);
  }

  // Dynamic switch test, ACC_PLAN test.
  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/1, {PlanTask(ALCC_PLAN)}, psmm,
        AssistStateProto::ASSIST_ACC_ACTIVE, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_TRUE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ACC_PLAN);
  }

  // Dynamic switch test, NOA test.
  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/1, {PlanTask(ALCC_PLAN)}, psmm,
        AssistStateProto::ASSIST_NOA_ACTIVE, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_TRUE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ON_ROAD_CRUISE_PLAN);
  }

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/1, {PlanTask(ON_ROAD_CRUISE_PLAN)},
        psmm, AssistStateProto::ASSIST_NOA_ACTIVE, RouteManagerOutput(),
        /*rerouted=*/true);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ON_ROAD_CRUISE_PLAN);
  }

  // Run Acc only test.
  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/2, /*current_task_queue=*/{}, psmm,
        AssistStateProto::ASSIST_ACC_ACTIVE, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ACC_PLAN);
  }

  // Run Alcc only test.
  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/3, /*current_task_queue=*/{}, psmm,
        AssistStateProto::ASSIST_LCC_ACTIVE, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ALCC_PLAN);
  }

  const auto mock_route_output = []() -> RouteManagerOutput {
    RouteManagerOutput route_output;
    route_output.update_id = 1;
    route_output.route_sections_from_current = RouteSections(
        /*start_fraction=*/0.0, /*end_fraction=*/1.0,
        {mapping::SectionId(12401)},
        mapping::LanePoint(mapping::ElementId(2448), /*fraction=*/1.0));

    route_output.route_navi_info.route_lane_info_map.emplace(
        mapping::ElementId(2448), RouteNaviInfo::RouteLaneInfo());
    return route_output;
  };

  // Run NOA only test.
  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/0, {PlanTask(ON_ROAD_CRUISE_PLAN)},
        psmm, AssistStateProto::ASSIST_NOA_ACTIVE, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_FALSE(result.switched);
    EXPECT_TRUE(result.new_task_queue.empty());
  }

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/0, {}, psmm,
        AssistStateProto::ASSIST_NOA_ACTIVE, mock_route_output(),
        /*rerouted=*/true);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ON_ROAD_CRUISE_PLAN);
  }

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/0, {PlanTask(ON_ROAD_CRUISE_PLAN)},
        psmm, AssistStateProto::ASSIST_NOA_ACTIVE, mock_route_output(),
        /*rerouted=*/false);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ON_ROAD_CRUISE_PLAN);
  }
}

TEST(SwitchPlanTask, UpdateDriverlessPlanTest) {
  const auto psmm = CreateDojoTestPSMMSharedPtr();

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/0, /*task_init_type=*/0, {PlanTask(ON_ROAD_CRUISE_PLAN)},
        psmm, AssistStateProto::ASSIST_OFF, RouteManagerOutput(),
        /*rerouted=*/false);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ON_ROAD_CRUISE_PLAN);
  }

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/0, /*task_init_type=*/0, {PlanTask(OFF_ROAD_PLAN)}, psmm,
        AssistStateProto::ASSIST_OFF, RouteManagerOutput(),
        /*rerouted=*/true);

    EXPECT_FALSE(result.switched);
    EXPECT_EQ(result.new_task_queue.front().type(), ON_ROAD_CRUISE_PLAN);
  }
}

TEST(SwitchPlanTask, ErrorPlanTest) {
  const auto psmm = CreateDojoTestPSMMSharedPtr();

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/3, /*task_init_type=*/0, {PlanTask(ON_ROAD_CRUISE_PLAN)},
        psmm, AssistStateProto::ASSIST_OFF, RouteManagerOutput(),
        /*rerouted=*/false);
  }

  {
    const auto result = SwitchPlanTask(
        /*run_mode=*/1, /*task_init_type=*/5, {PlanTask(ON_ROAD_CRUISE_PLAN)},
        psmm, AssistStateProto::ASSIST_OFF, RouteManagerOutput(),
        /*rerouted=*/false);
  }
}

TEST(SwitchPlanTask, ResetAssistPlanStateByTaskTypeTest) {
  // Noa/L4 test.
  {
    AssistPlanStateProto assist_plan_state;
    ExternalCommandStatus ext_cmd_status;
    for (const auto type :
         {ON_ROAD_CRUISE_PLAN, OFF_ROAD_PLAN, UTURN_PLAN, BLOCKED_PLAN}) {
      ResetAssistPlanStateByTaskType(type, &assist_plan_state, &ext_cmd_status);
      EXPECT_EQ(ext_cmd_status.lcc_state, QLCCState::LCC_OFF);
      EXPECT_EQ(ext_cmd_status.acc_state, QACCState::ACC_OFF);
      EXPECT_EQ(assist_plan_state.alc_state(), QALCState::ALC_STANDBY_ENABLE);
      EXPECT_EQ(assist_plan_state.lc_direction(),
                LaneChangeDirection::LCD_NONE);
      EXPECT_FALSE(assist_plan_state.has_origin_lane_path());
      EXPECT_FALSE(assist_plan_state.has_target_lane_path());
    }
  }

  // Alcc test.
  {
    AssistPlanStateProto assist_plan_state;
    ExternalCommandStatus ext_cmd_status;
    for (const auto type : {ALCC_PLAN}) {
      ResetAssistPlanStateByTaskType(type, &assist_plan_state, &ext_cmd_status);
      EXPECT_EQ(ext_cmd_status.acc_state, QACCState::ACC_OFF);
      EXPECT_EQ(assist_plan_state.alc_state(), QALCState::ALC_STANDBY_ENABLE);
      EXPECT_EQ(assist_plan_state.lc_direction(),
                LaneChangeDirection::LCD_NONE);
      EXPECT_FALSE(assist_plan_state.has_origin_lane_path());
      EXPECT_FALSE(assist_plan_state.has_target_lane_path());
    }
  }

  // Acc test.
  {
    AssistPlanStateProto assist_plan_state;
    ExternalCommandStatus ext_cmd_status;
    for (const auto type : {ACC_PLAN}) {
      ResetAssistPlanStateByTaskType(type, &assist_plan_state, &ext_cmd_status);
      EXPECT_EQ(ext_cmd_status.lcc_state, QLCCState::LCC_OFF);
      EXPECT_FALSE(assist_plan_state.has_acc_task());
    }
  }
}

TEST(SwitchPlanTask, UpdatePlannerStateOnTaskSwitch) {
  // TODO(jiayu): Add test util to scheduler multi alcc/cruise taskss.
  // TODO(jiayu): Check tja_state.
  constexpr int kCruiseAsyncLowFreqCycleIterations = 2;
  constexpr int kAlccAsyncLowFreqCycleIterations = 2;
  // ACC -> ALCC
  {
    PlannerState planner_state;
    UpdatePlannerStateOnTaskSwitch(
        /*prev_task*/ ACC_PLAN,
        /*new_task*/ ALCC_PLAN, kCruiseAsyncLowFreqCycleIterations,
        kAlccAsyncLowFreqCycleIterations, &planner_state);
    EXPECT_EQ(planner_state.async_planner_state.secondary_counter,
              kAsyncCounterInitVal);
  }

  // ACC -> NOA
  {
    PlannerState planner_state;
    for (const auto task_type :
         {ON_ROAD_CRUISE_PLAN, OFF_ROAD_PLAN, UTURN_PLAN, BLOCKED_PLAN}) {
      UpdatePlannerStateOnTaskSwitch(
          /*prev_task*/ ACC_PLAN,
          /*new_task*/ task_type, kCruiseAsyncLowFreqCycleIterations,
          kAlccAsyncLowFreqCycleIterations, &planner_state);
      EXPECT_EQ(planner_state.async_planner_state.secondary_counter,
                kAsyncCounterInitVal);
    }
  }

  // ALCC -> NOA
  // Async test, task future_multi_task_est_status is not valid.
  {
    PlannerState planner_state;
    for (const auto task_type :
         {ON_ROAD_CRUISE_PLAN, OFF_ROAD_PLAN, UTURN_PLAN, BLOCKED_PLAN}) {
      UpdatePlannerStateOnTaskSwitch(
          /*prev_task*/ ALCC_PLAN,
          /*new_task*/ task_type, kCruiseAsyncLowFreqCycleIterations,
          kAlccAsyncLowFreqCycleIterations, &planner_state);
      EXPECT_FALSE(planner_state.async_planner_state.task_transition);
      EXPECT_EQ(planner_state.async_planner_state.secondary_counter,
                kAsyncCounterInitVal);
    }
  }

  // NOA -> ALCC
  {
    PlannerState planner_state;
    for (const auto task_type :
         {ON_ROAD_CRUISE_PLAN, OFF_ROAD_PLAN, UTURN_PLAN, BLOCKED_PLAN}) {
      UpdatePlannerStateOnTaskSwitch(
          /*prev_task*/ task_type,
          /*new_task*/ ALCC_PLAN, kCruiseAsyncLowFreqCycleIterations,
          kAlccAsyncLowFreqCycleIterations, &planner_state);
      EXPECT_FALSE(planner_state.async_planner_state.task_transition);
      EXPECT_EQ(planner_state.async_planner_state.secondary_counter,
                kAsyncCounterInitVal);
    }
  }

  // NOA -> ACC
  {
    PlannerState planner_state;
    for (const auto task_type :
         {ON_ROAD_CRUISE_PLAN, OFF_ROAD_PLAN, UTURN_PLAN, BLOCKED_PLAN}) {
      UpdatePlannerStateOnTaskSwitch(
          /*prev_task*/ task_type,
          /*new_task*/ ACC_PLAN, kCruiseAsyncLowFreqCycleIterations,
          kAlccAsyncLowFreqCycleIterations, &planner_state);
      EXPECT_TRUE(planner_state.prev_target_lane_path.IsEmpty());
      EXPECT_TRUE(planner_state.prev_low_freq_psmm == nullptr);
    }
  }

  // ALCC -> ACC
  {
    PlannerState planner_state;
    UpdatePlannerStateOnTaskSwitch(
        /*prev_task*/ ALCC_PLAN,
        /*new_task*/ ACC_PLAN, kCruiseAsyncLowFreqCycleIterations,
        kAlccAsyncLowFreqCycleIterations, &planner_state);
    EXPECT_TRUE(planner_state.prev_target_lane_path.IsEmpty());
    EXPECT_TRUE(planner_state.prev_low_freq_psmm == nullptr);
    EXPECT_TRUE(planner_state.online_map_drift_buffer.empty());
  }
}
}  // namespace
}  // namespace qcraft::planner
