#ifndef ONBOARD_PLANNER_PLAN_ACC_ACC_FLAGS_H_
#define ONBOARD_PLANNER_PLAN_ACC_ACC_FLAGS_H_

#include "gflags/gflags.h"

DECLARE_bool(planner_acc_corridor_draw_target_lane_ref_path);

DECLARE_bool(planner_acc_corridor_draw_target_lane_choice_debug);

DECLARE_bool(planner_draw_acc_target_st_trajectories);

DECLARE_bool(planner_acc_plan_for_all_sources);

DECLARE_bool(planner_acc_avoid_crowded_scene_cutin);

#endif  // ONBOARD_PLANNER_PLAN_ACC_ACC_FLAGS_H_
