#include "onboard/planner/plan/acc/acc_flags.h"

DEFINE_bool(planner_acc_corridor_draw_target_lane_ref_path, false,
            "Whether to draw reference converging path and probing traj. (with "
            "map case)");

DEFINE_bool(planner_acc_corridor_draw_target_lane_choice_debug, false,
            "Whether to draw target lane choice visualization for debugging.");

DEFINE_bool(planner_draw_acc_target_st_trajectories, false,
            "Whether we draw target spacetime trajectories.");

DEFINE_bool(planner_acc_plan_for_all_sources, false,
            "Whether calculate all solutions in acc task internal. (set true "
            "for debug)");

DEFINE_bool(planner_acc_avoid_crowded_scene_cutin, false,
            "Whether to avoid crowded scene cutin.");
