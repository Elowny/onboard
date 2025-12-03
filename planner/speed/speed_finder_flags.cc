#include "onboard/planner/speed/speed_finder_flags.h"

DEFINE_bool(
    planner_draw_st_boundary_canvas, false,
    "Whether to export upstream input speed profile to st-graph chart.");

DEFINE_bool(planner_send_speed_path_chart_data, false,
            "Whether to send speed path chart data.");

DEFINE_bool(planner_print_speed_finder_time_stats, false,
            "Whether to print speed finder time stats.");

DEFINE_bool(planner_send_path_data_to_debug, false,
            "Whether to render path in vantage.");

DEFINE_bool(planner_send_speed_optimizer_debug, false,
            "Whether to send speed optimizer debug.");
DEFINE_bool(planner_send_interactive_speed_to_chart, false,
            "Whether to send interactive speed profiles to chart.");

DEFINE_bool(planner_enable_moving_close_traj_speed_limit, true,
            "Whether to enable moving close object speed_limit.");
