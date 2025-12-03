#pragma once

#include "gflags/gflags.h"

DECLARE_bool(random_route);

// Example: destinations { name: DEST_M4_WORK } destinations { name:
// DEST_M4_HOTEL } destinations { name: DEST_M4_GARAGE_W }
DECLARE_string(route);

DECLARE_string(route_str);

DECLARE_string(multi_stops_route);

DECLARE_bool(route_recover_multi_stops_from_log);

DECLARE_bool(route_recover_destinations_from_log);

// The default route params file path
DECLARE_string(route_default_params_file);

DECLARE_bool(route_auto_next_stop);

DECLARE_bool(route_match_filter_enable);

// Allow freeway or city express
DECLARE_bool(route_allow_express_way);

DECLARE_bool(route_recover_request_from_destinations);

DECLARE_bool(router_send_route_core_state_to_canvas);

DECLARE_double(route_mainloop_period);

DECLARE_double(route_arrive_distance);

DECLARE_bool(route_enable_alternative_path);

DECLARE_bool(router_enable_off_road_search);

// The time gap between OnInit() and LiteModule::ok()
DECLARE_double(route_wait_ok_secs);

DECLARE_bool(route_noa_mode);

DECLARE_bool(route_build_online_map_smm);
DECLARE_bool(route_snapshot_sim_mode);
DECLARE_string(route_inject_teleop_proto);
DECLARE_bool(route_fix_6);
DECLARE_bool(route_ignore_kickout);
DECLARE_bool(route_send_lane_graph_to_canvas);
DECLARE_double(route_sd_max_project_distance);
DECLARE_bool(route_trunc_mpp_by_nav);
DECLARE_bool(route_use_sdroute);
DECLARE_string(route_navinfo_restrict);

DECLARE_double(route_overwrite_ramp_speed_kph);
DECLARE_double(route_overwrite_highway_junction_speed_kph);
