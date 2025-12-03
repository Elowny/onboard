#include "onboard/planner/router/router_flags.h"

DEFINE_bool(random_route, false,
            "Generate a random route connecting all nearby named spots.");

// Example: destinations { name: DEST_M4_WORK } destinations { name:
// DEST_M4_HOTEL } destinations { name: DEST_M4_GARAGE_W }
DEFINE_string(route, "",
              "Proto file or text that describes the RoutingRequestProto.");

DEFINE_string(route_str, "",
              "A string with named spots separated by space as a route");

DEFINE_string(multi_stops_route, "", "Multiple stops for robobus operation.");

DEFINE_bool(route_recover_multi_stops_from_log, false,
            "Recover multiple stops route request from log.");

DEFINE_bool(route_recover_destinations_from_log, false,
            "Recover route destination request from log.");

DEFINE_string(route_default_params_file,
              "onboard/planner/router/params/route_default_params.pb.txt",
              "Default params, route map match and search cost");

DEFINE_bool(
    route_auto_next_stop, false,
    "Auto goto next stop when near stop(<kReachRouteDestinationRadius)");

DEFINE_bool(route_match_filter_enable, true,
            "Enable filter avoid lane when matching.");

DEFINE_bool(route_allow_express_way, true,
            "Allow to search route with express ways.");

DEFINE_bool(route_recover_request_from_destinations, true,
            "Recover MultiStops from destinations if exist.");

DEFINE_bool(router_send_route_core_state_to_canvas, false,
            "Whether to send route search state to vantage.");

DEFINE_double(route_mainloop_period, 500.0, "The routing module period (ms)");

DEFINE_double(route_arrive_distance, 25.0, "arrived stop distance.");

DEFINE_bool(route_enable_alternative_path, false,
            "Whether to enable alternative route.");
DEFINE_bool(router_enable_off_road_search, false,
            "Whether to enable off road searcher.");

DEFINE_double(route_wait_ok_secs, 60.0, "The max secs to wait for sending ok.");

DEFINE_bool(route_noa_mode, false, "Set noa mode, get routing result outside.");

DEFINE_bool(route_build_online_map_smm, false,
            "Force to build smm through OnlineSemanticMapProto");
DEFINE_bool(
    route_snapshot_sim_mode, false,
    "Replay sim mode,like playback mode, but update route with pose changed.");

DEFINE_string(route_inject_teleop_proto, "", "Inject proto");
DEFINE_bool(route_fix_6, false, "Noa mode, a fix request part of QR00215");

DEFINE_bool(route_send_lane_graph_to_canvas, false,
            "Send route lane graph to canvas.");
DEFINE_bool(route_ignore_kickout, false,
            "Ignore kickout, support for the LCC/ACC mode.");
DEFINE_double(route_sd_max_project_distance, 100.0,
              "Project max lateral error in noa mode.");
DEFINE_bool(route_trunc_mpp_by_nav, true,
            "Truncate mpp with part_of_calculated_route.");
DEFINE_bool(route_use_sdroute, false, "Use sd route as routing result");
DEFINE_string(route_navinfo_restrict, "", "Set navinfo avoid lanes.");

DEFINE_double(route_overwrite_ramp_speed_kph, 60.0,
              "Set ramp overwrite speed, unit: kph.");
DEFINE_double(route_overwrite_highway_junction_speed_kph, 60.0,
              "Set highway junction overwrite speed, unit: kph.");
