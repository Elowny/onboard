load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Protos

# Libraries.

cc_library(
    name = "routing_module",
    srcs = ["routing_module.cc"],
    hdrs = ["routing_module.h"],
    data = [
        "//onboard/planner/params:param_files",
        "//onboard/planner/params/X9:X9_param_files",
        "//onboard/planner/router/params:param_files",
    ],
    deps = [
        ":multi_stops_request",
        ":road_conditions_process",
        ":route_error",
        ":route_manager",
        ":route_manager_output",
        ":route_manager_state",
        ":route_module_input",
        ":route_util",
        ":router_flags",
        "//common/proto:drive_mission_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/hmi/states:run_event_states_util",
        "//onboard/lite:lite_client_base",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/lite:qissue_trans",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/maps:online_map_utils",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:online_semantic_map_cc_proto",
        "//onboard/maps/v2:semantic_map_listener",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/planner/assist/proto:external_command_status_cc_proto",
        "//onboard/planner/common:global_pose",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/planner/router/interface:generate_route",
        "//onboard/planner/router/noa:noa_main_loop",
        "//onboard/planner/router/noa:route_mpp",
        "//onboard/planner/router/noa:sd_route_util",
        "//onboard/planner/router/preprocess:future_pos_estimation",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/planner/router/sim:snapshot_sim_state",
        "//onboard/planner/router/task:route_reset_action",
        "//onboard/planner/router/thirdparty:external_route_manager",
        "//onboard/planner/router/util:async_retry",
        "//onboard/planner/router/util:map_index",
        "//onboard/planner/util:vehicle_util",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:hmi_content_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:q_issue_cc_proto",
        "//onboard/proto:q_run_event_cc_proto",
        "//onboard/proto:q_run_event_states_cc_proto",
        "//onboard/proto:remote_assist_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:proto_util",
        "//onboard/utils:time_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

filegroup(
    name = "route_data",
    data = glob([
        "testdata/*.pb.txt",
        "testdata/routes/**",
    ]),
)

cc_library(
    name = "route_util",
    srcs = ["route_util.cc"],
    hdrs = ["route_util.h"],
    deps = [
        ":drive_passage",
        ":route_manager_output",
        ":route_sections_info",
        ":router_flags",
        "//common/proto:map_geometry_cc_proto",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_path_util",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps:spatial_search_util",
        "//onboard/math:coordinate_converter",
        "//onboard/math:piecewise_const_function",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router/navi:route_navi_info",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/planner/router/util:map_index",
        "//onboard/planner/router/util:route_extend_path",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:proto_util",
        "//onboard/utils:time_util",
        "@boost//:geometry",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "plot_util",
    srcs = ["plot_util.cc"],
    hdrs = ["plot_util.h"],
    deps = [
        ":drive_passage",
        ":route_sections",
        ":route_util",
        "//common/proto:map_geometry_cc_proto",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//offboard/vis/vantage/vantage_server/proto:vantage_server_cc_proto",
        "//onboard/container:strong_int",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/math:coordinate_converter",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router/lane_graph/v2:lane_graph",
        "//onboard/planner/router/util:route_struct",
        "//onboard/planner/util:lane_point_util",
        "//onboard/proto:lite_msg_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "route_manager_output",
    srcs = ["route_manager_output.cc"],
    hdrs = ["route_manager_output.h"],
    deps = [
        ":road_constraints",
        ":route_sections",
        "//common/proto:drive_mission_cc_proto",
        "//onboard/container:strong_int",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/planner/router/navi:route_navi_info",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/proto:turn_signal_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "route_manager",
    srcs = ["route_manager.cc"],
    hdrs = ["route_manager.h"],
    deps = [
        ":multi_stops_request",
        ":off_road_route_searcher",
        ":road_conditions_process",
        ":route_core",
        ":route_core_searcher",
        ":route_ego_tracker",
        ":route_error",
        ":route_generator",
        ":route_manager_output",
        ":route_manager_state",
        ":route_manager_util",
        ":route_module_input",
        ":route_sections_util",
        ":route_util",
        ":router_defs",
        ":router_flags",
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/eval:qevent",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:safe_unit",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/router/alternate:alternative_route",
        "//onboard/planner/router/navi:navi_output",
        "//onboard/planner/router/navi:route_navi_info_builder",
        "//onboard/planner/router/preprocess:future_pos_estimation",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/planner/router/util:map_index",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:remote_assist_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf_headers",
    ],
)

cc_library(
    name = "router_flags",
    srcs = ["router_flags.cc"],
    hdrs = ["router_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "drive_passage",
    srcs = ["drive_passage.cc"],
    hdrs = ["drive_passage.h"],
    deps = [
        "//onboard/container:strong_vector",
        "//onboard/global:trace",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner:planner_defs",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "drive_passage_builder",
    srcs = ["drive_passage_builder.cc"],
    hdrs = ["drive_passage_builder.h"],
    deps = [
        ":drive_passage",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:circle",
        "//onboard/math:circle_fitter",
        "//onboard/math:fitter_def",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/util:lane_path_preprocess_util",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:lane_point_util",
        "//onboard/utils:map_util",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "route_test_util",
    testonly = True,
    srcs = ["route_test_util.cc"],
    hdrs = ["route_test_util.h"],
    deps = [
        ":plot_util",
        ":route_sections",
        ":route_sections_util",
        ":router_flags",
        "//onboard/maps/v2:semantic_map_loader",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner/router/util:map_index",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/proto:positioning_cc_proto",
    ],
)

cc_library(
    name = "multi_stops_request",
    srcs = ["multi_stops_request.cc"],
    hdrs = ["multi_stops_request.h"],
    deps = [
        ":road_conditions_process",
        ":route_util",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "route_manager_util",
    srcs = ["route_manager_util.cc"],
    hdrs = ["route_manager_util.h"],
    deps = [
        ":map_match",
        ":multi_stops_request",
        ":route_core",
        ":router_flags",
        "//onboard/global:clock",
        "//onboard/lite:logging",
        "//onboard/maps:compatibility_layer",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router/navi:route_navi_info_builder",
        "//onboard/planner/router/request:routing_request_context",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "route_sections",
    srcs = ["route_sections.cc"],
    hdrs = ["route_sections.h"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/proto:route_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "route_sections_info",
    srcs = ["route_sections_info.cc"],
    hdrs = ["route_sections_info.h"],
    deps = [
        ":route_sections",
        "//onboard/container:strong_int",
        "//onboard/lite:check",
        "//onboard/maps:lane_point",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "route_sections_util",
    srcs = ["route_sections_util.cc"],
    hdrs = ["route_sections_util.h"],
    deps = [
        ":route_sections",
        ":route_sections_info",
        ":route_util",
        "//onboard/container:strong_int",
        "//onboard/global:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:vec",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router/navi:route_navi_info",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:lane_point_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "map_match",
    srcs = ["map_match.cc"],
    hdrs = ["map_match.h"],
    deps = [
        ":route_util",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_library(
    name = "route_error",
    srcs = ["route_error.cc"],
    hdrs = ["route_error.h"],
    deps = [
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "off_road_route_searcher",
    srcs = ["off_road_route_searcher.cc"],
    hdrs = ["off_road_route_searcher.h"],
    deps = [
        ":multi_stops_request",
        ":road_conditions_process",
        ":route_core",
        ":route_core_searcher",
        ":route_manager_util",
        ":route_sections",
        ":route_util",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:spatial_search_util",
        "//onboard/math:vec",
        "//onboard/math/geometry:intersection_util",
        "//onboard/math/geometry:polyline2d",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router/request:routing_request_context",
        "//onboard/planner/router/util:map_index",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "route_portal_util",
    srcs = ["route_portal_util.cc"],
    hdrs = ["route_portal_util.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/proto:q_command_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
        "@com_google_protobuf//:protobuf_headers",
    ],
)

cc_library(
    name = "route_manager_state",
    srcs = ["route_manager_state.cc"],
    hdrs = ["route_manager_state.h"],
    deps = [
        ":multi_stops_request",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner:composite_lane_path",
        "//onboard/proto:hmi_content_cc_proto",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_library(
    name = "route_ego_tracker",
    srcs = ["route_ego_tracker.cc"],
    hdrs = ["route_ego_tracker.h"],
    deps = [
        ":route_manager_state",
        ":route_util",
        "//onboard/container:strong_int",
        "//onboard/global:clock",
        "//onboard/lite:logging",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:spatial_search_util",
        "//onboard/maps/proto:lane_path_cc_proto",
        "//onboard/math:vec",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router/util:map_index",
        "//onboard/proto:route_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "road_conditions_process",
    srcs = ["road_conditions_process.cc"],
    hdrs = ["road_conditions_process.h"],
    deps = [
        ":road_constraints",
        "//common/proto:drive_mission_cc_proto",
        "//common/proto:map_geometry_cc_proto",
        "//onboard/container:strong_int",
        "//onboard/global:clock",
        "//onboard/lite:logging",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "route_core",
    srcs = ["route_core.cc"],
    hdrs = ["route_core.h"],
    deps = [
        ":road_conditions_process",
        ":route_sections",
        ":route_sections_util",
        ":route_util",
        "//onboard/global:timer",
        "//onboard/lite:check",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps:spatial_search_util",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:multi_timer_util",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/planner/router/util:route_struct",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "route_core_searcher",
    srcs = ["route_core_searcher.cc"],
    hdrs = ["route_core_searcher.h"],
    deps = [
        ":route_core",
        ":route_sections",
        ":route_sections_util",
        ":route_util",
        "//onboard/container:strong_int",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:spatial_search_util",
        "//onboard/maps/proto:lane_path_cc_proto",
        "//onboard/math:vec",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner/router:map_match",
        "//onboard/planner/router/request:routing_request_context",
        "//onboard/planner/router/util:map_index",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "route_generator",
    srcs = ["route_generator.cc"],
    hdrs = ["route_generator.h"],
    deps = [
        ":map_match",
        ":multi_stops_request",
        ":route_core",
        ":router_flags",
        "//onboard/eval:qevent",
        "//onboard/global:clock",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router:off_road_route_searcher",
        "//onboard/planner/router:route_core_searcher",
        "//onboard/planner/router/request:routing_request_context",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "route_module_input",
    hdrs = ["route_module_input.h"],
    deps = [
        "//onboard/maps/proto:online_semantic_map_cc_proto",
        "//onboard/math:vec",
        "//onboard/planner/router/proto:route_external_command_cc_proto",
        "//onboard/proto:adasis_cc_proto",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:remote_assist_cc_proto",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_library(
    name = "router_defs",
    srcs = ["router_defs.cc"],
    hdrs = ["router_defs.h"],
    deps = [
        "//onboard/planner:planner_defs",
    ],
)

cc_library(
    name = "road_constraints",
    srcs = ["road_constraints.cc"],
    hdrs = ["road_constraints.h"],
    deps = [
        "//onboard/maps:semantic_map_defs",
        "//onboard/proto:route_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "route_speed_limit_util",
    srcs = ["route_speed_limit_util.cc"],
    hdrs = ["route_speed_limit_util.h"],
    deps = [
        ":router_defs",
        ":router_flags",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/proto:online_map_cc_proto",
        "//onboard/proto:route_cc_proto",
    ],
)

###############################################################################
# Proto.

###############################################################################
# Binaries.

cc_binary(
    name = "drive_passage_bm",
    testonly = True,
    srcs = ["drive_passage_bm.cc"],
    tags = ["bm"],
    deps = [
        ":drive_passage",
        ":drive_passage_builder",
        "//onboard/maps/v2:semantic_map_loader",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/object:spacetime_object_state",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/planner/test_util:route_builder",
        "@com_github_google_benchmark//:benchmark",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "route_util_test",
    size = "large",
    srcs = ["route_util_test.cc"],
    deps = [
        ":route_test_util",
        ":route_util",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/planner/router/navi:route_navi_info_builder",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "drive_passage_test",
    srcs = ["drive_passage_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":drive_passage",
        ":drive_passage_builder",
        ":plot_util",
        ":route_sections",
        ":route_sections_util",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/base",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_flags",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_object_state",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/planner/test_util:route_builder",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_test",
    srcs = ["route_test.cc"],
    deps = [
        "//common/proto:drive_mission_cc_proto",
        "//onboard/container:strong_int",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:map_util",
        "//onboard/utils:test_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_manager_test",
    srcs = ["route_manager_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":route_ego_tracker",
        ":route_manager",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "drive_passage_builder_test",
    testonly = True,
    srcs = ["drive_passage_builder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":drive_passage_builder",
        ":plot_util",
        ":route_sections",
        ":route_sections_util",
        ":route_test_util",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/base",
        "//onboard/global:test_main",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_flags_convert_test",
    srcs = ["route_flags_convert_test.cc"],
    deps = [
        ":route_util",
        ":router_flags",
        "//common/proto:drive_mission_cc_proto",
        "//onboard/global:test_main",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "multi_stops_request_test",
    srcs = ["multi_stops_request_test.cc"],
    deps = [
        ":multi_stops_request",
        "//common/proto:map_geometry_cc_proto",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/router/util:map_index",
        "//onboard/utils:test_util",
        "@com_google_absl//absl/hash",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_manager_util_test",
    srcs = ["route_manager_util_test.cc"],
    deps = [
        ":route_core",
        ":route_manager_util",
        ":route_test_util",
        ":route_util",
        ":router_flags",
        "//common/proto:lane_point_cc_proto",
        "//common/proto:map_geometry_cc_proto",
        "//onboard/base",
        "//onboard/global:logging",
        "//onboard/global:test_main",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:map_selector",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:spatial_search_util",
        "//onboard/maps/proto:lane_path_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/utils:file_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_sections_info_test",
    srcs = ["route_sections_info_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":plot_util",
        ":route_sections_info",
        ":route_test_util",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/maps/v2:semantic_map_loader",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_sections_util_test",
    srcs = ["route_sections_util_test.cc"],
    deps = [
        ":route_sections_util",
        "//onboard/global:test_main",
        "//onboard/maps:smm_proto_util",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/router/navi:route_navi_info_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "off_road_route_searcher_test",
    srcs = ["off_road_route_searcher_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":off_road_route_searcher",
        ":plot_util",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_params_test",
    srcs = ["route_params_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "route_portal_util_test",
    srcs = ["route_portal_util_test.cc"],
    deps = [
        ":route_portal_util",
        "//onboard/base",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "road_conditions_process_test",
    srcs = ["road_conditions_process_test.cc"],
    deps = [
        ":road_conditions_process",
        ":route_test_util",
        "//onboard/base",
        "//onboard/global:test_main",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_core_test",
    srcs = ["route_core_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":route_core",
        ":route_test_util",
        ":route_util",
        ":router_flags",
        "//onboard/base",
        "//onboard/global:test_main",
        "//onboard/utils:file_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_core_searcher_test",
    srcs = ["route_core_searcher_test.cc"],
    deps = [
        ":route_core",
        ":route_core_searcher",
        ":route_test_util",
        ":router_flags",
        "//common/proto:lane_point_cc_proto",
        "//onboard/base",
        "//onboard/global:test_main",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/params:param_manager",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/planner/router/request:routing_request_context",
        "//onboard/planner/router/util:map_index",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "map_match_test",
    srcs = ["map_match_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":map_match",
        "//onboard/global:test_main",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:smm_proto_util",
        "//onboard/maps:spatial_search_util",
        "//onboard/params:param_manager",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/router/util:map_index",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_ego_tracker_test",
    srcs = ["route_ego_tracker_test.cc"],
    deps = [
        ":route_ego_tracker",
        ":route_util",
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/maps:lane_point",
        "//onboard/maps:maps_common",
        "//onboard/planner/router:route_test_util",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "routing_module_test",
    srcs = ["routing_module_test.cc"],
    copts = ["-fno-access-control"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":route_test_util",
        ":router_flags",
        ":routing_module",
        "//onboard/global:test_main",
        "//onboard/lite:lite_client_base",
        "//onboard/lite/unittest:lite2_unittest_helper",
        "//onboard/maps:map_or_die_macros",
        "//onboard/math:coordinate_converter",
        "@com_google_googletest//:gtest",
        "@folly",
    ],
)

cc_test(
    name = "route_generator_test",
    srcs = ["route_generator_test.cc"],
    copts = ["-fno-access-control"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":route_core",
        ":route_generator",
        ":router_flags",
        "//onboard/global:test_main",
        "//onboard/math:coordinate_converter",
        "//onboard/planner/router:route_test_util",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_test(
    name = "route_error_test",
    srcs = ["route_error_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":route_error",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "road_constraints_test",
    srcs = ["road_constraints_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":road_constraints",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_speed_limit_util_test",
    srcs = ["route_speed_limit_util_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":route_speed_limit_util",
        ":route_test_util",
        ":route_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
