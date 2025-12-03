load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "scheduler_input",
    hdrs = ["scheduler_input.h"],
    deps = [
        ":smooth_reference_line_result",
        "//onboard/maps:lane_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/decision/traffic_light:tl_info",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router/navi:route_navi_info",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
    ],
)

cc_library(
    name = "scheduler_output",
    hdrs = ["scheduler_output.h"],
    deps = [
        "//onboard/math:frenet_frame",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/proto:turn_signal_cc_proto",
        "@com_google_absl//absl/hash",
    ],
)

cc_library(
    name = "scheduler_util",
    srcs = ["scheduler_util.cc"],
    hdrs = ["scheduler_util.h"],
    deps = [
        ":scheduler_output",
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/container:strong_int",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/math:frenet_common",
        "//onboard/math/geometry:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/assist:assist_util",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision/proto:traffic_light_info_cc_proto",
        "//onboard/planner/decision/traffic_light:tl_info",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler:path_boundary_builder_helper",
        "//onboard/planner/scheduler:smooth_reference_line_result",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/planner/scheduler/proto:scheduler_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "path_boundary_builder",
    srcs = ["path_boundary_builder.cc"],
    hdrs = ["path_boundary_builder.h"],
    deps = [
        ":path_boundary_builder_helper",
        ":smooth_reference_line_result",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:route_util",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/planner/scheduler/proto:reference_line_qp_smoother_cc_proto",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:history_buffer",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "path_boundary_builder_helper",
    srcs = ["path_boundary_builder_helper.cc"],
    hdrs = ["path_boundary_builder_helper.h"],
    deps = [
        ":scheduler_plot_util",
        ":smooth_reference_line_result",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:range1d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "local_map_builder",
    srcs = ["local_map_builder.cc"],
    hdrs = ["local_map_builder.h"],
    deps = [
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router:route_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "scheduler_plot_util",
    srcs = ["scheduler_plot_util.cc"],
    hdrs = ["scheduler_plot_util.h"],
    deps = [
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router/lane_graph/v2:lane_graph",
        "//onboard/planner/util:lane_point_util",
        "//onboard/planner/util:planner_semantic_map_util",
    ],
)

cc_library(
    name = "target_lane_path_filter",
    srcs = ["target_lane_path_filter.cc"],
    hdrs = ["target_lane_path_filter.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/math:frenet_frame",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router/navi:route_navi_info",
        "//onboard/planner/util:lane_point_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "//onboard/planner/util:scene_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "multi_tasks_scheduler",
    srcs = ["multi_tasks_scheduler.cc"],
    hdrs = ["multi_tasks_scheduler.h"],
    deps = [
        ":path_boundary_builder",
        ":scheduler_input",
        ":scheduler_output",
        ":scheduler_util",
        "//onboard/async:parallel_for",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/util:planner_semantic_map_manager_builder",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "reference_line_qp_smoother",
    srcs = ["reference_line_qp_smoother.cc"],
    hdrs = ["reference_line_qp_smoother.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/qp:osqp_solver",
        "//onboard/planner/scheduler/proto:reference_line_qp_smoother_cc_proto",
    ],
)

cc_library(
    name = "smooth_reference_line_builder",
    srcs = ["smooth_reference_line_builder.cc"],
    hdrs = ["smooth_reference_line_builder.h"],
    deps = [
        ":path_boundary_builder",
        ":reference_line_qp_smoother",
        ":smooth_reference_line_result",
        "//onboard/maps:map_or_die_macros",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:lane_point_util",
        "//onboard/planner/util:planner_semantic_map_util",
    ],
)

cc_library(
    name = "smooth_reference_line_result",
    srcs = ["smooth_reference_line_result.cc"],
    hdrs = ["smooth_reference_line_result.h"],
    deps = [
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:piecewise_linear_function",
        "//onboard/utils:map_util",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "driving_map_topo_builder",
    srcs = ["driving_map_topo_builder.cc"],
    hdrs = ["driving_map_topo_builder.h"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/global:trace",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:online_semantic_map_cc_proto",
        "//onboard/math:frenet_common",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:driving_map_topo",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/util:online_semantic_map_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "straighten_reference_line",
    srcs = ["straighten_reference_line.cc"],
    hdrs = ["straighten_reference_line.h"],
    deps = [
        ":path_boundary_builder",
        ":smooth_reference_line_result",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/util:lane_path_util",
        "//onboard/vis/common:color",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "local_map_builder_test",
    srcs = ["local_map_builder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":local_map_builder",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status:statusor",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/base",
        # "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_defs",
        "//onboard/params:param_manager",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/router/navi:route_navi_info_builder",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
        "//onboard/planner/test_util:load_psmm_util",
    ],
)

cc_test(
    name = "scheduler_util_test",
    srcs = ["scheduler_util_test.cc"],
    deps = [
        ":scheduler_util",
        "//onboard/global:test_main",
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "path_boundary_builder_test",
    srcs = ["path_boundary_builder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":path_boundary_builder",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:online_map_converter",
        "//onboard/planner/util:planner_semantic_map_manager_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "target_lane_path_filter_test",
    srcs = ["target_lane_path_filter_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":scheduler_plot_util",
        ":target_lane_path_filter",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/router/navi:route_navi_info_builder",
        "//onboard/planner/scheduler/lane_graph:lane_graph_builder",
        "//onboard/planner/scheduler/lane_graph:lane_path_finder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "reference_line_qp_smoother_test",
    srcs = ["reference_line_qp_smoother_test.cc"],
    deps = [
        ":reference_line_qp_smoother",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "smooth_reference_line_builder_test",
    srcs = ["smooth_reference_line_builder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":path_boundary_builder",
        ":smooth_reference_line_builder",
        ":smooth_reference_line_result",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "smooth_reference_line_result_test",
    srcs = ["smooth_reference_line_result_test.cc"],
    deps = [
        ":smooth_reference_line_builder",
        ":smooth_reference_line_result",
        "//onboard/base",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "driving_map_topo_builder_test",
    srcs = ["driving_map_topo_builder_test.cc"],
    deps = [
        ":driving_map_topo_builder",
        "//onboard/base",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/util:online_map_converter",
        "//onboard/planner/util:planner_semantic_map_manager_builder",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "multi_tasks_scheduler_test",
    srcs = ["multi_tasks_scheduler_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":multi_tasks_scheduler",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/base",
        "//onboard/container:strong_int",
        "//onboard/global:test_main",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router/navi:route_navi_info_builder",
        "//onboard/planner/scheduler:scheduler_util",
        "//onboard/planner/scheduler:smooth_reference_line_builder",
        "//onboard/planner/scheduler/lane_graph:lane_graph_builder",
        "//onboard/planner/scheduler/lane_graph:lane_path_finder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:util",
        "@com_google_absl//absl/hash",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "straighten_reference_line_test",
    srcs = ["straighten_reference_line_test.cc"],
    deps = [
        ":straighten_reference_line",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
