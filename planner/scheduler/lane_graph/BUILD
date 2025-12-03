load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "lane_graph_builder",
    srcs = ["lane_graph_builder.cc"],
    hdrs = ["lane_graph_builder.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/maps:map_or_die_macros",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/router/lane_graph/v2:lane_graph",
        "//onboard/planner/router/lane_graph/v2:lane_graph_util",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:lane_point_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "lane_path_finder",
    srcs = ["lane_path_finder.cc"],
    hdrs = ["lane_path_finder.h"],
    deps = [
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/maps:map_or_die_macros",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router/lane_graph/v2:dijkstra",
        "//onboard/planner/router/lane_graph/v2:lane_graph",
        "//onboard/planner/router/lane_graph/v2:lane_graph_util",
        "//onboard/planner/router/navi:route_navi_info",
        "//onboard/utils:status_macros",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "lane_path_finder_test",
    srcs = ["lane_path_finder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":lane_graph_builder",
        ":lane_path_finder",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/router/navi:route_navi_info_builder",
        "//onboard/planner/scheduler:scheduler_plot_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
