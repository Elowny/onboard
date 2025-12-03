load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "lane_graph",
    hdrs = ["lane_graph.h"],
    deps = [
        "//onboard/container:flat_map",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_defs",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "dijkstra",
    hdrs = ["dijkstra.h"],
    deps = [
        ":lane_graph",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "lane_graph_util",
    srcs = ["lane_graph_util.cc"],
    hdrs = ["lane_graph_util.h"],
    deps = [
        ":lane_graph",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router:route_util",
        "//onboard/planner/router:router_defs",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "route_lane_graph_builder",
    srcs = ["route_lane_graph_builder.cc"],
    hdrs = ["route_lane_graph_builder.h"],
    deps = [
        ":lane_graph_util",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/router:route_sections_info",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "dijkstra_test",
    srcs = ["dijkstra_test.cc"],
    deps = [
        ":dijkstra",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
