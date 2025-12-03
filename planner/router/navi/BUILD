load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Protos

# Libraries.

cc_library(
    name = "navi_output",
    srcs = ["navi_output.cc"],
    hdrs = ["navi_output.h"],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/global:timer",
        "//onboard/global:trace",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:route_core_searcher",
        "//onboard/planner/router:route_util",
        "//onboard/proto:hmi_content_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "route_navi_info",
    srcs = ["route_navi_info.cc"],
    hdrs = ["route_navi_info.h"],
    deps = [
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner/router:route_sections",
        "//onboard/proto:route_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "route_navi_info_builder",
    srcs = ["route_navi_info_builder.cc"],
    hdrs = ["route_navi_info_builder.h"],
    deps = [
        ":route_navi_info",
        ":route_navi_util",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:router_defs",
        "//onboard/planner/router/lane_graph/v2:dijkstra",
        "//onboard/planner/router/lane_graph/v2:route_lane_graph_builder",
        "//onboard/planner/util:scene_util",
        "//onboard/proto:route_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "route_navi_util",
    srcs = ["route_navi_util.cc"],
    hdrs = ["route_navi_util.h"],
    deps = [
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner/router:route_sections_info",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
    ],
)

###############################################################################
# Proto.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "navi_output_test",
    srcs = ["navi_output_test.cc"],
    deps = [
        ":navi_output",
        "//onboard/base",
        "//onboard/global:test_main",
        "//onboard/planner/router:route_manager_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "route_navi_info_builder_test",
    srcs = ["route_navi_info_builder_test.cc"],
    deps = [
        ":route_navi_info_builder",
        "//onboard/global:test_main",
        "//onboard/maps/v2:semantic_map_loader",
        "//onboard/planner/router:route_core_searcher",
        "//onboard/planner/router:route_manager_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_test_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
