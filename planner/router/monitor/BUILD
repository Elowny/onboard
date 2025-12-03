load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])
###############################################################################
# Protos

# Libraries.

cc_library(
    name = "stop_monitor",
    srcs = ["stop_monitor.cc"],
    hdrs = ["stop_monitor.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:spatial_search_util",
        "//onboard/math:vec",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner/router:map_match",
        "//onboard/planner/router:multi_stops_request",
        "//onboard/planner/router:route_core",
        "//onboard/planner/router:route_generator",
        "//onboard/planner/router:route_manager_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/router/request:routing_request_context",
        "//onboard/planner/router/util:map_index",
        "@com_google_absl//absl/status:statusor",
    ],
)

# Tests

cc_test(
    name = "stop_monitor_test",
    srcs = ["stop_monitor_test.cc"],
    deps = [
        ":stop_monitor",
        "//common/proto:drive_mission_cc_proto",
        "//common/proto:lane_point_cc_proto",
        "//onboard/base",
        "//onboard/global:test_main",
        "//onboard/planner/router:multi_stops_request",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/planner/router/util:map_index",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
