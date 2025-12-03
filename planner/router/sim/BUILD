load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
cc_library(
    name = "snapshot_sim_state",
    srcs = ["snapshot_sim_state.cc"],
    hdrs = ["snapshot_sim_state.h"],
    deps = [
        "//common/proto:drive_mission_cc_proto",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/router:multi_stops_request",
        "//onboard/planner/router:route_manager_state",
        "//onboard/planner/router/util:map_index",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "snapshot_sim_state_test",
    srcs = ["snapshot_sim_state_test.cc"],
    deps = [
        ":snapshot_sim_state",
        "//onboard/global:test_main",
        "//onboard/planner/router:route_test_util",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:proto_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
