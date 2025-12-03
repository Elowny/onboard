load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "traffic_gap_finder_impl",
    srcs = ["traffic_gap_finder_impl.cc"],
    hdrs = ["traffic_gap_finder_impl.h"],
    deps = [
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:lane_change_safety_params",
        "//onboard/planner/common:lane_change_safety_util",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "traffic_gap_finder",
    srcs = ["traffic_gap_finder.cc"],
    hdrs = ["traffic_gap_finder.h"],
    deps = [
        ":traffic_gap_finder_impl",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:logging",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/planner/decision:traffic_gap_result",
        "//onboard/planner/decision/traffic_gap_v2/proto:traffic_gap_cc_proto",
        "//onboard/planner/object:spacetime_trajectory_manager",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "traffic_gap_finder_impl_test",
    srcs = ["traffic_gap_finder_impl_test.cc"],
    deps = [
        ":traffic_gap_finder_impl",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "traffic_gap_finder_test",
    srcs = ["traffic_gap_finder_test.cc"],
    deps = [
        ":traffic_gap_finder",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
