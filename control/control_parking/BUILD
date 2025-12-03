load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "stationary_steer_control",
    srcs = ["stationary_steer_control.cc"],
    hdrs = ["stationary_steer_control.h"],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control:steering_converter",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "control_parking_manager",
    srcs = ["control_parking_manager.cc"],
    hdrs = ["control_parking_manager.h"],
    deps = [
        ":stationary_steer_control",
        "//onboard/control:control_flags",
        "//onboard/control:steering_converter",
        "//onboard/control:trajectory_interface",
        "//onboard/control/controllers:controller_common",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
    ],
)

cc_library(
    name = "parking_path_to_trajectory",
    srcs = ["parking_path_to_trajectory.cc"],
    hdrs = ["parking_path_to_trajectory.h"],
    deps = [
        "//onboard/math:util",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/planner:planner_defs",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_protobuf//:protobuf",
    ],
)

###############################################################################
# Tests
cc_test(
    name = "stationary_steer_control_test",
    srcs = ["stationary_steer_control_test.cc"],
    deps = [
        ":stationary_steer_control",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "control_parking_manager_test",
    srcs = ["control_parking_manager_test.cc"],
    deps = [
        ":control_parking_manager",
        "//onboard/global:test_main",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "parking_path_to_trajectory_test",
    srcs = ["parking_path_to_trajectory_test.cc"],
    deps = [
        ":parking_path_to_trajectory",
        "//onboard/global:test_main",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
