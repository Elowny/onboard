load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "trajectory_validation",
    srcs = ["trajectory_validation.cc"],
    hdrs = ["trajectory_validation.h"],
    deps = [
        ":trajectory_validation_error",
        "//onboard/async:parallel_for",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/math:frenet_frame",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:planner_util",
        "//onboard/planner:trajectory_point",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/object:partial_spacetime_object_trajectory",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "trajectory_validation_error",
    hdrs = [
        "trajectory_validation_error.h",
    ],
    deps = [
        "//onboard/planner/proto:trajectory_validation_cc_proto",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "trajectory_validation_error_test",
    srcs = ["trajectory_validation_error_test.cc"],
    linkstatic = 1,
    deps = [
        ":trajectory_validation_error",
        "//onboard/global:test_main",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "trajectory_validation_test",
    srcs = ["trajectory_validation_test.cc"],
    linkstatic = 1,
    deps = [
        "//onboard/global:test_main",
        "//onboard/planner/trajectory_validation",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
