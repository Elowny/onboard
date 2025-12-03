load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "trajectory_validation",
    srcs = ["trajectory_validation.cc"],
    hdrs = ["trajectory_validation.h"],
    deps = [
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:trajectory_point",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/ml:model_pool",
        "//onboard/planner/ml/captain_net",
        "//onboard/planner/object:partial_spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/proto:trajectory_validation_cc_proto",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/planner/trajectory_validation",
        "//onboard/planner/trajectory_validation:trajectory_validation_error",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
