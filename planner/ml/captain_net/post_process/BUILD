load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "post_process",
    srcs = ["post_process.cc"],
    hdrs = ["post_process.h"],
    deps = [
        ":movability_fixer",
        "//onboard/planner/ml/captain_net",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "trajectory_alignment",
    srcs = ["trajectory_alignment.cc"],
    hdrs = ["trajectory_alignment.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/planner:planner_flags",
        "//onboard/planner/ml/captain_net",
        "//onboard/proto:trajectory_point_cc_proto",
    ],
)

cc_library(
    name = "model_output_process",
    srcs = ["model_output_process.cc"],
    hdrs = ["model_output_process.h"],
    deps = [
        "//onboard/planner/ml/captain_net",
    ],
)

cc_library(
    name = "movability_fixer",
    srcs = ["movability_fixer.cc"],
    hdrs = ["movability_fixer.h"],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/planner:planner_flags",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/ml/captain_net",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
