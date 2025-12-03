load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])
###############################################################################
# Libraries.

cc_library(
    name = "auto_tuning_common_flags",
    srcs = ["auto_tuning_common_flags.cc"],
    hdrs = ["auto_tuning_common_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "auto_tuning_utils",
    srcs = ["auto_tuning_utils.cc"],
    hdrs = ["auto_tuning_utils.h"],
    deps = [
        ":auto_tuning_common_flags",
        "//onboard/planner:planner_defs",
        "//onboard/planner:trajectory_point",
        "//onboard/planner/ml/optimizer_auto_tuning/proto:auto_tuning_cc_proto",
        "//onboard/planner/optimization/ddp:ddp_optimizer",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
        "//onboard/proto:trajectory_cc_proto",
        "@eigen",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

###############################################################################
# Files.

filegroup(
    name = "optimizer_auto_tuning_files",
    data = glob([
        "traj_opt_params.pb.txt",
    ]),
)
