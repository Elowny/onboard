load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "model_pool",
    srcs = ["model_pool.cc"],
    hdrs = ["model_pool.h"],
    deps = [
        "//onboard/global:run_context",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner:planner_flags",
        "//onboard/planner/ml/captain_net/inference:captain_net_inference",
        "//onboard/planner/ml/captain_net/inference:captain_net_inferencer",
        "//onboard/planner/ml/captain_net/inference:captain_net_j5_inferencer",
        "//onboard/planner/ml/selector_models:selector_scoring_net_inference",
        "//onboard/prediction/inferencer:act_net_speed_inferencer",
        "//onboard/utils:errors",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "planner_ml_defs",
    hdrs = ["planner_ml_defs.h"],
)

cc_library(
    name = "planner_ml_qnn",
    srcs = [
        "planner_ml_qnn.cc",
    ],
    hdrs = [
        "planner_ml_qnn.h",
    ],
    deps = [
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/nets/qnn/qnn",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
