load("//bazel:common.bzl", "if_x86_64_cuda", "requires_x86_64_cuda")
load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "selector_feature_extractor",
    srcs = ["selector_feature_extractor.cc"],
    hdrs = ["selector_feature_extractor.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/planner:est_planner_output",
        "//onboard/planner:planner_flags",
        "//onboard/planner/ml/condition_feature_extractor",
        "//onboard/planner/ml/condition_feature_extractor:condition_feature_proto_converter",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/planner/selector:candidate_stats",
        "//onboard/planner/selector:selector_defs",
        "//onboard/planner/selector:selector_input",
        "//onboard/planner/selector/proto:selector_debug_cc_proto",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
    ],
)

cc_library(
    name = "selector_scorer",
    srcs = ["selector_scorer.cc"],
    hdrs = ["selector_scorer.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/planner/common:planner_status",
        "//onboard/planner/ml/common:feature_extraction_utils",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/selector:candidate_stats",
        "//onboard/planner/selector:common_feature",
        "//onboard/planner/selector:selector_input",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "selector_scoring_net",
    hdrs = [
        "selector_scoring_net.h",
    ],
)

cc_library(
    name = "selector_scoring_net_onnx",
    srcs = [
        "selector_scoring_net_onnx.cc",
    ],
    hdrs = [
        "selector_scoring_net_onnx.h",
    ],
    deps = [
        ":selector_scoring_net",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/nets/trt:onnxruntime_net",
        "//onboard/utils:source_location",
        "//third_party/onnxruntime",
    ],
)

cc_library(
    name = "selector_scoring_net_torch",
    srcs = [
        "selector_scoring_net_torch.cc",
    ],
    hdrs = [
        "selector_scoring_net_torch.h",
    ],
    target_compatible_with = requires_x86_64_cuda(),
    deps = [
        ":selector_scoring_net",
        "//onboard/global:trace",
        "//onboard/utils/lfs_access",
        "//third_party/torch",
    ],
)

cc_library(
    name = "selector_scoring_net_inference",
    srcs = [
        "selector_scoring_net_inference.cc",
    ],
    hdrs = ["selector_scoring_net_inference.h"],
    data = [
        "//onboard/nets:data",
        "//onboard/params/nets:selector_scoring_net_model_data",
    ],
    deps = if_x86_64_cuda([
        ":selector_scoring_net_torch",
    ]) + [
        ":selector_scoring_net_onnx",
        "//onboard/global:car_common",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params/utils:param_util",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
