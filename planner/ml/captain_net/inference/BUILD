load("//bazel:common.bzl", "if_x86_64_cuda", "requires_x86_64_cuda")
load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "captain_net_onnx",
    srcs = [
        "captain_net_onnx.cc",
    ],
    hdrs = [
        "captain_net_onnx.h",
    ],
    deps = [
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/nets/trt:onnxruntime_net",
        "//onboard/planner/ml/captain_net",
        "//onboard/utils:source_location",
        "//third_party/onnxruntime",
    ],
)

cc_library(
    name = "captain_net_torch",
    srcs = [
        "captain_net_torch.cc",
    ],
    hdrs = [
        "captain_net_torch.h",
    ],
    target_compatible_with = requires_x86_64_cuda(),
    deps = [
        "//onboard/global:trace",
        "//onboard/planner/ml/captain_net",
        "//onboard/utils/lfs_access",
        "//third_party/torch",
    ],
)

cc_library(
    name = "captain_net_inference",
    srcs = [
        "captain_net_inference.cc",
    ],
    hdrs = ["captain_net_inference.h"],
    data = [
        "//onboard/params/nets:captain_net_model_data",
    ],
    deps = if_x86_64_cuda([
        ":captain_net_torch",
    ]) + [
        ":captain_net_onnx",
        "//onboard/global:car_common",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params/utils:param_util",
    ],
)

cc_library(
    name = "captain_net_inferencer",
    srcs = [
        "captain_net_inferencer.cc",
    ],
    hdrs = ["captain_net_inferencer.h"],
    data = [
        "//onboard/params/nets:captain_net_model_data",
    ],
    deps = [
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params/utils:param_util",
        "//onboard/planner/ml:planner_ml_qnn",
        "//onboard/planner/ml/captain_net",
        "//onboard/planner/ml/common:feature_extraction_utils",
        "//onboard/utils:source_location",
    ],
)

cc_library(
    name = "captain_net_j5_inferencer",
    srcs = [
        "captain_net_j5_inferencer.cc",
    ],
    hdrs = ["captain_net_j5_inferencer.h"],
    data = [
        "//onboard/params/nets:captain_net_model_data",
    ],
    deps = [
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params/utils:param_util",
        "//onboard/planner/ml/captain_net",
        "//onboard/planner/ml/captain_net_j5:horizon_tensor_wrapper",
        "//onboard/planner/ml/captain_net_j5:planner_ml_j5_qnn",
        "//onboard/planner/ml/common:feature_extraction_utils",
        "//onboard/utils:source_location",
    ],
)

cc_binary(
    name = "captain_net_inferencer_bm",
    srcs = [
        "captain_net_inferencer_bm.cc",
    ],
    tags = ["bm"],
    deps = [
        ":captain_net_inferencer",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/utils:errors",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_library(
    name = "input_assemble",
    srcs = ["input_assemble.cc"],
    hdrs = ["input_assemble.h"],
    deps = [
        "//onboard/planner/ml/captain_net",
        "//onboard/planner/ml/condition_feature_extractor:condition_feature",
        "//onboard/planner/ml/context_feature_extractor:context_feature",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "captain_net_sync_test",
    srcs = [
        "captain_net_sync_test.cc",
    ],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    tags = [
        "exclusive",
        "manual",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_manager",
        "//onboard/planner/ml:planner_ml_qnn",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
