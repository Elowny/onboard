load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

# Library

cc_library(
    name = "planner_ml_j5_qnn",
    srcs = [
        "planner_ml_j5_qnn.cc",
    ],
    hdrs = [
        "planner_ml_j5_qnn.h",
    ],
    deps = [
        ":horizon_tensor_wrapper",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/lite:qissue_trans",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/nets/qnn/qnn",
        "//onboard/params:param_converter",
        "//onboard/proto:q_issue_cc_proto",
    ],
)

cc_library(
    name = "horizon_tensor_wrapper",
    srcs = [
        "horizon_tensor_wrapper.cc",
    ],
    hdrs = [
        "horizon_tensor_wrapper.h",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/nets/qnn/qnn",
        "@com_github_google_glog//:glog",
    ],
)

# Test
cc_test(
    name = "horizon_tensor_wrapper_test",
    srcs = ["horizon_tensor_wrapper_test.cc"],
    deps = [
        ":horizon_tensor_wrapper",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/nets/qnn/qnn",
        "@com_google_googletest//:gtest",
    ],
)

# cc_test(
#     name = "planner_ml_j5_qnn_test",
#     srcs = ["planner_ml_j5_qnn_test.cc"],
#     data = [
#         "//onboard/params/nets:actnet_j5_model_data",
#     ],
#     deps = [
#         ":prediction_j5_qnn",
#         "//onboard/global:test_main",
#         "//onboard/lite:logging",
#         "//onboard/nets/proto:net_param_cc_proto",
#         "//onboard/params:param_finder",
#         "//onboard/params:param_finder_without_nets",
#         "//onboard/prediction/test_util:net_input_generator",
#         "@com_google_googletest//:gtest",
#     ],
# )

cpplint()
