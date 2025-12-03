load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################

cc_library(
    name = "future_pos_estimation",
    srcs = ["future_pos_estimation.cc"],
    hdrs = ["future_pos_estimation.h"],
    deps = [
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "future_pos_estimation_test",
    srcs = ["future_pos_estimation_test.cc"],
    deps = [
        ":future_pos_estimation",
        "//onboard/global:test_main",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
