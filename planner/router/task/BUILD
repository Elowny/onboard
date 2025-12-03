load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Protos

# Libraries.

cc_library(
    name = "route_reset_action",
    srcs = ["route_reset_action.cc"],
    hdrs = ["route_reset_action.h"],
    data = [
    ],
    deps = [
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/planner/router:route_manager_state",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/proto:route_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "route_reset_action_test",
    srcs = ["route_reset_action_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":route_reset_action",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
