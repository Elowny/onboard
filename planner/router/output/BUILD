load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Protos

# Libraries.
cc_library(
    name = "routing_module_output",
    srcs = ["routing_module_output.cc"],
    hdrs = ["routing_module_output.h"],
    deps = [
        "//onboard/lite:lite_module",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/proto:hmi_content_cc_proto",
        "//onboard/proto:route_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

# Tests
cc_test(
    name = "routing_module_output_test",
    srcs = ["routing_module_output_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":routing_module_output",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)
