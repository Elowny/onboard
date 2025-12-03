load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "openloop_control",
    srcs = [
        "openloop_control.cc",
    ],
    hdrs = [
        "openloop_control.h",
    ],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control:steering_converter",
        "//onboard/control/openloop_control/proto:openloop_cmd_cc_proto",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "openloop_control_test",
    srcs = ["openloop_control_test.cc"],
    data = [
        "//onboard/control/openloop_control/testdata:openloop_config",
        "//onboard/control/openloop_control/testdata:steer_angle_file",
    ],
    deps = [
        ":openloop_control",
        "//onboard/control:control_defs",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
