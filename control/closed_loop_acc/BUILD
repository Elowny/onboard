load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed_mode_manager",
    srcs = [
        "speed_mode_manager.cc",
    ],
    hdrs = [
        "speed_mode_manager.h",
    ],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
    ],
)

cc_library(
    name = "closed_loop_acc",
    srcs = [
        "closed_loop_acc.cc",
    ],
    hdrs = [
        "closed_loop_acc.h",
    ],
    deps = [
        "//onboard/proto:control_cmd_cc_proto",
    ],
)

cc_test(
    name = "speed_mode_manager_test",
    srcs = ["speed_mode_manager_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":speed_mode_manager",
        "//onboard/global:test_main",
        "//onboard/proto:control_cmd_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
