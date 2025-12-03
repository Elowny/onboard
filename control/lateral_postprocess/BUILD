load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mrac_control",
    srcs = ["mrac_control.cc"],
    hdrs = ["mrac_control.h"],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math/filters:digital_filter",
        "//onboard/math/filters:digital_filter_coefficients",
        "//onboard/proto:control_cmd_cc_proto",
        "@boost//:circular_buffer",
        "@boost//:move",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_library(
    name = "steer_calibration",
    srcs = ["steer_calibration.cc"],
    hdrs = [
        "steer_calibration.h",
    ],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control:steering_converter",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
    ],
)

cc_library(
    name = "compensate_curvature_gain",
    srcs = ["compensate_curvature_gain.cc"],
    hdrs = [
        "compensate_curvature_gain.h",
    ],
    deps = [
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
    ],
)

# ##############################################################################
# Tests.

cc_test(
    name = "mrac_control_test",
    srcs = ["mrac_control_test.cc"],
    data = glob([
        "testdata/**",
    ]),
    deps = [
        ":mrac_control",
        "//onboard/control:control_defs",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "steer_calibration_test",
    srcs = ["steer_calibration_test.cc"],
    deps = [
        ":steer_calibration",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "compensate_curvature_gain_test",
    srcs = ["compensate_curvature_gain_test.cc"],
    deps = [
        ":compensate_curvature_gain",
        "//onboard/proto:control_cmd_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
