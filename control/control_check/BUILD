load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lon_wire_control_check",
    srcs = ["lon_wire_control_check.cc"],
    hdrs = ["lon_wire_control_check.h"],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control/math:util",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "lat_wire_control_check",
    srcs = ["lat_wire_control_check.cc"],
    hdrs = ["lat_wire_control_check.h"],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control:steering_converter",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
        "@boost//:circular_buffer",
        "@boost//:move",
    ],
)

cc_test(
    name = "lon_wire_control_check_test",
    srcs = ["lon_wire_control_check_test.cc"],
    deps = [
        ":lon_wire_control_check",
        "//onboard/control:control_defs",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lat_wire_control_check_test",
    srcs = ["lat_wire_control_check_test.cc"],
    deps = [
        ":lat_wire_control_check",
        "//onboard/control:control_defs",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
