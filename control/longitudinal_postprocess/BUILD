load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lon_postprocess",
    srcs = ["lon_postprocess.cc"],
    hdrs = ["lon_postprocess.h"],
    deps = [
        ":calibration_manager",
        "//onboard/control:control_defs",
        "//onboard/control:control_flags",
        "//onboard/control:steering_protection",
        "//onboard/control:trajectory_interface",
        "//onboard/control/closed_loop_acc:speed_mode_manager",
        "//onboard/control/controllers:controller_common",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math/filters:mean_filter",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "calibration_manager",
    srcs = ["calibration_manager.cc"],
    hdrs = ["calibration_manager.h"],
    deps = [
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:interpolation_2d",
        "//onboard/math:piecewise_linear_function",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "speed_mode_manager_v2",
    srcs = ["speed_mode_manager_v2.cc"],
    hdrs = ["speed_mode_manager_v2.h"],
    deps = [
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "lon_postprocess_test",
    srcs = ["lon_postprocess_test.cc"],
    deps = [
        ":lon_postprocess",
        "//onboard/control:control_defs",
        "//onboard/control:control_flags",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/math/proto:piecewise_linear_function_cc_proto",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "calibration_manager_test",
    srcs = ["calibration_manager_test.cc"],
    deps = [
        ":calibration_manager",
        "//onboard/global:test_main",
        "//onboard/math/proto:piecewise_linear_function_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "speed_mode_manager_v2_test",
    srcs = ["speed_mode_manager_v2_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":speed_mode_manager_v2",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
    ],
)

cpplint()
