load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "param_util",
    srcs = ["param_util.cc"],
    hdrs = ["param_util.h"],
    deps = [
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
    ],
)

cc_library(
    name = "control_param_integrator",
    srcs = ["control_param_integrator.cc"],
    hdrs = ["control_param_integrator.h"],
    data = [
        "//onboard/control/param/param_file/ts_pkmpc_controller_conf:ts_pkmpc_controller_conf_proto",
    ],
    deps = [
        ":param_util",
        "//onboard/control:control_flags",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/lite:check",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "variable_steer_ratio",
    srcs = ["variable_steer_ratio.cc"],
    hdrs = ["variable_steer_ratio.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/proto:vehicle_cc_proto",
        "@boost//:range",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "param_util_test",
    srcs = ["param_util_test.cc"],
    deps = [
        ":param_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "control_param_integrator_test",
    srcs = ["control_param_integrator_test.cc"],
    deps = [
        ":control_param_integrator",
        "//onboard/base",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "variable_steer_ratio_test",
    srcs = ["variable_steer_ratio_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":variable_steer_ratio",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
