load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_calibration_module",
    srcs = ["control_calibration_module.cc"],
    hdrs = ["control_calibration_module.h"],
    deps = [
        "//onboard/control/calibration/calibration_tools",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/global:car_common",
        "//onboard/lite:check",
        "//onboard/lite:lite_client_base",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/utils:status_macros",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "control_calibration_module_test",
    srcs = ["control_calibration_module_test.cc"],
    deps = [
        ":control_calibration_module",
        "//onboard/global:test_main",
        "//onboard/lite/unittest:lite2_unittest_helper",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
