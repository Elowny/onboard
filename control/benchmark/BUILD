load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "controller_bm",
    srcs = [
        "controller_bm.cc",
    ],
    data = [
        "//onboard/control/testdata/control_input_sample:input_proto",
        "//onboard/params/run_params:run_param_data",
    ],
    tags = ["bm"],
    deps = [
        ":control_test_config_setup",
        "//onboard/control:steering_converter",
        "//onboard/control:steering_protection",
        "//onboard/control:trajectory_interface",
        "//onboard/control:vehicle_state_interface",
        "//onboard/control/controllers:lat_km_mpc_controller",
        "//onboard/control/controllers:lon_mpc_controller",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/lite:check",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_library(
    name = "control_test_config_setup",
    srcs = ["control_test_config_setup.cc"],
    hdrs = ["control_test_config_setup.h"],
    data = [
        "//onboard/control/testdata/control_input_sample:input_proto",
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:file_util",
    ],
)

# ##############################################################################
# Tests.

cc_test(
    name = "control_test_config_setup_test",
    srcs = ["control_test_config_setup_test.cc"],
    deps = [
        ":control_test_config_setup",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
