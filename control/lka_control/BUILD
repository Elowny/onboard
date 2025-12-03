load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lka_control",
    srcs = ["lka_control.cc"],
    hdrs = ["lka_control.h"],
    data = [
        "//onboard/lane_support/params:param_files",
    ],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control:control_flags",
        "//onboard/control:steering_converter",
        "//onboard/control:steering_protection",
        "//onboard/control:trajectory_interface",
        "//onboard/control:vehicle_state_interface",
        "//onboard/control/controllers:controller_common",
        "//onboard/control/controllers:lat_km_mpc_controller",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/controllers:torque_controller",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "path_to_trajectory",
    srcs = ["path_to_trajectory.cc"],
    hdrs = ["path_to_trajectory.h"],
    deps = [
        "//onboard/global:clock",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/planner:planner_defs",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:turn_signal_cc_proto",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
    ],
)

cc_test(
    name = "path_to_trajectory_test",
    srcs = ["path_to_trajectory_test.cc"],
    deps = [
        ":path_to_trajectory",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lka_control_test",
    srcs = ["lka_control_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":lka_control",
        ":path_to_trajectory",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/global:test_main",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:file_util",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
