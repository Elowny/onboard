load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "controller_common",
    srcs = ["controller_common.cc"],
    hdrs = ["controller_common.h"],
    deps = [
        "//onboard/control:control_cache_manager",
        "//onboard/control:control_defs",
        "//onboard/control:control_flags",
        "//onboard/control:trajectory_interface",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:turn_signal_cc_proto",
        "//onboard/proto:vehicle_signal_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "lat_dm_mpc_controller",
    srcs = ["lat_dm_mpc_controller.cc"],
    hdrs = ["lat_dm_mpc_controller.h"],
    deps = [
        ":controller_common",
        "//onboard/control:control_defs",
        "//onboard/control:steering_converter",
        "//onboard/control:trajectory_interface",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/controllers/model:mpc_cost_constraint",
        "//onboard/control/controllers/model:single_track_dynamic_model",
        "//onboard/control/controllers/model:state_space",
        "//onboard/control/math:mpc_solver",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/control/proto:dynamic_model_conf_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "lat_km_mpc_controller",
    srcs = ["lat_km_mpc_controller.cc"],
    hdrs = ["lat_km_mpc_controller.h"],
    deps = [
        ":controller_common",
        "//onboard/control:control_defs",
        "//onboard/control:steering_converter",
        "//onboard/control:trajectory_interface",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/controllers/model:mpc_cost_constraint",
        "//onboard/control/controllers/model:state_space",
        "//onboard/control/controllers/model:tob_tv_kinematic_model",
        "//onboard/control/math:mpc_solver",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:fast_math",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "lon_mpc_controller",
    srcs = [
        "lon_mpc_controller.cc",
    ],
    hdrs = ["lon_mpc_controller.h"],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control:trajectory_interface",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/math:mpc_solver",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:vec",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "lon_tob_mpc_controller",
    srcs = [
        "lon_tob_mpc_controller.cc",
    ],
    hdrs = ["lon_tob_mpc_controller.h"],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control:control_flags",
        "//onboard/control:trajectory_interface",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/math:mpc_solver",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "controller_util",
    hdrs = [
        "controller_util.h",
    ],
    deps = [
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "predict_vehicle_pose",
    srcs = ["predict_vehicle_pose.cc"],
    hdrs = [
        "predict_vehicle_pose.h",
    ],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control:steering_converter",
        "//onboard/control/controllers/model:single_track_dynamic_model",
        "//onboard/control/controllers/model:state_space",
        "//onboard/control/proto:dynamic_model_conf_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "torque_controller",
    srcs = ["torque_controller.cc"],
    hdrs = [
        "torque_controller.h",
    ],
    deps = [
        ":pid_controller",
        "//onboard/control:control_defs",
        "//onboard/control:control_flags",
        "//onboard/control:steering_converter",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:interpolation_2d",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math/filters:digital_filter",
        "//onboard/math/filters:digital_filter_coefficients",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/utils:file_util",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "pid_controller",
    srcs = [
        "pid_controller.cc",
    ],
    hdrs = [
        "pid_controller.h",
    ],
    deps = ["//onboard/control:control_defs"],
)

# ##############################################################################
# Tests.
cc_test(
    name = "controller_common_test",
    srcs = ["controller_common_test.cc"],
    deps = [
        ":controller_common",
        "//onboard/base",
        "//onboard/control:control_defs",
        "//onboard/control:trajectory_interface",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "controller_util_test",
    srcs = ["controller_util_test.cc"],
    deps = [
        ":controller_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "torque_controller_test",
    srcs = ["torque_controller_test.cc"],
    data = [
        "//onboard/control/testdata/steer_torque:steer_torque_config",
    ],
    deps = [
        ":torque_controller",
        "//onboard/control:control_defs",
        "//onboard/control:steering_converter",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "pid_controller_test",
    srcs = [
        "pid_controller_test.cc",
    ],
    deps = [
        ":pid_controller",
        "//onboard/control:control_defs",
        "//onboard/math:util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lat_km_mpc_controller_test",
    srcs = [
        "lat_km_mpc_controller_test.cc",
    ],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":lat_km_mpc_controller",
        "//onboard/base",
        "//onboard/control:steering_protection",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lat_dm_mpc_controller_test",
    srcs = [
        "lat_dm_mpc_controller_test.cc",
    ],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":lat_dm_mpc_controller",
        "//onboard/base",
        "//onboard/control:steering_protection",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        # "//onboard/control/proto:dynamic_model_conf_cc_proto",
        "//onboard/math:vec",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_test(
    name = "lon_mpc_controller_test",
    srcs = [
        "lon_mpc_controller_test.cc",
    ],
    data = [
        "//onboard/control/testdata/control_input_sample:input_proto",
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":lon_mpc_controller",
        ":lon_tob_mpc_controller",
        "//onboard/control:steering_converter",
        "//onboard/control:trajectory_interface",
        "//onboard/control:vehicle_state_interface",
        "//onboard/control/param:control_param_integrator",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/lite:check",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "predict_vehicle_pose_test",
    srcs = [
        "predict_vehicle_pose_test.cc",
    ],
    deps = [
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/lite:logging",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
