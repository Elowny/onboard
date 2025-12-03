load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "vehicle_control_module",
    srcs = ["vehicle_control_module.cc"],
    hdrs = ["vehicle_control_module.h"],
    deps = [
        ":control_cache_manager",
        ":control_defs",
        ":control_flags",
        ":control_monitoring",
        ":control_validation",
        ":controller_agent",
        ":steering_converter",
        ":steering_protection",
        ":trajectory_interface",
        ":vehicle_state_interface",
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/control/closed_loop_acc:speed_mode_manager",
        "//onboard/control/control_check:lat_wire_control_check",
        "//onboard/control/control_check:lon_wire_control_check",
        "//onboard/control/control_parking:control_parking_manager",
        "//onboard/control/controllers:controller_common",
        "//onboard/control/controllers:torque_controller",
        "//onboard/control/lateral_postprocess:mrac_control",
        "//onboard/control/lateral_postprocess:steer_calibration",
        "//onboard/control/longitudinal_postprocess:lon_postprocess",
        "//onboard/control/openloop_control",
        "//onboard/control/param:control_param_integrator",
        "//onboard/control/parameter_identification",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/global:counter",
        "//onboard/global:run_context",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:lite_client_base",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/lite:qissue_trans",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/math:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/params/dynamic_param:dynamic_param_cc_proto",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:q_issue_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_builder",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "control_cache_manager",
    srcs = ["control_cache_manager.cc"],
    hdrs = [
        "control_cache_manager.h",
    ],
    deps = [
        ":control_defs",
        ":steering_converter",
        ":trajectory_interface",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "@boost//:circular_buffer",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "controller_agent",
    srcs = [
        "controller_agent.cc",
    ],
    hdrs = [
        "controller_agent.h",
    ],
    deps = [
        ":control_cache_manager",
        ":control_defs",
        ":control_monitoring",
        ":steering_converter",
        ":trajectory_interface",
        "//onboard/control:control_flags",
        "//onboard/control/control_parking:control_parking_manager",
        "//onboard/control/controllers:lat_dm_mpc_controller",
        "//onboard/control/controllers:lat_km_mpc_controller",
        "//onboard/control/controllers:lon_mpc_controller",
        "//onboard/control/controllers:lon_tob_mpc_controller",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/global:logging",
        "//onboard/lite:check",
        "//onboard/lite:qissue_trans",
        "//onboard/math:util",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:q_issue_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "control_flags",
    srcs = ["control_flags.cc"],
    hdrs = ["control_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "steering_protection",
    srcs = ["steering_protection.cc"],
    hdrs = ["steering_protection.h"],
    deps = [
        ":control_cache_manager",
        ":control_defs",
        ":steering_converter",
        "//offboard/vis/vantage/charts:chart_util",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/vis/common:color",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "steering_converter",
    srcs = ["steering_converter.cc"],
    hdrs = ["steering_converter.h"],
    deps = [
        "//onboard/control/param:variable_steer_ratio",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "control_defs",
    hdrs = ["control_defs.h"],
)

cc_library(
    name = "trajectory_interface",
    srcs = [
        "trajectory_interface.cc",
    ],
    hdrs = [
        "trajectory_interface.h",
    ],
    deps = [
        ":control_flags",
        ":control_util",
        "//onboard/control/control_parking:parking_path_to_trajectory",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "vehicle_state_interface",
    srcs = [
        "vehicle_state_interface.cc",
    ],
    hdrs = [
        "vehicle_state_interface.h",
    ],
    deps = [
        ":steering_converter",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/localization/visual:visual_localization_constants",
        "//onboard/localization/visual/proto:data_type_cc_proto",
        "//onboard/math:util",
        "//onboard/math/filters:exponential_smoothing",
        "//onboard/math/filters:mean_filter",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "control_validation",
    srcs = [
        "control_validation.cc",
    ],
    hdrs = [
        "control_validation.h",
    ],
    deps = [
        ":control_defs",
        ":steering_converter",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "control_monitoring",
    srcs = ["control_monitoring.cc"],
    hdrs = [
        "control_monitoring.h",
    ],
    deps = [
        ":control_cache_manager",
        ":control_defs",
        ":steering_converter",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:counter",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:positioning_cc_proto",
    ],
)

cc_library(
    name = "control_util",
    srcs = ["control_util.cc"],
    hdrs = [
        "control_util.h",
    ],
    deps = [
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
    ],
)
###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "steering_protection_test",
    srcs = ["steering_protection_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":control_defs",
        ":steering_protection",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "trajectory_interface_test",
    srcs = ["trajectory_interface_test.cc"],
    deps = [
        ":control_flags",
        ":trajectory_interface",
        "//onboard/base",
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/utils:time_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "control_cache_manager_test",
    srcs = ["control_cache_manager_test.cc"],
    deps = [
        ":control_cache_manager",
        ":control_defs",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "control_monitoring_test",
    srcs = ["control_monitoring_test.cc"],
    deps = [
        ":control_cache_manager",
        ":control_defs",
        ":control_monitoring",
        ":steering_converter",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval/collectors:qevent_collector",
        "//onboard/eval/collectors:qevent_log_collector",
        "//onboard/eval/proto:qevent_cc_proto",
        "//onboard/eval/proto:qevents_lite_cc_proto",
        "//onboard/global:counter",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:counter_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "control_validation_test",
    srcs = ["control_validation_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":control_defs",
        ":control_validation",
        ":steering_converter",
        ":steering_protection",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "steering_converter_test",
    srcs = ["steering_converter_test.cc"],
    deps = [
        ":steering_converter",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "control_util_test",
    srcs = ["control_util_test.cc"],
    deps = [
        ":control_util",
        "//onboard/global:test_main",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "vehicle_state_interface_test",
    srcs = ["vehicle_state_interface_test.cc"],
    deps = [
        ":steering_converter",
        ":vehicle_state_interface",
        "//onboard/global:test_main",
        "//onboard/localization/visual/proto:data_type_cc_proto",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "vehicle_control_module_test",
    srcs = ["vehicle_control_module_test.cc"],
    deps = [
        ":vehicle_control_module",
        "//onboard/base",
        "//onboard/control/benchmark:control_test_config_setup",
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/unittest:lite2_unittest_helper",
        "//onboard/utils:time_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
