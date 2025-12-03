load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "cost",
    hdrs = ["cost.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "cost_helper",
    hdrs = ["cost_helper.h"],
    deps = [
        "//onboard/lite:check",
    ],
)

cc_library(
    name = "third_order_bicycle",
    hdrs = ["third_order_bicycle.h"],
    deps = [
        ":cost",
        ":cost_helper",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/lite:logging",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:trajectory_point",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "mixed_fourth_order_bicycle",
    hdrs = ["mixed_fourth_order_bicycle.h"],
    deps = [
        ":cost",
        ":cost_helper",
        "//onboard/eval:qevent",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "//onboard/math:fast_math",
        "//onboard/planner:planner_defs",
        "//onboard/planner:trajectory_point",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "cost_convergence_test_util",
    testonly = True,
    hdrs = ["cost_convergence_test_util.h"],
    deps = [
        ":cost",
        "//onboard/math:convergence_order",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "cost_evaluation_test_util",
    testonly = True,
    hdrs = ["cost_evaluation_test_util.h"],
    deps = [
        ":cost",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "longitudinal_acceleration_cost",
    hdrs = ["longitudinal_acceleration_cost.h"],
    deps = [
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "lateral_acceleration_cost",
    hdrs = ["lateral_acceleration_cost.h"],
    deps = [
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "mfob_lateral_jerk_cost",
    hdrs = ["mfob_lateral_jerk_cost.h"],
    deps = [
        ":cost",
        "//onboard/math:util",
        "@eigen",
    ],
)

cc_library(
    name = "mfob_intrinsic_lateral_snap_cost",
    hdrs = ["mfob_intrinsic_lateral_snap_cost.h"],
    deps = [
        ":cost",
        "//onboard/math:util",
        "@eigen",
    ],
)

cc_library(
    name = "partitioned_object_cost",
    hdrs = ["partitioned_object_cost.h"],
    deps = [
        ":av_model_helper",
        ":cost",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "unidirectional_object_cost",
    hdrs = ["unidirectional_object_cost.h"],
    deps = [
        ":cost",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "aggregate_static_object_cost",
    hdrs = ["aggregate_static_object_cost.h"],
    deps = [
        ":cost",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "scatter_object_cost",
    hdrs = ["scatter_object_cost.h"],
    deps = [
        ":cost",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "longitudinal_jerk_cost",
    hdrs = ["longitudinal_jerk_cost.h"],
    deps = [
        ":cost",
        ":third_order_bicycle",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "intrinsic_jerk_cost",
    hdrs = ["intrinsic_jerk_cost.h"],
    deps = [
        ":cost",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "curvature_cost",
    hdrs = ["curvature_cost.h"],
    deps = [
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "tob_curvature_rate_cost",
    hdrs = ["tob_curvature_rate_cost.h"],
    deps = [
        ":cost",
        ":third_order_bicycle",
    ],
)

cc_library(
    name = "mfob_curvature_rate_cost",
    hdrs = ["mfob_curvature_rate_cost.h"],
    deps = [
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/math:util",
        "@eigen",
    ],
)

cc_library(
    name = "mfob_curvature_rate_rate_cost",
    hdrs = ["mfob_curvature_rate_rate_cost.h"],
    deps = [
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/math:util",
        "@eigen",
    ],
)

cc_library(
    name = "center_line_query_helper",
    hdrs = ["center_line_query_helper.h"],
    deps = [
        ":cost_helper",
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "//onboard/math:frenet_frame",
        "//onboard/planner/proto:planner_params_cc_proto",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "av_model_helper",
    hdrs = ["av_model_helper.h"],
    deps = [
        ":cost_helper",
        "//onboard/math:eigen",
        "//onboard/math:frenet_frame",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "reference_line_deviation_cost",
    hdrs = ["reference_line_deviation_cost.h"],
    deps = [
        ":center_line_query_helper",
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "segmented_speed_limit_cost",
    hdrs = ["segmented_speed_limit_cost.h"],
    deps = [
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "segmented_speed_limit_cost_v2",
    hdrs = ["segmented_speed_limit_cost_v2.h"],
    deps = [
        ":center_line_query_helper",
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:logging",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "backward_speed_cost",
    hdrs = ["backward_speed_cost.h"],
    deps = [
        ":cost",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "forward_speed_cost",
    hdrs = ["forward_speed_cost.h"],
    deps = [
        ":cost",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "reference_state_deviation_cost",
    hdrs = ["reference_state_deviation_cost.h"],
    deps = [
        ":cost",
        ":mixed_fourth_order_bicycle",
        ":third_order_bicycle",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "reference_control_deviation_cost",
    hdrs = ["reference_control_deviation_cost.h"],
    deps = [
        ":cost",
        ":mixed_fourth_order_bicycle",
        ":third_order_bicycle",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "reference_single_state_deviation_cost",
    hdrs = ["reference_single_state_deviation_cost.h"],
    deps = [
        ":cost",
        ":mixed_fourth_order_bicycle",
        ":third_order_bicycle",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "end_heading_cost",
    hdrs = ["end_heading_cost.h"],
    deps = [
        ":center_line_query_helper",
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "mfob_path_boundary_cost",
    hdrs = ["mfob_path_boundary_cost.h"],
    deps = [
        ":center_line_query_helper",
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/base",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "static_boundary_cost",
    hdrs = ["static_boundary_cost.h"],
    deps = [
        ":cost",
        "//onboard/base",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner/optimization/problem:center_line_query_helper",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "static_boundary_cost_lite",
    hdrs = ["static_boundary_cost_lite.h"],
    deps = [
        ":cost",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "msd_static_boundary_cost",
    hdrs = ["msd_static_boundary_cost.h"],
    deps = [
        ":center_line_query_helper",
        ":cost",
        "//onboard/planner/util:min_segment_distance_problem",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "msd_static_boundary_cost_v2",
    hdrs = ["msd_static_boundary_cost_v2.h"],
    deps = [
        ":center_line_query_helper",
        ":cost",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner/util:min_segment_distance_problem",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "curvature_deviation_cost",
    hdrs = ["curvature_deviation_cost.h"],
    deps = [
        ":cost",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "reference_longitudinal_jerk_deviation_cost",
    hdrs = ["reference_longitudinal_jerk_deviation_cost.h"],
    deps = [
        ":cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/math:util",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "partitioned_static_boundary_cost",
    hdrs = ["partitioned_static_boundary_cost.h"],
    deps = [
        ":cost",
        ":third_order_bicycle",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.
cc_binary(
    name = "cost_bm",
    srcs = ["cost_bm.cc"],
    tags = ["bm"],
    deps = [
        ":reference_control_deviation_cost",
        ":reference_state_deviation_cost",
        "@com_github_google_benchmark//:benchmark",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "third_order_bicycle_test",
    srcs = ["third_order_bicycle_test.cc"],
    deps = [
        ":third_order_bicycle",
        "//onboard/global:test_main",
        "//onboard/math:convergence_order",
        "//onboard/planner/test_util:util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "mixed_fourth_order_bicycle_test",
    srcs = ["mixed_fourth_order_bicycle_test.cc"],
    deps = [
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "//onboard/math:convergence_order",
        "//onboard/planner/test_util:util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "longitudinal_acceleration_cost_test",
    srcs = ["longitudinal_acceleration_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":longitudinal_acceleration_cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "lateral_acceleration_cost_test",
    srcs = ["lateral_acceleration_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":lateral_acceleration_cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "mfob_lateral_jerk_cost_test",
    srcs = ["mfob_lateral_jerk_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mfob_lateral_jerk_cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "mfob_intrinsic_lateral_snap_cost_test",
    srcs = ["mfob_intrinsic_lateral_snap_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mfob_intrinsic_lateral_snap_cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "longitudinal_jerk_cost_test",
    srcs = ["longitudinal_jerk_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":longitudinal_jerk_cost",
        ":third_order_bicycle",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "curvature_cost_test",
    srcs = ["curvature_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":curvature_cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "curvature_deviation_cost_test",
    srcs = ["curvature_deviation_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":curvature_deviation_cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "mfob_curvature_rate_cost_test",
    srcs = ["mfob_curvature_rate_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mfob_curvature_rate_cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "mfob_curvature_rate_rate_cost_test",
    srcs = ["mfob_curvature_rate_rate_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mfob_curvature_rate_rate_cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "reference_line_deviation_cost_test",
    srcs = ["reference_line_deviation_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":reference_line_deviation_cost",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "end_heading_cost_test",
    srcs = ["end_heading_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":end_heading_cost",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "segmented_speed_limit_cost_test",
    srcs = ["segmented_speed_limit_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":segmented_speed_limit_cost",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "segmented_speed_limit_cost_v2_test",
    srcs = ["segmented_speed_limit_cost_v2_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":segmented_speed_limit_cost_v2",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "reference_state_deviation_cost_test",
    srcs = ["reference_state_deviation_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":reference_state_deviation_cost",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "reference_control_deviation_cost_test",
    srcs = ["reference_control_deviation_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":reference_control_deviation_cost",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "reference_single_state_deviation_cost_test",
    srcs = ["reference_single_state_deviation_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":reference_single_state_deviation_cost",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "mfob_path_boundary_cost_test",
    srcs = ["mfob_path_boundary_cost_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mfob_path_boundary_cost",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "static_boundary_cost_test",
    srcs = ["static_boundary_cost_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":static_boundary_cost",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "static_boundary_cost_lite_test",
    srcs = ["static_boundary_cost_lite_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":static_boundary_cost_lite",
        ":third_order_bicycle",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/test_util:util",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "msd_static_boundary_cost_test",
    srcs = ["msd_static_boundary_cost_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":msd_static_boundary_cost",
        ":third_order_bicycle",
        "//onboard/global:test_main",
        "//onboard/math/geometry:segment2d",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "msd_static_boundary_cost_v2_test",
    srcs = ["msd_static_boundary_cost_v2_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":msd_static_boundary_cost_v2",
        "//onboard/global:test_main",
        "//onboard/math/geometry:segment2d",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "partitioned_object_cost_test",
    srcs = ["partitioned_object_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":partitioned_object_cost",
        ":third_order_bicycle",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "unidirectional_object_cost_test",
    srcs = ["unidirectional_object_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":unidirectional_object_cost",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "aggregate_static_object_cost_test",
    srcs = ["aggregate_static_object_cost_test.cc"],
    deps = [
        ":aggregate_static_object_cost",
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "scatter_object_cost_test",
    srcs = ["scatter_object_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":scatter_object_cost",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "reference_longitudinal_jerk_deviation_cost_test",
    srcs = ["reference_longitudinal_jerk_deviation_cost_test.cc"],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":mixed_fourth_order_bicycle",
        ":reference_longitudinal_jerk_deviation_cost",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_test(
    name = "partitioned_static_boundary_cost_test",
    srcs = ["partitioned_static_boundary_cost_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":cost_convergence_test_util",
        ":cost_evaluation_test_util",
        ":partitioned_static_boundary_cost",
        ":third_order_bicycle",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
