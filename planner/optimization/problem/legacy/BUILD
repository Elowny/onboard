load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "speed_limit_cost",
    hdrs = ["speed_limit_cost.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner/optimization/problem:cost",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "tob_lateral_jerk_cost",
    hdrs = ["tob_lateral_jerk_cost.h"],
    deps = [
        "//onboard/math:util",
        "//onboard/planner/optimization/problem:cost",
    ],
)

cc_library(
    name = "tob_s_limiting_object_cost",
    hdrs = ["tob_s_limiting_object_cost.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner/optimization/problem:cost",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "tob_speed_limiting_object_cost",
    hdrs = ["tob_speed_limiting_object_cost.h"],
    deps = [
        "//onboard/planner/optimization/problem:cost",
        "//onboard/planner/optimization/problem:third_order_bicycle",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "msd_keep_close_to_center_line_cost",
    hdrs = ["msd_keep_close_to_center_line_cost.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:fast_math",
        "//onboard/math:vec",
        "//onboard/planner/optimization/problem:cost",
        "//onboard/planner/util:min_segment_distance_problem",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "tob_projective_object_cost",
    hdrs = ["tob_projective_object_cost.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner/optimization/problem:cost",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "mfob_gradient_fixed_projective_object_cost",
    hdrs = ["mfob_gradient_fixed_projective_object_cost.h"],
    deps = [
        "//onboard/math/geometry:segment2d",
        "//onboard/planner/optimization/problem:cost",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "mfob_emeraude_object_cost",
    hdrs = ["mfob_emeraude_object_cost.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner/optimization/problem:cost",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "tob_lateral_jerk_cost_test",
    srcs = ["tob_lateral_jerk_cost_test.cc"],
    deps = [
        ":tob_lateral_jerk_cost",
        "//onboard/global:test_main",
        "//onboard/planner/optimization/problem:cost_convergence_test_util",
        "//onboard/planner/optimization/problem:cost_evaluation_test_util",
        "//onboard/planner/optimization/problem:third_order_bicycle",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "speed_limit_cost_test",
    srcs = ["speed_limit_cost_test.cc"],
    deps = [
        ":speed_limit_cost",
        "//onboard/global:test_main",
        "//onboard/planner/optimization/problem:cost_convergence_test_util",
        "//onboard/planner/optimization/problem:cost_evaluation_test_util",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "tob_s_limiting_object_cost_test",
    srcs = ["tob_s_limiting_object_cost_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":tob_s_limiting_object_cost",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/optimization/problem:cost_convergence_test_util",
        "//onboard/planner/optimization/problem:cost_evaluation_test_util",
        "//onboard/planner/optimization/problem:third_order_bicycle",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "msd_keep_close_to_center_line_cost_test",
    srcs = ["msd_keep_close_to_center_line_cost_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":msd_keep_close_to_center_line_cost",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/optimization/problem:cost_convergence_test_util",
        "//onboard/planner/optimization/problem:cost_evaluation_test_util",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "tob_projective_object_cost_test",
    timeout = "long",
    srcs = ["tob_projective_object_cost_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":tob_projective_object_cost",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/optimization/problem:cost_convergence_test_util",
        "//onboard/planner/optimization/problem:cost_evaluation_test_util",
        "//onboard/planner/optimization/problem:third_order_bicycle",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "mfob_emeraude_object_cost_test",
    srcs = ["mfob_emeraude_object_cost_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":mfob_emeraude_object_cost",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/optimization/problem:cost_convergence_test_util",
        "//onboard/planner/optimization/problem:cost_evaluation_test_util",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
