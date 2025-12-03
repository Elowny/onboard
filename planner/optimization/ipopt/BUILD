load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "ipopt_optimizer_debug_hook",
    hdrs = ["ipopt_optimizer_debug_hook.h"],
    deps = [
        "//onboard/math:eigen",
        "//onboard/math:vec",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "ipopt_optimizer",
    hdrs = ["ipopt_optimizer.h"],
    copts = ["-DHAVE_CSTDDEF"],
    deps = [
        ":ipopt_optimizer_debug_hook",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:trajectory_point",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@ipopt",
    ],
)

cc_library(
    name = "ipopt_adapter",
    hdrs = ["ipopt_adapter.h"],
    deps = [
        ":ipopt_optimizer",
        ":ipopt_util",
        "//onboard/planner:trajectory_point",
        "@com_google_absl//absl/status:statusor",
        "@ipopt",
    ],
)

cc_library(
    name = "ipopt_util",
    srcs = ["ipopt_util.cc"],
    hdrs = ["ipopt_util.h"],
    deps = [
        "@ipopt",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "ipopt_optimizer_test",
    srcs = ["ipopt_optimizer_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":ipopt_adapter",
        ":ipopt_optimizer_debug_hook",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/planner/optimization/problem:curvature_cost",
        "//onboard/planner/optimization/problem:intrinsic_jerk_cost",
        "//onboard/planner/optimization/problem:longitudinal_acceleration_cost",
        "//onboard/planner/optimization/problem:mfob_curvature_rate_cost",
        "//onboard/planner/optimization/problem:mfob_curvature_rate_rate_cost",
        "//onboard/planner/optimization/problem:reference_control_deviation_cost",
        "//onboard/planner/optimization/problem:reference_line_deviation_cost",
        "//onboard/planner/optimization/problem:reference_state_deviation_cost",
        "//onboard/planner/optimization/problem:segmented_speed_limit_cost",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:trajectory_plot_util",
        "@com_google_googletest//:gtest",
        "@ipopt",
    ],
)

cpplint()
