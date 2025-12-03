load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "piecewise_jerk_qp_solver",
    srcs = ["piecewise_jerk_qp_solver.cc"],
    hdrs = ["piecewise_jerk_qp_solver.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math/qp:osqp_return_status",
        "//onboard/planner:planner_flags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
        "@osqp",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "piecewise_jerk_qp_solver_test",
    srcs = ["piecewise_jerk_qp_solver_test.cc"],
    deps = [
        ":piecewise_jerk_qp_solver",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
