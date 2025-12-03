load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "spiral",
    srcs = ["spiral.cc"],
    hdrs = ["spiral.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/math:fast_math",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "cubic_spiral_boundary_value_problem",
    srcs = ["cubic_spiral_boundary_value_problem.cc"],
    hdrs = ["cubic_spiral_boundary_value_problem.h"],
    deps = [
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner/math:spiral",
        "@com_github_yixuan_lbfgspp//:lbfgs",
    ],
)

cc_library(
    name = "quintic_spiral_boundary_value_problem",
    srcs = ["quintic_spiral_boundary_value_problem.cc"],
    hdrs = ["quintic_spiral_boundary_value_problem.h"],
    deps = [
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner/math:spiral",
        "@com_github_yixuan_lbfgspp//:lbfgs",
    ],
)

cc_library(
    name = "intelligent_driver_model",
    srcs = ["intelligent_driver_model.cc"],
    hdrs = ["intelligent_driver_model.h"],
)

###############################################################################
# Tests.

cc_test(
    name = "spiral_test",
    srcs = ["spiral_test.cc"],
    deps = [
        ":spiral",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "intelligent_driver_model_test",
    srcs = ["intelligent_driver_model_test.cc"],
    deps = [
        ":intelligent_driver_model",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "cubic_spiral_boundary_value_problem_test",
    srcs = ["cubic_spiral_boundary_value_problem_test.cc"],
    deps = [
        ":cubic_spiral_boundary_value_problem",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "quintic_spiral_boundary_value_problem_test",
    srcs = ["quintic_spiral_boundary_value_problem_test.cc"],
    deps = [
        ":quintic_spiral_boundary_value_problem",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
