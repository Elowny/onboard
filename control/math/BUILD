load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "mpc_solver",
    srcs = [
        "mpc_solver.cc",
    ],
    hdrs = [
        "mpc_solver.h",
    ],
    deps = [
        "//onboard/control/controllers/model:mpc_cost_constraint",
        "//onboard/control/controllers/model:state_space",
        "//onboard/global:car_common",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "//onboard/math/qp:osqp_solver",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "coordinate_transform",
    srcs = [
        "coordinate_transform.cc",
    ],
    hdrs = [
        "coordinate_transform.h",
    ],
    deps = [
        "//onboard/math:util",
        "//onboard/proto:trajectory_cc_proto",
    ],
)

cc_library(
    name = "util",
    hdrs = ["util.h"],
    deps = [
        "@boost//:circular_buffer",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "mpc_solver_test",
    size = "small",
    srcs = [
        "mpc_solver_test.cc",
    ],
    deps = [
        ":mpc_solver",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "coordinate_transform_test",
    srcs = [
        "coordinate_transform_test.cc",
    ],
    deps = [
        ":coordinate_transform",
        "//onboard/math:util",
        "//onboard/proto:chassis_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "util_test",
    srcs = [
        "util_test.cc",
    ],
    deps = [
        ":util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
