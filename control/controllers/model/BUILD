load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "state_space",
    srcs = ["state_space.cc"],
    hdrs = ["state_space.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "//onboard/math:util",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "tob_tv_kinematic_model",
    srcs = ["tob_tv_kinematic_model.cc"],
    hdrs = ["tob_tv_kinematic_model.h"],
    deps = [
        ":state_space",
        "//onboard/lite:check",
        "//onboard/math:eigen",
        "//onboard/math:fast_math",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math/proto:piecewise_linear_function_cc_proto",
    ],
)

cc_library(
    name = "mpc_cost_constraint",
    srcs = ["mpc_cost_constraint.cc"],
    hdrs = ["mpc_cost_constraint.h"],
    deps = [
        ":state_space",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "single_track_dynamic_model",
    srcs = ["single_track_dynamic_model.cc"],
    hdrs = ["single_track_dynamic_model.h"],
    deps = [
        ":state_space",
        "//onboard/control:control_defs",
        "//onboard/control/proto:dynamic_model_conf_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "@eigen",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "state_space_test",
    srcs = ["state_space_test.cc"],
    deps = [
        ":state_space",
        "//onboard/global:test_main",
        "//onboard/math:eigen",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "tob_tv_kinematic_model_test",
    srcs = ["tob_tv_kinematic_model_test.cc"],
    deps = [
        ":tob_tv_kinematic_model",
        "//onboard/global:test_main",
        "//onboard/math:eigen",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "mpc_cost_constraint_test",
    srcs = ["mpc_cost_constraint_test.cc"],
    deps = [
        ":mpc_cost_constraint",
        ":state_space",
        "//onboard/global:test_main",
        "//onboard/math:eigen",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "single_track_dynamic_model_test",
    srcs = ["single_track_dynamic_model_test.cc"],
    deps = [
        ":single_track_dynamic_model",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cpplint()
