load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "parameter_identification",
    srcs = [
        "parameter_identification.cc",
    ],
    hdrs = [
        "parameter_identification.h",
    ],
    deps = [
        "//onboard/control:control_cache_manager",
        "//onboard/control:control_defs",
        "//onboard/control:steering_converter",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:clock",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math/filters:exponential_smoothing",
        "//onboard/math/proto:piecewise_linear_function_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/utils:time_util",
        "@boost//:circular_buffer",
    ],
)

cc_test(
    name = "parameter_identification_test",
    srcs = ["parameter_identification_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":parameter_identification",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
