load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "long_term_behavior_estimator",
    srcs = ["long_term_behavior_estimator.cc"],
    hdrs = ["long_term_behavior_estimator.h"],
    deps = [
        ":slow_front_vehicle_recognizer",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/prediction/container:av_context",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:object_history_span",
        "//onboard/prediction/container:prediction_object",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:elements_history",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "slow_front_vehicle_recognizer",
    srcs = ["slow_front_vehicle_recognizer.cc"],
    hdrs = ["slow_front_vehicle_recognizer.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/util:perception_util",
        "//onboard/prediction/container:av_context",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:object_history_span",
        "//onboard/prediction/container:prediction_object",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:elements_history",
        "@boost//:circular_buffer",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "long_term_behavior_estimator_test",
    srcs = ["long_term_behavior_estimator_test.cc"],
    deps = [
        ":long_term_behavior_estimator",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "slow_front_vehicle_recognizer_test",
    srcs = ["slow_front_vehicle_recognizer_test.cc"],
    deps = [
        ":slow_front_vehicle_recognizer",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
