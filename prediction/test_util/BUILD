load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "object_history_builder",
    testonly = True,
    srcs = ["object_history_builder.cc"],
    hdrs = ["object_history_builder.h"],
    deps = [
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:prediction_object",
        "//onboard/prediction/util:kinematic_model",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "prediction_context_builder",
    testonly = True,
    srcs = ["prediction_context_builder.cc"],
    hdrs = ["prediction_context_builder.h"],
    deps = [
        "//onboard/global:clock",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction/container:av_context",
        "//onboard/prediction/container:objects_history",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/container:prediction_input",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "net_input_generator",
    testonly = True,
    srcs = ["net_input_generator.cc"],
    hdrs = ["net_input_generator.h"],
    deps = [
        "//onboard/nets/proto:net_param_cc_proto",
    ],
)
###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "object_history_builder_test",
    srcs = ["object_history_builder_test.cc"],
    deps = [
        ":object_history_builder",
        "//onboard/global:test_main",
        "//onboard/math/geometry:box2d",
        "//onboard/prediction/container:object_history_span",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "prediction_context_builder_test",
    srcs = ["prediction_context_builder_test.cc"],
    deps = [
        ":prediction_context_builder",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction/container:objects_history",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "net_input_generator_test",
    srcs = ["net_input_generator_test.cc"],
    deps = [
        ":net_input_generator",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
