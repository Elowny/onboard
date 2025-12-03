load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "context_feature",
    hdrs = ["context_feature.h"],
    deps = [
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner/ml:planner_ml_defs",
        "//onboard/prediction/feature_extractor:act_net_feature",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "context_feature_extractor",
    srcs = ["context_feature_extractor.cc"],
    hdrs = ["context_feature_extractor.h"],
    deps = [
        ":context_feature",
        ":object_history_sampler",
        "//onboard/global:trace",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math/geometry:box2d",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/ml:planner_ml_defs",
        "//onboard/planner/object:object_vector",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:av_context",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:prediction_object",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/feature_extractor:act_net_feature",
        "//onboard/prediction/feature_extractor:act_net_feature_extractor",
        "//onboard/prediction/feature_extractor:map_sampler",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "object_history_sampler",
    srcs = ["object_history_sampler.cc"],
    hdrs = ["object_history_sampler.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:box2d_cc_proto",
        "//onboard/planner/object:object_vector",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "context_feature_proto_converter",
    srcs = ["context_feature_proto_converter.cc"],
    hdrs = ["context_feature_proto_converter.h"],
    deps = [
        ":context_feature",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/planner/ml/context_feature_extractor/proto:context_feature_cc_proto",
        "//onboard/prediction/feature_extractor:act_net_feature",
        "//onboard/prediction/proto:act_net_cc_proto",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "object_history_sampler_test",
    srcs = ["object_history_sampler_test.cc"],
    deps = [
        ":object_history_sampler",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
