load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "condition_feature",
    hdrs = ["condition_feature.h"],
    deps = [
    ],
)

cc_library(
    name = "condition_feature_extractor",
    srcs = ["condition_feature_extractor.cc"],
    hdrs = ["condition_feature_extractor.h"],
    deps = [
        ":condition_feature",
        "//onboard/maps:lane_path",
        "//onboard/maps:maps_common",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/ml/context_feature_extractor:context_feature",
        "//onboard/planner/util:lane_path_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "condition_feature_proto_converter",
    srcs = ["condition_feature_proto_converter.cc"],
    hdrs = ["condition_feature_proto_converter.h"],
    deps = [
        ":condition_feature",
        "//onboard/planner/ml/condition_feature_extractor/proto:condition_feature_cc_proto",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
