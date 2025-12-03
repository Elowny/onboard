load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "context_groundtruth_extractor",
    srcs = ["context_groundtruth_extractor.cc"],
    hdrs = ["context_groundtruth_extractor.h"],
    deps = [
        "//onboard/planner/ml:planner_ml_defs",
        "//onboard/planner/ml/context_feature_extractor:context_feature",
        "//onboard/planner/ml/context_groundtruth_extractor/proto:context_groundtruth_cc_proto",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
