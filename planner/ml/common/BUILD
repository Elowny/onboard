load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "feature_extraction_utils",
    srcs = ["feature_extraction_utils.cc"],
    hdrs = ["feature_extraction_utils.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/planner:est_planner_output",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/util:lane_path_util",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "data_filtering_utils",
    srcs = ["data_filtering_utils.cc"],
    hdrs = ["data_filtering_utils.h"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:frenet_common",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:trajectory_point",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
