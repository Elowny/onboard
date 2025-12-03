load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "act_net_speed_decider",
    srcs = ["act_net_speed_decider.cc"],
    hdrs = ["act_net_speed_decider.h"],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:clock",
        "//onboard/global:logging",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/ml:model_pool",
        "//onboard/planner/ml/context_feature_extractor:object_history_sampler",
        "//onboard/planner/object:spacetime_object_state",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/speed:overlap_info",
        "//onboard/planner/speed:st_boundary",
        "//onboard/planner/speed:st_boundary_with_decision",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:av_context",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/feature_extractor:act_net_speed_feature",
        "//onboard/prediction/feature_extractor:act_net_speed_feature_extractor",
        "//onboard/prediction/inferencer:act_net_speed_inferencer",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:map_util",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
