load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "captain_net",
    hdrs = [
        "captain_net.h",
    ],
    deps = [
        "//onboard/planner:trajectory_point",
        "//onboard/planner/ml/captain_net/proto:captain_net_debug_cc_proto",
    ],
)

cc_library(
    name = "captain_net_generator",
    srcs = ["captain_net_generator.cc"],
    hdrs = ["captain_net_generator.h"],
    deps = [
        ":captain_net",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:trace",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:plan_start_point_info",
        "//onboard/planner/common:planner_status",
        "//onboard/planner/ml:model_pool",
        "//onboard/planner/ml/captain_net/inference:input_assemble",
        "//onboard/planner/ml/captain_net/post_process",
        "//onboard/planner/ml/captain_net/post_process:model_output_process",
        "//onboard/planner/ml/captain_net/post_process:trajectory_alignment",
        "//onboard/planner/ml/captain_net/validation:trajectory_validation",
        "//onboard/planner/ml/condition_feature_extractor",
        "//onboard/planner/ml/context_feature_extractor:context_feature",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/status",
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
