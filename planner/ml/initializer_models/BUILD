load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "complete_motion_form",
    srcs = ["complete_motion_form.cc"],
    hdrs = ["complete_motion_form.h"],
    deps = [
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/initializer:motion_form",
        "//onboard/planner/initializer:motion_state",
        "//onboard/planner/initializer/geometry:geometry_form",
        "//onboard/planner/initializer/geometry:geometry_form_builder",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:trajectory_point_cc_proto",
    ],
)

cc_library(
    name = "initializer_feature_extractor",
    srcs = ["initializer_feature_extractor.cc"],
    hdrs = ["initializer_feature_extractor.h"],
    deps = [
        ":complete_motion_form",
        "//onboard/lite:logging",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/initializer:initializer_output",
        "//onboard/planner/initializer:motion_graph",
        "//onboard/planner/initializer:motion_search_util",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "initializer_post_evaluator",
    srcs = ["initializer_post_evaluator.cc"],
    hdrs = ["initializer_post_evaluator.h"],
    deps = [
        ":complete_motion_form",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/container:small_set",
        "//onboard/eval:qevent",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/math:frenet_frame",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/initializer:cost_provider",
        "//onboard/planner/initializer:dp_motion_searcher_defs",
        "//onboard/planner/initializer:dp_motion_searcher_util",
        "//onboard/planner/initializer:initializer_input",
        "//onboard/planner/initializer:initializer_output",
        "//onboard/planner/initializer:motion_graph",
        "//onboard/planner/initializer:motion_graph_cache",
        "//onboard/planner/initializer:multi_traj_selector",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/utils:history_buffer",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
