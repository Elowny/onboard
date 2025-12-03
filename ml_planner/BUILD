load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "ml_planner_module",
    srcs = ["ml_planner_module.cc"],
    hdrs = ["ml_planner_module.h"],
    deps = [
        ":ml_planner_defs",
        ":ml_planner_flags",
        ":ml_planner_input",
        ":ml_task_dispatcher",
        "//onboard/global:run_context",
        "//onboard/lite:lite_client_base",
        "//onboard/lite:lite_module",
        "//onboard/maps:online_map_gflags",
        "//onboard/maps:semantic_map_io",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:online_semantic_map_cc_proto",
        "//onboard/maps/v2:semantic_map_listener",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/maps/v2:semantic_map_spatial_index",
        "//onboard/ml_planner/proto:ml_planner_debug_cc_proto",
        "//onboard/ml_planner/proto:ml_planner_output_cc_proto",
        "//onboard/planner:planner_main_loop_internal",
        "//onboard/planner:planner_state",
        "//onboard/planner/util:planner_semantic_map_manager_builder",
        "//onboard/prediction:prediction_message_compressor",
        "//onboard/proto:positioning_cc_proto",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "ml_planner_input",
    hdrs = ["ml_planner_input.h"],
    deps = [
        "//onboard/params:vehicle_param_api",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/ml:model_pool",
        "//onboard/prediction/container:av_context",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
    ],
)

cc_library(
    name = "ml_planner_flags",
    srcs = ["ml_planner_flags.cc"],
    hdrs = ["ml_planner_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "ml_planner_defs",
    hdrs = ["ml_planner_defs.h"],
)

cc_library(
    name = "ml_task_dispatcher",
    srcs = ["ml_task_dispatcher.cc"],
    hdrs = ["ml_task_dispatcher.h"],
    deps = [
        ":ml_cruise_task",
        ":ml_planner_flags",
        ":ml_planner_input",
        ":ml_planner_status",
        "//onboard/async:thread_pool",
        "//onboard/ml_planner/proto:ml_planner_debug_cc_proto",
        "//onboard/ml_planner/proto:ml_planner_output_cc_proto",
        "//onboard/planner/object:low_likelihood_filter",
        "//onboard/planner/object:motion_state_filter",
        "//onboard/planner/object:planner_object_manager_builder",
        "//onboard/planner/object:predicted_motion_filter",
        "//onboard/planner/object:reflected_object_in_proximity_filter",
        "//onboard/planner/object:unknown_roadway_position_object_filter",
        "//onboard/planner/plan:plan_task_helper",
    ],
)

cc_library(
    name = "ml_planner_status",
    hdrs = ["ml_planner_status.h"],
    deps = [
        "//onboard/ml_planner/proto:ml_planner_status_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "ml_cruise_task_input",
    hdrs = ["ml_cruise_task_input.h"],
    deps = [
        ":ml_planner_input",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
    ],
)

cc_library(
    name = "ml_cruise_task_output",
    hdrs = ["ml_cruise_task_output.h"],
    deps = [
    ],
)

cc_library(
    name = "ml_cruise_task",
    srcs = ["ml_cruise_task.cc"],
    hdrs = ["ml_cruise_task.h"],
    deps = [
        ":ml_cruise_task_input",
        ":ml_planner_input",
        ":ml_planner_status",
        ":multi_conditions_ml_cruise_planner",
        ":multi_conditions_ml_cruise_planner_input",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:lane_path_cc_proto",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/ml_planner/proto:ml_planner_debug_cc_proto",
        "//onboard/ml_planner/proto:ml_planner_output_cc_proto",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/ml/context_feature_extractor",
        "//onboard/planner/ml/context_feature_extractor:context_feature",
        "//onboard/planner/util:lane_point_util",
        "//onboard/planner/util:spatial_search_util",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/util:drive_passage_util",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/proto:positioning_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "multi_conditions_ml_cruise_planner_input",
    hdrs = ["multi_conditions_ml_cruise_planner_input.h"],
    deps = [
        ":ml_planner_input",
        "//onboard/planner/common:plan_start_point_info",
        "//onboard/planner/ml/context_feature_extractor:context_feature",
    ],
)

cc_library(
    name = "multi_conditions_ml_cruise_planner_output",
    hdrs = ["multi_conditions_ml_cruise_planner_output.h"],
    deps = [
    ],
)

cc_library(
    name = "multi_conditions_ml_cruise_planner",
    srcs = ["multi_conditions_ml_cruise_planner.cc"],
    hdrs = ["multi_conditions_ml_cruise_planner.h"],
    deps = [
        ":ml_planner_flags",
        ":ml_planner_input",
        ":ml_planner_status",
        ":multi_conditions_ml_cruise_planner_input",
        ":multi_conditions_ml_cruise_planner_output",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/global:timer",
        "//onboard/global:trace",
        "//onboard/ml_planner/proto:ml_planner_debug_cc_proto",
        "//onboard/ml_planner/proto:ml_planner_output_cc_proto",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:plan_start_point_info",
        "//onboard/planner/ml:model_pool",
        "//onboard/planner/ml/captain_net",
        "//onboard/planner/ml/captain_net/inference:input_assemble",
        "//onboard/planner/ml/captain_net/post_process:model_output_process",
        "//onboard/planner/ml/captain_net/post_process:trajectory_alignment",
        "//onboard/planner/ml/condition_feature_extractor",
        "//onboard/planner/ml/context_feature_extractor:context_feature",
    ],
)

###############################################################################
# Proto.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
