load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "decider_input",
    hdrs = ["decider_input.h"],
    deps = [
        "//onboard/math:frenet_common",
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision/traffic_light:tl_info",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/proto:driving_style_cc_proto",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "decider_output",
    hdrs = ["decider_output.h"],
    deps = [
        ":constraint_manager",
        "//onboard/planner/decision/proto:constraint_cc_proto",
    ],
)

cc_library(
    name = "pedestrians_decider",
    srcs = ["pedestrians_decider.cc"],
    hdrs = ["pedestrians_decider.h"],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/maps:lane_path",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/math:frenet_common",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:source_location",
        "//onboard/utils:status_macros",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "turn_signal_decider",
    srcs = ["turn_signal_decider.cc"],
    hdrs = ["turn_signal_decider.h"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:frenet_common",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/assist:external_command_info",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:turn_signal_cc_proto",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "door_open_decider",
    srcs = ["door_open_decider.cc"],
    hdrs = ["door_open_decider.h"],
    deps = [
        "//onboard/planner:planner_flags",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "stop_sign_decider",
    srcs = ["stop_sign_decider.cc"],
    hdrs = ["stop_sign_decider.h"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/global:trace",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:frenet_common",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:halfplane",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/decision/proto:stop_sign_state_cc_proto",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/util:perception_util",
        "//onboard/planner/util:spatial_search_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "crosswalk_decider",
    srcs = ["crosswalk_decider.cc"],
    hdrs = ["crosswalk_decider.h"],
    deps = [
        ":decision_util",
        "//onboard/container:strong_int",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:logging",
        "//onboard/lite:check",
        "//onboard/maps:lane_path",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:frenet_common",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:halfplane",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/decision/proto:crosswalk_state_cc_proto",
        "//onboard/planner/object:object_vector",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:source_location",
        "//onboard/utils:status_macros",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "constraint_manager",
    hdrs = ["constraint_manager.h"],
    deps = [
        ":traffic_gap_finder",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:halfplane",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "constraint_builder",
    srcs = ["constraint_builder.cc"],
    hdrs = ["constraint_builder.h"],
    deps = [
        ":beyond_length_along_route",
        ":cautious_brake_decider",
        ":crosswalk_decider",
        ":decider_input",
        ":decider_output",
        ":decision_util",
        ":end_of_current_lane_path",
        ":end_of_path_boundary",
        ":inferred_object_decider",
        ":lc_end_of_current_lane_constraint",
        ":leading_groups_builder",
        ":leading_object",
        ":no_block",
        ":parking_brake_release",
        ":pedestrians_decider",
        ":solid_line_within_boundary",
        ":speed_bump",
        ":standstill_decider",
        ":stop_polyline_decider",
        ":stop_sign_decider",
        ":toll_decider",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/maps:lane_path",
        "//onboard/math/geometry:halfplane",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:lane_change_safety_util",
        "//onboard/planner/decision/traffic_light:traffic_light_decider",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/util:lane_point_util",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "leading_object",
    srcs = ["leading_object.cc"],
    hdrs = ["leading_object.h"],
    deps = [
        ":decision_util",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "leading_groups_builder",
    srcs = ["leading_groups_builder.cc"],
    hdrs = ["leading_groups_builder.h"],
    deps = [
        ":decision_util",
        "//onboard/global:trace",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/ml/captain_net",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "no_block",
    srcs = ["no_block.cc"],
    hdrs = ["no_block.h"],
    deps = [
        ":decision_util",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "cautious_brake_decider",
    srcs = ["cautious_brake_decider.cc"],
    hdrs = ["cautious_brake_decider.h"],
    deps = [
        ":decision_util",
        "//onboard/container:strong_int",
        "//onboard/maps:lane_path",
        "//onboard/maps:maps_common",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:planner_util",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "end_of_path_boundary",
    srcs = ["end_of_path_boundary.cc"],
    hdrs = ["end_of_path_boundary.h"],
    deps = [
        "//onboard/math/geometry:halfplane",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/router:drive_passage",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "end_of_current_lane_path",
    srcs = ["end_of_current_lane_path.cc"],
    hdrs = ["end_of_current_lane_path.h"],
    deps = [
        "//onboard/maps:lane_path",
        "//onboard/math/geometry:halfplane",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/router:drive_passage",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "speed_bump",
    srcs = ["speed_bump.cc"],
    hdrs = ["speed_bump.h"],
    deps = [
        "//onboard/maps:maps_helper",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "decision_util",
    srcs = ["decision_util.cc"],
    hdrs = ["decision_util.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/common:speed_profile",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "lc_end_of_current_lane_constraint",
    srcs = ["lc_end_of_current_lane_constraint.cc"],
    hdrs = ["lc_end_of_current_lane_constraint.h"],
    deps = [
        "//onboard/maps:lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:planner_cc_proto",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "beyond_length_along_route",
    srcs = ["beyond_length_along_route.cc"],
    hdrs = ["beyond_length_along_route.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math/proto:piecewise_linear_function_cc_proto",
        "//onboard/planner:planner_defs",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "parking_brake_release",
    srcs = ["parking_brake_release.cc"],
    hdrs = ["parking_brake_release.h"],
    deps = [
        "//onboard/math/geometry:halfplane",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "toll_decider",
    srcs = ["toll_decider.cc"],
    hdrs = ["toll_decider.h"],
    deps = [
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/router:drive_passage",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "standstill_decider",
    srcs = ["standstill_decider.cc"],
    hdrs = ["standstill_decider.h"],
    deps = [
        "//onboard/math/geometry:halfplane",
        "//onboard/planner/router:drive_passage",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "traffic_gap_result",
    hdrs = ["traffic_gap_result.h"],
    deps = [
        "//onboard/planner/decision/traffic_gap_v2/proto:traffic_gap_cc_proto",
    ],
)

cc_library(
    name = "traffic_gap_finder",
    srcs = ["traffic_gap_finder.cc"],
    hdrs = ["traffic_gap_finder.h"],
    deps = [
        ":traffic_gap_result",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "solid_line_within_boundary",
    srcs = ["solid_line_within_boundary.cc"],
    hdrs = ["solid_line_within_boundary.h"],
    deps = [
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "inferred_object_decider",
    srcs = ["inferred_object_decider.cc"],
    hdrs = ["inferred_object_decider.h"],
    deps = [
        "//common/proto:lane_point_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_speed_limit",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math/proto:piecewise_linear_function_cc_proto",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "stop_polyline_decider",
    srcs = ["stop_polyline_decider.cc"],
    hdrs = ["stop_polyline_decider.h"],
    deps = [
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/router:drive_passage",
        "@com_google_absl//absl/status:statusor",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "constraint_builder_test",
    srcs = ["constraint_builder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":constraint_builder",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/decision/traffic_light:tl_info",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "leading_object_test",
    srcs = ["leading_object_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":leading_object",
        "//onboard/global:test_main",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "leading_groups_builder_test",
    srcs = ["leading_groups_builder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":leading_groups_builder",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/scheduler:scheduler_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:time_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "no_block_test",
    srcs = ["no_block_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":no_block",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "cautious_brake_decider_test",
    srcs = ["cautious_brake_decider_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":cautious_brake_decider",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "end_of_path_boundary_test",
    srcs = ["end_of_path_boundary_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":end_of_path_boundary",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/base",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/scheduler:smooth_reference_line_result",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "end_of_current_lane_path_test",
    srcs = ["end_of_current_lane_path_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":end_of_current_lane_path",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/proto:positioning_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "speed_bump_test",
    srcs = ["speed_bump_test.cc"],
    deps = [
        ":speed_bump",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "decision_util_test",
    srcs = ["decision_util_test.cc"],
    deps = [
        ":decision_util",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "parking_brake_release_test",
    srcs = ["parking_brake_release_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":parking_brake_release",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "traffic_gap_finder_test",
    srcs = ["traffic_gap_finder_test.cc"],
    deps = [
        ":traffic_gap_finder",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "solid_line_within_boundary_test",
    srcs = ["solid_line_within_boundary_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":solid_line_within_boundary",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "beyond_length_along_route_test",
    srcs = ["beyond_length_along_route_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":beyond_length_along_route",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lc_end_of_current_lane_constraint_test",
    srcs = ["lc_end_of_current_lane_constraint_test.cc"],
    deps = [
        ":lc_end_of_current_lane_constraint",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "inferred_object_decider_test",
    srcs = ["inferred_object_decider_test.cc"],
    deps = [
        ":inferred_object_decider",
        "//onboard/base",
        "//onboard/common/vehicle_pose",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:vec",
        "//onboard/perception/lidar_pipeline/sensor_fov",
        "//onboard/perception/lidar_pipeline/sensor_fov:sensor_fov_builder",
        "//onboard/perception/lidar_pipeline/sensor_fov:sensor_fov_test_util",
        "//onboard/perception/lidar_pipeline/test_util:obstacle_builder",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/scene:occluded_interaction_lane_reasoning",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "turn_signal_decider_test",
    srcs = ["turn_signal_decider_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        "turn_signal_decider",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:vehicle_geometry_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "stop_polyline_decider_test",
    srcs = ["stop_polyline_decider_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":stop_polyline_decider",
        "//onboard/math:piecewise_linear_function",
        "//onboard/params:param_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/util:online_map_converter",
        "//onboard/planner/util:planner_semantic_map_manager_builder",
        "//onboard/planner/util:vehicle_geometry_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "stop_sign_decider_test",
    srcs = ["stop_sign_decider_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":stop_sign_decider",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "door_open_decider_test",
    srcs = ["door_open_decider_test.cc"],
    deps = [
        ":door_open_decider",
        "//onboard/global:clock",
        "//onboard/utils:time_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "standstill_decider_test",
    srcs = ["standstill_decider_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":standstill_decider",
        "//onboard/math/geometry:halfplane",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "crosswalk_decider_test",
    srcs = ["crosswalk_decider_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":crosswalk_decider",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/object:planner_object_manager_builder",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:object_prediction",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "toll_decider_test",
    srcs = ["toll_decider_test.cc"],
    deps = [
        ":toll_decider",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
    ],
)

cc_test(
    name = "pedestrians_decider_test",
    srcs = ["pedestrians_decider_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":pedestrians_decider",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/object:object_vector",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:planner_object_manager_builder",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:object_prediction",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:status_macros",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
