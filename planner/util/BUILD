load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])
###############################################################################
# Libraries.

cc_library(
    name = "trajectory_plot_util",
    srcs = ["trajectory_plot_util.cc"],
    hdrs = ["trajectory_plot_util.h"],
    deps = [
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/math:vec",
        "//onboard/planner:planner_defs",
        "//onboard/planner:trajectory_point",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "vehicle_geometry_util",
    srcs = ["vehicle_geometry_util.cc"],
    hdrs = ["vehicle_geometry_util.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:offset_rect",
        "//onboard/math/geometry/proto:aabox3d_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "perception_util",
    srcs = ["perception_util.cc"],
    hdrs = ["perception_util.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/math/geometry/proto:box2d_cc_proto",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "path_util",
    hdrs = ["path_util.h"],
    deps = [
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/proto:trajectory_point_cc_proto",
    ],
)

cc_library(
    name = "planner_status_macros",
    hdrs = ["planner_status_macros.h"],
    deps = [
        "//onboard/planner/common:planner_status",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "min_segment_distance_problem",
    srcs = ["min_segment_distance_problem.cc"],
    hdrs = ["min_segment_distance_problem.h"],
    deps = [
        ":qtfm_segment_matcher_v2",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/vis/canvas",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "motion_util",
    srcs = ["motion_util.cc"],
    hdrs = ["motion_util.h"],
    deps = [
        "//onboard/math:util",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "quad_tree",
    hdrs = ["quad_tree.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "quad_tree_field_map",
    hdrs = ["quad_tree_field_map.h"],
    deps = [
        ":quad_tree",
        "//onboard/math/geometry:grid_frame",
    ],
)

cc_library(
    name = "qtfm_segment_matcher_util",
    srcs = ["qtfm_segment_matcher_util.cc"],
    hdrs = ["qtfm_segment_matcher_util.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:segment2d",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "qtfm_segment_matcher_v2",
    srcs = ["qtfm_segment_matcher_v2.cc"],
    hdrs = ["qtfm_segment_matcher_v2.h"],
    deps = [
        ":qtfm_segment_matcher_util",
        ":quad_tree_field_map",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:circle2d",
        "//onboard/math/geometry:grid_frame",
        "//onboard/math/geometry:grid_frame_util",
        "//onboard/math/geometry:intersection_util",
        "//onboard/math/geometry:segment2d",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "hmi_content_util",
    srcs = ["hmi_content_util.cc"],
    hdrs = ["hmi_content_util.h"],
    deps = [
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:qissue_trans",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:frenet_common",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/geometry:util",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "//onboard/proto:hmi_content_cc_proto",
        "//onboard/proto:lane_change_type_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:source_location",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@com_google_protobuf//:protobuf_lite",
    ],
)

cc_library(
    name = "lane_path_util",
    srcs = ["lane_path_util.cc"],
    hdrs = ["lane_path_util.h"],
    deps = [
        ":lane_point_util",
        ":planner_semantic_map_util",
        ":spatial_search_util",
        "//onboard/base",
        "//onboard/container:strong_int",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:qissue_trans",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_path_data",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:driving_map_topo",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:q_issue_cc_proto",
        "//onboard/utils:source_location",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "lane_point_util",
    srcs = ["lane_point_util.cc"],
    hdrs = ["lane_point_util.h"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/global:logging",
        "//onboard/lite:check",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/maps/v2:semantic_map_object",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "planner_semantic_map_util",
    srcs = ["planner_semantic_map_util.cc"],
    hdrs = ["planner_semantic_map_util.h"],
    deps = [
        "//common/proto:map_geometry_cc_proto",
        "//onboard/container:strong_int",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/lite:qissue_trans",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:level_id",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/proto:lane_path_cc_proto",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:range1d",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/proto:q_issue_cc_proto",
        "//onboard/utils:source_location",
        "//onboard/utils:status_builder",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "spatial_search_util",
    srcs = ["spatial_search_util.cc"],
    hdrs = ["spatial_search_util.h"],
    deps = [
        ":lane_point_util",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:level_id",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:maps_helper",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/utils:source_location",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "scene_util",
    srcs = ["scene_util.cc"],
    hdrs = ["scene_util.h"],
    deps = [
        "//onboard/global:logging",
        "//onboard/maps:compatibility_layer",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/util:lane_point_util",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "online_map_converter",
    srcs = ["online_map_converter.cc"],
    hdrs = ["online_map_converter.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:online_map_constants",
        "//onboard/maps:online_map_data_types",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:online_semantic_map_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/math:frenet_common",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/proto:localization_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "planner_semantic_map_manager_builder",
    srcs = ["planner_semantic_map_manager_builder.cc"],
    hdrs = ["planner_semantic_map_manager_builder.h"],
    deps = [
        "//onboard/async:async_util",
        "//onboard/async:future",
        "//onboard/async:thread_pool",
        "//onboard/base",
        "//onboard/container:strong_int",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/maps:online_map_utils",
        "//onboard/maps/proto:online_semantic_map_cc_proto",
        "//onboard/maps/v2:semantic_map_definition",
        "//onboard/maps/v2:semantic_map_listener",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/maps/v2:semantic_map_spatial_index",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:planner_state",
        "//onboard/proto:adasis_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:online_map_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "vehicle_util",
    srcs = ["vehicle_util.cc"],
    hdrs = ["vehicle_util.h"],
    deps = [
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
    ],
)

cc_library(
    name = "online_semantic_map_util",
    srcs = ["online_semantic_map_util.cc"],
    hdrs = ["online_semantic_map_util.h"],
    deps = [
        ":lane_path_util",
        ":planner_semantic_map_util",
        "//onboard/container:strong_int",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/maps:lane_path",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:online_semantic_map_cc_proto",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:vec",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/util:lane_point_util",
        "//onboard/utils:source_location",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "lane_path_preprocess_util",
    srcs = ["lane_path_preprocess_util.cc"],
    hdrs = ["lane_path_preprocess_util.h"],
    deps = [
        ":lane_point_util",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:gradient_points_smoother",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:range1d",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:plot_util",
        "//onboard/utils:source_location",
        "//onboard/vis/common:color",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "perception_util_test",
    srcs = ["perception_util_test.cc"],
    deps = [
        ":perception_util",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/test_util:util",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "vehicle_geometry_util_test",
    srcs = ["vehicle_geometry_util_test.cc"],
    deps = [
        ":vehicle_geometry_util",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "//onboard/math:vec",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "path_util_test",
    srcs = ["path_util_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/planner/util:path_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "min_segment_distance_problem_test",
    srcs = ["min_segment_distance_problem_test.cc"],
    deps = [
        ":min_segment_distance_problem",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "quad_tree_test",
    srcs = ["quad_tree_test.cc"],
    deps = [
        ":quad_tree",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "qtfm_segment_matcher_v2_test",
    srcs = ["qtfm_segment_matcher_v2_test.cc"],
    deps = [
        ":qtfm_segment_matcher_v2",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/utils:map_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "qtfm_segment_matcher_util_test",
    srcs = ["qtfm_segment_matcher_util_test.cc"],
    deps = [
        ":qtfm_segment_matcher_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "motion_util_test",
    srcs = ["motion_util_test.cc"],
    deps = [
        ":motion_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "hmi_content_util_test",
    srcs = ["hmi_content_util_test.cc"],
    deps = [
        ":hmi_content_util",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:vec",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf_lite",
    ],
)

cc_test(
    name = "lane_path_util_test",
    srcs = ["lane_path_util_test.cc"],
    deps = [
        ":lane_path_util",
        ":lane_point_util",
        "//onboard/base",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/scheduler:driving_map_topo_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lane_point_util_test",
    srcs = ["lane_point_util_test.cc"],
    deps = [
        ":lane_point_util",
        "//onboard/async:future",
        "//onboard/base",
        "//onboard/container:strong_int",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/v2:semantic_map_loader",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/maps/v2:semantic_map_object",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_semantic_map_util_test",
    srcs = ["planner_semantic_map_util_test.cc"],
    deps = [
        ":planner_semantic_map_util",
        "//onboard/global:test_main",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/v2:semantic_map_loader",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/math/geometry:box2d",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "spatial_search_util_test",
    srcs = [
        "spatial_search_util_test.cc",
    ],
    deps = [
        ":spatial_search_util",
        "//onboard/base",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/util:lane_path_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "online_map_converter_test",
    srcs = ["online_map_converter_test.cc"],
    deps = [
        ":online_map_converter",
        "//onboard/global:buffered_logger",
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_semantic_map_manager_builder_test",
    srcs = ["planner_semantic_map_manager_builder_test.cc"],
    deps = [
        ":planner_semantic_map_manager_builder",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps/v2:semantic_map_gflags",
        "//onboard/maps/v2:semantic_map_loader",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/util:online_map_converter",
        "//onboard/proto:adasis_cc_proto",
        "//onboard/proto:online_map_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "vehicle_util_test",
    srcs = ["vehicle_util_test.cc"],
    deps = [
        ":vehicle_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "online_semantic_map_util_test",
    srcs = ["online_semantic_map_util_test.cc"],
    deps = [
        ":lane_path_util",
        ":online_map_converter",
        ":online_semantic_map_util",
        ":planner_semantic_map_manager_builder",
        ":planner_semantic_map_util",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_path_data",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/utils:status_macros",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "scene_util_test",
    srcs = ["scene_util_test.cc"],
    deps = [
        ":scene_util",
        "//onboard/container:strong_int",
        "//onboard/global:test_main",
        "//onboard/maps:lane_point",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
