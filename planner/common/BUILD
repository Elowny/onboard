load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "path_sl_boundary",
    srcs = ["path_sl_boundary.cc"],
    hdrs = ["path_sl_boundary.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/scheduler/proto:scheduler_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "plot_util",
    srcs = ["plot_util.cc"],
    hdrs = ["plot_util.h"],
    deps = [
        ":discretized_path",
        ":path_sl_boundary",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/maps:lane_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/util:planner_semantic_map_util",
        "//onboard/vis/common:color",
    ],
)

cc_library(
    name = "lane_path_info",
    srcs = ["lane_path_info.cc"],
    hdrs = ["lane_path_info.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:frenet_common",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/util:planner_semantic_map_util",
    ],
)

cc_library(
    name = "multi_timer_util",
    srcs = ["multi_timer_util.cc"],
    hdrs = ["multi_timer_util.h"],
    deps = [
        "//onboard/global:timer",
        "//onboard/lite:logging",
        "//onboard/proto:timer_report_cc_proto",
        "//onboard/utils:map_util",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "planner_status",
    srcs = ["planner_status.cc"],
    hdrs = ["planner_status.h"],
    deps = [
        "//onboard/planner/common/proto:planner_status_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "plan_start_point_info",
    hdrs = ["plan_start_point_info.h"],
    deps = [
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "speed_profile",
    hdrs = ["speed_profile.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "discretized_path",
    srcs = ["discretized_path.cc"],
    hdrs = ["discretized_path.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:frenet_common",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/util:path_util",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "path_approx",
    srcs = ["path_approx.cc"],
    hdrs = ["path_approx.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:offset_rect",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "global_pose",
    srcs = ["global_pose.cc"],
    hdrs = ["global_pose.h"],
    deps = [
        "//common/proto:global_pose_cc_proto",
        "//common/proto:map_geometry_cc_proto",
        "//onboard/math:vec",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "path_approx_overlap",
    srcs = ["path_approx_overlap.cc"],
    hdrs = ["path_approx_overlap.h"],
    deps = [
        ":path_approx",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:halfplane",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:polygon2d_util",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
    ],
)

cc_library(
    name = "vehicle_octagon_model",
    srcs = ["vehicle_octagon_model.cc"],
    hdrs = ["vehicle_octagon_model.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:gjk2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
    ],
)

cc_library(
    name = "circle_path_overlap",
    srcs = ["circle_path_overlap.cc"],
    hdrs = ["circle_path_overlap.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
    ],
)

cc_library(
    name = "vehicle_shape",
    hdrs = ["vehicle_shape.h"],
    deps = [
        ":vehicle_octagon_model",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:circle2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/geometry/proto:aabox3d_cc_proto",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "driving_map_topo",
    srcs = ["driving_map_topo.cc"],
    hdrs = ["driving_map_topo.h"],
    deps = [
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:vec",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "lane_change_safety_params",
    hdrs = ["lane_change_safety_params.h"],
)

cc_library(
    name = "lane_change_safety_util",
    srcs = ["lane_change_safety_util.cc"],
    hdrs = ["lane_change_safety_util.h"],
    deps = [
        ":lane_change_safety_params",
        "//onboard/math:frenet_common",
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/object:planner_object",
        "//onboard/prediction:object_prediction",
        "//onboard/proto:driving_style_cc_proto",
    ],
)

cc_library(
    name = "lane_change_safety",
    srcs = ["lane_change_safety.cc"],
    hdrs = ["lane_change_safety.h"],
    deps = [
        ":lane_change_safety_params",
        ":lane_change_safety_util",
        "//onboard/math:frenet_frame",
        "//onboard/math/geometry:box2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common/proto:lane_change_safety_cc_proto",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:driving_style_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "path_generator_util",
    srcs = ["path_generator_util.cc"],
    hdrs = ["path_generator_util.h"],
    deps = [
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:online_map_utils",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:spatial_search_util",
        "//onboard/maps/proto:online_semantic_map_cc_proto",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/util:path_util",
        "//onboard/proto:trajectory_point_cc_proto",
    ],
)

cc_library(
    name = "map_path_generator",
    srcs = ["map_path_generator.cc"],
    hdrs = ["map_path_generator.h"],
    deps = [
        ":discretized_path",
        ":path_generator_util",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/maps:lane_point",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:cubic_spline",
        "//onboard/math:fast_math",
        "//onboard/math:frenet_cartesian_converter",
        "//onboard/math:frenet_common",
        "//onboard/math:gradient_points_smoother",
        "//onboard/math:polynomial_connector",
        "//onboard/math:polynomialxd",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/math:cubic_spiral_boundary_value_problem",
        "//onboard/planner/math:spiral",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/util:lane_point_util",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:spatial_search_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:map_util",
        "//onboard/utils:status_macros",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "geometry_path_util",
    srcs = ["geometry_path_util.cc"],
    hdrs = ["geometry_path_util.h"],
    deps = [
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner/common:circle_path_overlap",
    ],
)

###############################################################################
# Binaries.

cc_binary(
    name = "vehicle_octagon_model_bm",
    srcs = ["vehicle_octagon_model_bm.cc"],
    tags = ["bm"],
    deps = [
        ":vehicle_octagon_model",
        "//onboard/math/geometry:polygon2d",
        "@com_github_google_benchmark//:benchmark",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "path_sl_boundary_test",
    srcs = ["path_sl_boundary_test.cc"],
    deps = [
        ":path_sl_boundary",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lane_path_info_test",
    srcs = ["lane_path_info_test.cc"],
    deps = [
        ":lane_path_info",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/util:lane_path_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "speed_profile_test",
    srcs = ["speed_profile_test.cc"],
    linkstatic = 1,
    deps = [
        ":speed_profile",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "discretized_path_test",
    srcs = ["discretized_path_test.cc"],
    linkstatic = 1,
    deps = [
        ":discretized_path",
        "//onboard/global:test_main",
        "//onboard/math:frenet_frame",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "path_approx_test",
    srcs = ["path_approx_test.cc"],
    deps = [
        ":path_approx",
        "//onboard/math:test_util",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "path_approx_overlap_test",
    srcs = ["path_approx_overlap_test.cc"],
    deps = [
        ":path_approx_overlap",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vehicle_octagon_model_test",
    srcs = ["vehicle_octagon_model_test.cc"],
    deps = [
        ":vehicle_octagon_model",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vehicle_shape_test",
    srcs = ["vehicle_shape_test.cc"],
    deps = [
        ":vehicle_shape",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:vehicle_geometry_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "circle_path_overlap_test",
    srcs = ["circle_path_overlap_test.cc"],
    deps = [
        ":circle_path_overlap",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "path_generator_util_test",
    srcs = ["path_generator_util_test.cc"],
    deps = [
        ":path_generator_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "map_path_generator_test",
    srcs = ["map_path_generator_test.cc"],
    deps = [
        ":map_path_generator",
        "//onboard/global:test_main",
        "//onboard/maps:online_map_converter",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:planner_semantic_map_manager_builder",
    ],
)

cc_test(
    name = "geometry_path_util_test",
    srcs = ["geometry_path_util_test.cc"],
    deps = [
        ":geometry_path_util",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_status_test",
    srcs = ["planner_status_test.cc"],
    deps = [
        ":planner_status",
        "//onboard/global:test_main",
        "//onboard/planner/common/proto:planner_status_cc_proto",
    ],
)

cc_test(
    name = "global_pose_test",
    srcs = ["global_pose_test.cc"],
    deps = [
        ":global_pose",
        "//onboard/global:test_main",
        "//onboard/utils:proto_util",
    ],
)

cpplint()
