load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "scene_understanding_output",
    hdrs = ["scene_understanding_output.h"],
    deps = ["//onboard/planner/scene/proto:scene_understanding_cc_proto"],
)

cc_library(
    name = "scene_understanding",
    srcs = ["scene_understanding.cc"],
    hdrs = ["scene_understanding.h"],
    deps = [
        ":occluded_objects_reasoning",
        ":scene_understanding_output",
        ":traffic_flow_reasoning",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/maps:lane_path",
        "//onboard/perception/lidar_pipeline/sensor_fov",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/decision/traffic_light:tl_info",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "traffic_flow_reasoning",
    srcs = ["traffic_flow_reasoning.cc"],
    hdrs = ["traffic_flow_reasoning.h"],
    deps = [
        ":scene_understanding_output",
        "//onboard/async:parallel_for",
        "//onboard/maps:lane_path",
        "//onboard/maps:map_or_die_macros",
        "//onboard/math:piecewise_linear_function",
        "//onboard/perception/lidar_pipeline/sensor_fov",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/decision/traffic_light:tl_info",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:perception_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "//onboard/planner/util:spatial_search_util",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "objects_reasoning",
    srcs = ["objects_reasoning.cc"],
    hdrs = ["objects_reasoning.h"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/math:util",
        "//onboard/planner/object:object_vector",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status:statusor",
        "@eigen",
    ],
)

cc_library(
    name = "off_road_scene_reasoning",
    srcs = ["off_road_scene_reasoning.cc"],
    hdrs = ["off_road_scene_reasoning.h"],
    deps = [
        ":objects_reasoning",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "scene_reasoning_util",
    srcs = ["scene_reasoning_util.cc"],
    hdrs = ["scene_reasoning_util.h"],
    deps = [
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "occluded_objects_reasoning",
    srcs = ["occluded_objects_reasoning.cc"],
    hdrs = ["occluded_objects_reasoning.h"],
    deps =
        [
            ":occluded_interaction_lane_reasoning",
            "//onboard/maps:map_or_die_macros",
            "//onboard/perception/lidar_pipeline/sensor_fov",
            "//onboard/planner:planner_flags",
            "//onboard/planner:planner_semantic_map_manager",
            "//onboard/planner/router:route_sections",
            "//onboard/planner/router:route_sections_util",
            "//onboard/planner/scene/proto:scene_understanding_cc_proto",
            "@com_google_absl//absl/container:flat_hash_set",
            "@com_google_absl//absl/status:statusor",
        ],
)

cc_library(
    name = "occluded_interaction_lane_reasoning",
    srcs = ["occluded_interaction_lane_reasoning.cc"],
    hdrs = ["occluded_interaction_lane_reasoning.h"],
    deps =
        [
            "//onboard/container:strong_int",
            "//onboard/global:logging",
            "//onboard/global:trace",
            "//onboard/maps:lane_path",
            "//onboard/maps:lane_point",
            "//onboard/maps:map_or_die_macros",
            "//onboard/maps:maps_common",
            "//onboard/maps:semantic_map_defs",
            "//onboard/maps/proto:semantic_map_cc_proto",
            "//onboard/math:vec",
            "//onboard/math/geometry:box2d",
            "//onboard/math/geometry:util",
            "//onboard/perception/lidar_pipeline/sensor_fov",
            "//onboard/planner:planner_semantic_map_manager",
            "//onboard/planner/router:route_sections",
            "//onboard/planner/router:route_sections_util",
            "//onboard/planner/scene/proto:scene_understanding_cc_proto",
            "//onboard/planner/util:lane_path_util",
            "//onboard/planner/util:lane_point_util",
            "//onboard/proto:perception_cc_proto",
            "//onboard/utils:status_macros",
            "@com_google_absl//absl/container:flat_hash_set",
            "@com_google_absl//absl/hash",
            "@com_google_absl//absl/status:statusor",
            "@com_google_absl//absl/strings",
            "@com_google_absl//absl/types:span",
        ],
)

cc_library(
    name = "bus_station_stalled_objects_filter",
    srcs = ["bus_station_stalled_objects_filter.cc"],
    hdrs = ["bus_station_stalled_objects_filter.h"],
    deps = [
        "//onboard/maps:lane_path",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/util:lane_point_util",
        "//onboard/planner/util:spatial_search_util",
        "//onboard/proto:perception_cc_proto",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "traffic_flow_reasoning_test",
    srcs = ["traffic_flow_reasoning_test.cc"],
    deps = [
        ":traffic_flow_reasoning",
        "//onboard/planner:planner_flags",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "off_road_scene_reasoning_test",
    srcs = ["off_road_scene_reasoning_test.cc"],
    deps = [
        ":off_road_scene_reasoning",
        "//onboard/global:buffered_logger",
        "//onboard/maps:map_selector",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:planner_object_manager_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "occluded_objects_reasoning_test",
    srcs = ["occluded_objects_reasoning_test.cc"],
    deps = [
        ":occluded_objects_reasoning",
        "//onboard/common/vehicle_pose",
        "//onboard/global:buffered_logger",
        "//onboard/global:run_context",
        "//onboard/maps:map_selector",
        "//onboard/math:vec",
        "//onboard/perception/lidar_pipeline/sensor_fov:sensor_fov_builder",
        "//onboard/perception/lidar_pipeline/sensor_fov:sensor_fov_test_util",
        "//onboard/perception/lidar_pipeline/test_util:obstacle_builder",
        "//onboard/planner:planner_flags",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "occluded_interaction_lane_reasoning_test",
    srcs = ["occluded_interaction_lane_reasoning_test.cc"],
    deps = [
        ":occluded_interaction_lane_reasoning",
        # "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/perception/lidar_pipeline/sensor_fov:sensor_fov_test_util",
        "//onboard/perception/lidar_pipeline/test_util:obstacle_builder",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest_main",
        "//onboard/planner/test_util:load_psmm_util",
    ],
)

cc_test(
    name = "bus_station_stalled_objects_filter_test",
    srcs = ["bus_station_stalled_objects_filter_test.cc"],
    deps = [
        ":bus_station_stalled_objects_filter",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
