load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "planner_object",
    srcs = ["planner_object.cc"],
    hdrs = ["planner_object.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/planner/util:perception_util",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
    ],
)

cc_library(
    name = "planner_cluster_object",
    srcs = ["planner_cluster_object.cc"],
    hdrs = ["planner_cluster_object.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/proto/perception/parking:parking_freespace_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "object_vector",
    hdrs = ["object_vector.h"],
    deps = [
        "//onboard/container:strong_vector",
    ],
)

cc_library(
    name = "spacetime_object_state",
    srcs = ["spacetime_object_state.cc"],
    hdrs = ["spacetime_object_state.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
    ],
)

cc_library(
    name = "spacetime_object_trajectory",
    srcs = ["spacetime_object_trajectory.cc"],
    hdrs = ["spacetime_object_trajectory.h"],
    deps = [
        ":spacetime_object_state",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/object:planner_object",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "spacetime_planner_object_trajectories",
    hdrs = ["spacetime_planner_object_trajectories.h"],
    deps = [
        ":spacetime_object_trajectory",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "low_likelihood_filter",
    srcs = ["low_likelihood_filter.cc"],
    hdrs = ["low_likelihood_filter.h"],
    deps = [
        ":trajectory_filter",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/prediction:predicted_trajectory",
    ],
)

cc_library(
    name = "spacetime_trajectory_manager",
    srcs = ["spacetime_trajectory_manager.cc"],
    hdrs = ["spacetime_trajectory_manager.h"],
    deps = [
        ":planner_object",
        ":spacetime_object_trajectory",
        ":trajectory_filter",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "planner_object_manager",
    srcs = ["planner_object_manager.cc"],
    hdrs = ["planner_object_manager.h"],
    deps = [
        "//onboard/planner/object:object_vector",
        "//onboard/planner/object:planner_object",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "planner_object_util",
    srcs = ["planner_object_util.cc"],
    hdrs = ["planner_object_util.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/planner:est_planner_output",
        "//onboard/planner/object:partial_spacetime_object_trajectory",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "planner_object_manager_builder",
    srcs = ["planner_object_manager_builder.cc"],
    hdrs = ["planner_object_manager_builder.h"],
    deps = [
        ":object_vector",
        ":planner_object",
        ":planner_object_manager",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/container:strong_int",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/planner/object:trajectory_filter",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "planner_cluster_object_manager",
    srcs = ["planner_cluster_object_manager.cc"],
    hdrs = ["planner_cluster_object_manager.h"],
    deps = [
        ":planner_cluster_object",
        "//onboard/lite:logging",
        "//onboard/proto/perception/parking:parking_freespace_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
    ],
)

cc_library(
    name = "trajectory_filter",
    hdrs = ["trajectory_filter.h"],
    deps = [
        ":planner_object",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "drive_passage_filter",
    srcs = ["drive_passage_filter.cc"],
    hdrs = ["drive_passage_filter.h"],
    deps = [
        ":planner_object",
        ":trajectory_filter",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:logging",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:util",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "reflected_object_in_proximity_filter",
    srcs = ["reflected_object_in_proximity_filter.cc"],
    hdrs = ["reflected_object_in_proximity_filter.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/planner/object:trajectory_filter",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "motion_state_filter",
    srcs = ["motion_state_filter.cc"],
    hdrs = ["motion_state_filter.h"],
    deps = [
        ":planner_object",
        ":trajectory_filter",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "freespace_region_filter",
    srcs = ["freespace_region_filter.cc"],
    hdrs = ["freespace_region_filter.h"],
    deps = [
        ":planner_object",
        ":trajectory_filter",
        "//onboard/lite:check",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
    ],
)

cc_library(
    name = "predicted_motion_filter",
    srcs = ["predicted_motion_filter.cc"],
    hdrs = ["predicted_motion_filter.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:aabox_kdtree2d",
        "//onboard/math/geometry:box2d",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:trajectory_filter",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "path_boundary_filter",
    srcs = ["path_boundary_filter.cc"],
    hdrs = ["path_boundary_filter.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/math:frenet_frame",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:trajectory_filter",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/planner/plan/acc:acc_defs",
        "//onboard/planner/plan/acc:acc_target_util",
        "//onboard/prediction:predicted_trajectory",
    ],
)

cc_library(
    name = "spacetime_planner_object_trajectories_finder",
    srcs = ["spacetime_planner_object_trajectories_finder.cc"],
    hdrs = ["spacetime_planner_object_trajectories_finder.h"],
    deps = [
        ":planner_object_manager",
        ":spacetime_planner_object_trajectories",
        "//onboard/container:strong_int",
        "//onboard/maps:lane_path",
        "//onboard/math:frenet_frame",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "spacetime_planner_object_trajectories_filter",
    srcs = ["spacetime_planner_object_trajectories_filter.cc"],
    hdrs = ["spacetime_planner_object_trajectories_filter.h"],
    deps = [
        ":planner_object_manager",
        ":spacetime_planner_object_trajectories",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_util",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/util:perception_util",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "spacetime_planner_object_trajectories_builder",
    srcs = ["spacetime_planner_object_trajectories_builder.cc"],
    hdrs = ["spacetime_planner_object_trajectories_builder.h"],
    deps = [
        ":planner_object_manager",
        ":spacetime_planner_object_trajectories",
        ":spacetime_planner_object_trajectories_filter",
        ":spacetime_planner_object_trajectories_finder",
        "//onboard/maps:lane_path",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "spacetime_trajectory_manager_builder",
    srcs = ["spacetime_trajectory_manager_builder.cc"],
    hdrs = ["spacetime_trajectory_manager_builder.h"],
    deps = [
        ":drive_passage_filter",
        ":low_likelihood_filter",
        ":planner_object_manager",
        ":spacetime_trajectory_manager",
        ":trajectory_filter",
        "//onboard/async:thread_pool",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/planner:planner_flags",
        "//onboard/planner/assist:alcc_target_selector",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "plot_util",
    srcs = ["plot_util.cc"],
    hdrs = ["plot_util.h"],
    deps = [
        ":object_vector",
        ":planner_object",
        ":spacetime_object_state",
        ":spacetime_object_trajectory",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "partial_spacetime_object_trajectory",
    hdrs = ["partial_spacetime_object_trajectory.h"],
    deps = [
        ":spacetime_object_trajectory",
        "//onboard/math:range1d",
        "//onboard/math:util",
    ],
)

cc_library(
    name = "unknown_roadway_position_object_filter",
    srcs = ["unknown_roadway_position_object_filter.cc"],
    hdrs = ["unknown_roadway_position_object_filter.h"],
    deps = [
        "//onboard/perception/tracker/multi_camera_fusion/proto:multi_camera_fusion_cc_proto",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:trajectory_filter",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto/perception/fusion:object_cc_proto",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.
cc_binary(
    name = "planner_object_manager_builder_bm",
    testonly = True,
    srcs = ["planner_object_manager_builder_bm.cc"],
    deps = [
        ":planner_object_manager_builder",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/math:vec",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:object_prediction",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_github_google_benchmark//:benchmark",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "planner_cluster_object_test",
    srcs = ["planner_cluster_object_test.cc"],
    deps = [
        ":planner_cluster_object",
        "//onboard/math:test_util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "spacetime_object_trajectory_test",
    srcs = ["spacetime_object_trajectory_test.cc"],
    deps = [
        ":spacetime_object_trajectory",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/math:test_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:file_util",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "spacetime_trajectory_manager_test",
    srcs = ["spacetime_trajectory_manager_test.cc"],
    deps = [
        ":spacetime_object_state",
        ":spacetime_trajectory_manager",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:test_util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_object_manager_test",
    srcs = ["planner_object_manager_test.cc"],
    deps = [
        ":planner_object_manager",
        ":planner_object_manager_builder",
        "//onboard/global:test_main",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_object_util_test",
    srcs = ["planner_object_util_test.cc"],
    deps = [
        ":planner_object_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_object_manager_builder_test",
    srcs = ["planner_object_manager_builder_test.cc"],
    deps = [
        ":planner_object_manager_builder",
        "//onboard/base",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:object_prediction",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_cluster_object_manager_test",
    srcs = ["planner_cluster_object_manager_test.cc"],
    deps = [
        ":planner_cluster_object_manager",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/proto/perception/parking:parking_freespace_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "motion_state_filter_test",
    srcs = ["motion_state_filter_test.cc"],
    deps = [
        ":motion_state_filter",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "predicted_motion_filter_test",
    srcs = ["predicted_motion_filter_test.cc"],
    deps = [
        ":predicted_motion_filter",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:vec",
        "//onboard/planner/object:object_vector",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/proto:perception_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "spacetime_planner_object_trajectories_test",
    srcs = ["spacetime_planner_object_trajectories_test.cc"],
    deps = [
        ":spacetime_planner_object_trajectories",
        "//onboard/planner/object:planner_object_manager_builder",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/object/proto:planner_object_cc_proto",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
