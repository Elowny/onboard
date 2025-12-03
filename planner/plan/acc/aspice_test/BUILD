load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "aspice_test_util",
    testonly = True,
    srcs = ["aspice_test_util.cc"],
    hdrs = ["aspice_test_util.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_object_state",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/proto/perception/fusion:object_cc_proto",
    ],
)

###############################################################################
# Proto.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "acc_preliminary_speed_test",
    srcs = ["acc_preliminary_speed_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
        "//onboard/planner/params:param_files",
    ],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/plan/acc:acc_preliminary_speed",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "acc_corridor_util_test",
    srcs = ["acc_corridor_util_test.cc"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/plan/acc:acc_corridor_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "acc_corridor_combined_test",
    srcs = ["acc_corridor_combined_test.cc"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/maps:map_selector",
        "//onboard/planner/common:driving_map_topo",
        "//onboard/planner/plan/acc:acc_corridor_combined",
        "//onboard/planner/scheduler:driving_map_topo_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "acc_corridor_with_map_test",
    srcs = ["acc_corridor_with_map_test.cc"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/maps:map_selector",
        "//onboard/math:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner/common:path_generator_util",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/plan/acc:acc_corridor_with_map",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "acc_corridor_without_map_test",
    srcs = ["acc_corridor_without_map_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/planner/plan/acc:acc_corridor_without_map",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "acc_target_util_test",
    srcs = ["acc_target_util_test.cc"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/common:path_generator_util",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/plan/acc:acc_target_util",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "acc_task_internal_test",
    srcs = ["acc_task_internal_test.cc"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/math/proto:piecewise_linear_function_cc_proto",
        "//onboard/planner/common/proto:planner_status_cc_proto",
        "//onboard/planner/plan/acc:acc_task_internal",
        "//onboard/planner/util:path_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "acc_speed_decision_test",
    srcs = ["acc_speed_decision_test.cc"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_object_state",
        "//onboard/planner/plan/acc:acc_speed_decision",
        "//onboard/planner/speed:overlap_info",
        "//onboard/planner/speed:speed_finder_flags",
        "//onboard/planner/speed:st_point",
        "//onboard/planner/speed:vt_point",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "acc_speed_finder_test",
    srcs = ["acc_speed_finder_test.cc"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/plan/acc:acc_speed_finder",
        "//onboard/planner/speed:speed_finder_flags",
        "//onboard/planner/speed:st_boundary",
        "//onboard/planner/speed:st_boundary_with_decision",
        "//onboard/planner/speed:st_point",
        "//onboard/planner/speed:vt_point",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:path_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
