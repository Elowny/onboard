load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "trajectory_developer",
    srcs = ["trajectory_developer.cc"],
    hdrs = ["trajectory_developer.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:line2d",
        "//onboard/math:line_fitter",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/speed:speed_point",
        "//onboard/planner/speed:speed_vector",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:prediction_object",
        "//onboard/prediction/util:kinematic_model",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "pole_placement_util",
    srcs = ["pole_placement_util.cc"],
    hdrs = ["pole_placement_util.h"],
    deps = [
        "//onboard/global:logging",
        "//onboard/lite:check",
        "//onboard/math:cubic_spline",
        "//onboard/math:fast_math",
        "//onboard/math:frenet_common",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:polynomial_fitter",
        "//onboard/math:polynomialxd",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/util:kinematic_model",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "safe_guard_util",
    srcs = ["safe_guard_util.cc"],
    hdrs = ["safe_guard_util.h"],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:logging",
        "//onboard/lite:check",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:vec",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/proto:prediction_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "kinematic_model",
    srcs = ["kinematic_model.cc"],
    hdrs = ["kinematic_model.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/prediction:prediction_defs",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "lane_path_finder",
    srcs = ["lane_path_finder.cc"],
    hdrs = ["lane_path_finder.h"],
    deps = [
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
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/util:lane_path_util",
        "//onboard/planner/util:planner_semantic_map_util",
        "//onboard/planner/util:spatial_search_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:map_util",
        "//onboard/utils:source_location",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "lane_path_util",
    srcs = ["lane_path_util.cc"],
    hdrs = ["lane_path_util.h"],
    deps = [
        ":kinematic_model",
        "//onboard/container:strong_int",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/util:spatial_search_util",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/prediction/util:trajectory_developer",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "transform_util",
    srcs = ["transform_util.cc"],
    hdrs = ["transform_util.h"],
    deps = [
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:box2d_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
    ],
)

cc_library(
    name = "perception_util",
    srcs = ["perception_util.cc"],
    hdrs = ["perception_util.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "drive_passage_util",
    srcs = ["drive_passage_util.cc"],
    hdrs = ["drive_passage_util.h"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:frenet_frame",
        "//onboard/math:vec",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/util:lane_point_util",
        "//onboard/planner/util:spatial_search_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "lane_follow_util",
    srcs = ["lane_follow_util.cc"],
    hdrs = ["lane_follow_util.h"],
    deps = [
        "//onboard/math:frenet_common",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.
cc_binary(
    name = "drive_passage_util_bm",
    testonly = True,
    srcs = ["drive_passage_util_bm.cc"],
    data = [
    ],
    tags = ["bm"],
    deps = [
        ":drive_passage_util",
        "//onboard/container:strong_int",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:vec",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:objects_history",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/prediction/predictor:lane_selection_net_j5_predictor",
        "//onboard/prediction/test_util:prediction_context_builder",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_benchmark//:benchmark",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "trajectory_developer_test",
    srcs = ["trajectory_developer_test.cc"],
    deps = [
        ":lane_path_finder",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/maps:lane_path",
        "//onboard/math:vec",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/speed:speed_point",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction/post_process:conflict_resolver_util",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lane_path_finder_test",
    srcs = ["lane_path_finder_test.cc"],
    deps = [
        "//onboard/base",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "transform_util_test",
    srcs = ["transform_util_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/math/geometry/proto:box2d_cc_proto",
        "//onboard/prediction/util:transform_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "kinematic_model_test",
    srcs = ["kinematic_model_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/prediction/util:kinematic_model",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "perception_util_test",
    srcs = ["perception_util_test.cc"],
    deps = [
        ":perception_util",
        "//onboard/math:test_util",
        "//onboard/math:vec",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pole_placement_util_test",
    srcs = ["pole_placement_util_test.cc"],
    deps = [
        ":pole_placement_util",
        "//onboard/math:test_util",
        "//onboard/math:vec",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "drive_passage_util_test",
    srcs = ["drive_passage_util_test.cc"],
    deps = [
        ":drive_passage_util",
        "//onboard/container:strong_int",
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/util:spatial_search_util",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
