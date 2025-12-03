load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "prediction_module",
    srcs = ["prediction_module.cc"],
    hdrs = ["prediction_module.h"],
    deps = [
        "//onboard/async:future",
        "//onboard/async:thread_pool",
        "//onboard/container:strong_int",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:clock",
        "//onboard/global:run_context",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:lite_client_base",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/lite:qissue_trans",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/maps:online_map_gflags",
        "//onboard/maps:semantic_map_io",
        "//onboard/maps/proto:online_semantic_map_cc_proto",
        "//onboard/maps/v2:semantic_map_listener",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/maps/v2:semantic_map_spatial_index",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:planner_state",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/planner/util:planner_semantic_map_manager_builder",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:av_context",
        "//onboard/prediction/container:model_pool",
        "//onboard/prediction/container:objects_history",
        "//onboard/prediction/container:prediction_input",
        "//onboard/prediction/container:prediction_runner",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:online_map_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:q_issue_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "predicted_trajectory",
    srcs = ["predicted_trajectory.cc"],
    hdrs = ["predicted_trajectory.h"],
    deps = [
        ":prediction_defs",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/prediction:prediction_message_compressor",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "prediction_defs",
    hdrs = ["prediction_defs.h"],
    deps = [
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "object_prediction",
    srcs = ["object_prediction.cc"],
    hdrs = ["object_prediction.h"],
    deps = [
        ":predicted_trajectory",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math/geometry:polygon2d",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/prediction/util:perception_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "prediction_flags",
    srcs = ["prediction_flags.cc"],
    hdrs = ["prediction_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "prediction_util",
    srcs = ["prediction_util.cc"],
    hdrs = ["prediction_util.h"],
    deps = [
        ":object_prediction",
        ":predicted_trajectory",
        ":prediction_defs",
        "//onboard/container:strong_int",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:lane_path_cc_proto",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:object_history_span",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/prediction/util:perception_util",
        "//onboard/proto:assist_state_cc_proto",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "prediction_message_compressor",
    srcs = ["prediction_message_compressor.cc"],
    hdrs = ["prediction_message_compressor.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:prediction_cc_proto",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "prediction_util_test",
    srcs = ["prediction_util_test.cc"],
    deps = [
        ":prediction_defs",
        ":prediction_util",
        "//onboard/container:strong_int",
        "//onboard/global:test_main",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/proto:assist_state_cc_proto",
        "//onboard/utils:proto_util",
        "//onboard/utils:test_util",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "prediction_message_compressor_test",
    srcs = ["prediction_message_compressor_test.cc"],
    deps = [
        ":prediction_defs",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_message_compressor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "predicted_trajectory_test",
    srcs = ["predicted_trajectory_test.cc"],
    deps = [
        ":predicted_trajectory",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "prediction_module_test",
    srcs = ["prediction_module_test.cc"],
    data = ["//onboard/params/run_params:run_param_data"],
    deps = [
        ":prediction_module",
        "//onboard/global:test_main",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/unittest:lite2_unittest_helper",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "object_prediction_test",
    srcs = ["object_prediction_test.cc"],
    deps = [
        ":object_prediction",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
