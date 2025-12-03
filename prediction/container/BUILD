load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "prediction_object",
    srcs = ["prediction_object.cc"],
    hdrs = ["prediction_object.h"],
    deps = [
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/util:transform_util",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "object_history_span",
    srcs = ["object_history_span.cc"],
    hdrs = ["object_history_span.h"],
    deps = [
        ":prediction_object",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/prediction:prediction_defs",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:elements_history",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "object_history",
    srcs = ["object_history.cc"],
    hdrs = ["object_history.h"],
    deps = [
        ":object_history_span",
        ":prediction_object",
        "//onboard/prediction:prediction_defs",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:elements_history",
    ],
)

cc_library(
    name = "objects_history",
    srcs = ["objects_history.cc"],
    hdrs = ["objects_history.h"],
    deps = [
        ":object_history",
        ":object_history_span",
        ":prediction_object",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:elements_history",
        "//onboard/utils:time_util",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "object_prediction_result",
    hdrs = ["object_prediction_result.h"],
    deps = [
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "av_context",
    srcs = ["av_context.cc"],
    hdrs = ["av_context.h"],
    deps = [
        ":object_history",
        ":prediction_object",
        "//onboard/math:coordinate_converter",
        "//onboard/planner/util:perception_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/util:transform_util",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:elements_history",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "prediction_input",
    hdrs = ["prediction_input.h"],
    deps = [
        ":av_context",
        ":objects_history",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "prediction_runner",
    srcs = ["prediction_runner.cc"],
    hdrs = ["prediction_runner.h"],
    deps = [
        ":av_context",
        ":model_pool",
        ":object_history",
        ":object_prediction_result",
        ":prediction_input",
        "//onboard/async:thread_pool",
        "//onboard/global:timer",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math/geometry:util",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/predictor:predictor_util",
        "//onboard/prediction/scheduler",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/utils:objects_view",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "map_cache_builder",
    srcs = ["map_cache_builder.cc"],
    hdrs = ["map_cache_builder.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/util:lane_point_util",
        "//onboard/planner/util:spatial_search_util",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/util:drive_passage_util",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "prediction_context",
    srcs = ["prediction_context.cc"],
    hdrs = ["prediction_context.h"],
    deps = [
        ":av_context",
        ":object_history",
        ":objects_history",
        ":prediction_input",
        ":prediction_object",
        ":traffic_light_manager",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:route_manager_output",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/router/navi:route_navi_info",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:map_cache_builder",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/prediction/util:drive_passage_util",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:elements_history",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "traffic_light_manager",
    srcs = ["traffic_light_manager.cc"],
    hdrs = ["traffic_light_manager.h"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
    ],
)

cc_library(
    name = "model_pool",
    srcs = ["model_pool.cc"],
    hdrs = ["model_pool.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/global:run_context",
        "//onboard/lite:module_config_helper",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/params:param_manager",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/inferencer:act_net_inferencer",
        "//onboard/prediction/inferencer:cutin_sl_net_inferencer",
        "//onboard/prediction/net/horizon:act_net_j5_inferencer",
        "//onboard/prediction/net/horizon:act_net_local_j5_inferencer",
        "//onboard/prediction/net/horizon:cutin_sl_net_j5_inferencer",
        "//onboard/prediction/net/horizon:lane_selection_net_j5_inferencer",
        "//onboard/utils:status_macros",
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
    name = "model_pool_test",
    srcs = ["model_pool_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":model_pool",
        "//onboard/global:buffered_logger",
        "//onboard/global:run_context",
        "//onboard/lite:check",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "objects_history_test",
    srcs = ["objects_history_test.cc"],
    deps = [
        ":objects_history",
        "//onboard/math:coordinate_converter",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "object_prediction_result_test",
    srcs = ["object_prediction_result_test.cc"],
    deps = [
        "object_prediction_result",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:object_prediction",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "prediction_context_test",
    srcs = ["prediction_context_test.cc"],
    deps = [
        ":objects_history",
        ":prediction_context",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction/test_util:prediction_context_builder",
        "//onboard/proto:assist_state_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "map_cache_builder_test",
    srcs = ["map_cache_builder_test.cc"],
    deps = [
        ":map_cache_builder",
        "//onboard/container:strong_int",
        "//onboard/global:test_main",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "prediction_input_test",
    srcs = ["prediction_input_test.cc"],
    deps = [
        ":object_history",
        ":objects_history",
        ":prediction_input",
        "//onboard/global:clock",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "traffic_light_manager_test",
    srcs = ["traffic_light_manager_test.cc"],
    deps = [
        ":traffic_light_manager",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "prediction_runner_test",
    srcs = ["prediction_runner_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":prediction_runner",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/params:param_manager",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/prediction/test_util:prediction_context_builder",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
