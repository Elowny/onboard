load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "post_process",
    srcs = ["post_process.cc"],
    hdrs = ["post_process.h"],
    deps = [
        ":conflict_resolver",
        ":conflict_resolver_params",
        ":post_process_defs",
        ":trajectory_modifier",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/container:strong_int",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_prediction_result",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "trajectory_modifier",
    srcs = ["trajectory_modifier.cc"],
    hdrs = ["trajectory_modifier.h"],
    deps = [
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "conflict_resolver",
    srcs = ["conflict_resolver.cc"],
    hdrs = ["conflict_resolver.h"],
    deps = [
        ":conflict_resolver_input",
        ":conflict_resolver_params",
        ":object_conflict_manager",
        ":object_conflict_resolver",
        ":post_process_defs",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "//onboard/utils:source_location",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "object_conflict_manager",
    srcs = ["object_conflict_manager.cc"],
    hdrs = ["object_conflict_manager.h"],
    deps = [
        ":conflict_resolver_util",
        ":post_process_defs",
        ":relation_analyzer",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/container:strong_int",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:buffered_logger",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/speed:speed_vector",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:map_util",
        "//onboard/utils:status_macros",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "conflict_resolver_input",
    hdrs = ["conflict_resolver_input.h"],
    deps = [
        ":conflict_resolver_params",
        ":object_conflict_manager",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "object_svt_sample",
    srcs = ["object_svt_sample.cc"],
    hdrs = ["object_svt_sample.h"],
    deps = [
        "//onboard/container:strong_vector",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "object_st_map",
    srcs = ["object_st_map.cc"],
    hdrs = ["object_st_map.h"],
    deps = [
        ":conflict_resolver_params",
        ":object_conflict_manager",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/container:strong_int",
        "//onboard/global:trace",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/speed:overlap_info",
        "//onboard/planner/speed:speed_vector",
        "//onboard/planner/speed:st_boundary",
        "//onboard/planner/speed:st_point",
        "//onboard/planner/speed:vt_point",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:status_macros",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "svt_feature_cost",
    srcs = ["svt_feature_cost.cc"],
    hdrs = ["svt_feature_cost.h"],
    deps = [
        ":object_svt_sample",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/planner/speed:speed_point",
        "//onboard/planner/speed:speed_vector",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "svt_cost_provider",
    srcs = ["svt_cost_provider.cc"],
    hdrs = ["svt_cost_provider.h"],
    deps = [
        ":object_svt_sample",
        ":svt_feature_cost",
        "//onboard/lite:check",
        "//onboard/planner/speed:speed_vector",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "conflict_resolver_params",
    srcs = ["conflict_resolver_params.cc"],
    hdrs = ["conflict_resolver_params.h"],
    data = [
        ":prediction_post_process_default_params.pb.txt",
    ],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:file_util",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "conflict_resolver_util",
    srcs = ["conflict_resolver_util.cc"],
    hdrs = ["conflict_resolver_util.h"],
    deps = [
        ":object_svt_sample",
        "//onboard/global:trace",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/speed:speed_point",
        "//onboard/planner/speed:speed_vector",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "object_svt_graph",
    srcs = ["object_svt_graph.cc"],
    hdrs = ["object_svt_graph.h"],
    deps = [
        ":conflict_resolver_util",
        ":object_svt_sample",
        ":svt_cost_provider",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/planner/speed:speed_point",
        "//onboard/planner/speed:speed_vector",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "object_conflict_resolver",
    srcs = ["object_conflict_resolver.cc"],
    hdrs = ["object_conflict_resolver.h"],
    deps = [
        ":conflict_resolver_input",
        ":conflict_resolver_params",
        ":object_conflict_manager",
        ":object_st_map",
        ":object_svt_graph",
        ":relation_analyzer",
        ":svt_cost_provider",
        "//onboard/async:thread_pool",
        "//onboard/container:strong_int",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/planner/speed:speed_vector",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:status_macros",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "relation_analyzer",
    srcs = ["relation_analyzer.cc"],
    hdrs = ["relation_analyzer.h"],
    deps = [
        "//onboard/container:strong_int",
        "//onboard/container:strong_vector",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:fast_math",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/geometry/proto:box2d_cc_proto",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner:trajectory_point",
        "//onboard/planner:trajectory_util",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:status_macros",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "post_process_defs",
    hdrs = ["post_process_defs.h"],
    deps = [
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto/perception/fusion:object_cc_proto",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

cc_binary(
    name = "post_process_bm",
    testonly = True,
    srcs = ["post_process_bm.cc"],
    deps = [
        ":conflict_resolver_params",
        ":post_process",
        "//onboard/global:trace",
        "//onboard/math:vec",
        "//onboard/planner/object:planner_object_manager_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_message_compressor",
        "//onboard/prediction/container:object_prediction_result",
        "//onboard/prediction/proto:conflict_resolver_cc_proto",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "@com_github_google_benchmark//:benchmark",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "post_process_test",
    srcs = ["post_process_test.cc"],
    deps = [
        ":post_process",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "conflict_resolver_test",
    srcs = ["conflict_resolver_test.cc"],
    deps = [
        ":conflict_resolver",
        ":post_process_defs",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/container:object_prediction_result",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "object_svt_sample_test",
    srcs = ["object_svt_sample_test.cc"],
    deps = [
        ":object_svt_sample",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "svt_feature_cost_test",
    srcs = ["svt_feature_cost_test.cc"],
    deps = [
        ":conflict_resolver_params",
        ":object_svt_sample",
        ":svt_feature_cost",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/planner/speed:speed_point",
        "//onboard/planner/speed:st_boundary",
        "//onboard/proto:perception_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "object_svt_graph_test",
    srcs = ["object_svt_graph_test.cc"],
    deps = [
        ":conflict_resolver_params",
        ":conflict_resolver_util",
        ":object_svt_graph",
        ":object_svt_sample",
        ":svt_cost_provider",
        "//onboard/container:strong_int",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/planner/speed:speed_point",
        "//onboard/planner/speed:speed_vector",
        "//onboard/proto:perception_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "relation_analyzer_test",
    srcs = ["relation_analyzer_test.cc"],
    deps = [
        ":relation_analyzer",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/util:kinematic_model",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "trajectory_modifier_test",
    srcs = ["trajectory_modifier_test.cc"],
    deps = [
        ":conflict_resolver_params",
        ":trajectory_modifier",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "conflict_resolver_util_test",
    srcs = ["conflict_resolver_util_test.cc"],
    deps = [
        ":conflict_resolver_util",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
