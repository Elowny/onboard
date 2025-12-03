load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

# Library

cc_library(
    name = "prediction_j5_qnn",
    srcs = [
        "prediction_j5_qnn.cc",
    ],
    hdrs = [
        "prediction_j5_qnn.h",
    ],
    deps = [
        ":horizon_tensor_wrapper",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/lite:qissue_trans",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/nets/qnn/qnn",
        "//onboard/params:param_converter",
        "//onboard/proto:q_issue_cc_proto",
    ],
)

cc_library(
    name = "horizon_tensor_wrapper",
    srcs = [
        "horizon_tensor_wrapper.cc",
    ],
    hdrs = [
        "horizon_tensor_wrapper.h",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/nets/qnn/qnn",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "act_net_j5",
    srcs = [
    ],
    hdrs = [
        "act_net_j5.h",
    ],
    deps = [
        "//onboard/math:piecewise_linear_function",
    ],
)

cc_library(
    name = "cutin_sl_net_j5",
    srcs = [
    ],
    hdrs = [
        "cutin_sl_net_j5.h",
    ],
    deps = [
        "//onboard/math:piecewise_linear_function",
    ],
)

cc_library(
    name = "act_net_j5_inferencer",
    srcs = [
        "act_net_j5_inferencer.cc",
    ],
    hdrs = [
        "act_net_j5_inferencer.h",
    ],
    data = [
        "//onboard/params/nets:actnet_j5_model_data",
    ],
    deps = [
        ":act_net_j5",
        ":horizon_tensor_wrapper",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/feature_extractor:act_net_feature",
        "//onboard/prediction/feature_extractor:map_sampler",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/net/horizon:prediction_j5_qnn",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "lane_selection_net_j5_inferencer",
    srcs = [
        "lane_selection_net_j5_inferencer.cc",
    ],
    hdrs = [
        "lane_selection_net_j5_inferencer.h",
    ],
    data = [
        "//onboard/params/nets:lane_selection_net_j5_model_data",
    ],
    deps = [
        "//onboard/global:trace",
        "//onboard/math:frenet_common",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/feature_extractor:lane_selection_feature",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/net/horizon:horizon_tensor_wrapper",
        "//onboard/prediction/net/horizon:lane_selection_net",
        "//onboard/prediction/net/horizon:prediction_j5_qnn",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "cutin_sl_net_j5_inferencer",
    srcs = [
        "cutin_sl_net_j5_inferencer.cc",
    ],
    hdrs = [
        "cutin_sl_net_j5_inferencer.h",
    ],
    data = [
        "//onboard/params/nets:cutin_sl_net_j5_model_data",
    ],
    deps = [
        ":cutin_sl_net_j5",
        ":horizon_tensor_wrapper",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:frenet_common",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/feature_extractor:cutin_sl_feature",
        "//onboard/prediction/feature_extractor:map_sampler",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/net/horizon:prediction_j5_qnn",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "act_net_local_j5",
    hdrs = ["act_net_local_j5.h"],
    deps = [
        "//onboard/math:piecewise_linear_function",
    ],
)

cc_library(
    name = "act_net_local_j5_inferencer",
    srcs = ["act_net_local_j5_inferencer.cc"],
    hdrs = ["act_net_local_j5_inferencer.h"],
    data = [
        "//onboard/params/nets:actnet_local_j5_model_data",
    ],
    deps = [
        ":act_net_local_j5",
        ":horizon_tensor_wrapper",
        ":prediction_j5_qnn",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/container:strong_int",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:fast_math",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/nets/proto:net_param_cc_proto",
        # "//onboard/planner/common:plot_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/feature_extractor:act_net_feature",
        "//onboard/prediction/feature_extractor:map_sampler",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/feature_extractor:qc_net_feature",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "lane_selection_net",
    srcs = [
    ],
    hdrs = [
        "lane_selection_net.h",
    ],
    data = [
    ],
    deps = [
    ],
)

cc_library(
    name = "tensor_id_name",
    hdrs = ["tensor_id_name.h"],
    deps = [
        "//onboard/utils:map_util",
    ],
)

# Benchmark
cc_binary(
    name = "act_net_j5_inferencer_bm",
    testonly = True,
    srcs = [
        "act_net_j5_inferencer_bm.cc",
    ],
    data = [
        "//onboard/params/nets:actnet_j5_model_data",
    ],
    tags = ["bm"],
    deps = [
        ":act_net_j5_inferencer",
        "//onboard/async:thread_pool",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:objects_history",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/feature_extractor:map_sampler",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/prediction/test_util:prediction_context_builder",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:errors",
        "@com_github_google_benchmark//:benchmark",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ],
)

cc_binary(
    name = "act_net_local_j5_inferencer_bm",
    testonly = True,
    srcs = [
        "act_net_local_j5_inferencer_bm.cc",
    ],
    data = [
        "//onboard/params/nets:actnet_local_j5_model_data",
    ],
    tags = ["bm"],
    deps = [
        ":act_net_local_j5_inferencer",
        "//onboard/async:thread_pool",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:objects_history",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/feature_extractor:map_sampler",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/prediction/test_util:prediction_context_builder",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:errors",
        "@com_github_google_benchmark//:benchmark",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ],
)

cc_binary(
    name = "lane_selection_net_j5_inferencer_bm",
    testonly = True,
    srcs = [
        "lane_selection_net_j5_inferencer_bm.cc",
    ],
    data = [
        "//onboard/params/nets:lane_selection_net_j5_model_data",
    ],
    tags = ["bm"],
    deps = [
        ":lane_selection_net_j5_inferencer",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:objects_history",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/net/horizon:lane_selection_net",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/prediction/predictor:lane_selection_net_j5_predictor",
        "//onboard/prediction/test_util:prediction_context_builder",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:errors",
        "@com_github_google_benchmark//:benchmark",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ],
)

cc_binary(
    name = "cutin_sl_net_j5_inferencer_bm",
    testonly = True,
    srcs = ["cutin_sl_net_j5_inferencer_bm.cc"],
    data = [
        "//onboard/params/nets:cutin_sl_net_j5_model_data",
    ],
    tags = ["bm"],
    deps = [
        ":cutin_sl_net_j5_inferencer",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/feature_extractor:map_sampler",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/prediction/test_util:prediction_context_builder",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_benchmark//:benchmark",
    ],
)

# Test
cc_test(
    name = "horizon_tensor_wrapper_test",
    srcs = ["horizon_tensor_wrapper_test.cc"],
    deps = [
        ":horizon_tensor_wrapper",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/nets/qnn/qnn",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "prediction_j5_qnn_test",
    srcs = ["prediction_j5_qnn_test.cc"],
    data = [
        "//onboard/params/nets:actnet_j5_model_data",
    ],
    deps = [
        ":prediction_j5_qnn",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/prediction/test_util:net_input_generator",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "act_net_j5_inferencer_test",
    srcs = ["act_net_j5_inferencer_test.cc"],
    data = [
        "//onboard/params/nets:actnet_j5_model_data",
    ],
    deps = [
        ":act_net_j5_inferencer",
        ":horizon_tensor_wrapper",
        "//onboard/async:thread_pool",
        "//onboard/global:test_main",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:objects_history",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/feature_extractor:map_sampler",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/prediction/test_util:prediction_context_builder",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:errors",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "cutin_sl_net_j5_inferencer_test",
    srcs = ["cutin_sl_net_j5_inferencer_test.cc"],
    data = [
        "//onboard/params/nets:cutin_sl_net_j5_model_data",
    ],
    deps = [
        ":cutin_sl_net_j5_inferencer",
        "//onboard/global:test_main",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/prediction/test_util:prediction_context_builder",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lane_selection_net_j5_inferencer_test",
    srcs = ["lane_selection_net_j5_inferencer_test.cc"],
    data = [
        "//onboard/params/nets:lane_selection_net_j5_model_data",
    ],
    deps = [
        ":lane_selection_net_j5_inferencer",
        "//onboard/global:test_main",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:objects_history",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/net/horizon:lane_selection_net",
        "//onboard/prediction/post_process:conflict_resolver_params",
        "//onboard/prediction/predictor:lane_selection_net_j5_predictor",
        "//onboard/prediction/test_util:prediction_context_builder",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:errors",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "actnet_j5_sync_test",
    srcs = [
        "actnet_j5_sync_test.cc",
    ],
    data = [
        "//onboard/params/nets:actnet_j5_model_data",
        "//onboard/params/run_params:run_param_data",
    ],
    tags = [
        "exclusive",
        "manual",
    ],
    deps = [
        ":horizon_tensor_wrapper",
        ":prediction_j5_qnn",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/prediction/test_util:net_input_generator",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "tensor_id_name_test",
    srcs = ["tensor_id_name_test.cc"],
    deps = [
        ":tensor_id_name",
        "@com_github_neargye_magic_enum//:magic_enum",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
