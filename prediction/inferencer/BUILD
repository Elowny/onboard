load("//bazel:common.bzl", "if_cuda")
load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "prediction_qnn",
    srcs = [
        "prediction_qnn.cc",
    ],
    hdrs = [
        "prediction_qnn.h",
    ],
    deps = [
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/nets/qnn/qnn",
        "//onboard/lite:qissue_trans",
    ] + if_cuda([
        "//onboard/nets/qcv:qcv_cuda_base",
    ]),
)

cc_library(
    name = "act_net_inferencer",
    srcs = [
        "act_net_inferencer.cc",
    ],
    hdrs = ["act_net_inferencer.h"],
    data = [
        "//onboard/params/nets:actnet_model_data",
    ],
    deps = [
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/feature_extractor:act_net_feature",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/prediction/inferencer:prediction_qnn",
        "//onboard/simulation:sim_cache",
        "@boost//:functional",
        "@boost//:serialization",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_binary(
    name = "act_net_inferencer_bm",
    srcs = [
        "act_net_inferencer_bm.cc",
    ],
    tags = ["bm"],
    deps = [
        ":act_net_inferencer",
        "//onboard/math:vec",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/feature_extractor:act_net_feature",
        "//onboard/utils:errors",
        "@com_github_google_benchmark//:benchmark",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "act_net_speed_inferencer",
    srcs = [
        "act_net_speed_inferencer.cc",
    ],
    hdrs = ["act_net_speed_inferencer.h"],
    data = [
        "//onboard/params/nets:actnet_speed_model_data",
    ],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/global:timer",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/feature_extractor:act_net_feature",
        "//onboard/prediction/feature_extractor:act_net_speed_feature",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/prediction/inferencer:prediction_qnn",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_binary(
    name = "act_net_speed_inferencer_bm",
    srcs = [
        "act_net_speed_inferencer_bm.cc",
    ],
    tags = ["bm"],
    deps = [
        ":act_net_speed_inferencer",
        "//onboard/math:vec",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/feature_extractor:act_net_feature",
        "//onboard/prediction/feature_extractor:act_net_speed_feature",
        "//onboard/utils:errors",
        "@com_github_google_benchmark//:benchmark",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "cutin_sl_net_inferencer",
    srcs = [
        "cutin_sl_net_inferencer.cc",
    ],
    hdrs = ["cutin_sl_net_inferencer.h"],
    data = [
        "//onboard/params/nets:cutin_sl_net_model_data",
    ],
    deps = [
        "//onboard/global:trace",
        "//onboard/math:util",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/feature_extractor:cutin_sl_feature",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/prediction/inferencer:prediction_qnn",
        "@com_google_absl//absl/types:span",
    ],
)

# Benchmark
cc_binary(
    name = "cutin_sl_net_inferencer_bm",
    testonly = True,
    srcs = [
        "cutin_sl_net_inferencer_bm.cc",
    ],
    tags = ["bm"],
    deps = [
        "//onboard/math:vec",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/feature_extractor:cutin_sl_feature",
        "//onboard/prediction/feature_extractor:cutin_sl_feature_extractor",
        "//onboard/prediction/feature_extractor:map_sampler",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/inferencer:cutin_sl_net_inferencer",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/utils:errors",
        "@com_github_google_benchmark//:benchmark",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "actnet_sync_test",
    srcs = [
        "actnet_sync_test.cc",
    ],
    data = [
        "//onboard/params/nets:actnet_model_data",
        "//onboard/params/run_params:run_param_data",
    ],
    tags = [
        "exclusive",
        "manual",
    ],
    deps = [
        ":prediction_qnn",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_manager",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "actnet_speed_sync_test",
    srcs = [
        "actnet_speed_sync_test.cc",
    ],
    data = [
        "//onboard/params/nets:actnet_speed_model_data",
        "//onboard/params/run_params:run_param_data",
    ],
    tags = [
        "exclusive",
        "manual",
    ],
    deps = [
        ":prediction_qnn",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_manager",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "prediction_qnn_test",
    srcs = ["prediction_qnn_test.cc"],
    data = [
        "//onboard/params/nets:actnet_model_data",
    ],
    deps = [
        ":prediction_qnn",
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
    name = "act_net_inferencer_test",
    srcs = ["act_net_inferencer_test.cc"],
    deps = [
        ":act_net_inferencer",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/utils:errors",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "act_net_speed_inferencer_test",
    srcs = ["act_net_speed_inferencer_test.cc"],
    deps = [
        ":act_net_speed_inferencer",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/utils:errors",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "cutin_sl_net_inferencer_test",
    srcs = ["cutin_sl_net_inferencer_test.cc"],
    deps = [
        ":cutin_sl_net_inferencer",
        "//onboard/global:test_main",
        "//onboard/params:param_finder",
        "//onboard/params:param_finder_without_nets",
        "//onboard/prediction/feature_extractor:cutin_sl_feature",
        "//onboard/utils:errors",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "cutin_sl_net_sync_test",
    srcs = [
        "cutin_sl_net_sync_test.cc",
    ],
    data = [
        "//onboard/params/nets:cutin_sl_net_model_data",
        "//onboard/params/run_params:run_param_data",
    ],
    tags = [
        "exclusive",
        "manual",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params:param_finder",
        "//onboard/params:param_manager",
        "//onboard/prediction/inferencer:prediction_qnn",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
