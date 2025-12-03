load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "object_history_sampler",
    srcs = ["object_history_sampler.cc"],
    hdrs = ["object_history_sampler.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/global:car_common",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:fast_math",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:polynomial",
        "//onboard/math:polynomial_fitter",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/math/geometry/proto:box2d_cc_proto",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:object_history_span",
        "//onboard/prediction/container:prediction_object",
        "//onboard/prediction/util:perception_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "//onboard/utils:elements_history",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "map_sampler",
    srcs = ["map_sampler.cc"],
    hdrs = ["map_sampler.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/maps:lane_speed_limit",
        "//onboard/maps:level_id",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame_util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "feature_extraction_util",
    srcs = ["feature_extraction_util.cc"],
    hdrs = ["feature_extraction_util.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/maps:maps_common",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:prediction_message_compressor",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "act_net_feature",
    hdrs = ["act_net_feature.h"],
    deps = [
        "//onboard/maps:semantic_map_defs",
        "//onboard/prediction:prediction_defs",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "act_net_speed_feature",
    hdrs = ["act_net_speed_feature.h"],
    deps = [":act_net_feature"],
)

cc_library(
    name = "act_net_feature_extractor",
    srcs = ["act_net_feature_extractor.cc"],
    hdrs = ["act_net_feature_extractor.h"],
    deps = [
        ":act_net_feature",
        ":map_sampler",
        ":object_history_sampler",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:fast_math",
        "//onboard/math:frenet_common",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/geometry:util",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/proto:act_net_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "act_net_speed_feature_extractor",
    srcs = ["act_net_speed_feature_extractor.cc"],
    hdrs = ["act_net_speed_feature_extractor.h"],
    deps = [
        ":act_net_feature",
        ":act_net_feature_extractor",
        ":act_net_speed_feature",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/object:spacetime_object_state",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "cutin_sl_feature",
    hdrs = ["cutin_sl_feature.h"],
    deps = [
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:vec",
        "//onboard/prediction:prediction_defs",
    ],
)

cc_library(
    name = "lane_selection_feature",
    hdrs = ["lane_selection_feature.h"],
    deps = [
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:vec",
        "//onboard/prediction:prediction_defs",
    ],
)

cc_library(
    name = "cutin_sl_feature_extractor",
    srcs = ["cutin_sl_feature_extractor.cc"],
    hdrs = ["cutin_sl_feature_extractor.h"],
    deps = [
        ":map_sampler",
        ":object_history_sampler",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/maps:maps_common",
        "//onboard/math:fast_math",
        "//onboard/math:frenet_common",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/feature_extractor:cutin_sl_feature",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/prediction/proto:prediction_cutin_sl_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "lane_selection_feature_extractor",
    srcs = ["lane_selection_feature_extractor.cc"],
    hdrs = ["lane_selection_feature_extractor.h"],
    deps = [
        ":map_sampler",
        ":object_history_sampler",
        "//onboard/math:frenet_common",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/prediction/feature_extractor:lane_selection_feature",
        "//onboard/prediction/proto:prediction_lane_selection_cc_proto",
    ],
)

cc_library(
    name = "qc_net_feature",
    hdrs = ["qc_net_feature.h"],
    deps = [
        ":act_net_feature",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:vec",
        "//onboard/prediction:prediction_defs",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "feature_extraction_util_test",
    srcs = ["feature_extraction_util_test.cc"],
    deps = [
        ":feature_extraction_util",
        "//onboard/global:test_main",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:util",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "map_sampler_test",
    srcs = ["map_sampler_test.cc"],
    deps = [
        ":map_sampler",
        "//onboard/global:test_main",
        "//onboard/maps:maps_common",
        "//onboard/planner/test_util:load_psmm_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "act_net_feature_extractor_test",
    srcs = ["act_net_feature_extractor_test.cc"],
    deps = [
        ":act_net_feature",
        ":act_net_feature_extractor",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "cutin_sl_feature_extractor_test",
    srcs = ["cutin_sl_feature_extractor_test.cc"],
    deps = [
        ":cutin_sl_feature",
        ":cutin_sl_feature_extractor",
        "//onboard/global:test_main",
        "//onboard/math:frenet_common",
        "//onboard/math:vec",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lane_selection_feature_extractor_test",
    srcs = ["lane_selection_feature_extractor_test.cc"],
    deps = [
        ":lane_selection_feature",
        ":lane_selection_feature_extractor",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:vec",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "object_history_sampler_test",
    srcs = ["object_history_sampler_test.cc"],
    deps = [
        ":object_history_sampler",
        "//onboard/global:test_main",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:prediction_object",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "act_net_speed_feature_extractor_test",
    srcs = ["act_net_speed_feature_extractor_test.cc"],
    deps = [
        ":act_net_feature",
        ":act_net_speed_feature_extractor",
        ":object_history_sampler",
        "//onboard/global:test_main",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:prediction_object",
        "//onboard/prediction/test_util:object_history_builder",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
