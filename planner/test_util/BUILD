load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])
###############################################################################
# Libraries.

cc_library(
    name = "planner_test_setup",
    testonly = True,
    srcs = ["planner_test_setup.cc"],
    hdrs = ["planner_test_setup.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/lite:lite_module",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:vec",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_input",
        "//onboard/planner/proto:planner_output_cc_proto",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/proto:positioning_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "perception_object_builder",
    testonly = True,
    srcs = ["perception_object_builder.cc"],
    hdrs = ["perception_object_builder.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "predicted_trajectory_builder",
    testonly = True,
    srcs = ["predicted_trajectory_builder.cc"],
    hdrs = ["predicted_trajectory_builder.h"],
    deps = [
        "//onboard/planner:planner_defs",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "@com_google_absl//absl/memory",
    ],
)

cc_library(
    name = "object_prediction_builder",
    testonly = True,
    srcs = ["object_prediction_builder.cc"],
    hdrs = ["object_prediction_builder.h"],
    deps = [
        ":predicted_trajectory_builder",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math/geometry:util",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction/proto:prediction_common_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/memory",
    ],
)

cc_library(
    name = "planner_object_builder",
    testonly = True,
    srcs = ["planner_object_builder.cc"],
    hdrs = ["planner_object_builder.h"],
    deps = [
        ":object_prediction_builder",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/math/geometry/proto:box2d_cc_proto",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/object:planner_object",
        "//onboard/prediction:object_prediction",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
    ],
)

cc_library(
    name = "route_builder",
    testonly = True,
    srcs = ["route_builder.cc"],
    hdrs = ["route_builder.h"],
    data = [
        "//onboard/planner/params/X9:X9_param_files",
        "//onboard/planner/router/params:param_files",
    ],
    deps = [
        "//common/proto:drive_mission_cc_proto",
        "//common/proto:lane_point_cc_proto",
        "//common/proto:map_geometry_cc_proto",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner/router:route_core",
        "//onboard/planner/router:route_core_searcher",
        "//onboard/planner/router:route_util",
        "//onboard/planner/router:router_flags",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/planner/router/request:routing_request_context",
        "//onboard/planner/router/util:map_index",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    data = [
        "//onboard/planner/params:param_files",
        "//onboard/planner/params/X9:X9_param_files",
    ],
    deps = [
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:proto_util",
    ],
)

cc_library(
    name = "load_psmm_util",
    testonly = True,
    srcs = ["load_psmm_util.cc"],
    hdrs = ["load_psmm_util.h"],
    deps = [
        "//onboard/maps:map_selector",
        "//onboard/maps/v2:semantic_map_loader",
        "//onboard/math:coordinate_converter",
        "//onboard/planner:planner_semantic_map_manager",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "perception_object_builder_test",
    srcs = ["perception_object_builder_test.cc"],
    deps = [
        ":perception_object_builder",
        "//onboard/global:test_main",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/math/geometry/proto:box2d_cc_proto",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "predicted_trajectory_builder_test",
    srcs = ["predicted_trajectory_builder_test.cc"],
    deps = [
        ":predicted_trajectory_builder",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "object_prediction_builder_test",
    srcs = ["object_prediction_builder_test.cc"],
    deps = [
        ":object_prediction_builder",
        ":perception_object_builder",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "util_test",
    srcs = ["util_test.cc"],
    deps = [
        ":util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_object_builder_test",
    srcs = ["planner_object_builder_test.cc"],
    deps = [
        ":perception_object_builder",
        ":planner_object_builder",
        ":predicted_trajectory_builder",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/prediction:object_prediction",
        "//onboard/prediction:predicted_trajectory",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
