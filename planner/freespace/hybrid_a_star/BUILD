load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "node_3d",
    srcs = ["node_3d.cc"],
    hdrs = ["node_3d.h"],
    deps = [
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "reeds_shepp",
    srcs = ["reeds_shepp.cc"],
    hdrs = ["reeds_shepp.h"],
    deps = [
        ":node_3d",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "fast_reeds_shepp",
    srcs = ["fast_reeds_shepp.cc"],
    hdrs = [
        "fast_reeds_shepp.h",
        "fast_reeds_shepp_table.h",
    ],
    deps = [
        ":node_3d",
        ":reeds_shepp",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "a_star_heuristics",
    srcs = ["a_star_heuristics.cc"],
    hdrs = ["a_star_heuristics.h"],
    deps = [
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner/freespace:freespace_planner_defs",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
    ],
)

cc_library(
    name = "hybrid_a_star",
    srcs = ["hybrid_a_star.cc"],
    hdrs = ["hybrid_a_star.h"],
    deps = [
        ":a_star_heuristics",
        ":fast_reeds_shepp",
        ":node_3d",
        ":reeds_shepp",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/common:vehicle_octagon_model",
        "//onboard/planner/freespace:directional_path",
        "//onboard/planner/freespace:freespace_planner_defs",
        "//onboard/planner/freespace:freespace_util",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@boost//:heap",
        "@boost//:intrusive",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Proto.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "reeds_shepp_test",
    srcs = ["reeds_shepp_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":reeds_shepp",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "hybrid_a_star_test",
    srcs = ["hybrid_a_star_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":hybrid_a_star",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/test_util:util",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
