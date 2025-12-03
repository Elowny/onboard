load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "geometry_method_defs",
    srcs = [],
    hdrs = ["geometry_method_defs.h"],
    deps = [
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "geometry_method_util",
    srcs = ["geometry_method_util.cc"],
    hdrs = ["geometry_method_util.h"],
    deps = [
        ":geometry_method_defs",
        ":geometry_path_collision_checker",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner/common:geometry_path_util",
        "//onboard/planner/common:vehicle_octagon_model",
        "//onboard/planner/freespace:freespace_planner_defs",
        "//onboard/planner/freespace:freespace_util",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "geometry_connection",
    srcs = ["geometry_connection.cc"],
    hdrs = ["geometry_connection.h"],
    deps = [
        ":geometry_method_defs",
        ":geometry_method_util",
        "//onboard/math:util",
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "geometry_path_collision_checker",
    srcs = ["geometry_path_collision_checker.cc"],
    hdrs = ["geometry_path_collision_checker.h"],
    deps = [
        ":geometry_method_defs",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner/common:circle_path_overlap",
        "//onboard/planner/common:vehicle_octagon_model",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "parallel_parking",
    srcs = ["parallel_parking.cc"],
    hdrs = ["parallel_parking.h"],
    deps = [
        ":geometry_connection",
        ":geometry_method_defs",
        ":geometry_method_util",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner/freespace:freespace_planner_defs",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "perpendicular_parking",
    srcs = ["perpendicular_parking.cc"],
    hdrs = ["perpendicular_parking.h"],
    deps = [
        ":geometry_connection",
        ":geometry_method_util",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner/freespace:freespace_planner_defs",
        "//onboard/planner/freespace/geometry_method:geometry_method_defs",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "forward_perpendicular_parking",
    srcs = ["forward_perpendicular_parking.cc"],
    hdrs = ["forward_perpendicular_parking.h"],
    deps = [
        ":geometry_connection",
        ":geometry_method_util",
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner/freespace:freespace_planner_defs",
        "//onboard/planner/freespace/geometry_method:geometry_method_defs",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "perpendicular_parking_out",
    srcs = ["perpendicular_parking_out.cc"],
    hdrs = ["perpendicular_parking_out.h"],
    deps = [
        ":geometry_method_util",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner/freespace:freespace_planner_defs",
        "//onboard/planner/freespace/geometry_method:geometry_method_defs",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:parking_spot_finder_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "geometry_parking",
    srcs = ["geometry_parking.cc"],
    hdrs = ["geometry_parking.h"],
    deps = [
        ":geometry_method_defs",
        ":parallel_parking",
        ":perpendicular_parking",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/maps:maps_common",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/freespace:directional_path",
        "//onboard/planner/freespace:freespace_planner_defs",
        "//onboard/planner/freespace:freespace_util",
        "//onboard/planner/freespace/geometry_method:geometry_method_util",
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
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
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
    name = "geometry_connection_test",
    srcs = ["geometry_connection_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":geometry_connection",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "geometry_path_collision_checker_test",
    srcs = ["geometry_path_collision_checker_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":geometry_method_util",
        ":geometry_path_collision_checker",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "geometry_method_util_test",
    srcs = ["geometry_method_util_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":geometry_method_util",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/freespace:freespace_util",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "parallel_parking_test",
    srcs = ["parallel_parking_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":geometry_method_util",
        ":parallel_parking",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/test_util:util",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "perpendicular_parking_test",
    srcs = ["perpendicular_parking_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":perpendicular_parking",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "forward_perpendicular_parking_test",
    srcs = ["forward_perpendicular_parking_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":forward_perpendicular_parking",
        ":geometry_method_util",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/test_util:util",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "perpendicular_parking_out_test",
    srcs = ["perpendicular_parking_out_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":geometry_method_util",
        ":perpendicular_parking_out",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/test_util:util",
        "//onboard/vis/canvas",
        "//onboard/vis/canvas/proto:canvas_buffer_cc_proto",
        "//onboard/vis/common:color",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "geometry_parking_test",
    srcs = ["geometry_parking_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":geometry_parking",
        "//onboard/container:strong_int",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
