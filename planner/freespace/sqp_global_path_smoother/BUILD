load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "convex_region_generator",
    srcs = ["convex_region_generator.cc"],
    hdrs = ["convex_region_generator.h"],
    deps = [
        ":convex_region_defs",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:eigen",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:hyperplane",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner/common:discretized_path",
        "//onboard/proto:trajectory_point_cc_proto",
    ],
)

cc_library(
    name = "convex_region_defs",
    hdrs = ["convex_region_defs.h"],
    deps = [
        "//onboard/math:eigen",
        "//onboard/math:vec",
        "//onboard/math/geometry:gjk2d",
        "//onboard/math/geometry:hyperplane",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
    ],
)

cc_library(
    name = "geometric_utils",
    srcs = ["geometric_utils.cc"],
    hdrs = ["geometric_utils.h"],
    deps = [
        ":convex_region_defs",
        "//onboard/math:fast_math",
        "//onboard/math:vec",
        "//onboard/math/geometry:hyperplane",
        "//onboard/math/geometry:intersection_util",
    ],
)

cc_library(
    name = "hpipm_solver_defs",
    hdrs = ["hpipm_solver_defs.h"],
    deps = [
        "//onboard/math:eigen",
    ],
)

cc_library(
    name = "hpipm_solver",
    srcs = ["hpipm_solver.cc"],
    hdrs = ["hpipm_solver.h"],
    deps = [
        ":hpipm_solver_defs",
        "//onboard/global:buffered_logger",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/math:eigen",
        "//third_party/hpipm",
        "//third_party/hpipm:blasfeo",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "sqp_global_path_smoother",
    srcs = ["sqp_global_path_smoother.cc"],
    hdrs = ["sqp_global_path_smoother.h"],
    deps = [
        ":convex_region_defs",
        ":convex_region_generator",
        ":geometric_utils",
        ":hpipm_solver",
        ":hpipm_solver_defs",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:circle2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner/common:discretized_path",
        "//onboard/planner/common:vehicle_shape",
        "//onboard/planner/freespace:directional_path",
        "//onboard/planner/freespace:freespace_planner_defs",
        "//onboard/planner/freespace:freespace_util",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/vis/canvas",
        "//onboard/vis/common:color",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
    ],
)
###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "convex_region_generator_test",
    srcs = ["convex_region_generator_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":convex_region_generator",
        ":geometric_utils",
        "//onboard/global:test_main",
        "//onboard/math:eigen",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "sqp_global_path_smoother_test",
    srcs = ["sqp_global_path_smoother_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":sqp_global_path_smoother",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/freespace/hybrid_a_star",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/test_util:util",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
