load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "geometry_state",
    hdrs = ["geometry_state.h"],
    deps = [
        "//onboard/math:vec",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "geometry_form",
    srcs = ["geometry_form.cc"],
    hdrs = ["geometry_form.h"],
    deps = [
        ":geometry_state",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "geometry_graph",
    srcs = ["geometry_graph.cc"],
    hdrs = ["geometry_graph.h"],
    deps = [
        ":geometry_form",
        "//onboard/container:strong_vector",
        "//onboard/math:frenet_frame",
        "//onboard/math:vec",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
    ],
)

cc_library(
    name = "geometry_graph_debug",
    hdrs = ["geometry_graph_debug.h"],
    deps = [
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "geometry_graph_builder",
    srcs = ["geometry_graph_builder.cc"],
    hdrs = ["geometry_graph_builder.h"],
    deps = [
        ":geometry_form_builder",
        ":geometry_graph",
        ":geometry_graph_cache",
        ":geometry_graph_debug",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math/geometry:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision:leading_groups_builder",
        "//onboard/planner/initializer:collision_checker",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "geometry_graph_cache",
    srcs = ["geometry_graph_cache.cc"],
    hdrs = ["geometry_graph_cache.h"],
    deps = [
        ":geometry_form",
        ":geometry_graph",
        ":geometry_graph_debug",
        ":geometry_state",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "geometry_form_builder",
    srcs = ["geometry_form_builder.cc"],
    hdrs = ["geometry_form_builder.h"],
    deps = [
        ":geometry_form",
        "//onboard/lite:logging",
        "//onboard/math:cubic_spline",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/math:cubic_spiral_boundary_value_problem",
        "//onboard/planner/math:quintic_spiral_boundary_value_problem",
        "//onboard/planner/math:spiral",
        "//onboard/planner/router:drive_passage",
        "//onboard/utils:status_macros",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "geometry_graph_test",
    srcs = ["geometry_graph_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":geometry_graph_builder",
        ":geometry_graph_cache",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/planner:planner_flags",
        "//onboard/planner/initializer:initializer_util",
        "//onboard/planner/initializer:test_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "geometry_form_test",
    srcs = ["geometry_form_test.cc"],
    deps = [
        ":geometry_form",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "geometry_form_builder_test",
    srcs = ["geometry_form_builder_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":geometry_form_builder",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:test_util",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/initializer:initializer_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/proto:positioning_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
