load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "generate_route",
    srcs = ["generate_route.cc"],
    hdrs = ["generate_route.h"],
    data = [
        "//onboard/planner/router/params:param_files",
    ],
    deps = [
        "//onboard/maps:lane_path_util",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner/router:multi_stops_request",
        "//onboard/planner/router:route_core",
        "//onboard/planner/router:route_manager",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/router:router_flags",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/planner/router/util:map_index",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:file_util",
        "@com_google_absl//absl/strings",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "generate_route_test",
    srcs = ["generate_route_test.cc"],
    deps = [
        ":generate_route",
        "//onboard/global:test_main",
        "//onboard/planner/router:route_test_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
