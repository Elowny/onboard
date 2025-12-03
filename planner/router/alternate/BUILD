load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "alternative_route_util",
    srcs = ["alternative_route_util.cc"],
    hdrs = ["alternative_route_util.h"],
    deps = [
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_library(
    name = "alternative_route",
    srcs = ["alternative_route.cc"],
    hdrs = ["alternative_route.h"],
    deps = [
        ":alternative_route_util",
        "//common/proto:drive_mission_cc_proto",
        "//onboard/lite:logging",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:spatial_search_util",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner/router:multi_stops_request",
        "//onboard/planner/router:road_conditions_process",
        "//onboard/planner/router:route_core",
        "//onboard/planner/router:route_core_searcher",
        "//onboard/planner/router/request:routing_request_context",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_protobuf//:protobuf",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "alternative_route_util_test",
    srcs = ["alternative_route_util_test.cc"],
    deps = [
        ":alternative_route_util",
        "//common/proto:lane_point_cc_proto",
        "//onboard/planner/router:route_test_util",
        "//onboard/proto:route_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "alternative_route_test",
    srcs = ["alternative_route_test.cc"],
    deps = [
        ":alternative_route",
        "//common/proto:lane_point_cc_proto",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math:vec",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/router:route_util",
        "//onboard/proto:route_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
