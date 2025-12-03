load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
cc_library(
    name = "route_struct",
    hdrs = ["route_struct.h"],
)

cc_library(
    name = "async_retry",
    srcs = ["async_retry.cc"],
    hdrs = ["async_retry.h"],
)

cc_library(
    name = "route_extend_path",
    srcs = ["route_extend_path.cc"],
    hdrs = ["route_extend_path.h"],
    deps = [
        ":route_struct",
        "//onboard/container:strong_int",
        "//onboard/global:logging",
        "//onboard/maps:lane_path_util",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:lane_path_cc_proto",
        "//onboard/planner:composite_lane_path",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "map_index",
    srcs = ["map_index.cc"],
    hdrs = ["map_index.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:spatial_search_util",
        "//onboard/math:vec",
        "//onboard/planner/router/geometry:gfc",
        "@boost//:geometry",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "async_retry_test",
    srcs = ["async_retry_test.cc"],
    deps = [
        ":async_retry",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_extend_path_test",
    srcs = ["route_extend_path_test.cc"],
    deps = [
        ":route_extend_path",
        "//onboard/global:test_main",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/router:route_test_util",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "map_index_test",
    srcs = ["map_index_test.cc"],
    deps = [
        ":map_index",
        "//onboard/global:test_main",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner/router:route_test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_struct_test",
    srcs = ["route_struct_test.cc"],
    deps = [
        ":route_struct",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
