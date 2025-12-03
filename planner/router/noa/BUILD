load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Protos

# Libraries.

cc_library(
    name = "hdmap_util",
    srcs = ["hdmap_util.cc"],
    hdrs = ["hdmap_util.h"],
    deps = [
        "//onboard/global:logging",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner/router/geometry:gfc",
    ],
)

cc_library(
    name = "hd_hmi_adapter",
    srcs = ["hd_hmi_adapter.cc"],
    hdrs = ["hd_hmi_adapter.h"],
    deps = [
        "//common/proto:drive_mission_cc_proto",
        "//onboard/global:logging",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/ehr/proto:odd_cc_proto",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/planner/router:road_conditions_process",
        "//onboard/planner/router:route_manager_util",
        "//onboard/planner/router:router_defs",
        "//onboard/planner/router/noa:hdmap_util",
        "//onboard/proto:adasis_cc_proto",
        "//onboard/proto:route_cc_proto",
        "@boost//:range",
    ],
)

cc_library(
    name = "route_mpp",
    srcs = ["route_mpp.cc"],
    hdrs = ["route_mpp.h"],
    deps = [
        ":hd_hmi_adapter",
        ":hdmap_util",
        "//common/proto:drive_mission_cc_proto",
        "//onboard/global:logging",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_or_die_macros",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps/ehr/proto:odd_cc_proto",
        "//onboard/maps/v2:semantic_map_manager",
        "//onboard/planner/router:road_conditions_process",
        "//onboard/planner/router:route_manager_util",
        "//onboard/planner/router:router_defs",
        "//onboard/proto:adasis_cc_proto",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_library(
    name = "sd_route_util",
    srcs = ["sd_route_util.cc"],
    hdrs = ["sd_route_util.h"],
    deps = [
        "//onboard/math/geometry:util",
        "//onboard/planner/router:route_manager_state",
        "//onboard/proto:adasis_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/proto:route_navigation_cc_proto",
        "//onboard/utils:source_location",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "noa_main_loop",
    srcs = ["noa_main_loop.cc"],
    hdrs = ["noa_main_loop.h"],
    deps = [
        ":hd_hmi_adapter",
        ":route_mpp",
        "//onboard/maps/v2:semantic_map_listener",
        "//onboard/maps/v2:semantic_map_spatial_index",
        "//onboard/math/geometry:util",
        "//onboard/planner/router:route_error",
        "//onboard/planner/router:route_manager_state",
        "//onboard/planner/router:route_module_input",
        "//onboard/planner/router:route_util",
        "//onboard/planner/router:router_defs",
        "//onboard/planner/router/noa:sd_route_util",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/planner/router/sim:snapshot_sim_state",
        "//onboard/planner/router/thirdparty:external_route_manager",
        "//onboard/proto:adasis_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/proto:route_navigation_cc_proto",
        "//onboard/utils:source_location",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

# ###############################################################################
# Tests.
cc_test(
    name = "hdmap_util_test",
    srcs = ["hdmap_util_test.cc"],
    deps = [
        ":hdmap_util",
        "//onboard/global:test_main",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_mpp_test",
    srcs = ["route_mpp_test.cc"],
    deps = [
        ":route_mpp",
        "//onboard/global:test_main",
        "//onboard/maps/v2:semantic_map_loader",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "sd_route_util_test",
    srcs = ["sd_route_util_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":sd_route_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "noa_main_loop_test",
    srcs = ["noa_main_loop_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":noa_main_loop",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "hd_hmi_adapter_test",
    srcs = ["hd_hmi_adapter_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":hd_hmi_adapter",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
