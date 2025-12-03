load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "tl_info",
    srcs = ["tl_info.cc"],
    hdrs = ["tl_info.h"],
    deps = [
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner/decision/proto:traffic_light_info_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "traffic_light_info_collector",
    srcs = ["traffic_light_info_collector.cc"],
    hdrs = ["traffic_light_info_collector.h"],
    deps = [
        ":tl_info",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:route_sections",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:time_util",
    ],
)

cc_library(
    name = "traffic_light_decider",
    srcs = ["traffic_light_decider.cc"],
    hdrs = ["traffic_light_decider.h"],
    deps = [
        ":tl_info",
        "//onboard/maps:map_or_die_macros",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:speed_profile",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:route_sections",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/time",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "traffic_light_info_collector_test",
    srcs = ["traffic_light_info_collector_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":traffic_light_info_collector",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//onboard/global:test_main",
        "//onboard/maps:map_or_die_macros",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "tl_info_test",
    srcs = ["tl_info_test.cc"],
    deps = [
        ":tl_info",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "traffic_light_decider_test",
    srcs = ["traffic_light_decider_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":traffic_light_decider",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/decision:decision_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/test_util:load_psmm_util",
    ],
)

cpplint()
