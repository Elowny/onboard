load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Protos

# Libraries.

cc_library(
    name = "external_route_request",
    srcs = ["external_route_request.cc"],
    hdrs = ["external_route_request.h"],
    data = [
    ],
    deps = [
        ":coordinate_transformer",
        ":tencent_action_format",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:http_client",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "coordinate_transformer",
    srcs = ["coordinate_transformer.cc"],
    hdrs = ["coordinate_transformer.h"],
    data = [
    ],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:gcj_transform",
    ],
)

cc_library(
    name = "tencent_action_format",
    srcs = ["tencent_action_format.cc"],
    hdrs = ["tencent_action_format.h"],
    deps = ["//onboard/proto:route_navigation_cc_proto"],
)

cc_library(
    name = "external_route_manager",
    srcs = ["external_route_manager.cc"],
    hdrs = ["external_route_manager.h"],
    deps = [
        ":coordinate_transformer",
        ":external_route_request",
        ":external_route_util",
        "//common/proto:drive_mission_cc_proto",
        "//onboard/planner/router:route_manager_state",
        "//onboard/planner/router/geometry:gfc",
        "//onboard/planner/router/noa:route_mpp",
        "//onboard/planner/router/noa:sd_route_util",
        "//onboard/proto:route_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "external_route_util",
    srcs = ["external_route_util.cc"],
    hdrs = ["external_route_util.h"],
    deps = [
        ":external_route_request",
        "//common/proto:drive_mission_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
        "@com_google_protobuf//:protobuf",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "external_route_request_test",
    srcs = ["external_route_request_test.cc"],
    deps = [
        ":external_route_request",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "coordinate_transformer_test",
    srcs = ["coordinate_transformer_test.cc"],
    deps = [
        ":coordinate_transformer",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "external_route_manager_test",
    srcs = ["external_route_manager_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":external_route_manager",
        "//onboard/global:test_main",
        "//onboard/planner/router/geometry:gfc",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
