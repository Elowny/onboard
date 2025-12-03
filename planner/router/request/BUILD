load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Protos

# Libraries.

cc_library(
    name = "routing_request_context",
    hdrs = ["routing_request_context.h"],
    deps = [
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/planner/router/util:map_index",
        "//onboard/proto:route_cc_proto",
    ],
)

cpplint()
