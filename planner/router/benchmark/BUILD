load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libs.

###############################################################################
# Binaries.

cc_binary(
    name = "graph_bm",
    srcs = [
        "graph_bm.cc",
    ],
    tags = ["bm"],
    deps = [
        "//onboard/planner/router/lane_graph/v2:dijkstra",
        "@com_github_google_benchmark//:benchmark",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_binary(
    name = "nav_info_graph_bm",
    srcs = [
        "nav_info_graph_bm.cc",
    ],
    tags = ["bm"],
    deps = [
        "//onboard/maps:lane_point",
        "//onboard/planner/router:route_manager_util",
        "//onboard/utils:file_util",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cpplint()
