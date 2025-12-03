load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libs.

###############################################################################
# Binaries.
cc_binary(
    name = "geometry_graph_bm",
    srcs = [
        "geometry_graph_bm.cc",
    ],
    tags = ["bm"],
    deps = [
        "//onboard/container:strong_vector",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "@com_github_google_benchmark//:benchmark",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cpplint()
