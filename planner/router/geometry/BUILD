load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libs.
cc_library(
    name = "gfc",
    hdrs = ["gfc.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:segment2d",
    ],
)

###############################################################################
# Binaries.
cc_binary(
    name = "gfc_bm",
    srcs = [
        "gfc_bm.cc",
    ],
    tags = ["bm"],
    deps = [
        "gfc",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_benchmark//:benchmark",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "gfc_test",
    srcs = ["gfc_test.cc"],
    deps = [
        ":gfc",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
