load("//bazel:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "input_proto",
    data = glob([
        "*.pb.txt",
    ]),
)

cpplint()
