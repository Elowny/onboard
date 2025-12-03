load("//bazel:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "ts_pkmpc_controller_conf_proto",
    data = glob([
        "*.pb.txt",
    ]),
)

cpplint()
