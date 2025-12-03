load("//bazel:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "steer_torque_config",
    data = glob([
        "steer_torque_config.pb.txt",
    ]),
)

cpplint()
