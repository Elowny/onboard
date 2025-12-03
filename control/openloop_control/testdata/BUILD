load("//bazel:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "steer_angle_file",
    data = [
        "steer_angle.txt",
    ],
)

filegroup(
    name = "openloop_config",
    data = [
        "openloop.pb.txt",
    ],
)

cpplint()
