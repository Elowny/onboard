load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

###############################################################################
# Proto.

###############################################################################
# Binaries.

cc_binary(
    name = "acc_preliminary_speed_timing",
    testonly = True,
    srcs = ["acc_preliminary_speed_timing.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
        "//onboard/planner/params:param_files",
    ],
    deps = [
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:driving_map_topo",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/plan/acc:acc_corridor",
        "//onboard/planner/plan/acc:acc_corridor_util",
        "//onboard/planner/plan/acc:acc_corridor_with_map",
        "//onboard/planner/plan/acc:acc_preliminary_speed",
        "//onboard/planner/plan/acc:acc_target",
        "//onboard/planner/plan/acc:acc_target_selector_with_map",
        "//onboard/planner/plan/acc:test_util",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/speed/proto:speed_finder_params_cc_proto",
        "//onboard/planner/test_util:load_psmm_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/planner/test_util:util",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/proto/perception/fusion:object_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Tests.

cpplint()
