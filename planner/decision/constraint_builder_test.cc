#include "onboard/planner/decision/constraint_builder.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
TEST(BuildConstraintsTest, BuildBrakeToStopConstraintTest) {
  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geometry_params =
      vehicle_params.vehicle_geometry_params();
  const auto& vehicle_model = vehicle_params.vehicle_params().model();

  const PlannerParamsProto planner_params = DefaultPlannerParams();

  const auto& psmm = CreateDojoTestPSMM();

  // Plan start point.
  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(0.0);
  plan_start_point.mutable_path_point()->set_y(0.0);
  plan_start_point.set_v(10.0);

  // Build drive passage.
  const auto current_lane_path = mapping::LanePath(
      psmm.semantic_map_manager(), /*lane_ids=*/
      {mapping::ElementId(2448), mapping::ElementId(1), mapping::ElementId(34)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, current_lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  const Box2d ego_box = ComputeAvBox(
      Vec2dFromApolloTrajectoryPointProto(plan_start_point),
      plan_start_point.path_point().theta(), vehicle_geometry_params);
  ASSIGN_OR_DIE(const auto sl_box, drive_passage.QueryFrenetBoxAt(ego_box));

  // Build path boundary.
  ASSIGN_OR_DIE(const auto path_boundary,
                BuildPathBoundaryFromDrivePassage(psmm, drive_passage));

  const auto plan_time = absl::Now();
  const auto parking_brake_release_time = plan_time - absl::Seconds(1.1);

  const LaneChangeStateProto lc_state;
  const SpacetimeTrajectoryManager st_traj_mgr;
  const PlannerObjectManager obj_mgr;
  const TrafficLightInfoMap tl_info_map;
  const DeciderStateProto decider_state_proto;
  const SceneOutputProto scene_output_proto;

  // Enable pull over test.
  {
    const DeciderInput decider_input{
        .vehicle_geometry_params = &vehicle_geometry_params,
        .motion_constraint_params = &planner_params.motion_constraint_params(),
        .config = &planner_params.decision_constraint_config(),
        .planner_semantic_map_manager = &psmm,
        .lc_state = &lc_state,
        .plan_start_point = &plan_start_point,
        .lane_path_before_lc = nullptr,
        .passage = &drive_passage,
        .sl_boundary = &path_boundary,
        .ego_frenet_box = &sl_box,
        .borrow_lane_boundary = false,
        .obj_mgr = &obj_mgr,
        .st_traj_mgr = &st_traj_mgr,
        .tl_info_map = &tl_info_map,
        .pre_decider_state = &decider_state_proto,
        .parking_brake_release_time = parking_brake_release_time,
        .teleop_enable_traffic_light_stop = true,
        .enable_pull_over = true,
        .brake_to_stop = std::nullopt,
        .max_reach_length = std::numeric_limits<double>::max(),
        .vehicle_model = vehicle_model,
        .plan_time = plan_time,
        .scene_reasoning = &scene_output_proto,
        .enable_stop_polyline_stopping = false,
        .is_engage_steer_only = false,
        .enable_force_stop = false};

    ASSIGN_OR_DIE(auto decider_output, BuildConstraints(decider_input));
    const auto constraint_manager =
        std::move(decider_output.constraint_manager);
    const auto& stop_lines = constraint_manager.StopLine();
    const auto iter = std::find_if(stop_lines.begin(), stop_lines.end(),
                                   [](const auto& stop_line) {
                                     return stop_line.source().type_case() ==
                                            SourceProto::TypeCase::kBrakeToStop;
                                   });
    EXPECT_NE(iter, stop_lines.end());
    const auto& stop_line = *iter;
    EXPECT_NEAR(stop_line.s(), 55.095, 1e-2);
    EXPECT_EQ(stop_line.id(), "pull_over");
  }

  // Brake to stop test.
  {
    const DeciderInput decider_input{
        .vehicle_geometry_params = &vehicle_geometry_params,
        .motion_constraint_params = &planner_params.motion_constraint_params(),
        .config = &planner_params.decision_constraint_config(),
        .planner_semantic_map_manager = &psmm,
        .lc_state = &lc_state,
        .plan_start_point = &plan_start_point,
        .lane_path_before_lc = nullptr,
        .passage = &drive_passage,
        .sl_boundary = &path_boundary,
        .ego_frenet_box = &sl_box,
        .borrow_lane_boundary = false,
        .obj_mgr = &obj_mgr,
        .st_traj_mgr = &st_traj_mgr,
        .tl_info_map = &tl_info_map,
        .pre_decider_state = &decider_state_proto,
        .parking_brake_release_time = parking_brake_release_time,
        .teleop_enable_traffic_light_stop = true,
        .enable_pull_over = false,
        .brake_to_stop = 1.0,
        .max_reach_length = std::numeric_limits<double>::max(),
        .vehicle_model = vehicle_model,
        .plan_time = plan_time,
        .scene_reasoning = &scene_output_proto,
        .enable_stop_polyline_stopping = false,
        .is_engage_steer_only = false,
        .enable_force_stop = false};

    ASSIGN_OR_DIE(auto decider_output, BuildConstraints(decider_input));
    const auto constraint_manager =
        std::move(decider_output.constraint_manager);
    const auto& stop_lines = constraint_manager.StopLine();
    const auto iter = std::find_if(stop_lines.begin(), stop_lines.end(),
                                   [](const auto& stop_line) {
                                     return stop_line.source().type_case() ==
                                            SourceProto::TypeCase::kBrakeToStop;
                                   });
    EXPECT_NE(iter, stop_lines.end());
    const auto& stop_line = *iter;
    EXPECT_NEAR(stop_line.s(), 55.095, 1e-2);
    EXPECT_EQ(stop_line.id(), "brake_to_stop");
  }
}
}  // namespace planner
}  // namespace qcraft
