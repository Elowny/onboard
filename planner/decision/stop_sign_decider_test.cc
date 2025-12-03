#include "onboard/planner/decision/stop_sign_decider.h"

#include <memory>
#include <optional>
#include <utility>

#include "absl/time/clock.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
TEST(StopSignDeciderTest, BaseTest) {
  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto& vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto& psmm = CreateDojoTestPSMM();

  const auto current_lane_path = mapping::LanePath(
      psmm.semantic_map_manager(), /*lane_ids=*/
      {mapping::ElementId(3), mapping::ElementId(951), mapping::ElementId(43)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, current_lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(15.0);
  plan_start_point.mutable_path_point()->set_y(-3.5);
  plan_start_point.set_v(8.0);

  const double plan_time = ToUnixDoubleSeconds(absl::Now());
  const SpacetimeTrajectoryManager st_traj_mgr;
  const ::google::protobuf::RepeatedPtrField<StopSignStateProto>
      last_stop_sign_states;

  ASSIGN_OR_DIE(auto output,
                BuildStopSignConstraints(
                    psmm, st_traj_mgr, vehicle_geometry_params, drive_passage,
                    plan_time, plan_start_point, last_stop_sign_states));

  auto stop_lines = std::move(output.stop_lines);
  auto stop_sign_states = std::move(output.stop_sign_states);
  EXPECT_EQ(stop_lines.size(), 1);
  const auto stop_line = stop_lines[0];
  EXPECT_NEAR(stop_line.s(), 70.0, 1e-1);
  EXPECT_EQ(stop_sign_states.size(), 1);
  const auto& state = stop_sign_states[0];
  EXPECT_EQ(state.stop_sign_lane_id(), 951);
  EXPECT_FALSE(state.has_stop_time());
  EXPECT_EQ(state.stop_state(), StopSignStateProto::APPROACHING);
}

TEST(StopSignDeciderTest, StopToPassTest) {
  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto& vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto& psmm = CreateDojoTestPSMM();

  const auto current_lane_path = mapping::LanePath(
      psmm.semantic_map_manager(), /*lane_ids=*/
      {mapping::ElementId(3), mapping::ElementId(951), mapping::ElementId(43)},
      /*start_fraction=*/0.8, /*end_fraction=*/1.0);

  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, current_lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(63.5);
  plan_start_point.mutable_path_point()->set_y(-3.671);
  plan_start_point.set_v(0.0);

  const double plan_time = ToUnixDoubleSeconds(absl::Now());
  const SpacetimeTrajectoryManager st_traj_mgr;

  // Mock last stop sign state.
  DeciderStateProto decider_state;
  auto* stop_sign_state = decider_state.add_stop_sign_state();
  stop_sign_state->set_stop_sign_lane_id(951);
  stop_sign_state->set_stop_time(plan_time - 2.0);
  stop_sign_state->set_stop_state(StopSignStateProto::STOPPED);

  ASSIGN_OR_DIE(auto output, BuildStopSignConstraints(
                                 psmm, st_traj_mgr, vehicle_geometry_params,
                                 drive_passage, plan_time, plan_start_point,
                                 decider_state.stop_sign_state()));

  auto stop_lines = std::move(output.stop_lines);
  auto stop_sign_states = std::move(output.stop_sign_states);
  EXPECT_TRUE(stop_lines.empty());
  EXPECT_EQ(stop_sign_states.size(), 1);
  const auto& state = stop_sign_states[0];
  EXPECT_EQ(state.stop_sign_lane_id(), 951);
  EXPECT_FALSE(state.has_stop_time());
  EXPECT_EQ(state.stop_state(), StopSignStateProto::PASSED);
}
}  // namespace
}  // namespace qcraft::planner
