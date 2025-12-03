#include "onboard/planner/decision/crosswalk_decider.h"

#include <initializer_list>
#include <memory>
#include <optional>
#include <utility>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
TEST(CrosswalkDeciderTest, EmptyObjectTest) {
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
      {mapping::ElementId(3), mapping::ElementId(950), mapping::ElementId(35)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, current_lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(48.55);
  plan_start_point.mutable_path_point()->set_y(-3.42);
  plan_start_point.set_v(10.0);

  const auto plan_time = absl::Now();

  const PlannerObjectManager empty_object_mgr;
  const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>
      last_crosswalk_states;

  ASSIGN_OR_DIE(
      auto output,
      BuildCrosswalkConstraints(CrosswalkDeciderInput{
          .vehicle_geometry_params = &vehicle_geometry_params,
          .psmm = &psmm,
          .plan_start_point = &plan_start_point,
          .passage = &drive_passage,
          .lane_path_from_start = &current_lane_path,
          .obj_mgr = &empty_object_mgr,
          .last_crosswalk_states = &last_crosswalk_states,
          .valid_cw_types = {mapping::CrosswalkProto::UNCONTROLLED,
                             mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED,
                             mapping::CrosswalkProto::MUST_YEILD},
          .now_in_seconds = ToUnixDoubleSeconds(plan_time),
          .s_offset = 0.0}));

  const auto stop_lines = std::move(output.stop_lines);
  const auto speed_regions = std::move(output.speed_regions);
  const auto crosswalk_states = std::move(output.crosswalk_states);
  EXPECT_TRUE(stop_lines.empty());
  EXPECT_TRUE(speed_regions.empty());
  EXPECT_EQ(crosswalk_states.size(), 2);
}

TEST(CrosswalkDeciderTest, UnMustYieldCrosswalkTest) {
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
      {mapping::ElementId(3), mapping::ElementId(950), mapping::ElementId(35)},
      /*start_fraction=*/0.69, /*end_fraction=*/1.0);

  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, current_lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(48.55);
  plan_start_point.mutable_path_point()->set_y(-3.42);
  plan_start_point.set_v(10.0);

  const auto plan_time = absl::Now();

  // Pedestrian object test.
  {
    ObjectsProto perception_objects;
    const auto object_1 = PerceptionObjectBuilder()
                              .set_id("PED_1")
                              .set_type(OT_PEDESTRIAN)
                              .set_pos(Vec2d(75.443, 2.587))
                              .set_yaw(-1.55)
                              .set_velocity(1.0)
                              .Build();
    *perception_objects.add_objects() = object_1;

    ObjectPredictionBuilder pred_builder;
    pred_builder.set_object(object_1)
        .add_predicted_trajectory()
        ->set_probability(1.0)
        .set_straight_line(/*start=*/Vec2d(75.443, 2.587),
                           /*end=*/Vec2d(75.443, -6.843),
                           /*init_v=*/1.0, /*last_v=*/1.0);

    const auto object_pred_1 = pred_builder.Build();
    ObjectsPredictionProto prediction_objects;
    object_pred_1.ToProto(prediction_objects.add_objects());

    PlannerObjectManager obj_mgr(BuildPlannerObjects(&perception_objects,
                                                     &prediction_objects,
                                                     /*align_time=*/0.0,
                                                     /*thread_pool=*/nullptr));
    const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>
        last_crosswalk_states;

    ASSIGN_OR_DIE(auto output,
                  BuildCrosswalkConstraints(CrosswalkDeciderInput{
                      .vehicle_geometry_params = &vehicle_geometry_params,
                      .psmm = &psmm,
                      .plan_start_point = &plan_start_point,
                      .passage = &drive_passage,
                      .lane_path_from_start = &current_lane_path,
                      .obj_mgr = &obj_mgr,
                      .last_crosswalk_states = &last_crosswalk_states,
                      .valid_cw_types =
                          {mapping::CrosswalkProto::UNCONTROLLED,
                           mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED,
                           mapping::CrosswalkProto::MUST_YEILD},
                      .now_in_seconds = ToUnixDoubleSeconds(plan_time),
                      .s_offset = 0.0}));

    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_regions = std::move(output.speed_regions);
    const auto crosswalk_states = std::move(output.crosswalk_states);
    EXPECT_FALSE(stop_lines.empty());
    const auto& stop_line = stop_lines.front();
    EXPECT_EQ(stop_line.id(), "crosswalk_524");
    EXPECT_NEAR(stop_line.s(), 34.76, 1e-2);
    EXPECT_TRUE(speed_regions.empty());
    EXPECT_EQ(crosswalk_states.size(), 2);
  }

  // Invalid object type test.
  {
    for (const auto object_type : {
             OT_UNKNOWN_STATIC,
             OT_VEHICLE,
             OT_LARGE_VEHICLE,
             OT_MOTORCYCLIST,
             OT_FOD,
             OT_UNKNOWN_MOVABLE,
             OT_VEGETATION,
             OT_BARRIER,
             OT_CONE,
             OT_WARNING_TRIANGLE,
         }) {
      ObjectsProto perception_objects;
      const auto object_1 = PerceptionObjectBuilder()
                                .set_id("object_1")
                                .set_type(object_type)
                                .set_pos(Vec2d(75.443, 2.587))
                                .set_yaw(-1.55)
                                .set_velocity(1.0)
                                .Build();
      *perception_objects.add_objects() = object_1;

      ObjectPredictionBuilder pred_builder;
      pred_builder.set_object(object_1)
          .add_predicted_trajectory()
          ->set_probability(1.0)
          .set_straight_line(/*start=*/Vec2d(75.443, 2.587),
                             /*end=*/Vec2d(75.443, -6.843),
                             /*init_v=*/1.0, /*last_v=*/1.0);

      const auto object_pred_1 = pred_builder.Build();
      ObjectsPredictionProto prediction_objects;
      object_pred_1.ToProto(prediction_objects.add_objects());

      PlannerObjectManager obj_mgr(
          BuildPlannerObjects(&perception_objects, &prediction_objects,
                              /*align_time=*/0.0,
                              /*thread_pool=*/nullptr));
      const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>
          last_crosswalk_states;

      ASSIGN_OR_DIE(auto output,
                    BuildCrosswalkConstraints(CrosswalkDeciderInput{
                        .vehicle_geometry_params = &vehicle_geometry_params,
                        .psmm = &psmm,
                        .plan_start_point = &plan_start_point,
                        .passage = &drive_passage,
                        .lane_path_from_start = &current_lane_path,
                        .obj_mgr = &obj_mgr,
                        .last_crosswalk_states = &last_crosswalk_states,
                        .valid_cw_types =
                            {mapping::CrosswalkProto::UNCONTROLLED,
                             mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED,
                             mapping::CrosswalkProto::MUST_YEILD},
                        .now_in_seconds = ToUnixDoubleSeconds(plan_time),
                        .s_offset = 0.0}));

      const auto stop_lines = std::move(output.stop_lines);
      const auto speed_regions = std::move(output.speed_regions);
      const auto crosswalk_states = std::move(output.crosswalk_states);
      EXPECT_TRUE(stop_lines.empty());
      EXPECT_TRUE(speed_regions.empty());
      EXPECT_EQ(crosswalk_states.size(), 2);
    }
  }
}

// NOLINTNEXTLINE
TEST(CrosswalkDeciderTest, MustYieldCrosswalkTest) {
  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto& vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  const auto& psmm = CreateDojoTestPSMM();

  // Mock must yield crosswalk.
  const mapping::ElementId must_yield_cw_id(524);
  {
    const mapping::CrosswalkInfo* cw_info =
        psmm.FindCrosswalkByIdOrNull(must_yield_cw_id);
    auto* mutable_cw_proto =
        const_cast<mapping::CrosswalkProto*>(cw_info->proto);
    mutable_cw_proto->set_type(mapping::CrosswalkProto::MUST_YEILD);
  }

  const auto current_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(1), mapping::ElementId(34)},
                        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, current_lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  const auto plan_time = absl::Now();

  // Passed -> Passed.
  {
    // Mock last crosswalk state.
    DeciderStateProto decider_state;
    auto* cw_state = decider_state.add_crosswalk_state();
    cw_state->set_crosswalk_id(must_yield_cw_id.value());
    cw_state->set_stop_state(CrosswalkStateProto::PASSED);

    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(78.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(10.0);

    const PlannerObjectManager empty_obj_mgr;

    ASSIGN_OR_DIE(auto output,
                  BuildCrosswalkConstraints(CrosswalkDeciderInput{
                      .vehicle_geometry_params = &vehicle_geometry_params,
                      .psmm = &psmm,
                      .plan_start_point = &plan_start_point,
                      .passage = &drive_passage,
                      .lane_path_from_start = &current_lane_path,
                      .obj_mgr = &empty_obj_mgr,
                      .last_crosswalk_states = &decider_state.crosswalk_state(),
                      .valid_cw_types =
                          {mapping::CrosswalkProto::UNCONTROLLED,
                           mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED,
                           mapping::CrosswalkProto::MUST_YEILD},
                      .now_in_seconds = ToUnixDoubleSeconds(plan_time),
                      .s_offset = 0.0}));

    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_regions = std::move(output.speed_regions);
    const auto crosswalk_states = std::move(output.crosswalk_states);
    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_regions.empty());
    EXPECT_EQ(crosswalk_states.size(), 2);
    const auto& cw_state_first = crosswalk_states[0];
    EXPECT_EQ(cw_state_first.crosswalk_id(), must_yield_cw_id.value());
    EXPECT_EQ(cw_state_first.stop_state(), CrosswalkStateProto::PASSED);
  }

  // Approaching -> Stopped.
  {
    // Mock last crosswalk state.
    DeciderStateProto decider_state;
    auto* cw_state = decider_state.add_crosswalk_state();
    cw_state->set_crosswalk_id(must_yield_cw_id.value());
    cw_state->set_stop_state(CrosswalkStateProto::APPROACHING);

    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(67.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(0.0);

    const PlannerObjectManager empty_obj_mgr;

    ASSIGN_OR_DIE(auto output,
                  BuildCrosswalkConstraints(CrosswalkDeciderInput{
                      .vehicle_geometry_params = &vehicle_geometry_params,
                      .psmm = &psmm,
                      .plan_start_point = &plan_start_point,
                      .passage = &drive_passage,
                      .lane_path_from_start = &current_lane_path,
                      .obj_mgr = &empty_obj_mgr,
                      .last_crosswalk_states = &decider_state.crosswalk_state(),
                      .valid_cw_types =
                          {mapping::CrosswalkProto::UNCONTROLLED,
                           mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED,
                           mapping::CrosswalkProto::MUST_YEILD},
                      .now_in_seconds = ToUnixDoubleSeconds(plan_time),
                      .s_offset = 0.0}));

    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_regions = std::move(output.speed_regions);
    const auto crosswalk_states = std::move(output.crosswalk_states);
    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_regions.empty());
    EXPECT_EQ(crosswalk_states.size(), 2);
    const auto& cw_state_first = crosswalk_states[0];
    EXPECT_EQ(cw_state_first.crosswalk_id(), must_yield_cw_id.value());
    EXPECT_EQ(cw_state_first.stop_state(), CrosswalkStateProto::STOPPED);
  }

  // Approaching -> Passed.
  {
    // Mock last crosswalk state.
    DeciderStateProto decider_state;
    auto* cw_state = decider_state.add_crosswalk_state();
    cw_state->set_crosswalk_id(must_yield_cw_id.value());
    cw_state->set_stop_state(CrosswalkStateProto::APPROACHING);

    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(78.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(5.0);

    const PlannerObjectManager empty_obj_mgr;

    ASSIGN_OR_DIE(auto output,
                  BuildCrosswalkConstraints(CrosswalkDeciderInput{
                      .vehicle_geometry_params = &vehicle_geometry_params,
                      .psmm = &psmm,
                      .plan_start_point = &plan_start_point,
                      .passage = &drive_passage,
                      .lane_path_from_start = &current_lane_path,
                      .obj_mgr = &empty_obj_mgr,
                      .last_crosswalk_states = &decider_state.crosswalk_state(),
                      .valid_cw_types =
                          {mapping::CrosswalkProto::UNCONTROLLED,
                           mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED,
                           mapping::CrosswalkProto::MUST_YEILD},
                      .now_in_seconds = ToUnixDoubleSeconds(plan_time),
                      .s_offset = 0.0}));

    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_regions = std::move(output.speed_regions);
    const auto crosswalk_states = std::move(output.crosswalk_states);
    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_regions.empty());
    EXPECT_EQ(crosswalk_states.size(), 2);
    const auto& cw_state_first = crosswalk_states[0];
    EXPECT_EQ(cw_state_first.crosswalk_id(), must_yield_cw_id.value());
    EXPECT_EQ(cw_state_first.stop_state(), CrosswalkStateProto::PASSED);
  }

  // Approaching -> Approaching.
  {
    // Mock last crosswalk state.
    DeciderStateProto decider_state;
    auto* cw_state = decider_state.add_crosswalk_state();
    cw_state->set_crosswalk_id(must_yield_cw_id.value());
    cw_state->set_stop_state(CrosswalkStateProto::APPROACHING);

    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(70.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(5.0);

    const PlannerObjectManager empty_obj_mgr;

    ASSIGN_OR_DIE(auto output,
                  BuildCrosswalkConstraints(CrosswalkDeciderInput{
                      .vehicle_geometry_params = &vehicle_geometry_params,
                      .psmm = &psmm,
                      .plan_start_point = &plan_start_point,
                      .passage = &drive_passage,
                      .lane_path_from_start = &current_lane_path,
                      .obj_mgr = &empty_obj_mgr,
                      .last_crosswalk_states = &decider_state.crosswalk_state(),
                      .valid_cw_types =
                          {mapping::CrosswalkProto::UNCONTROLLED,
                           mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED,
                           mapping::CrosswalkProto::MUST_YEILD},
                      .now_in_seconds = ToUnixDoubleSeconds(plan_time),
                      .s_offset = 0.0}));

    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_regions = std::move(output.speed_regions);
    const auto crosswalk_states = std::move(output.crosswalk_states);
    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_regions.empty());
    EXPECT_EQ(crosswalk_states.size(), 2);
    const auto& cw_state_first = crosswalk_states[0];
    EXPECT_EQ(cw_state_first.crosswalk_id(), must_yield_cw_id.value());
    EXPECT_EQ(cw_state_first.stop_state(), CrosswalkStateProto::APPROACHING);
  }

  // Stopped -> Passed.
  {
    // Mock last crosswalk state.
    DeciderStateProto decider_state;
    auto* cw_state = decider_state.add_crosswalk_state();
    cw_state->set_crosswalk_id(must_yield_cw_id.value());
    cw_state->set_stop_state(CrosswalkStateProto::STOPPED);
    cw_state->set_stop_time(
        ToUnixDoubleSeconds(plan_time - absl::Seconds(8.1)));

    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(70.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(5.0);

    const PlannerObjectManager empty_obj_mgr;

    ASSIGN_OR_DIE(auto output,
                  BuildCrosswalkConstraints(CrosswalkDeciderInput{
                      .vehicle_geometry_params = &vehicle_geometry_params,
                      .psmm = &psmm,
                      .plan_start_point = &plan_start_point,
                      .passage = &drive_passage,
                      .lane_path_from_start = &current_lane_path,
                      .obj_mgr = &empty_obj_mgr,
                      .last_crosswalk_states = &decider_state.crosswalk_state(),
                      .valid_cw_types =
                          {mapping::CrosswalkProto::UNCONTROLLED,
                           mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED,
                           mapping::CrosswalkProto::MUST_YEILD},
                      .now_in_seconds = ToUnixDoubleSeconds(plan_time),
                      .s_offset = 0.0}));

    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_regions = std::move(output.speed_regions);
    const auto crosswalk_states = std::move(output.crosswalk_states);
    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_regions.empty());
    EXPECT_EQ(crosswalk_states.size(), 2);
    const auto& cw_state_first = crosswalk_states[0];
    EXPECT_EQ(cw_state_first.crosswalk_id(), must_yield_cw_id.value());
    EXPECT_EQ(cw_state_first.stop_state(), CrosswalkStateProto::PASSED);
  }

  // Stopped -> Stopped.
  {
    // Mock last crosswalk state.
    DeciderStateProto decider_state;
    auto* cw_state = decider_state.add_crosswalk_state();
    cw_state->set_crosswalk_id(must_yield_cw_id.value());
    cw_state->set_stop_state(CrosswalkStateProto::STOPPED);
    cw_state->set_stop_time(
        ToUnixDoubleSeconds(plan_time - absl::Seconds(7.9)));

    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(70.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(5.0);

    const PlannerObjectManager empty_obj_mgr;

    ASSIGN_OR_DIE(auto output,
                  BuildCrosswalkConstraints(CrosswalkDeciderInput{
                      .vehicle_geometry_params = &vehicle_geometry_params,
                      .psmm = &psmm,
                      .plan_start_point = &plan_start_point,
                      .passage = &drive_passage,
                      .lane_path_from_start = &current_lane_path,
                      .obj_mgr = &empty_obj_mgr,
                      .last_crosswalk_states = &decider_state.crosswalk_state(),
                      .valid_cw_types =
                          {mapping::CrosswalkProto::UNCONTROLLED,
                           mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED,
                           mapping::CrosswalkProto::MUST_YEILD},
                      .now_in_seconds = ToUnixDoubleSeconds(plan_time),
                      .s_offset = 0.0}));

    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_regions = std::move(output.speed_regions);
    const auto crosswalk_states = std::move(output.crosswalk_states);
    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_regions.empty());
    EXPECT_EQ(crosswalk_states.size(), 2);
    const auto& cw_state_first = crosswalk_states[0];
    EXPECT_EQ(cw_state_first.crosswalk_id(), must_yield_cw_id.value());
    EXPECT_EQ(cw_state_first.stop_state(), CrosswalkStateProto::STOPPED);
  }
}
}  // namespace
}  // namespace qcraft::planner
