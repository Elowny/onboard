#include "onboard/planner/initializer/searcher_cost_feature.h"

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

TEST(AccelerationFeatureCost, Cost) {
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  const AccelerationFeatureCost acc_cost(
      planner_params.motion_constraint_params());
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  const double duration = 2.0;
  GeometryState fake_state({.xy = Vec2d(0.0, 0.0)});
  const auto motion_form =
      std::make_unique<StationaryMotion>(duration, fake_state);
  edge_info.motion_form = motion_form.get();
  const MotionState dummy_state = MotionState{.t = 0.0, .a = 1.0};
  const MotionState dummy_state1 = MotionState{.t = 0.1, .a = 2.0};
  const MotionState dummy_state2 = MotionState{.t = 0.2, .a = 1.0};

  edge_info.equal_interval_states.push_back(dummy_state);
  edge_info.equal_interval_states.push_back(dummy_state1);
  edge_info.equal_interval_states.push_back(dummy_state2);

  std::vector<double> cost(1);
  acc_cost.ComputeCost(edge_info, absl::MakeSpan(cost));

  ASSERT_NEAR(
      cost[0],
      10.0 / Sqr(planner_params.motion_constraint_params().max_acceleration()) *
          0.1 * 0.5,
      1e-4);
}

TEST(LaneBoundaryFeatureCost, Cost) {
  int n = 10;
  std::vector<double> s_vec, left_l, right_l, target_left_l, target_right_l;
  s_vec.reserve(n);
  left_l.reserve(n);
  right_l.reserve(n);
  std::vector<Vec2d> left_xy, right_xy, target_left_xy, target_right_xy;
  left_xy.reserve(n);
  right_xy.reserve(n);

  const double step_s = 1.0;
  for (double s = 0.0; s < step_s * n - step_s * 0.1; s += step_s) {
    s_vec.emplace_back(s);
    left_l.emplace_back(2.0);
    right_l.emplace_back(-2.0);
    left_xy.emplace_back(s, 2.0);
    right_xy.emplace_back(s, -2.0);
  }
  target_left_l = left_l;
  target_right_l = right_l;
  target_left_xy = left_xy;
  target_right_xy = right_xy;
  PathSlBoundary path_bound(s_vec, right_l, left_l, target_right_l,
                            target_left_l, right_xy, left_xy, target_right_xy,
                            target_left_xy);
  const auto vehicle_geom = DefaultVehicleGeometry();
  const double sdc_half_width = vehicle_geom.width() * 0.5;
  const LaneBoundaryFeatureCost lb_cost(&path_bound, sdc_half_width);
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  const double duration = 0.2;
  GeometryState fake_state({.xy = Vec2d(0.0, 0.0)});
  const auto motion_form =
      std::make_unique<StationaryMotion>(duration, fake_state);
  edge_info.motion_form = motion_form.get();
  const MotionState dummy_state = MotionState{.t = 0.0, .l = 1.0};
  const MotionState dummy_state1 = MotionState{.t = 0.1, .l = 2.0};
  const MotionState dummy_state2 = MotionState{.t = 0.2, .l = 1.0};

  edge_info.equal_interval_states.push_back(dummy_state);
  edge_info.equal_interval_states.push_back(dummy_state1);
  edge_info.equal_interval_states.push_back(dummy_state2);

  std::vector<double> cost(3);
  lb_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  EXPECT_NEAR(
      cost[0],
      0.05 * (2.0 * Sqr(std::clamp(1.0 / kDefaultHalfLaneWidth, 0.0, 1.0)) +
              2.0 * Sqr(std::clamp(2.0 / kDefaultHalfLaneWidth, 0.0, 1.0))),
      1e-4);
  EXPECT_NEAR(
      cost[1],
      0.05 * (2.0 * Sqr(std::clamp(1.0 / (2.0 - sdc_half_width), 0.0, 1.0)) +
              2.0 * Sqr(std::clamp(2.0 / (2.0 - sdc_half_width), 0.0, 1.0))),
      1e-4);
  EXPECT_NEAR(
      cost[2],
      0.05 * (2.0 * Sqr(std::clamp(1.0 / (2.0 - sdc_half_width), 0.0, 1.0)) +
              2.0 * Sqr(std::clamp(2.0 / (2.0 - sdc_half_width), 0.0, 1.0))),
      1e-4);
}

TEST(LateralAccelerationFeatureCost, ZeroCost) {
  const LateralAccelerationFeatureCost lat_cost(false);
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  const MotionState dummy_state = MotionState{.t = 0.0, .l = 1.0};
  const MotionState dummy_state1 = MotionState{.t = 0.1, .l = 2.0};
  const MotionState dummy_state2 = MotionState{.t = 0.2, .l = 1.0};

  edge_info.equal_interval_states.push_back(dummy_state);
  edge_info.equal_interval_states.push_back(dummy_state1);
  edge_info.equal_interval_states.push_back(dummy_state2);
  std::vector<double> cost(3);

  lat_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(cost[0], 0.0, 1e-4);
  ASSERT_NEAR(cost[1], 0.0, 1e-4);
  lat_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(cost[1], 0, 1e-4);
}

TEST(LateralAccelerationFeatureCost, NonZeroCost) {
  const LateralAccelerationFeatureCost lat_cost(false);
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  const MotionState dummy_state =
      MotionState{.k = 0.2, .ref_k = 0.1, .t = 0.0, .v = 10, .dddl = 1.0};
  const MotionState dummy_state1 =
      MotionState{.k = 0.3, .ref_k = 0.2, .t = 0.1, .v = 10, .dddl = 2.0};
  const MotionState dummy_state2 =
      MotionState{.k = 0.2, .ref_k = 0.1, .t = 0.2, .v = 10, .dddl = 1.0};

  edge_info.equal_interval_states.push_back(dummy_state);
  edge_info.equal_interval_states.push_back(dummy_state1);
  edge_info.equal_interval_states.push_back(dummy_state2);
  std::vector<double> cost(3);

  lat_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(cost[0], 0.2, 1e-4);
  ASSERT_NEAR(cost[1], 81.0 * 4 * 0.1 * 0.5, 1e-4);
  ASSERT_NEAR(cost[2], 2.5 * 0.1 * 0.5, 1e-4);
}

TEST(StopConstraintFeatureCost, Cost) {
  const std::vector<double> stop_s = {10.0};
  const StopConstraintFeatureCost stop_cost(stop_s);
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  const MotionState dummy_state = MotionState{.t = 0.0, .accumulated_s = 9};
  const MotionState dummy_state1 = MotionState{.t = 0.1, .accumulated_s = 11};

  edge_info.equal_interval_states.push_back(dummy_state);
  edge_info.equal_interval_states.push_back(dummy_state1);

  std::vector<double> cost(1);
  stop_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(cost[0], 1.0, 1e-4);
}

TEST(FinalProgressCost, Cost) {
  // Create fake path boundary.
  const int n = 100;
  std::vector<double> s_vec, left_l, right_l, target_left_l, target_right_l;
  s_vec.reserve(n);
  left_l.reserve(n);
  right_l.reserve(n);
  std::vector<Vec2d> left_xy, right_xy, target_left_xy, target_right_xy;
  left_xy.reserve(n);
  right_xy.reserve(n);

  const double step_s = 1.0;
  for (double s = 0.0; s < step_s * n - step_s * 0.1; s += step_s) {
    s_vec.emplace_back(s);
    left_l.emplace_back(2.0);
    right_l.emplace_back(-2.0);
    left_xy.emplace_back(s, 2.0);
    right_xy.emplace_back(s, -2.0);
  }
  target_left_l = left_l;
  target_right_l = right_l;
  target_left_xy = left_xy;
  target_right_xy = right_xy;
  PathSlBoundary path_bound(s_vec, right_l, left_l, target_right_l,
                            target_left_l, right_xy, left_xy, target_right_xy,
                            target_left_xy);

  const double max_accumulated_s = 50.0;
  const FinalProgressFeatureCost progress_cost(/*traj_horizon=*/9.9,
                                               &path_bound, max_accumulated_s);
  // Create a straight geometry form.
  const int state_size = 11;
  std::vector<GeometryState> states;
  states.reserve(state_size);
  const double step_length = 1.0;
  const double s_offset = 2.0;
  for (int i = 0; i < state_size; ++i) {
    states.push_back(GeometryState{
        .xy = Vec2d(0.0, s_offset + step_length * i),
        .accumulated_s = s_offset + step_length * i,
        .l = 0.0,
    });
  }
  std::unique_ptr<GeometryForm> ptr_geometry_form =
      std::make_unique<PiecewiseLinearGeometry>(states);
  ASSERT_NEAR(ptr_geometry_form->length(), 10.0, 1e-4);

  // Build motion forms. Brake to stationary state.
  std::unique_ptr<MotionForm> ptr_decelerate_to_stationary =
      std::make_unique<ConstAccelMotion>(/*traj_horizon=*/9.9, 2.0, -1.0,
                                         ptr_geometry_form.get());
  ASSERT_NEAR(ptr_decelerate_to_stationary->duration(), 9.9, 1e-4);
  auto motion_states =
      ptr_decelerate_to_stationary->SampleEqualIntervalStates();
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  edge_info.equal_interval_states = std::move(motion_states);
  edge_info.motion_form = ptr_decelerate_to_stationary.get();

  std::vector<double> costs(2);
  progress_cost.ComputeCost(edge_info, absl::MakeSpan(costs));
  EXPECT_NEAR(costs[0], 3.6, 1e-4);
  EXPECT_EQ(costs[1], 0.0);

  // Const velocity.
  std::unique_ptr<MotionForm> ptr_const_v = std::make_unique<ConstAccelMotion>(
      /*traj_horizon=*/9.9, 2.0, 0.0, ptr_geometry_form.get());
  ASSERT_NEAR(ptr_const_v->duration(), 5.0, 1e-4);
  auto motion_states_2 = ptr_const_v->SampleEqualIntervalStates();
  MotionEdgeInfo edge_info_2;
  edge_info_2.start_t = 1.0;
  edge_info_2.equal_interval_states = std::move(motion_states_2);
  edge_info_2.motion_form = ptr_const_v.get();

  progress_cost.ComputeCost(edge_info_2, absl::MakeSpan(costs));
  EXPECT_EQ(costs[0], 0.0);
  EXPECT_EQ(costs[1], 0.0);

  edge_info_2.start_t = 6.0;
  progress_cost.ComputeCost(edge_info_2, absl::MakeSpan(costs));
  EXPECT_NEAR(costs[0], 3.02, 1e-4);
  EXPECT_EQ(costs[1], 0.0);
}

TEST(LeadingObjectFeatureCost, Cost) {
  SetMap("dojo");
  const auto& psmm = planner::CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const std::vector<Vec2d> object_positions = {
      Vec2d(90.0, 0.0), Vec2d(95.0, 0.0), Vec2d(110.0, 0.0), Vec2d(115.0, 0.0)};

  std::vector<std::vector<std::string>> leading_groups;
  std::vector<std::string> leading_trajs;
  ObjectVector<PlannerObject> objects;

  for (int i = 0; i < object_positions.size(); ++i) {
    const auto& pos = object_positions[i];
    const auto perc_obj = PerceptionObjectBuilder()
                              .set_id(absl::StrFormat("Leading%d", i))
                              .set_type(ObjectType::OT_VEHICLE)
                              .set_timestamp(0.0)
                              .set_velocity(1.0)
                              .set_yaw(0.0)
                              .set_length_width(4.0, 2.0)
                              .set_box_center(pos)
                              .Build();
    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perc_obj)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(0.5)
        .set_straight_line(Vec2d::Zero() + pos, Vec2d(10.0, 0.0) + pos,
                           /*init_v=*/1.0, /*last_v=*/1.0);
    leading_trajs.push_back(absl::StrFormat("Leading%d-idx%d", i, 0));
    // leading traj 0, 1 belong to the first group; traj 2, 3 belong to the
    // other
    if (i == 1) leading_trajs.swap(leading_groups.emplace_back());
    objects.push_back(builder.Build());
  }
  leading_trajs.swap(leading_groups.emplace_back());

  const auto object_mgr = PlannerObjectManager(objects);

  const SpacetimeTrajectoryManager st_traj_mgr(object_mgr.planner_objects(),
                                               /*thread_pool=*/nullptr);

  // build drive passage
  const mapping::LanePath target_lane_path(
      smm, {mapping::ElementId(2471), mapping::ElementId(53)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, RouteSections::BuildFromLanePath(psmm, target_lane_path),
          target_lane_path,
          /*extend_len=*/50.0);
  EXPECT_OK(backward_extended_lane_path);
  ASSIGN_OR_DIE(
      const auto drive_passage,
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr, target_lane_path,
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        target_lane_path.length(), target_lane_path.back(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt));

  const double ego_front_to_ra = 4.0;
  const double ego_back_to_ra = 1.0;

  // follow leading group 1, and should overtake group 0 in the end
  int leading_group_idx = 1;
  const LeadingObjectFeatureCost leading_cost(
      /*traj_horizon=*/9.9, drive_passage, st_traj_mgr, leading_groups,
      leading_group_idx, ego_front_to_ra, ego_back_to_ra);

  const int state_size = 11;
  std::vector<GeometryState> states;
  states.reserve(state_size);
  const double step_length = 2.0;
  const double x_offset = 0.0;

  // set s_offset as 67.0 to check "not overtake group 0", cost == 1.0
  // as 69.0 to check "overtake group 0 and behind group 1", cost == 0.0
  // as 92.0 to check "not behind group 1", cost == 1.0
  const double s_offset = 67.0;
  for (int i = 0; i < state_size; ++i) {
    states.push_back(GeometryState{
        .xy = Vec2d(x_offset + step_length * i, 0.0),
        .accumulated_s = s_offset + step_length * i,
        .l = 0.0,
    });
  }
  std::unique_ptr<GeometryForm> ptr_geometry_form =
      std::make_unique<PiecewiseLinearGeometry>(states);
  ASSERT_NEAR(ptr_geometry_form->length(), 20.0, 1e-4);

  // Const velocity.
  std::vector<double> cost(1);
  std::unique_ptr<MotionForm> ptr_const_v = std::make_unique<ConstAccelMotion>(
      /*traj_horizon=*/9.9, 2.0, 0.0, ptr_geometry_form.get());
  ASSERT_NEAR(ptr_const_v->duration(), 10.0, 1e-4);
  auto motion_states = ptr_const_v->SampleEqualIntervalStates();
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  edge_info.equal_interval_states = std::move(motion_states);
  edge_info.motion_form = ptr_const_v.get();

  leading_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  EXPECT_EQ(cost[0], 1.0);
}

TEST(AstarLeadingObjectFeatureCost, Cost) {
  SetMap("dojo");
  const auto& psmm = planner::CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const std::vector<Vec2d> object_positions = {
      Vec2d(90.0, 0.0), Vec2d(95.0, 0.0), Vec2d(110.0, 0.0), Vec2d(115.0, 0.0)};

  std::vector<std::vector<std::string>> leading_groups;
  std::vector<std::string> leading_trajs;
  ObjectVector<PlannerObject> objects;

  for (int i = 0; i < object_positions.size(); ++i) {
    const auto& pos = object_positions[i];
    const auto perc_obj = PerceptionObjectBuilder()
                              .set_id(absl::StrFormat("Leading%d", i))
                              .set_type(ObjectType::OT_VEHICLE)
                              .set_timestamp(0.0)
                              .set_velocity(1.0)
                              .set_yaw(0.0)
                              .set_length_width(4.0, 2.0)
                              .set_box_center(pos)
                              .Build();
    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perc_obj)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(0.5)
        .set_straight_line(Vec2d::Zero() + pos, Vec2d(10.0, 0.0) + pos,
                           /*init_v=*/1.0, /*last_v=*/1.0);
    leading_trajs.push_back(absl::StrFormat("Leading%d-idx%d", i, 0));
    // leading traj 0, 1 belong to the first group; traj 2, 3 belong to the
    // other
    if (i == 1) leading_trajs.swap(leading_groups.emplace_back());
    objects.push_back(builder.Build());
  }
  leading_trajs.swap(leading_groups.emplace_back());

  const auto object_mgr = PlannerObjectManager(objects);

  const SpacetimeTrajectoryManager st_traj_mgr(object_mgr.planner_objects(),
                                               /*thread_pool=*/nullptr);

  // build drive passage
  const mapping::LanePath target_lane_path(
      smm, {mapping::ElementId(2471), mapping::ElementId(53)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, RouteSections::BuildFromLanePath(psmm, target_lane_path),
          target_lane_path,
          /*extend_len=*/50.0);
  EXPECT_OK(backward_extended_lane_path);
  ASSIGN_OR_DIE(
      const auto drive_passage,
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr, target_lane_path,
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        target_lane_path.length(), target_lane_path.back(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt));

  const double ego_front_to_ra = 4.0;
  const double ego_back_to_ra = 1.0;

  bool able_to_overtake_leading_behind = false;

  // follow leading group 1, and should overtake group 0 in the end
  int leading_group_idx = 1;
  const AstarLeadingObjectFeatureCost leading_cost(
      /*traj_horizon=*/9.9, drive_passage, st_traj_mgr, leading_groups,
      leading_group_idx, able_to_overtake_leading_behind, ego_front_to_ra,
      ego_back_to_ra);

  const int state_size = 11;
  std::vector<GeometryState> states;
  states.reserve(state_size);
  const double step_length = 2.0;
  const double x_offset = 0.0;

  // When s_offset is 67.0, it actually fails to overtake group 0.
  // BUt when able_to_overtake_leading_behind is false, the cost will be 0.0
  // instead of 1.0, since overtaking cost is not considered
  const double s_offset = 67.0;
  for (int i = 0; i < state_size; ++i) {
    states.push_back(GeometryState{
        .xy = Vec2d(x_offset + step_length * i, 0.0),
        .accumulated_s = s_offset + step_length * i,
        .l = 0.0,
    });
  }
  std::unique_ptr<GeometryForm> ptr_geometry_form =
      std::make_unique<PiecewiseLinearGeometry>(states);
  ASSERT_NEAR(ptr_geometry_form->length(), 20.0, 1e-4);

  // Const velocity.
  std::vector<double> cost(1);
  std::unique_ptr<MotionForm> ptr_const_v = std::make_unique<ConstAccelMotion>(
      /*traj_horizon=*/9.9, 2.0, 0.0, ptr_geometry_form.get());
  ASSERT_NEAR(ptr_const_v->duration(), 10.0, 1e-4);
  auto motion_states = ptr_const_v->SampleEqualIntervalStates();
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  edge_info.equal_interval_states = std::move(motion_states);
  edge_info.motion_form = ptr_const_v.get();

  leading_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  EXPECT_EQ(cost[0], 0.0);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
