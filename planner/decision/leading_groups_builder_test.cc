#include "onboard/planner/decision/leading_groups_builder.h"

#include <algorithm>
#include <limits>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {

namespace {

PlannerObjectManager BuildMultiplePhantomVehicles(
    const std::vector<Vec2d>& object_positions) {
  ObjectVector<PlannerObject> objects;
  for (int i = 0; i < object_positions.size(); ++i) {
    const auto& pos = object_positions[i];
    const auto perc_obj = PerceptionObjectBuilder()
                              .set_id(absl::StrFormat("Phantom%d", i))
                              .set_type(ObjectType::OT_VEHICLE)
                              .set_pos(pos)
                              .set_length_width(4.5, 2.2)
                              .set_yaw(0.0)
                              .Build();
    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perc_obj)
        .set_stationary(true)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(0.5)
        .set_stationary_traj(pos, /*theta=*/0.0);
    objects.push_back(builder.Build());
  }
  return PlannerObjectManager(objects);
}

TEST(LeadingGroupsBuilder, LCIgnoreStalledObjects) {
  VehicleGeometryParamsProto vehicle_geom = DefaultVehicleGeometry();

  // Construct sdc pose.
  absl::Time timestamp = absl::UnixEpoch();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(timestamp),
                 Vec2d(116.5, kDefaultLaneWidth), 0.0, Vec2d(11.0, 0.0));
  const auto plan_start_point = ConvertToTrajPointProto(sdc_pose);

  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  // Create multiple objects on target lane.
  const std::vector<Vec2d> object_positions = {
      Vec2d(117.57, 0.081), Vec2d(127.031, 0.081), Vec2d(134.936, -1.144),
      Vec2d(142.5, -1.478), Vec2d(163.3, -1.478)};
  const auto object_mgr = BuildMultiplePhantomVehicles(object_positions);
  DrawPlannerObjectManagerToCanvas(object_mgr, "LCIgnoreStalledObjects/objects",
                                   vis::Color::kLightGreen);
  // Create stalled objects.
  const absl::flat_hash_set<std::string> stalled_objects{"Phantom1",
                                                         "Phantom2"};

  // Drive Passage.
  const mapping::LanePath target_lane_path(
      smm, {mapping::ElementId(2471), mapping::ElementId(53)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  DrawLanePathToCanvas(psmm, target_lane_path,
                       "LCIgnoreStalledObjects/target_lane_path",
                       vis::Color::kMint);
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, RouteSections::BuildFromLanePath(psmm, target_lane_path),
          target_lane_path,
          /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  ASSIGN_OR_DIE(
      const auto drive_passage,
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr, target_lane_path,
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        target_lane_path.length(), target_lane_path.back(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt));
  SendDrivePassageToCanvas(drive_passage,
                           "LCIgnoreStalledObjects/drive_passage");

  const SpacetimeTrajectoryManager st_traj_mgr(object_mgr.planner_objects(),
                                               /*thread_pool=*/nullptr);

  SmoothedReferenceLineResultMap smooth_result_map;
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  ASSIGN_OR_DIE(
      auto ego_frenet_box,
      drive_passage.QueryFrenetBoxAt(ComputeAvBox(
          ego_pos, plan_start_point.path_point().theta(), vehicle_geom)));

  const mapping::LanePath prev_target_lane_path(smm, {mapping::ElementId(2470)},
                                                /*start_fraction=*/0.0,
                                                /*end_fraction=*/1.0);

  const double ref_center_l =
      CalcAvhRefCenterL(psmm, drive_passage, ego_frenet_box, smooth_result_map,
                        /*should_smooth=*/false);
  const auto lc_state = *MakeLaneChangeState(
      drive_passage, ego_pos, ego_frenet_box, prev_target_lane_path,
      prev_target_lane_path, MakeNoneLaneChangeState(), ref_center_l,
      AutonomyStateProto::AUTO_DRIVE);
  EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_EXECUTING);

  // Path Sl Boundary.
  ASSIGN_OR_DIE(const auto path_sl_boundary,
                BuildPathBoundaryFromPose(
                    psmm, drive_passage, plan_start_point, vehicle_geom,
                    st_traj_mgr, lc_state, smooth_result_map,
                    /*borrow_lane_boundary=*/false,
                    /*should_smooth=*/false, /*unsafe_object_ids=*/{}));
  DrawPathSlBoundaryToCanvas(path_sl_boundary,
                             "LCIgnoreStalledObjects/path_boundary");

  const double cur_ego_v = std::numeric_limits<double>::max();
  const auto leading_groups = FindMultipleLeadingGroups(
      drive_passage, path_sl_boundary, lc_state.lc_left(), st_traj_mgr,
      stalled_objects, plan_start_point.path_point().theta(), ego_frenet_box,
      vehicle_geom, cur_ego_v);

  EXPECT_EQ(leading_groups.size(), 2);
}

}  // namespace
}  // namespace qcraft::planner
