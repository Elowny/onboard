#include "onboard/planner/decision/end_of_path_boundary.h"

#include <optional>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {
// Data generated from 'dojo.planner.end_of_drive_passage.pb.txt'.
TEST(BuildEndOfPathBoundaryConstraintsTest, BuildEndOfPathBoundaryConstraints) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(124.0);
  pose.mutable_pos_smooth()->set_y(63.0);
  const auto route_path =
      RoutingToNameSpot(*smm, cc, pose, /*name_spot=*/"8c_s2");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  SendRouteLanePathToCanvas(
      psmm, route_path, "test/route_build_end_of_path_boundary_constraints");

  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            route_path.lane_paths().front(),
                                            /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                                  route_path.lane_paths().front(),
                                  *backward_extended_lane_path,
                                  /*anchor_point=*/mapping::LanePoint(),
                                  route_sections.planning_horizon(psmm),
                                  route_sections.destination(),
                                  /*all_lanes_virtual=*/false,
                                  /*override_speed_limit_mps=*/std::nullopt),
                "Building drive passage failed!");
  SendDrivePassageToCanvas(
      drive_passage,
      "test/drive_passage_build_end_of_path_boundary_constraints");

  const auto vehicle_geom = DefaultVehicleGeometry();
  const SpacetimeTrajectoryManager st_traj_mgr;
  LaneChangeStateProto lc_state;
  SmoothedReferenceLineResultMap smooth_result_map;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);

  ASSIGN_OR_DIE(const auto path_bound_lk,
                BuildPathBoundaryFromPose(
                    psmm, drive_passage, ConvertToTrajPointProto(pose),
                    vehicle_geom, st_traj_mgr, lc_state, smooth_result_map,
                    /*borrow_lane_boundary=*/false,
                    /*should_smooth=*/false, /*unsafe_object_ids=*/{}));

  const auto end_of_path_boundary =
      BuildEndOfPathBoundaryConstraint(drive_passage, path_bound_lk);
  EXPECT_TRUE(end_of_path_boundary.ok());
  EXPECT_NEAR(end_of_path_boundary.value().s(),
              drive_passage.stations().back().accumulated_s(), 0.01);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
