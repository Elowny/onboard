#include "onboard/planner/decision/end_of_current_lane_path.h"

#include <optional>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace planner {
namespace {
// Data generated from 'dojo.planner.end_of_drive_passage.pb.txt'.
TEST(BuildEndOfCurrentLanePathConstraintsTest,
     BuildEndOfCurrentLanePathConstraints) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(124.0);
  pose.mutable_pos_smooth()->set_y(63.0);
  const auto route_path =
      RoutingToNameSpot(*smm, cc, pose, /*name_spot=*/"end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);

  SendRouteLanePathToCanvas(
      psmm, route_path,
      "test/route_build_end_of_current_lane_path_constraints");

  const auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";
  SendDrivePassageToCanvas(
      drive_passage.value(),
      "test/drive_passage_build_end_of_current_lane_path_constraints");

  const auto& passage = drive_passage.value();
  const auto end_of_current_lane_path =
      BuildEndOfCurrentLanePathConstraint(passage);
  EXPECT_TRUE(end_of_current_lane_path.ok());
  EXPECT_NEAR(end_of_current_lane_path.value().s(), 61.78, 0.1);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
