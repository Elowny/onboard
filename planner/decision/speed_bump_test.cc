#include "onboard/planner/decision/speed_bump.h"

#include <optional>

#include "absl/status/statusor.h"

#include "gtest/gtest.h"

#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace planner {
namespace {
// Data generated from `dojo.planner.speed_bump.pb.txt`.
TEST(BuildSpeedBumpConstraintsTest, BuildSpeedBumpConstraints) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(-10.5);
  pose.mutable_pos_smooth()->set_y(21.4);
  pose.set_yaw(1.55);
  const auto route_path = RoutingToNameSpot(*smm, cc, pose,
                                            /*name_spot=*/"a7_e2_start");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";

  const auto speed_bumps = BuildSpeedBumpConstraints(psmm, *drive_passage);
  EXPECT_EQ(speed_bumps.size(), 2);
  EXPECT_GT(speed_bumps[0].end_s() - speed_bumps[0].start_s(), 0.1);
  EXPECT_GT(speed_bumps[1].end_s() - speed_bumps[1].start_s(), 0.1);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
