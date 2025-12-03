#include "onboard/planner/decision/no_block.h"

#include <optional>

#include "absl/status/statusor.h"

#include "gtest/gtest.h"

#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/composite_lane_path.h"
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
// Data generated from 'dojo.planner.no_block_1.pb.txt'.
TEST(BuildNoBlockConstraintsTest, BuildNoBlockConstraints) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(53.7);
  pose.mutable_pos_smooth()->set_y(66.3);
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

  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";

  const auto& passage = drive_passage.value();

  SendDrivePassageToCanvas(passage, "test/drive_passage");

  const auto speed_regions = BuildNoBlockConstraints(
      psmm, passage, passage.lane_path(), /*s_offset=*/0.0);
  EXPECT_EQ(speed_regions.size(), 1);
  for (const auto& speed_region : speed_regions) {
    EXPECT_LT(speed_region.min_speed(), speed_region.max_speed());
    EXPECT_GE(speed_region.end_s() - speed_region.start_s(), 0.1);
  }
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
