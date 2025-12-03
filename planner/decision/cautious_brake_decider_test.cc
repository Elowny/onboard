#include "onboard/planner/decision/cautious_brake_decider.h"

#include <optional>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/object/planner_object.h"
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
TEST(BuildCautiousBrakeDeciderConstraintsTest,
     BuildCautiousBrakeDeciderConstraints) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  std::vector<PlannerObject> planner_objects;
  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(53.7);
  pose.mutable_pos_smooth()->set_y(66.3);
  const auto route_path =
      RoutingToNameSpot(*smm, cc, pose, /*name_spot=*/"a9_e2_end");
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

  const auto speed_regions =
      BuildCautiousBrakeConstraints(psmm, passage, passage.lane_path(),
                                    /*s_offset=*/0.0, st_traj_mgr);
  EXPECT_GT(speed_regions.size(), 1);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
