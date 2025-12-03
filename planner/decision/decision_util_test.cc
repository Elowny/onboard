#include "onboard/planner/decision/decision_util.h"

#include <algorithm>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "gtest/gtest.h"

#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace planner {
TEST(CreateSpeedProfileTest, CreateSpeedProfile) {
  // Load map
  SetMap("dojo");

  auto start_time = absl::Now();

  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  start_time = absl::Now();
  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "7a_n2");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);

  start_time = absl::Now();
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

  const double v_now = 5.0;  // m/s
  std::vector<ConstraintProto::SpeedRegionProto> speed_zone_vector;
  auto& speed_zone = speed_zone_vector.emplace_back();
  speed_zone.set_start_s(20.0);
  speed_zone.set_end_s(25.0);
  speed_zone.set_max_speed(10.0);

  std::vector<ConstraintProto::StopLineProto> stop_point_vector;
  auto& stop_point = stop_point_vector.emplace_back();
  stop_point.set_s(60.0);

  absl::Span<ConstraintProto::SpeedRegionProto> speed_zones =
      absl::MakeSpan(speed_zone_vector);
  absl::Span<ConstraintProto::StopLineProto> stop_points =
      absl::MakeSpan(stop_point_vector);

  const SpeedProfile speed_profile =
      CreateSpeedProfile(v_now, passage, speed_zones, stop_points);
}

}  // namespace planner
}  // namespace qcraft
