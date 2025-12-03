#include "onboard/planner/decision/inferred_object_decider.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/common/vehicle_pose/vehicle_pose.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/vec.h"
#include "onboard/perception/lidar_pipeline/sensor_fov/sensor_fov.h"
#include "onboard/perception/lidar_pipeline/sensor_fov/sensor_fov_builder.h"
#include "onboard/perception/lidar_pipeline/sensor_fov/sensor_fov_test_util.h"
#include "onboard/perception/lidar_pipeline/test_util/obstacle_builder.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scene/occluded_interaction_lane_reasoning.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {

TEST(OccludedObjectsReasoningTest, BaseTest) {
  // Build planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Build route sections.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(
      ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const VehiclePose vehicle_pose(sdc_pose);

  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);

  // Build sensor fov.
  auto sensor_fov_builder = sensor_fov::BuildSensorFovBuilder();
  const auto& [clusters, obstacle_refs] =
      sensor_fov::BuildSegmentedClusters(vehicle_pose);
  const auto obstacle_ptrs =
      ConstructObstaclePtrsFromObstacleRefVector(obstacle_refs);
  sensor_fov_builder->Compute(obstacle_ptrs, {0.1, vehicle_pose});
  const auto sensor_fovs =
      sensor_fov_builder->Compute(obstacle_ptrs, {3.0, vehicle_pose});
  const auto sensor_fov = GetLidarViewSensorFov(sensor_fovs);

  auto inferred_objects =
      InferOccludedObjectsOnInteractionLanes(psmm, route_sections, *sensor_fov);
  SceneOutputProto reasoning_output;
  reasoning_output.mutable_inferred_objects()->Reserve(inferred_objects.size());
  for (auto& inferred_object : inferred_objects) {
    *reasoning_output.add_inferred_objects() = std::move(inferred_object);
  }

  const auto speed_profile_or = BuildInferredObjectConstraint(
      psmm, reasoning_output, route_path.lane_paths().front(),
      /*ego_init_v=*/11.0);
  EXPECT_OK(speed_profile_or);
  EXPECT_EQ(speed_profile_or->source().occluded_object().id(), "occ-lane-40-1");
}
}  // namespace
}  // namespace qcraft::planner
