#include "onboard/planner/scene/occluded_interaction_lane_reasoning.h"

#include <memory>

#include "onboard/planner/test_util/load_psmm_util.h"
// #include <string>

#include "absl/time/clock.h"
#include "absl/time/time.h"
// #include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/common/vehicle_pose/vehicle_pose.h"
#include "onboard/maps/map_selector.h"
// #include "onboard/math/geometry/proto/box2d.pb.h"
#include "onboard/math/vec.h"
#include "onboard/perception/lidar_pipeline/sensor_fov/sensor_fov_builder.h"
#include "onboard/perception/lidar_pipeline/sensor_fov/sensor_fov_test_util.h"
#include "onboard/perception/lidar_pipeline/test_util/obstacle_builder.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
// #include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/time_util.h"
// #include "onboard/vis/canvas/canvas.h"
// #include "onboard/vis/common/color.h"

namespace qcraft::planner {
namespace {

// void DrawSensorFovToCanvas(const sensor_fov::SensorFov& sensor_fov,
//                            const std::string& channel) {
//   auto& canvas = vantage_client_man::GetCanvas(channel);
//   canvas.SetGroundZero(1);

//   const auto blind_zones =
//       sensor_fov.ComputeAllBlindZones(sensor_fov::kDefaultOcclusionHeight);
//   for (const auto& blind_zone : blind_zones) {
//     canvas.DrawPolygon(blind_zone, /*z=*/0.0, vis::Color::kWhite);
//   }
// }

// void DrawInferredObjectsToCanvas(
//     absl::Span<const InferredObjectProto> inferred_objects,
//     const std::string& channel) {
//   auto& canvas = vantage_client_man::GetCanvas(channel);
//   canvas.SetGroundZero(1);

//   for (const auto& inferred_object : inferred_objects) {
//     const auto& box = inferred_object.object_info().bounding_box();
//     canvas.DrawBox(Vec3d(box.x(), box.y(), 0.0), box.heading(),
//                    Vec2d(box.length(), box.width()), vis::Color::kLightGreen,
//                    1);
//   }
// }

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
  // DrawSensorFovToCanvas(*sensor_fov, "sensor_fov");

  const auto result =
      InferOccludedObjectsOnInteractionLanes(psmm, route_sections, *sensor_fov);
  // DrawInferredObjectsToCanvas(result, "objects");

  EXPECT_FALSE(result.empty());
  for (const auto& inferred_object : result) {
    EXPECT_TRUE(inferred_object.has_object_info());
    EXPECT_TRUE(inferred_object.has_infer_source());
    EXPECT_TRUE(inferred_object.has_confidence());
    EXPECT_TRUE(inferred_object.infer_source().has_interaction_lane());
    EXPECT_TRUE(inferred_object.infer_source()
                    .interaction_lane()
                    .has_interaction_point());
    EXPECT_TRUE(inferred_object.infer_source()
                    .interaction_lane()
                    .has_dist_to_interaction());
  }
}
}  // namespace
}  // namespace qcraft::planner
