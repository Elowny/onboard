#include "onboard/planner/scene/occluded_objects_reasoning.h"

#include <memory>

#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "gtest/gtest.h"

#include "onboard/common/vehicle_pose/vehicle_pose.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/run_context.h"
#include "onboard/math/vec.h"
#include "onboard/perception/lidar_pipeline/sensor_fov/sensor_fov_builder.h"
#include "onboard/perception/lidar_pipeline/sensor_fov/sensor_fov_test_util.h"
#include "onboard/perception/lidar_pipeline/test_util/obstacle_builder.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
TEST(OccludedObjectsReasoningTest, BaseTest) {
  FLAGS_enable_context_test = 1;
  FLAGS_planner_enable_crosswalk_occluded_objects_inference = true;

  // Build planner semantic map.
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Build route sections.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(ToUnixDoubleSeconds(plan_time),
                                        Vec2d(0.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);

  // Build sensor fov.
  auto sensor_fov_builder = sensor_fov::BuildSensorFovBuilder();
  const auto& [clusters, obstacle_refs] = sensor_fov::BuildSegmentedClusters();
  const auto obstacle_ptrs =
      ConstructObstaclePtrsFromObstacleRefVector(obstacle_refs);
  const auto sensor_fovs =
      sensor_fov_builder->Compute(obstacle_ptrs, {0.0, VehiclePose()});
  const auto sensor_fov_lidar = GetLidarViewSensorFov(sensor_fovs);

  ASSIGN_OR_DIE(const auto result,
                RunOccludedObjectsReasoning(OccludedObjectsReasoningInput{
                    .psmm = &psmm,
                    .sensor_fov = sensor_fov_lidar.get(),
                    .route_sections = &route_sections}));
}
}  // namespace
}  // namespace qcraft::planner
