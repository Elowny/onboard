#ifndef ONBOARD_PLANNER_SCENE_OCCLUDED_OBJECTS_REASONING_H_
#define ONBOARD_PLANNER_SCENE_OCCLUDED_OBJECTS_REASONING_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/perception/lidar_pipeline/sensor_fov/sensor_fov.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"

namespace qcraft::planner {

struct OccludedObjectsReasoningInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  // Sensor fov is a necessary condition to reason occluded objects.
  const sensor_fov::SensorFov* sensor_fov = nullptr;
  // Collect crosswalks along route sections.
  const RouteSections* route_sections = nullptr;
};

absl::StatusOr<std::vector<InferredObjectProto>> RunOccludedObjectsReasoning(
    const OccludedObjectsReasoningInput& input);

}  // namespace qcraft::planner
#endif
