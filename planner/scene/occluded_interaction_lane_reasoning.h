#ifndef ONBOARD_PLANNER_SCENE_OCCLUDED_INTERACTION_LANE_REASONING_H_
#define ONBOARD_PLANNER_SCENE_OCCLUDED_INTERACTION_LANE_REASONING_H_

#include <vector>

#include "onboard/perception/lidar_pipeline/sensor_fov/sensor_fov.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"

namespace qcraft::planner {

std::vector<InferredObjectProto> InferOccludedObjectsOnInteractionLanes(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const sensor_fov::SensorFov& sensor_fov);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCENE_OCCLUDED_INTERACTION_LANE_REASONING_H_
