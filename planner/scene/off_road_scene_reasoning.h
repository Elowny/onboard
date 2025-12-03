
#ifndef ONBOARD_PLANNER_SCENE_OFFROAD_SCENE_REASONING_H_
#define ONBOARD_PLANNER_SCENE_OFFROAD_SCENE_REASONING_H_

#include "absl/status/statusor.h"

#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
namespace qcraft::planner {

struct OffRoadSceneReasoningInput {
  const PlannerObjectManager* object_mgr = nullptr;
};

// Scene reasoning on unstructured road.
absl::StatusOr<SceneOutputProto> RunOffRoadSceneReasoning(
    const OffRoadSceneReasoningInput& input);

}  // namespace qcraft::planner
#endif
