
#ifndef ONBOARD_PLANNER_SCENE_OBJECTS_REASONING_H_
#define ONBOARD_PLANNER_SCENE_OBJECTS_REASONING_H_
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
namespace qcraft::planner {

struct ObjectsReasoningOutput {
  std::vector<ObjectAnnotationProto> objects_annotation;
};

// Reasoning object annotation without structured road info(e.g. lane path).
absl::StatusOr<ObjectsReasoningOutput> RunObjectsReasoning(
    const PlannerObjectManager& object_mgr);
}  // namespace qcraft::planner

#endif
