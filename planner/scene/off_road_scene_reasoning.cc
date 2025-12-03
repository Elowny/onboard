#include "onboard/planner/scene/off_road_scene_reasoning.h"

#include <utility>
#include <vector>

#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/planner/scene/objects_reasoning.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

absl::StatusOr<SceneOutputProto> RunOffRoadSceneReasoning(
    const OffRoadSceneReasoningInput& input) {
  SCOPED_QTRACE("OffRoadSceneReasoning");
  SceneOutputProto output;
  QCHECK_NOTNULL(input.object_mgr);

  const auto& object_mgr = *input.object_mgr;

  ASSIGN_OR_RETURN(auto objects_reasoning_output,
                   RunObjectsReasoning(object_mgr));

  output.mutable_objects_annotation()->Reserve(
      objects_reasoning_output.objects_annotation.size());
  for (auto& object_annotation : objects_reasoning_output.objects_annotation) {
    *output.add_objects_annotation() = std::move(object_annotation);
  }

  return output;
}
}  // namespace qcraft::planner
