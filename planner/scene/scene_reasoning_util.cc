#include "onboard/planner/scene/scene_reasoning_util.h"

#include <string>
#include <utility>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/object/planner_object.h"

namespace qcraft::planner {

void ParseObjectAnnotationToDebugProto(
    const ::google::protobuf::RepeatedPtrField<ObjectAnnotationProto>&
        objects_annotation,
    const PlannerObjectManager& object_manager,
    PlannerDebugProto* debug_proto) {
  for (const auto& object_annotation : objects_annotation) {
    constexpr double kStalledProbThreshold = 0.6;
    if (object_annotation.stalled_vehicle_likelyhood() <
        kStalledProbThreshold) {
      continue;
    }
    const auto& object_id = object_annotation.object_id();

    const auto* object_ptr = object_manager.FindObjectById(object_id);
    if (object_ptr == nullptr) continue;
    const auto& object_box = object_ptr->bounding_box();

    PlannerDebugProto::ObjectInfo object;
    object.set_id(object_id);
    Vec2dToProto(object_box.center(), object.mutable_pos());
    object.set_heading(object_box.heading());

    *debug_proto->add_stalled_objects() = std::move(object);
  }
}

void ParseTrafficWaitingQueueToDebugProto(
    const ::google::protobuf::RepeatedPtrField<TrafficWaitingQueueProto>&
        traffic_waiting_queues,
    const PlannerObjectManager& object_manager,
    PlannerDebugProto* debug_proto) {
  for (const auto& traffic_waiting_queue : traffic_waiting_queues) {
    for (const auto& object_id : traffic_waiting_queue.object_id()) {
      const auto* object_ptr = object_manager.FindObjectById(object_id);
      if (object_ptr == nullptr) continue;
      const auto& object_box = object_ptr->bounding_box();
      PlannerDebugProto::ObjectInfo object;
      object.set_id(object_id);
      Vec2dToProto(object_box.center(), object.mutable_pos());
      object.set_heading(object_box.heading());

      *debug_proto->add_traffic_waiting_objects() = std::move(object);
    }
  }
}

}  // namespace qcraft::planner
