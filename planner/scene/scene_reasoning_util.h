
#ifndef ONBOARD_PLANNER_SCENE_SCENE_REASONING_UTIL_H_
#define ONBOARD_PLANNER_SCENE_SCENE_REASONING_UTIL_H_

#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {
// Parse object annotation reasoning result to planner debug proto. Using
// PlannerObjectManager to calculate object position, this debug info
// will used to show stall objects in vantage.
void ParseObjectAnnotationToDebugProto(
    const ::google::protobuf::RepeatedPtrField<ObjectAnnotationProto>&
        objects_annotation,
    const PlannerObjectManager& object_manager, PlannerDebugProto* debug);

// Parse traffic waiting queue reasoning result to planner debug prot. Using
// PlannerObjectManager to calculate object position, this debug info will used
// to show traffic waiting objects in vantage.
void ParseTrafficWaitingQueueToDebugProto(
    const ::google::protobuf::RepeatedPtrField<TrafficWaitingQueueProto>&
        traffic_waiting_queues,
    const PlannerObjectManager& object_manager, PlannerDebugProto* debug);
}  // namespace qcraft::planner

#endif
