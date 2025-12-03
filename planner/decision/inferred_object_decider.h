#ifndef ONBOARD_PLANNER_DECISION_INFERRED_OBJECT_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_INFERRED_OBJECT_DECIDER_H_

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<ConstraintProto::SpeedProfileProto>
BuildInferredObjectConstraint(const PlannerSemanticMapManager& psmm,
                              const SceneOutputProto& scene_reasoning,
                              const mapping::LanePath& lane_path_from_start,
                              double ego_init_v);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_INFERRED_OBJECT_DECIDER_H_
