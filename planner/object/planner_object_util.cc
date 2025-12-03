#include "onboard/planner/object/planner_object_util.h"

#include <algorithm>  // for max
#include <string>     // for basic_string, string

#include "absl/container/flat_hash_set.h"  // for flat_hash_set, operator!=
#include "google/protobuf/repeated_ptr_field.h"  // for RepeatedPtrField

#include "onboard/global/trace.h"  // for SCOPED_QTRACE, ScopedTrace
#include "onboard/planner/object/object_vector.h"   // for ObjectVector
#include "onboard/planner/object/planner_object.h"  // for PlannerObject
#include "onboard/planner/object/spacetime_object_trajectory.h"  // for SpacetimeObjectTrajectory
#include "onboard/prediction/object_prediction.h"  // for ObjectPrediction

namespace qcraft {
namespace planner {

std::vector<PartialSpacetimeObjectTrajectory>
CollectSpeedConsiderObjectsPartialStTrajectory(
    const std::vector<EstPlannerOutput>& est_outputs,
    const std::vector<PartialSpacetimeObjectTrajectory>&
        fallback_considered_st_objects) {
  std::vector<PartialSpacetimeObjectTrajectory> all_part_st_traj;
  for (int i = 0; i < est_outputs.size(); ++i) {
    for (const auto& part_st_traj : est_outputs[i].considered_st_objects) {
      all_part_st_traj.push_back(part_st_traj);
    }
  }
  for (const auto& part_st_traj : fallback_considered_st_objects) {
    all_part_st_traj.push_back(part_st_traj);
  }
  return all_part_st_traj;
}

ObjectsPredictionProto CollectSpeedConsideredObjectsPrediction(
    const PlannerObjectManager& obj_mgr,
    const std::vector<PartialSpacetimeObjectTrajectory>& considered_st_objects,
    bool planner_export_all_prediction_to_speed_considered) {
  SCOPED_QTRACE("CollectSpeedConsideredObjectsPrediction");
  // Collect speed-considered object ids by all est and fallback branches.
  absl::flat_hash_set<std::string> speed_considered_object_ids;
  if (planner_export_all_prediction_to_speed_considered) {
    for (const auto& planner_object : obj_mgr.planner_objects()) {
      speed_considered_object_ids.insert(planner_object.id());
    }
  } else {
    for (const auto& part_st_traj : considered_st_objects) {
      speed_considered_object_ids.insert(
          std::string(part_st_traj.st_traj().object_id()));
    }
  }

  ObjectsPredictionProto speed_considered_objects_prediction;
  speed_considered_objects_prediction.mutable_objects()->Reserve(
      speed_considered_object_ids.size());
  for (const auto& object_id : speed_considered_object_ids) {
    const auto* object_ptr = obj_mgr.FindObjectById(object_id);
    if (object_ptr == nullptr) continue;
    object_ptr->prediction().ToProto(
        speed_considered_objects_prediction.add_objects());
  }

  return speed_considered_objects_prediction;
}

}  // namespace planner
}  // namespace qcraft
