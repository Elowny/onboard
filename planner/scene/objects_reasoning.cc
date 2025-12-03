#include "onboard/planner/scene/objects_reasoning.h"

#include <algorithm>
#include <utility>

#include "Eigen/Core"
#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"

#include "onboard/container/strong_int.h"
#include "onboard/math/util.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::planner {

absl::StatusOr<ObjectsReasoningOutput> RunObjectsReasoning(
    const PlannerObjectManager& object_mgr) {
  const auto& objects = object_mgr.planner_objects();
  // flat_hash_map: object index, stall_probability.
  absl::flat_hash_map<ObjectIndex, double> stall_analysis;
  for (const auto index : objects.index_range()) {
    const auto& object = objects[index];
    const auto object_type = object.type();

    if (!object.is_stationary()) continue;

    if (object_type == ObjectType::OT_UNKNOWN_STATIC ||
        object_type == ObjectType::OT_BARRIER ||
        object_type == ObjectType::OT_CONE ||
        object_type == ObjectType::OT_WARNING_TRIANGLE) {
      stall_analysis[index] = 1.0;
      continue;
    }

    constexpr double kStallSpeed = 0.01;  // m/s.
    if (object_type == ObjectType::OT_VEHICLE ||
        object_type == ObjectType::OT_MOTORCYCLIST ||
        object_type == ObjectType::OT_CYCLIST ||
        object_type == ObjectType::OT_TRICYCLIST ||
        object_type == ObjectType::OT_LARGE_VEHICLE) {
      if (object.velocity().squaredNorm() < Sqr(kStallSpeed)) {
        stall_analysis[index] = 0.5;
        continue;
      }
    }

    if (object.object_proto().has_parked() && object.object_proto().parked()) {
      stall_analysis[index] = 1.0;
      continue;
    }
  }

  std::vector<ObjectAnnotationProto> objects_annotation;
  objects_annotation.reserve(stall_analysis.size());
  for (const auto& [index, stall_probability] : stall_analysis) {
    ObjectAnnotationProto object_annotation;
    object_annotation.set_object_id(objects[index].id());
    object_annotation.set_stalled_vehicle_likelyhood(stall_probability);
    object_annotation.set_depart_soon_likelyhood(1.0 - stall_probability);
    objects_annotation.emplace_back(std::move(object_annotation));
  }

  return ObjectsReasoningOutput{.objects_annotation =
                                    std::move(objects_annotation)};
}
}  // namespace qcraft::planner
