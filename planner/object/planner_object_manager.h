#ifndef ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_MANAGER_H_
#define ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_MANAGER_H_

#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"

#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {

// A class that manages all planner objects.
class PlannerObjectManager {
 public:
  // Added because of `Status` requires T has a default constructor.
  PlannerObjectManager() = default;

  explicit PlannerObjectManager(ObjectVector<PlannerObject> planner_objects);

  const ObjectVector<PlannerObject>& planner_objects() const {
    return planner_objects_;
  }

  absl::Span<const PlannerObject* const> stationary_objects() const {
    return stationary_objects_;
  }

  const PlannerObject* FindObjectById(const absl::string_view id) const {
    const auto* found_ptr = FindOrNull(object_map_, id);
    if (found_ptr == nullptr) return nullptr;
    return *found_ptr;
  }

  absl::Span<const PlannerObject* const> moving_objects() const {
    return moving_objects_;
  }

  const PlannerObject& planner_object(const ObjectIndex index) const {
    return planner_objects_[index];
  }

  int size() const { return planner_objects_.size(); }

 private:
  ObjectVector<PlannerObject> planner_objects_;
  std::vector<const PlannerObject*> stationary_objects_;
  std::vector<const PlannerObject*> moving_objects_;
  absl::flat_hash_map<std::string_view, const PlannerObject*> object_map_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_MANAGER_H_
