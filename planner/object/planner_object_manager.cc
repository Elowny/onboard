#include "onboard/planner/object/planner_object_manager.h"

#include <algorithm>
#include <utility>

namespace qcraft {
namespace planner {

PlannerObjectManager::PlannerObjectManager(
    ObjectVector<PlannerObject> planner_objects)
    : planner_objects_(std::move(planner_objects)) {
  stationary_objects_.reserve(planner_objects_.size());
  moving_objects_.reserve(planner_objects_.size());
  object_map_.reserve(planner_objects_.size());
  for (auto& object : planner_objects_) {
    if (object.is_stationary()) {
      stationary_objects_.push_back(&object);
    } else {
      moving_objects_.push_back(&object);
    }
    object_map_[object.id()] = &object;
  }
}

}  // namespace planner
}  // namespace qcraft
