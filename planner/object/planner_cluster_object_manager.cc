#include "onboard/planner/object/planner_cluster_object_manager.h"

#include <algorithm>
#include <ostream>
#include <utility>
#include <vector>

#include "onboard/lite/logging.h"

namespace qcraft {
namespace planner {
namespace {

template <typename T>
void UpdateObjectMap(
    const std::vector<T>& objects,
    absl::flat_hash_map<PlannerClusterObject::Id, const T*>* objects_map) {
  objects_map->reserve(objects.size());
  for (const auto& object : objects) {
    (*objects_map)[object.id()] = &object;
  }
}

}  // namespace

PlannerClusterObjectManager::PlannerClusterObjectManager(
    std::vector<BoundaryClusterProto> objects) {
  for (auto& obj : objects) {
    // TODO(renjie): Remove the contiue logic when perception fix the empty
    // polygon cluster output problem.
    if (obj.points_size() == 0) {
      continue;
    }
    switch (obj.geometry_type()) {
      case BoundaryClusterProto::GT_SCATTER:
        scatter_objects_.emplace_back(std::move(obj));
        break;
      case BoundaryClusterProto::GT_POLYLINE:
        polyline_objects_.emplace_back(std::move(obj));
        break;
      case BoundaryClusterProto::GT_POLYGON:
        polygon_objects_.emplace_back(std::move(obj));
        break;
      case BoundaryClusterProto::GT_UNKNOWN:
        QLOG(FATAL) << "Unexpected GT_UNKNOWN geomtry type for cluster object "
                    << obj.id();
        break;
    }
  }

  UpdateObjectMap(scatter_objects_, &scatter_objects_map_);
  UpdateObjectMap(polyline_objects_, &polyline_objects_map_);
  UpdateObjectMap(polygon_objects_, &polygon_objects_map_);
}

}  // namespace planner
}  // namespace qcraft
