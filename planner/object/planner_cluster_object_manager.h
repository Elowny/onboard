#ifndef ONBOARD_PLANNER_OBJECT_PLANNER_CLUSTER_OBJECT_MANAGER_H_
#define ONBOARD_PLANNER_OBJECT_PLANNER_CLUSTER_OBJECT_MANAGER_H_

#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"

#include "onboard/planner/object/planner_cluster_object.h"
#include "onboard/proto/perception/parking/parking_freespace.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {

class PlannerClusterObjectManager {
 public:
  PlannerClusterObjectManager() = default;

  explicit PlannerClusterObjectManager(
      std::vector<BoundaryClusterProto> objects);

  const std::vector<PlannerScatterClusterObject>& scatter_objects() const {
    return scatter_objects_;
  }
  const PlannerScatterClusterObject* FindScatterObjectById(
      PlannerClusterObject::Id id) const {
    const auto* found_ptr = FindOrNull(scatter_objects_map_, id);
    if (found_ptr == nullptr) return nullptr;
    return *found_ptr;
  }

  const std::vector<PlannerPolylineClusterObject>& polyline_objects() const {
    return polyline_objects_;
  }
  const PlannerPolylineClusterObject* FindPolylineObjectById(
      PlannerClusterObject::Id id) const {
    const auto* found_ptr = FindOrNull(polyline_objects_map_, id);
    if (found_ptr == nullptr) return nullptr;
    return *found_ptr;
  }

  const std::vector<PlannerPolygonClusterObject>& polygon_objects() const {
    return polygon_objects_;
  }
  const PlannerPolygonClusterObject* FindPolygonObjectById(
      PlannerClusterObject::Id id) const {
    const auto* found_ptr = FindOrNull(polygon_objects_map_, id);
    if (found_ptr == nullptr) return nullptr;
    return *found_ptr;
  }

 private:
  std::vector<PlannerScatterClusterObject> scatter_objects_;
  absl::flat_hash_map<PlannerClusterObject::Id,
                      const PlannerScatterClusterObject*>
      scatter_objects_map_;

  std::vector<PlannerPolylineClusterObject> polyline_objects_;
  absl::flat_hash_map<PlannerClusterObject::Id,
                      const PlannerPolylineClusterObject*>
      polyline_objects_map_;

  std::vector<PlannerPolygonClusterObject> polygon_objects_;
  absl::flat_hash_map<PlannerClusterObject::Id,
                      const PlannerPolygonClusterObject*>
      polygon_objects_map_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_PLANNER_CLUSTER_OBJECT_MANAGER_H_
