#ifndef ONBOARD_PLANNER_SCENE_BUS_STATION_STALLED_OBJECTS_FILTER_H_
#define ONBOARD_PLANNER_SCENE_BUS_STATION_STALLED_OBJECTS_FILTER_H_

#include <vector>

#include "onboard/maps/lane_path.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {
class BusStationStalledObjectsFilter {
 public:
  BusStationStalledObjectsFilter(const PlannerSemanticMapManager& psmm,
                                 const RouteSections& route_sections);
  bool IsFiltered(const PlannerSemanticMapManager& psmm, const Vec2d& obj_pos,
                  ObjectType obj_type);

 private:
  std::vector<mapping::LanePath> station_lane_paths_;
};
}  // namespace planner
}  // namespace qcraft
#endif
