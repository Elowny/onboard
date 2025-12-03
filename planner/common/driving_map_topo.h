#ifndef ONBOARD_PLANNER_COMMON_DRIVING_MAP_TOPO_H_
#define ONBOARD_PLANNER_COMMON_DRIVING_MAP_TOPO_H_

#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"

#include "onboard/maps/semantic_map_defs.h"

namespace qcraft::planner {

class DrivingMapTopo {
 public:
  struct Lane {
    mapping::ElementId id;
    double start_fraction;
    double end_fraction;

    // NOTE: outgoing and incoming lanes must exist in driving map.
    std::vector<mapping::ElementId> outgoing_lane_ids;
    std::vector<mapping::ElementId> incoming_lane_ids;
  };

  explicit DrivingMapTopo(std::vector<Lane> lanes,
                          std::vector<mapping::ElementId> starting_lane_ids);

  absl::Span<const Lane> lanes() const { return lanes_; }
  absl::Span<const mapping::ElementId> starting_lane_ids() const {
    return starting_lane_ids_;
  }
  const DrivingMapTopo::Lane* GetLaneById(mapping::ElementId id) const;

 private:
  std::vector<Lane> lanes_;
  std::vector<mapping::ElementId> starting_lane_ids_;
  absl::flat_hash_map<mapping::ElementId, int> id_index_map_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_DRIVING_MAP_TOPO_H_
