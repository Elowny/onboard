#ifndef ONBOARD_PLANNER_ROUTER_ROAD_CONSTRAINTS_H_
#define ONBOARD_PLANNER_ROUTER_ROAD_CONSTRAINTS_H_

#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {

struct AvoidLaneSegmentMap {
  struct LaneSegment {
    double start_fraction = 0.0;
    double end_fraction = 1.0;
  };
  absl::flat_hash_map<mapping::ElementId, LaneSegment> lane_segments_map;

  bool IsEmpty() const { return lane_segments_map.empty(); }

  bool ContainsLane(mapping::ElementId lane_id) const {
    return lane_segments_map.contains(lane_id);
  }

  bool ContainsLaneFrac(mapping::ElementId lane_id, double fraction) const {
    const auto iter = lane_segments_map.find(lane_id);
    if (iter == lane_segments_map.end()) return false;
    return fraction >= iter->second.start_fraction &&
           fraction <= iter->second.end_fraction;
  }

  void ToProto(AvoidLaneSegmentMapProto* proto) const;
  void FromProto(const AvoidLaneSegmentMapProto& proto);
};

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ROAD_CONSTRAINTS_H_
