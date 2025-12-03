#include "onboard/planner/router/road_constraints.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "onboard/container/strong_int.h"

namespace qcraft::planner::route {
void AvoidLaneSegmentMap::ToProto(AvoidLaneSegmentMapProto* proto) const {
  if (!proto) return;
  proto->Clear();
  auto* mutable_avoid_lane_segments = proto->mutable_avoid_lane_segment();
  mutable_avoid_lane_segments->Reserve(lane_segments_map.size());
  for (const auto& [lane_id, lane_seg] : lane_segments_map) {
    auto* avoid_lane_seg = mutable_avoid_lane_segments->Add();
    avoid_lane_seg->set_id(lane_id.value());
    auto* mutable_seg = avoid_lane_seg->mutable_segments()->Add();
    mutable_seg->set_start_fraction(lane_seg.start_fraction);
    mutable_seg->set_end_fraction(lane_seg.end_fraction);
  }
}

void AvoidLaneSegmentMap::FromProto(const AvoidLaneSegmentMapProto& proto) {
  lane_segments_map.clear();
  lane_segments_map.reserve(proto.avoid_lane_segment().size());
  for (const auto& lane_seg_proto : proto.avoid_lane_segment()) {
    auto& lane_segment =
        lane_segments_map[mapping::ElementId(lane_seg_proto.id())];
    lane_segment.start_fraction = lane_seg_proto.segments(0).start_fraction();
    lane_segment.end_fraction = lane_seg_proto.segments(0).end_fraction();
  }
}
}  // namespace qcraft::planner::route
