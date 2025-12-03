#include "onboard/planner/router/road_constraints.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"

namespace qcraft::planner::route {
namespace {
TEST(RoadConstraintsTest, AvoidLaneSegmentMapTest) {
  AvoidLaneSegmentMap avoid_lane_map;
  auto& lane_segments_map = avoid_lane_map.lane_segments_map;
  lane_segments_map.insert(
      {mapping::ElementId(1), AvoidLaneSegmentMap::LaneSegment{
                                  .start_fraction = 0.2, .end_fraction = 0.9}});
  lane_segments_map.insert(
      {mapping::ElementId(2), AvoidLaneSegmentMap::LaneSegment()});
  EXPECT_FALSE(avoid_lane_map.ContainsLane(mapping::ElementId(3)));
  EXPECT_TRUE(avoid_lane_map.ContainsLane(mapping::ElementId(2)));
  EXPECT_TRUE(
      avoid_lane_map.ContainsLaneFrac(mapping::ElementId(1), /*fraction=*/0.5));
  EXPECT_FALSE(avoid_lane_map.ContainsLaneFrac(mapping::ElementId(1),
                                               /*fraction=*/0.99));

  AvoidLaneSegmentMapProto proto;
  avoid_lane_map.ToProto(&proto);
  EXPECT_EQ(proto.avoid_lane_segment_size(), 2);
  AvoidLaneSegmentMap new_map;
  new_map.FromProto(proto);
  EXPECT_EQ(new_map.lane_segments_map.size(), 2);
  EXPECT_DOUBLE_EQ(
      new_map.lane_segments_map[mapping::ElementId(1)].start_fraction, 0.2);
  EXPECT_DOUBLE_EQ(
      new_map.lane_segments_map[mapping::ElementId(1)].end_fraction, 0.9);
}
}  // namespace
}  // namespace qcraft::planner::route
