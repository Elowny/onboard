#include "onboard/planner/router/map_match.h"

// IWYU pragma: no_include <algorithm>

#include "gtest/gtest.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/smm_proto_util.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/route_test_util.h"
namespace qcraft::planner::route::map_match {

namespace {

TEST(MapMatchTest, ExcludeOngoingLanesTest) {
  // "3" -> "951","1"
  const auto& smm = LoadDojoMap();
  std::vector<mapping::PointToLane> pt_lanes = {
      {
          .lane_proto = FindLaneProto(smm, mapping::ElementId(3)),
          .dist = 1.0,
      },
      {
          .lane_proto = FindLaneProto(smm, mapping::ElementId(1)),
          .dist = 2.0,
      },
      {
          .lane_proto = FindLaneProto(smm, mapping::ElementId(951)),
          .dist = 3.0,
      },
  };
  const auto exclude_lanes = ExcludeOngoingLanes(smm, pt_lanes);
  EXPECT_EQ(exclude_lanes.size(), 2);
  EXPECT_EQ(exclude_lanes[0].lane_proto->id(), 3);
  EXPECT_EQ(exclude_lanes[1].lane_proto->id(), 1);
}

TEST(MapMatchTest, SelectOnlyOneLaneInSection) {
  // "3", "2448","2" with the same section
  const auto& smm = LoadDojoMap();
  std::vector<mapping::PointToLane> pt_lanes = {
      {
          .lane_proto = FindLaneProto(smm, mapping::ElementId(3)),
          .dist = 1.0,
      },
      {
          .lane_proto = FindLaneProto(smm, mapping::ElementId(2448)),
          .dist = 2.0,
      },
      {
          .lane_proto = FindLaneProto(smm, mapping::ElementId(2)),
          .dist = 3.0,
      },
  };
  const auto select_lanes = SelectOnlyOneLaneInSection(pt_lanes);
  EXPECT_EQ(select_lanes.size(), 1);
  EXPECT_EQ(select_lanes[0].lane_proto->id(), 3);
}

TEST(MapMatchTest, PostprocessingSortedPointToLanes) {
  const auto& smm = LoadDojoMap();
  std::vector<mapping::PointToLane> ptls = {
      {
          .lane_proto = FindLaneProto(smm, mapping::ElementId(1)),
          .dist = 1.0,
      },
      {
          .lane_proto = FindLaneProto(smm, mapping::ElementId(938)),
          .dist = 2.0,
      },
      {
          .lane_proto = FindLaneProto(smm, mapping::ElementId(950)),
          .dist = 3.0,
      },
      {
          .lane_proto = FindLaneProto(smm, mapping::ElementId(34)),
          .dist = 3.0,
      },
  };
  const auto post_ptls = PostprocessingSortedPointToLanes(smm, ptls);
  EXPECT_EQ(post_ptls.size(), 1);
  EXPECT_EQ(post_ptls.front().lane_proto->id(), 1);
}

}  // namespace

}  // namespace qcraft::planner::route::map_match
