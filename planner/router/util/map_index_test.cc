#include "onboard/planner/router/util/map_index.h"

#include <set>

#include "gtest/gtest.h"

#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/router/route_test_util.h"
namespace qcraft::planner::route {
namespace {
TEST(MapIndex, PointToNearLanesTest) {
  const auto& smm = LoadDojoMap();
  MapIndex index;
  index.InitIndex(smm);
  const Vec2d global = {0.0, 0.0};
  std::vector<mapping::PointToLane> point_to_lanes =
      index.PointToNearLanes(global, 7.0, [](const auto*) { return true; });
  EXPECT_TRUE(!point_to_lanes.empty());
  std::set<int> ids;
  for (const auto& point_to_lane : point_to_lanes) {
    auto id = point_to_lane.lane_proto->id();
    ids.insert(id);
    if (id == 2 || id == 3) {
      EXPECT_NEAR(point_to_lane.fraction, 0, 0.01);
      EXPECT_NEAR(point_to_lane.segment.heading(), 0.0, 0.1);
    } else if (id == 56 || id == 97) {
      EXPECT_NEAR(point_to_lane.fraction, 1.0, 0.01);
    }
  }
  std::set<int> expect_ids = {2, 3, 2448, 96, 97, 267, 222, 55, 56, 57};
  EXPECT_EQ(expect_ids.size(), ids.size());
  for (const auto id : ids) {
    EXPECT_EQ(expect_ids.count(id), 1);
  }

  {
    const auto& smm_v1 = LoadDojoMapV1();
    MapIndex index_v1;
    index_v1.InitIndex(smm_v1);
    const Vec2d global = {0.0, 0.0};
    std::vector<mapping::PointToLane> point_to_lanes =
        index_v1.PointToNearLanes(global, 7.0,
                                  [](const auto*) { return true; });
    EXPECT_TRUE(!point_to_lanes.empty());
  }
}
}  // namespace
}  // namespace qcraft::planner::route
