#include "onboard/planner/router/geometry/gfc.h"

#include <cmath>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
namespace qcraft::planner::route {
namespace {

TEST(GfcTest, DistanceSquare) {
  // suzhou
  {
    Vec2d p1 = {2.1054831494605182, 0.548453257};
    Vec2d p2 = {2.1054831494605182, 0.548453257};
    double dist =
        std::sqrt(DistanceSquare(p1, p2, std::cos((p1.y() + p2.y()) / 2))) *
        kEarthRadius;
    EXPECT_NEAR(dist, 0, 0.1);
  }
  {
    Vec2d p1 = {2.1054831494605182, 0.548453257};
    Vec2d p2 = {2.10548448007, 0.54844869};
    const double dist =
        std::sqrt(DistanceSquare(p1, p2, std::cos((p1.y() + p2.y()) / 2))) *
        kEarthRadius;
    constexpr auto kExpectedValue = 29.98;
    EXPECT_NEAR(dist, kExpectedValue, kExpectedValue * 0.2);
    const double dist2 = HaversineDistance(p1, p2);
    EXPECT_NEAR(dist2, kExpectedValue, kExpectedValue * 0.003);
  }
}

TEST(GfcTest, ProjectToLineString) {
  // From dojo to get this points, as follows:
  // SetMap("dojo");
  // mapping::v2::SemanticMapManager semantic_map_manager;
  // semantic_map_manager.LoadWholeMap().Build();
  // const auto lane_ptr =
  //     semantic_map_manager.FindLaneInfoOrNull(mapping::ElementId(2));
  // if (lane_ptr == nullptr) {
  //   FAIL();
  // }
  // const mapping::GeoPolylineProto& proto = lane_ptr->proto->polyline();

  // std::vector<Vec2d> points;
  // for (const auto& pt : proto.points()) {
  //   points.emplace_back(pt.longitude(), pt.latitude());
  // }

  std::vector<Vec2d> points = {
      {1.0772844705713323e-08, 5.4525610704975843e-07},
      {3.9356629662373708e-06, 5.4575033263681034e-07},
      {7.8605530877690275e-06, 5.4624455822386225e-07},
      {8.7912876579756333e-06, 5.4636175713264859e-07}};

  {
    // Case 1: lowest
    Vec2d p1 = {-0.00000081617, -0.000000829};
    const auto t = ProjectToLineString(points, p1);
    EXPECT_EQ(t.match_point, points[0]);
    EXPECT_EQ(t.segment_index, 0);
    const auto dist = HaversineDistance(t.match_point, p1);
    EXPECT_NEAR(dist, 10.1, 0.3);
  }

  {
    // Case 2: highest
    Vec2d p2 = {1.0997795565427963e-05, 5.4423481391445072e-07};
    const auto t = ProjectToLineString(points, p2);
    EXPECT_EQ(t.match_point, points.back());
    EXPECT_EQ(t.segment_index, points.size() - 2);
    const auto dist = HaversineDistance(t.match_point, p2);
    EXPECT_NEAR(dist, 14.04, 0.5);
  }

  {
    // Case 3: mid
    Vec2d p3 = {7.8605530877690275e-06, 8.4624495822386225e-07};
    const auto t = ProjectToLineString(points, p3);
    EXPECT_EQ(t.segment_index, points.size() - 2);
    const auto dist = HaversineDistance(t.match_point, p3);
    EXPECT_NEAR(dist, 2.17, 0.5);
  }

  {
    // Case 4: on line string
    Vec2d p4 = {7.8605530877690275e-06, 5.4624495822386225e-07};
    const auto t = ProjectToLineString(points, p4);
    EXPECT_EQ(t.segment_index, 1);
    const auto dist = HaversineDistance(t.match_point, p4);
    EXPECT_NEAR(dist, 0, 0.1);
  }
}

}  // namespace

}  // namespace qcraft::planner::route
