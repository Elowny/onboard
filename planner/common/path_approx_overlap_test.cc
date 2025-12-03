#include "onboard/planner/common/path_approx_overlap.h"

#include <cmath>
#include <memory>

#include "absl/types/span.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/offset_rect.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {
namespace planner {
namespace {

MATCHER_P4(AgentOverlapNear, first_ra_s, last_ra_s, lat_dist, epsilon, "") {
  *result_listener << "Actual value: first_ra_s=" << arg.first_ra_s
                   << ", last_ra_s=" << arg.last_ra_s
                   << ", lat_dist=" << arg.lat_dist;
  return std::abs(arg.first_ra_s - first_ra_s) < epsilon &&
         std::abs(arg.last_ra_s - last_ra_s) < epsilon &&
         std::abs(arg.lat_dist - lat_dist) < epsilon;
}

std::unique_ptr<SegmentMatcherKdtree> BuildPathKdTree(
    absl::Span<const PathPoint> path_points) {
  std::vector<Vec2d> points;
  points.reserve(path_points.size());
  for (const auto& point : path_points) {
    points.emplace_back(point.x(), point.y());
  }
  return std::make_unique<SegmentMatcherKdtree>(points);
}

using ::testing::ElementsAre;

TEST(StraightTest, Works) {
  PathPoint p0, p1, p2, p3;
  TextToProto(R"(x: 0.0 y: 0.0 theta: 0.0, s: 0.0)", &p0);
  TextToProto(R"(x: 1.0 y: 0.0 theta: 0.0, s: 1.0)", &p1);
  TextToProto(R"(x: 2.0 y: 0.0 theta: 0.0, s: 2.0)", &p2);
  TextToProto(R"(x: 3.0 y: 0.0 theta: 0.0, s: 3.0)", &p3);
  const std::vector<PathPoint> path_points{p0, p1, p2, p3};
  const auto path_kd_tree = BuildPathKdTree(path_points);

  const auto vehicle_geom = DefaultVehicleGeometry();
  const auto vehicle_rect = CreateOffsetRectFromVehicleGeometry(vehicle_geom);
  const auto path_approx =
      BuildPathApprox(path_points, vehicle_rect,
                      /*tolerance=*/0.05, /*path_kd_tree=*/path_kd_tree.get());

  const Polygon2d box1(Box2d(Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/1.0,
                             /*width=*/1.0));

  constexpr double kBuffer = 0.2;  // m.
  const double av_radius = vehicle_rect.radius() + kBuffer;
  std::vector<AgentOverlap> overlaps;
  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 3, box1,
                                  /*max_lat_dist=*/10.0,
                                  av_radius + box1.CircleRadius());
  EXPECT_THAT(overlaps, ElementsAre(AgentOverlapNear(0.0, 1.5, 0.0, 1e-6)));

  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 1, box1,
                                  /*max_lat_dist=*/10.0,
                                  av_radius + box1.CircleRadius());
  EXPECT_THAT(overlaps, ElementsAre(AgentOverlapNear(0.0, 1.0, 0.0, 1e-6)));

  const Polygon2d box2(Box2d(Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/2.0,
                             /*width=*/1.0));
  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 3, box2,
                                  /*max_lat_dist=*/10.0,
                                  av_radius + box2.CircleRadius());
  EXPECT_THAT(overlaps, ElementsAre(AgentOverlapNear(0.0, 2.0, 0.0, 1e-6)));

  const Polygon2d box3(Box2d(Vec2d(0.0, 10.0), /*heading=*/0.0, /*length=*/2.0,
                             /*width=*/2.0));
  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 3, box3,
                                  /*max_lat_dist=*/10.0,
                                  av_radius + box3.CircleRadius() + 10.0);
  EXPECT_THAT(overlaps, ElementsAre(AgentOverlapNear(0.0, 0.0, 8.0, 1e-6)));

  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 3, box3,
                                  /*max_lat_dist=*/1.0,
                                  av_radius + box3.CircleRadius());
  EXPECT_TRUE(overlaps.empty());

  const Polygon2d box4(Box2d(Vec2d(-10.0, 0.0), /*heading=*/0.0, /*length=*/2.0,
                             /*width=*/2.0));
  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 3, box4,
                                  /*max_lat_dist=*/10.0,
                                  av_radius + box4.CircleRadius());
  EXPECT_TRUE(overlaps.empty());
}

TEST(StraightTest, LateralGap) {
  PathPoint p0, p1, p2, p3;
  TextToProto(R"(x: 0.0 y: 0.0 theta: 0.0, s: 0.0)", &p0);
  TextToProto(R"(x: 1.0 y: 0.0 theta: 0.0, s: 1.0)", &p1);
  TextToProto(R"(x: 2.0 y: 0.0 theta: 0.0, s: 2.0)", &p2);
  TextToProto(R"(x: 3.0 y: 0.0 theta: 0.0, s: 3.0)", &p3);
  const std::vector<PathPoint> path_points{p0, p1, p2, p3};
  const auto path_kd_tree = BuildPathKdTree(path_points);

  const auto vehicle_geom = DefaultVehicleGeometry();
  const auto vehicle_rect = CreateOffsetRectFromVehicleGeometry(vehicle_geom);
  const auto path_approx =
      BuildPathApprox(path_points, vehicle_rect,
                      /*tolerance=*/0.05, /*path_kd_tree=*/path_kd_tree.get());
  constexpr double kBuffer = 0.2;  // m.
  const double av_radius = vehicle_rect.radius() + kBuffer;

  constexpr double kMaxLatDist = 5.0;  // m.
  const Box2d box(Vec2d(0.0, 4.0), /*heading=*/0.0, /*length=*/1.0,
                  /*width=*/1.0);
  const Polygon2d polygon(box);
  std::vector<AgentOverlap> overlaps = ComputeAgentOverlapsWithBuffer(
      path_approx, /*step_length=*/1.0, /*first_index=*/0,
      /*last_index=*/3, polygon, kMaxLatDist, /*lat_buffer=*/3.0,
      /*lon_buffer=*/0.0, av_radius + polygon.CircleRadius());
  EXPECT_THAT(overlaps,
              ElementsAre(AgentOverlapNear(
                  0.0, box.width() * 0.5 + vehicle_geom.back_edge_to_center(),
                  0.0, 1e-6)));
  overlaps = ComputeAgentOverlapsWithBufferAndHeading(
      path_approx, /*step_length=*/1.0, /*first_index=*/0,
      /*last_index=*/3, polygon, kMaxLatDist, /*lat_buffer=*/3.0,
      /*lon_buffer=*/0.0, av_radius + polygon.CircleRadius(), /*theta=*/0.0,
      /*max_heading_diff=*/M_PI_4);
  EXPECT_THAT(overlaps,
              ElementsAre(AgentOverlapNear(
                  0.0, box.width() * 0.5 + vehicle_geom.back_edge_to_center(),
                  0.0, 1e-6)));
}

TEST(StraightTest, HalfPalneOverlap) {
  PathPoint p0, p1, p2, p3, p4;
  TextToProto(R"(x: 0.0 y: 0.0 theta: 0.0, s: 0.0)", &p0);
  TextToProto(R"(x: 1.0 y: 0.0 theta: 0.0, s: 1.0)", &p1);
  TextToProto(R"(x: 2.0 y: 0.0 theta: 0.0, s: 2.0)", &p2);
  TextToProto(R"(x: 3.0 y: 0.0 theta: 0.0, s: 3.0)", &p3);
  TextToProto(R"(x: 4.0 y: 0.0 theta: 0.0, s: 4.0)", &p4);
  const std::vector<PathPoint> path_points{p0, p1, p2, p3};
  const auto path_kd_tree = BuildPathKdTree(path_points);
  const auto vehicle_geom = DefaultVehicleGeometry();
  const auto vehicle_rect = CreateOffsetRectFromVehicleGeometry(vehicle_geom);
  const auto path_approx =
      BuildPathApprox(path_points, vehicle_rect,
                      /*tolerance=*/0.05, /*path_kd_tree=*/path_kd_tree.get());
  const HalfPlane half_plane(Vec2d(2.0, 2.0), Vec2d(2.0, -2.0));
  constexpr double kBuffer = 0.2;  // m.
  const double av_radius = vehicle_rect.radius() + kBuffer;
  auto min_ra_s = ComputeHalfPlaneOverlapsWithLateralGap(
      path_approx, /*step_length=*/1.0, /*first_index=*/0,
      /*last_index=*/4, half_plane, /*lateral_gap=*/0.0,
      av_radius + half_plane.length() * 0.5);
  EXPECT_NEAR(*min_ra_s, 0.0, 1e-6);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
