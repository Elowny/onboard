#include "onboard/planner/freespace/sqp_global_path_smoother/convex_region_generator.h"

#include <math.h>

#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/eigen.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/freespace/sqp_global_path_smoother/geometric_utils.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kFrontEdgeToCenter = 3.5;  // m
constexpr double kBackEdgeToCenter = 1.5;   // m
constexpr double kLocalBoxSize = 5.0;       // m

TEST(ConvexRegionsGeneratorTest, ConvexRegionsGeneratorTest) {
  DiscretizedPath path;
  PathPoint path_point;
  path_point.set_x(0.0);
  path_point.set_y(0.0);
  path_point.set_theta(M_PI_4);
  path.push_back(std::move(path_point));
  std::vector<bool> forwards;
  forwards.push_back(true);
  // Set map size
  const Vec2d bottom_left(-5.0, -5.0);
  const Vec2d top_right(5.0, 5.0);
  const AABox2d global_aabox(bottom_left, top_right);
  std::vector<std::pair<double, Segment2d>> boundaries_info;
  boundaries_info.push_back(
      std::pair<double, Segment2d>(0.3, Segment2d({4.0, 2.0}, {4.0, -4.0})));
  boundaries_info.push_back(
      std::pair<double, Segment2d>(1.5, Segment2d({-3.0, 0.0}, {0.0, 3.0})));
  std::vector<std::pair<double, Polygon2d>> objects_info;
  std::vector<Vec2d> object_points = {Vec2d{-3.0, -2.0}, Vec2d{-1.0, -3.0},
                                      Vec2d{-2.0, -1.0}};
  objects_info.push_back(std::pair<double, Polygon2d>(
      0.1, Polygon2d{object_points, /*is_convex=*/true}));
  object_points = {Vec2d{2.0, -1.0}, Vec2d{2.0, -3.0}, Vec2d{1.0, -2.0}};
  objects_info.push_back(std::pair<double, Polygon2d>(
      0.3, Polygon2d{object_points, /*is_convex=*/true}));

  ConvexRegionsGenerator generator(global_aabox);
  generator.Dilate(path, forwards, boundaries_info, objects_info, kLocalBoxSize,
                   kLocalBoxSize, kFrontEdgeToCenter, kBackEdgeToCenter,
                   /*set_start_to_origin=*/false);

  const auto& line_segment_decomps = generator.line_segment_decomps();
  EXPECT_EQ(line_segment_decomps.size(), path.size());
  for (int i = 0; i < path.size(); ++i) {
    const auto& convex_region = line_segment_decomps[i].convex_region();
    const auto vertices = CalVertices(convex_region);
    const Vec2d path_point_vec(path[i].x(), path[i].y());
    EXPECT_GE(vertices.size(), 3);
    Polygon2d polygon(vertices);
    for (const auto& boundary_info : boundaries_info) {
      EXPECT_FALSE(convex_region.IsPointIn(boundary_info.second.start()));
      EXPECT_FALSE(convex_region.IsPointStrictlyIn(boundary_info.second.end()));
      EXPECT_FALSE(polygon.IsPointIn(boundary_info.second.start()));
      EXPECT_FALSE(polygon.IsPointIn(boundary_info.second.end()));
    }
    for (const auto& object_info : objects_info) {
      for (const auto& point : object_info.second.points()) {
        EXPECT_FALSE(convex_region.IsPointIn(point));
        EXPECT_FALSE(polygon.IsPointIn(point));
      }
    }
    EXPECT_TRUE(polygon.is_convex());
  }

  // NOLINTNEXTLINE(readability-identifier-naming)
  Mat2d C = Mat2d::Identity();
  Vec2d d = {0.0, 0.0};
  C(0, 0) = 2.0;
  C(1, 1) = 1.0;
  GeneralEllipse ellipse(C, d);
  EXPECT_TRUE(ellipse.IsPointIn(Vec2d{2.0, 0.0}));
  EXPECT_FALSE(ellipse.IsPointStrictlyIn(Vec2d{2.0, 0.0}));
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
