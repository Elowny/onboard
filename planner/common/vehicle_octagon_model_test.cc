// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include "onboard/planner/common/vehicle_octagon_model.h"

#include <algorithm>
#include <random>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/math/geometry/polygon2d.h"

namespace qcraft::planner {
namespace {
constexpr double kCornerLengthRatio = 0.25;
std::vector<Vec2d> GenerateRandomVector(int n) {
  std::vector<Vec2d> v;
  v.reserve(n);

  static std::mt19937 gen;
  std::uniform_real_distribution<> dis(-5.0, 5.0);

  for (int i = 0; i < n; ++i) {
    v.push_back(Vec2d(dis(gen), dis(gen)));
  }
  return v;
}

std::vector<Box2d> GenerateRandomBox2d(int n) {
  const auto centers = GenerateRandomVector(n);
  auto tangents = GenerateRandomVector(n);
  for (auto& v : tangents) {
    v /= v.norm();
  }
  auto length_width = GenerateRandomVector(n);
  for (auto& lw : length_width) {
    lw /= 100.0;
  }
  std::vector<Box2d> boxes;
  boxes.reserve(n);
  for (int i = 0; i < n; ++i) {
    boxes.push_back(Box2d(centers[i], tangents[i],
                          std::max(0.1, length_width[i].x()),
                          std::max(0.1, length_width[i].y())));
  }
  return boxes;
}

std::vector<Segment2d> GenerateRandomSegment2d(int n) {
  const auto points = GenerateRandomVector(n + 1);
  std::vector<Segment2d> segments;
  segments.reserve(n);
  for (int i = 0; i < n; ++i) {
    segments.emplace_back(points[i], points[i + 1]);
  }
  return segments;
}

std::vector<Polygon2d> GenerateRandomPolygon2d(int n) {
  const auto centers = GenerateRandomVector(n);
  auto tangents = GenerateRandomVector(n);
  for (auto& v : tangents) {
    v /= v.norm();
  }
  const auto length_width = GenerateRandomVector(n);

  std::vector<Polygon2d> polys;
  polys.reserve(n);
  for (int i = 0; i < n; ++i) {
    polys.push_back(Polygon2d(Box2d(
        centers[i], tangents[i], std::max(0.1, std::fabs(length_width[i].x())),
        std::max(0.1, std::fabs(length_width[i].y())))));
  }
  return polys;
}

TEST(VehicleOctagonModel, IsPointInTest) {
  constexpr int kSize = 5;
  const auto boxes = GenerateRandomBox2d(kSize);
  const auto points = GenerateRandomVector(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    for (int j = 0; j < kSize; ++j) {
      EXPECT_EQ(octagon.IsPointIn(points[j]), polygon.IsPointIn(points[j]));
    }
  }
}

TEST(VehicleOctagonModel, IsPointInWithBufferTest) {
  constexpr int kSize = 500;
  constexpr double kBuffer = 0.5;
  const auto boxes = GenerateRandomBox2d(kSize);
  const auto points = GenerateRandomVector(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    for (int j = 0; j < kSize; ++j) {
      EXPECT_EQ(
          octagon.IsPointInWithBuffer(points[j], kBuffer, kBuffer),
          octagon.GetModelWithBuffer(kBuffer, kBuffer).IsPointIn(points[j]));
    }
  }
  // different buffer
  const Box2d box(/*center=*/Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/2.0,
                  /*width=*/1.0);
  const VehicleOctagonModel octagon(box, 1.0 * kCornerLengthRatio,
                                    0.5 * kCornerLengthRatio);
  const Vec2d point_up(0.0, 0.75);
  const Vec2d point_right(1.25, 0.0);
  EXPECT_TRUE(octagon.IsPointInWithBuffer(point_up, kBuffer, 0.0));
  EXPECT_FALSE(octagon.IsPointInWithBuffer(point_up, 0.0, kBuffer));
  EXPECT_TRUE(octagon.IsPointInWithBuffer(point_right, 0.0, kBuffer));
  EXPECT_FALSE(octagon.IsPointInWithBuffer(point_right, kBuffer, 0.0));
  // negtive buffer
  constexpr double kNegBuffer = -0.2;
  const Vec2d point_down(0.0, -0.4);
  const Vec2d point_left(-0.9, 0.0);
  EXPECT_FALSE(octagon.IsPointInWithBuffer(point_down, kNegBuffer, 0.0));
  EXPECT_TRUE(octagon.IsPointInWithBuffer(point_down, 0.0, kNegBuffer));
  EXPECT_FALSE(octagon.IsPointInWithBuffer(point_left, 0.0, kNegBuffer));
  EXPECT_TRUE(octagon.IsPointInWithBuffer(point_left, kNegBuffer, 0.0));
}

TEST(VehicleOctagonModel, ExtremePointTest) {
  constexpr int kSize = 5;
  constexpr double kEps = 1e-4;
  const auto boxes = GenerateRandomBox2d(kSize);
  const auto vecs = GenerateRandomVector(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    for (int j = 0; j < kSize; ++j) {
      const auto& extreme_point_of_octagon =
          octagon.points()[octagon.ExtremePoint(vecs[j])];
      const auto& extreme_point_of_polygon =
          polygon.points()[polygon.ExtremePoint(vecs[j])];
      EXPECT_NEAR(vecs[j].dot(extreme_point_of_octagon),
                  vecs[j].dot(extreme_point_of_polygon), kEps);
    }
  }
}

TEST(VehicleOctagonModel, DistanceToPointTest) {
  constexpr int kSize = 5;
  constexpr double kEps = 1e-4;
  const auto boxes = GenerateRandomBox2d(kSize);
  const auto points = GenerateRandomVector(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    for (int j = 0; j < kSize; ++j) {
      EXPECT_NEAR(octagon.DistanceTo(points[j]), polygon.DistanceTo(points[j]),
                  kEps);
    }
  }
}

TEST(VehicleOctagonModel, HasOverlapSegment2dTest) {
  constexpr int kSize = 5;
  constexpr double kEps = 1e-4;
  const auto boxes = GenerateRandomBox2d(kSize);
  const auto segments = GenerateRandomSegment2d(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    for (int j = 0; j < kSize; ++j) {
      if (octagon.HasOverlap(segments[j]) != polygon.HasOverlap(segments[j])) {
        EXPECT_LT(polygon.DistanceTo(segments[j]), kEps);
        EXPECT_LT(octagon.DistanceTo(segments[j]), kEps);
      }
    }
  }
}

TEST(VehicleOctagonModel, HasOverlapWithBufferSegment2dTest) {
  constexpr int kSize = 5;
  constexpr double kEps = 1e-4;
  constexpr double kBuffer = 0.5;
  const auto boxes = GenerateRandomBox2d(kSize);
  const auto segments = GenerateRandomSegment2d(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    for (int j = 0; j < kSize; ++j) {
      const double dist = octagon.DistanceTo(segments[j]);
      const bool has_overlap_with_buffer = dist < kBuffer;
      if (octagon.HasOverlapWithBuffer(segments[j], kBuffer, kBuffer) !=
          has_overlap_with_buffer) {
        EXPECT_NEAR(dist, kBuffer, kEps);
      }
    }
  }
  // different buffer
  const Box2d box(/*center=*/Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/2.0,
                  /*width=*/1.0);
  const VehicleOctagonModel octagon(box, 1.0 * kCornerLengthRatio,
                                    0.5 * kCornerLengthRatio);
  const Segment2d line_up = {Vec2d{-1.0, 0.75}, Vec2d{1.0, 0.75}};
  const Segment2d line_right = {Vec2d{1.25, 0.5}, Vec2d{1.25, -0.5}};
  EXPECT_TRUE(octagon.HasOverlapWithBuffer(line_up, kBuffer, 0.0));
  EXPECT_FALSE(octagon.HasOverlapWithBuffer(line_up, 0.0, kBuffer));
  EXPECT_TRUE(octagon.HasOverlapWithBuffer(line_right, 0.0, kBuffer));
  EXPECT_FALSE(octagon.HasOverlapWithBuffer(line_right, kBuffer, 0.0));
  // negtive buffer
  constexpr double kNegBuffer = -0.2;
  const Segment2d line_down = {Vec2d{-1.0, -0.4}, Vec2d{1.0, -0.4}};
  const Segment2d line_left = {Vec2d{-0.9, 0.5}, Vec2d{-0.9, -0.5}};
  EXPECT_FALSE(octagon.HasOverlapWithBuffer(line_down, kNegBuffer, 0.0));
  EXPECT_TRUE(octagon.HasOverlapWithBuffer(line_down, 0.0, kNegBuffer));
  EXPECT_FALSE(octagon.HasOverlapWithBuffer(line_left, 0.0, kNegBuffer));
  EXPECT_TRUE(octagon.HasOverlapWithBuffer(line_left, kNegBuffer, 0.0));
}

TEST(VehicleOctagonModel, DistanceToSegment2dTest) {
  constexpr int kSize = 5;
  constexpr double kEps = 1e-4;
  const auto boxes = GenerateRandomBox2d(kSize);
  const auto segments = GenerateRandomSegment2d(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    for (int j = 0; j < kSize; ++j) {
      EXPECT_NEAR(octagon.DistanceTo(segments[j]),
                  polygon.DistanceTo(segments[j]), kEps);
    }
  }
}

TEST(VehicleOctagonModel, DistanceToWithBufferSegment2dTest) {
  constexpr double kEps = 1e-4;
  constexpr double kLatBuffer = 0.1;
  constexpr double kLonBuffer = 0.15;
  constexpr double kAVLength = 2.0;
  constexpr double kAVWidth = 1.0;
  const Box2d box(/*center=*/Vec2d(0.0, 0.0), /*heading=*/0.0, kAVLength,
                  kAVWidth);
  const VehicleOctagonModel octagon(box, 1.0 * kCornerLengthRatio,
                                    0.5 * kCornerLengthRatio);
  const Segment2d segment_up = {Vec2d{-0.5, 1.5}, Vec2d{0.5, 2.0}};
  EXPECT_NEAR(octagon.DistanceToWithBuffer(segment_up, kLatBuffer, kLonBuffer),
              1.5 - kAVWidth / 2.0 - kLatBuffer, kEps);
  const Segment2d segment_left = {Vec2d{-2.0, 0.3}, Vec2d{-2.5, -0.3}};
  EXPECT_NEAR(
      octagon.DistanceToWithBuffer(segment_left, kLatBuffer, kLonBuffer),
      2.0 - kAVLength / 2.0 - kLonBuffer, kEps);
}

TEST(VehicleOctagonModel, HasOverlapBox2dTest) {
  constexpr int kSize = 5;
  const auto boxes = GenerateRandomBox2d(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    for (int j = 0; j < kSize; ++j) {
      EXPECT_EQ(octagon.HasOverlap(boxes[j]), polygon.HasOverlap(boxes[j]));
    }
  }
}

TEST(VehicleOctagonModel, DistanceToBox2dTest) {
  constexpr int kSize = 5;
  constexpr double kEps = 1e-4;
  const auto boxes = GenerateRandomBox2d(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    for (int j = 0; j < kSize; ++j) {
      EXPECT_NEAR(octagon.DistanceTo(boxes[j]), polygon.DistanceTo(boxes[j]),
                  kEps);
    }
  }
}

TEST(VehicleOctagonModel, HasOverlapPolygon2dTest) {
  constexpr int kSize = 5;
  const auto boxes = GenerateRandomBox2d(kSize);
  const auto polygons = GenerateRandomPolygon2d(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    for (int j = 0; j < kSize; ++j) {
      EXPECT_EQ(octagon.HasOverlap(polygons[j]),
                polygon.HasOverlap(polygons[j]));
    }
  }
}

TEST(VehicleOctagonModel, HasOverlapOctagonTest) {
  constexpr int kSize = 5;
  const auto boxes = GenerateRandomBox2d(kSize);
  std::vector<VehicleOctagonModel> octagons;
  std::vector<Polygon2d> polygons;
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    octagons.push_back(std::move(octagon));
    polygons.push_back(std::move(polygon));
  }
  for (int i = 0; i < kSize; ++i) {
    for (int j = 0; j < kSize; ++j) {
      EXPECT_EQ(octagons[i].HasOverlap(octagons[j]),
                polygons[i].HasOverlap(polygons[j]));
    }
  }
}

TEST(VehicleOctagonModel, HasOverlapWithBufferPolygon2dTest) {
  constexpr int kSize = 5;
  constexpr double kEps = 1e-4;
  constexpr double kBuffer = 0.5;
  const auto boxes = GenerateRandomBox2d(kSize);
  const auto polygons = GenerateRandomPolygon2d(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    for (int j = 0; j < kSize; ++j) {
      const auto dist = octagon.DistanceTo(polygons[j]);
      const bool has_overlap_with_buffer = dist < kBuffer;
      if (octagon.HasOverlapWithBuffer(polygons[j], kBuffer, kBuffer) !=
          has_overlap_with_buffer) {
        EXPECT_NEAR(dist, kBuffer, kEps);
      }
    }
  }
  // different buffer
  const Box2d box(/*center=*/Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/2.0,
                  /*width=*/1.0);
  const VehicleOctagonModel octagon(box, 1.0 * kCornerLengthRatio,
                                    0.5 * kCornerLengthRatio);
  std::vector<Vec2d> polygon_points = {Vec2d(-1.0, -0.75), Vec2d(1.0, -0.75),
                                       Vec2d(0.0, -1.0)};
  const Polygon2d polygon_down{polygon_points};
  polygon_points = {Vec2d(-1.25, 0.5), Vec2d(-1.25, -0.5), Vec2d(-1.5, 0.0)};
  const Polygon2d polygon_left{polygon_points};
  EXPECT_TRUE(octagon.HasOverlapWithBuffer(polygon_down, kBuffer, 0.0));
  EXPECT_FALSE(octagon.HasOverlapWithBuffer(polygon_down, 0.0, kBuffer));
  EXPECT_TRUE(octagon.HasOverlapWithBuffer(polygon_left, 0.0, kBuffer));
  EXPECT_FALSE(octagon.HasOverlapWithBuffer(polygon_left, kBuffer, 0.0));
  // negtive buffer
  constexpr double kNegBuffer = -0.2;
  polygon_points = {Vec2d(-1.0, 0.4), Vec2d(1.0, 0.4), Vec2d(0.0, 1.0)};
  const Polygon2d polygon_up{polygon_points};
  polygon_points = {Vec2d(0.9, 0.5), Vec2d(0.9, -0.5), Vec2d(1.5, 0.0)};
  const Polygon2d polygon_right{polygon_points};
  EXPECT_FALSE(octagon.HasOverlapWithBuffer(polygon_up, kNegBuffer, 0.0));
  EXPECT_TRUE(octagon.HasOverlapWithBuffer(polygon_up, 0.0, kNegBuffer));
  EXPECT_FALSE(octagon.HasOverlapWithBuffer(polygon_right, 0.0, kNegBuffer));
  EXPECT_TRUE(octagon.HasOverlapWithBuffer(polygon_right, kNegBuffer, 0.0));
}

TEST(VehicleOctagonModel, DistanceToPolygon2dTest) {
  constexpr int kSize = 5;
  constexpr double kEps = 1e-4;
  const auto boxes = GenerateRandomBox2d(kSize);
  const auto polygons = GenerateRandomPolygon2d(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    for (int j = 0; j < kSize; ++j) {
      EXPECT_NEAR(octagon.DistanceTo(polygons[j]),
                  polygon.DistanceTo(polygons[j]), kEps);
    }
  }
}

TEST(VehicleOctagonModel, DistanceToWithBufferPolygon2dTest) {
  constexpr double kEps = 1e-4;
  constexpr double kLatBuffer = 0.1;
  constexpr double kLonBuffer = 0.15;
  constexpr double kAVLength = 2.0;
  constexpr double kAVWidth = 1.0;
  const Box2d box(/*center=*/Vec2d(0.0, 0.0), /*heading=*/0.0, kAVLength,
                  kAVWidth);
  const VehicleOctagonModel octagon(box, 1.0 * kCornerLengthRatio,
                                    0.5 * kCornerLengthRatio);
  std::vector<Vec2d> polygon_points = {Vec2d(-0.5, 1.25), Vec2d(0.5, 1.5),
                                       Vec2d(0.0, 2.0)};
  const Polygon2d polygon_up{polygon_points};
  EXPECT_NEAR(octagon.DistanceToWithBuffer(polygon_up, kLatBuffer, kLonBuffer),
              1.25 - kAVWidth / 2.0 - kLatBuffer, kEps);
  polygon_points = {Vec2d(-1.25, 0.3), Vec2d(-1.5, -0.2), Vec2d(-2.0, 0.0)};
  const Polygon2d polygon_left{polygon_points};
  EXPECT_NEAR(
      octagon.DistanceToWithBuffer(polygon_left, kLatBuffer, kLonBuffer),
      1.25 - kAVLength / 2.0 - kLonBuffer, kEps);
}

TEST(VehicleOctagonModel, DistanceToOctagonTest) {
  constexpr int kSize = 5;
  constexpr double kEps = 1e-4;
  const auto boxes = GenerateRandomBox2d(kSize);
  std::vector<VehicleOctagonModel> octagons;
  std::vector<Polygon2d> polygons;
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    Polygon2d polygon(octagon.points(), /*is_convex=*/true);
    octagons.push_back(std::move(octagon));
    polygons.push_back(std::move(polygon));
  }
  for (int i = 0; i < kSize; ++i) {
    for (int j = 0; j < kSize; ++j) {
      EXPECT_NEAR(octagons[i].DistanceTo(octagons[j]),
                  polygons[i].DistanceTo(polygons[j]), kEps);
    }
  }
}

TEST(VehicleOctagonModel, GetCornersWithBufferCounterClockwise) {
  constexpr int kSize = 5;
  constexpr double kEps = 1e-4;
  constexpr double kBuffer1 = 0.3;
  constexpr double kBuffer2 = 0.5;
  const auto boxes = GenerateRandomBox2d(kSize);
  for (int i = 0; i < kSize; ++i) {
    VehicleOctagonModel octagon(boxes[i],
                                boxes[i].length() * kCornerLengthRatio,
                                boxes[i].width() * kCornerLengthRatio);
    VehicleOctagonModel buffered_octagon =
        octagon.GetModelWithBuffer(kBuffer1, kBuffer2);
    const auto& buffered_points1 = buffered_octagon.points();
    const auto buffered_points2 =
        octagon.GetCornersWithBufferCounterClockwise(kBuffer1, kBuffer2);
    for (int j = 0; j < kSize; ++j) {
      for (int k = 0; k < buffered_points1.size(); ++k) {
        EXPECT_NEAR(buffered_points1[k].x(), buffered_points2[k].x(), kEps);
        EXPECT_NEAR(buffered_points1[k].y(), buffered_points2[k].y(), kEps);
      }
    }
  }
}
}  // namespace
}  // namespace qcraft::planner
