// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include "onboard/planner/common/vehicle_shape.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {
namespace {
const auto kVehicleOctagonModelParams =
    DefaultPlannerParams()
        .vehicle_models_params()
        .freespace_vehicle_octagon_model_params();

VehicleGeometryParamsProto InitVehicleGeometryParamsProto() {
  VehicleGeometryParamsProto vehicle_geo_params = DefaultVehicleGeometry();
  ::qcraft::AABox3dProto left_mirror_proto;
  left_mirror_proto.set_x(kVehicleOctagonModelParams.mirror_offset_x());
  left_mirror_proto.set_y(kVehicleOctagonModelParams.mirror_offset_y());
  left_mirror_proto.set_length(kVehicleOctagonModelParams.mirror_radius() *
                               2.0);

  ::qcraft::AABox3dProto right_mirror_proto;
  right_mirror_proto.set_x(kVehicleOctagonModelParams.mirror_offset_x());
  right_mirror_proto.set_y(-kVehicleOctagonModelParams.mirror_offset_y());
  right_mirror_proto.set_length(kVehicleOctagonModelParams.mirror_radius() *
                                2.0);
  *vehicle_geo_params.mutable_left_mirror() = left_mirror_proto;
  *vehicle_geo_params.mutable_right_mirror() = right_mirror_proto;
  return vehicle_geo_params;
}

std::vector<double> GenerateRandomAngle(int n) {
  std::vector<double> angle;
  angle.reserve(n);

  static std::mt19937 gen;
  std::uniform_real_distribution<> dis(-M_PI, M_PI);

  for (int i = 0; i < n; ++i) {
    angle.push_back(dis(gen));
  }
  return angle;
}

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

VehicleBoxShape ComputeVehicleBoxShape(
    const Vec2d& rac, double theta,
    const VehicleGeometryParamsProto& vehicle_geom) {
  const double half_length = vehicle_geom.length() * 0.5;
  const double half_width = vehicle_geom.width() * 0.5;
  const double rac_to_center = half_length - vehicle_geom.back_edge_to_center();
  std::vector<VehicleShapeBasePtr> av_shapes;
  const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
  const Vec2d center = rac + tangent * rac_to_center;
  return VehicleBoxShape(vehicle_geom, rac, center, tangent, theta, half_length,
                         half_width);
}

VehicleOctagonShape ComputeVehicleOctagonShape(
    const Vec2d& rac, double theta,
    const VehicleGeometryParamsProto& vehicle_geom,
    const VehicleOctagonModelParamsProto& vehicle_model_params) {
  const double half_length = vehicle_geom.length() * 0.5;
  const double half_width = vehicle_geom.width() * 0.5;
  const double rac_to_center = half_length - vehicle_geom.back_edge_to_center();
  const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
  const Vec2d center = rac + tangent * rac_to_center;
  return VehicleOctagonShape(vehicle_geom, vehicle_model_params, rac, center,
                             tangent, theta, half_length, half_width);
}

VehicleOctagonModel ComputeVehicleOctagonModel(
    const Vec2d& rac, double theta,
    const VehicleGeometryParamsProto& vehicle_geom,
    const VehicleOctagonModelParamsProto& vehicle_model_params) {
  Box2d av_box = ComputeAvBox(rac, theta, vehicle_geom);
  return VehicleOctagonModel(av_box,
                             vehicle_model_params.front_corner_side_length(),
                             vehicle_model_params.rear_corner_side_length());
}

double DistanceTo(const Circle2d& circle, const Segment2d& seg) {
  return seg.DistanceTo(circle.center()) - circle.radius();
}

double DistanceTo(const Circle2d& circle, const Polygon2d& polygon) {
  return polygon.DistanceTo(circle.center()) - circle.radius();
}

constexpr double kEps = 1e-4;

void CheckVec2dNear(const Vec2d& lhs, const Vec2d& rhs) {
  EXPECT_NEAR(lhs.x(), rhs.x(), kEps);
  EXPECT_NEAR(lhs.y(), rhs.y(), kEps);
}

void CheckVec2dNearInSeq(const std::vector<Vec2d>& lhs,
                         const std::vector<Vec2d>& rhs) {
  EXPECT_EQ(lhs.size(), rhs.size());
  for (int i = 0; i + 1 < static_cast<int>(lhs.size()); ++i) {
    CheckVec2dNear(lhs[i], rhs[i]);
  }
}

void CheckCircleNear(const Circle2d& lhs, const std::optional<Circle2d>& rhs) {
  EXPECT_TRUE(rhs.has_value());
  CheckVec2dNear(lhs.center(), rhs->center());
  EXPECT_NEAR(lhs.radius(), rhs->radius(), kEps);
}

TEST(VehicleShape, VehicleBoxShape) {
  const auto vehicle_geo_params = InitVehicleGeometryParamsProto();

  const auto& init_left_mirror = vehicle_geo_params.left_mirror();
  const auto& init_right_mirror = vehicle_geo_params.right_mirror();
  const double mirror_radius = init_left_mirror.length() * 0.5;
  const Vec2d init_left_mirror_center(init_left_mirror.x(),
                                      init_left_mirror.y());
  const Vec2d init_right_mirror_center(init_right_mirror.x(),
                                       init_right_mirror.y());

  constexpr int kSize = 20;
  constexpr double kLatBuffer = 0.2;
  constexpr double kLonBuffer = 0.3;

  const auto angles = GenerateRandomAngle(kSize);
  const auto points = GenerateRandomVector(kSize);
  const auto segments = GenerateRandomSegment2d(kSize);
  const auto polygons = GenerateRandomPolygon2d(kSize);
  for (const auto theta : angles) {
    for (const auto& rac : points) {
      const VehicleShapeBasePtr av_shape = std::make_unique<VehicleBoxShape>(
          ComputeVehicleBoxShape(rac, theta, vehicle_geo_params));
      const Box2d av_box = ComputeAvBox(rac, theta, vehicle_geo_params);
      CheckVec2dNearInSeq(
          av_shape->GetCornersWithBufferCounterClockwise(kLatBuffer,
                                                         kLonBuffer),
          av_box.GetCornersWithBufferCounterClockwise(kLatBuffer, kLonBuffer));
      auto extended_box = av_box;
      extended_box.LongitudinalExtend(kLonBuffer * 2.0);
      extended_box.LateralExtend(kLatBuffer * 2.0);
      const Circle2d left_mirror(init_left_mirror_center.Rotate(theta) + rac,
                                 mirror_radius);
      const Circle2d right_mirror(init_right_mirror_center.Rotate(theta) + rac,
                                  mirror_radius);
      CheckVec2dNear(av_box.center(), av_shape->center());
      CheckCircleNear(left_mirror, av_shape->left_mirror());
      CheckCircleNear(right_mirror, av_shape->right_mirror());
      for (const auto& seg : segments) {
        EXPECT_NEAR(av_shape->MainBodyDistanceTo(seg), av_box.DistanceTo(seg),
                    kEps);
        EXPECT_EQ(
            av_shape->MainBodyHasOverlapWithBuffer(seg, kLatBuffer, kLonBuffer),
            extended_box.HasOverlap(seg));
        EXPECT_NEAR(av_shape->LeftMirrorDistanceTo(seg),
                    DistanceTo(left_mirror, seg), kEps);
        EXPECT_NEAR(av_shape->RightMirrorDistanceTo(seg),
                    DistanceTo(right_mirror, seg), kEps);
      }
      for (const auto& polygon : polygons) {
        EXPECT_NEAR(polygon.DistanceTo(av_box),
                    av_shape->MainBodyDistanceTo(polygon), kEps);
        const bool has_overlap =
            polygon.HasOverlap(extended_box) ||
            DistanceTo(left_mirror, polygon) <= kLatBuffer ||
            DistanceTo(right_mirror, polygon) <= kLatBuffer;
        EXPECT_EQ(
            av_shape->HasOverlapWithBuffer(polygon, kLatBuffer, kLonBuffer,
                                           /*consider_mirrors=*/true),
            has_overlap);
      }
    }
  }
}

TEST(VehicleShape, VehicleOctagonShape) {
  const auto vehicle_geo_params = InitVehicleGeometryParamsProto();

  const auto& init_left_mirror = vehicle_geo_params.left_mirror();
  const auto& init_right_mirror = vehicle_geo_params.right_mirror();
  const double mirror_radius = init_left_mirror.length() * 0.5;
  const Vec2d init_left_mirror_center(init_left_mirror.x(),
                                      init_left_mirror.y());
  const Vec2d init_right_mirror_center(init_right_mirror.x(),
                                       init_right_mirror.y());
  constexpr int kSize = 20;
  constexpr double kLatBuffer = 0.2;
  constexpr double kLonBuffer = 0.3;

  const auto angles = GenerateRandomAngle(kSize);
  const auto points = GenerateRandomVector(kSize);
  const auto segments = GenerateRandomSegment2d(kSize);
  const auto polygons = GenerateRandomPolygon2d(kSize);
  for (const auto theta : angles) {
    for (const auto& rac : points) {
      const VehicleShapeBasePtr av_shape =
          std::make_unique<VehicleOctagonShape>(ComputeVehicleOctagonShape(
              rac, theta, vehicle_geo_params, kVehicleOctagonModelParams));
      const VehicleOctagonModel av_octagon = ComputeVehicleOctagonModel(
          rac, theta, vehicle_geo_params, kVehicleOctagonModelParams);
      CheckVec2dNearInSeq(av_shape->GetCornersWithBufferCounterClockwise(
                              kLatBuffer, kLonBuffer),
                          av_octagon.GetCornersWithBufferCounterClockwise(
                              kLatBuffer, kLonBuffer));
      const Circle2d left_mirror(init_left_mirror_center.Rotate(theta) + rac,
                                 mirror_radius);
      const Circle2d right_mirror(init_right_mirror_center.Rotate(theta) + rac,
                                  mirror_radius);
      CheckVec2dNear(av_octagon.centroid(), av_shape->center());
      CheckCircleNear(left_mirror, av_shape->left_mirror());
      CheckCircleNear(right_mirror, av_shape->right_mirror());
      for (const auto& seg : segments) {
        EXPECT_NEAR(av_shape->MainBodyDistanceTo(seg),
                    av_octagon.DistanceTo(seg), kEps);
        EXPECT_NEAR(av_shape->LeftMirrorDistanceTo(seg),
                    DistanceTo(left_mirror, seg), kEps);
        EXPECT_NEAR(av_shape->RightMirrorDistanceTo(seg),
                    DistanceTo(right_mirror, seg), kEps);
      }
      for (const auto& polygon : polygons) {
        EXPECT_NEAR(av_octagon.DistanceTo(polygon),
                    av_shape->MainBodyDistanceTo(polygon), kEps);
        const bool has_overlap =
            av_octagon.HasOverlapWithBuffer(polygon, kLatBuffer, kLonBuffer) ||
            DistanceTo(left_mirror, polygon) <= kLatBuffer ||
            DistanceTo(right_mirror, polygon) <= kLatBuffer;
        EXPECT_EQ(
            av_shape->HasOverlapWithBuffer(polygon, kLatBuffer, kLonBuffer,
                                           /*consider_mirrors=*/true),
            has_overlap);
      }
    }
  }
}
}  // namespace
}  // namespace qcraft::planner
