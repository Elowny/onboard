#ifndef ONBOARD_PLANNER_COMMON_VEHICLE_SHAPE_H_
#define ONBOARD_PLANNER_COMMON_VEHICLE_SHAPE_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/circle2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/aabox3d.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/vehicle_octagon_model.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

class VehicleShapeBase {
 public:
  VehicleShapeBase(const VehicleGeometryParamsProto& vehicle_geo_params,
                   const Vec2d& rac, const Vec2d& tangent) {
    if (vehicle_geo_params.has_left_mirror() &&
        vehicle_geo_params.has_right_mirror()) {
      const Vec2d left_mirror_center =
          rac + vehicle_geo_params.left_mirror().x() * tangent +
          vehicle_geo_params.left_mirror().y() * tangent.Perp();
      const double left_mirror_radius =
          vehicle_geo_params.left_mirror().length() * 0.5;
      left_mirror_ = Circle2d(left_mirror_center, left_mirror_radius);

      const Vec2d right_mirror_center =
          rac + vehicle_geo_params.right_mirror().x() * tangent +
          vehicle_geo_params.right_mirror().y() * tangent.Perp();
      const double right_mirror_radius =
          vehicle_geo_params.right_mirror().length() * 0.5;
      right_mirror_ = Circle2d(right_mirror_center, right_mirror_radius);
    }
  }
  virtual ~VehicleShapeBase() = default;
  virtual const Vec2d& center() const = 0;
  virtual std::vector<Vec2d> GetCornersWithBufferCounterClockwise(
      double lat_buffer, double lon_buffer) const = 0;
  virtual double MainBodyDistanceTo(const Segment2d& seg) const = 0;
  virtual double MainBodyDistanceTo(const Polygon2d& polygon) const = 0;
  virtual double MainBodyHasOverlapWithBuffer(const Segment2d& seg,
                                              double lat_buffer,
                                              double lon_buffer) const = 0;
  virtual double MainBodyHasOverlapWithBuffer(const Polygon2d& polygon,
                                              double lat_buffer,
                                              double lon_buffer) const = 0;
  const std::optional<Circle2d>& left_mirror() const { return left_mirror_; }
  const std::optional<Circle2d>& right_mirror() const { return right_mirror_; }

  template <typename T>
  double LeftMirrorDistanceTo(const T& target) const {
    if (!left_mirror_.has_value())
      return std::numeric_limits<double>::infinity();
    return target.DistanceTo(left_mirror_->center()) - left_mirror_->radius();
  }
  template <typename T>
  double RightMirrorDistanceTo(const T& target) const {
    if (!right_mirror_.has_value())
      return std::numeric_limits<double>::infinity();
    return target.DistanceTo(right_mirror_->center()) - right_mirror_->radius();
  }

  template <typename T>
  bool LeftMirrorHasOverlapWithBuffer(const T& target, double buffer) const {
    if (!left_mirror_.has_value()) return false;
    return target.DistanceSquareTo(left_mirror_->center()) <=
           Sqr(buffer + left_mirror_->radius());
  }
  template <typename T>
  bool RightMirrorHasOverlapWithBuffer(const T& target, double buffer) const {
    if (!right_mirror_.has_value()) return false;
    return target.DistanceSquareTo(right_mirror_->center()) <=
           Sqr(buffer + right_mirror_->radius());
  }

  /**
   * @brief Check if the ego vehicle has overlap with a target(segment/polygon)
   * with buffer
   *
   * @param target segment or polygon
   * @param lat_buffer lateral buffer for the ego vehicle
   * @param lon_buffer longitude buffer for the ego vehicle
   * @param consider_mirrors Whether consider the mirrors
   */
  template <typename T>
  bool HasOverlapWithBuffer(const T& target, double lat_buffer,
                            double lon_buffer, bool consider_mirrors) const {
    bool has_overlap =
        MainBodyHasOverlapWithBuffer(target, lat_buffer, lon_buffer);
    if (consider_mirrors) {
      has_overlap = has_overlap ||
                    LeftMirrorHasOverlapWithBuffer(target, lat_buffer) ||
                    RightMirrorHasOverlapWithBuffer(target, lat_buffer);
    }
    return has_overlap;
  }

 protected:
  std::optional<Circle2d> left_mirror_;
  std::optional<Circle2d> right_mirror_;
};
using VehicleShapeBasePtr = std::unique_ptr<VehicleShapeBase>;

class VehicleBoxShape : public VehicleShapeBase {
 public:
  VehicleBoxShape(const VehicleGeometryParamsProto& vehicle_geo_params,
                  const Vec2d& rac, const Vec2d& center, const Vec2d& tangent,
                  double theta, double half_length, double half_width)
      : VehicleShapeBase(vehicle_geo_params, rac, tangent),
        main_body_(Box2d(half_length, half_width, center, theta, tangent)) {}
  const Vec2d& center() const override { return main_body_.center(); }
  std::vector<Vec2d> GetCornersWithBufferCounterClockwise(
      double lat_buffer, double lon_buffer) const override {
    return main_body_.GetCornersWithBufferCounterClockwise(lat_buffer,
                                                           lon_buffer);
  }
  double MainBodyDistanceTo(const Segment2d& seg) const override {
    return main_body_.DistanceTo(seg);
  }
  double MainBodyDistanceTo(const Polygon2d& polygon) const override {
    return polygon.DistanceTo(main_body_);
  }
  double MainBodyHasOverlapWithBuffer(const Segment2d& seg, double lat_buffer,
                                      double lon_buffer) const override {
    return main_body_.HasOverlapWithBuffer(seg, lat_buffer, lon_buffer);
  }
  double MainBodyHasOverlapWithBuffer(const Polygon2d& polygon,
                                      double lat_buffer,
                                      double lon_buffer) const override {
    return polygon.HasOverlapWithBuffer(main_body_, lat_buffer, lon_buffer);
  }

 private:
  Box2d main_body_;
};

class VehicleOctagonShape : public VehicleShapeBase {
 public:
  VehicleOctagonShape(
      const VehicleGeometryParamsProto& vehicle_geo_params,
      const VehicleOctagonModelParamsProto& vehicle_model_params,
      const Vec2d& rac, const Vec2d& center, const Vec2d& tangent, double theta,
      double half_length, double half_width)
      : VehicleShapeBase(vehicle_geo_params, rac, tangent),
        main_body_(VehicleOctagonModel(
            half_length, half_width, center, theta, tangent,
            vehicle_model_params.front_corner_side_length(),
            vehicle_model_params.rear_corner_side_length())) {}
  const Vec2d& center() const override { return main_body_.centroid(); }
  std::vector<Vec2d> GetCornersWithBufferCounterClockwise(
      double lat_buffer, double lon_buffer) const override {
    return main_body_.GetCornersWithBufferCounterClockwise(lat_buffer,
                                                           lon_buffer);
  }
  template <typename T>
  double DistanceTo(const T& target, bool consider_mirrors) const {
    constexpr double kEps = 1e-4;
    const double main_body_distance = main_body_.DistanceTo(target);
    if (main_body_distance < kEps || !consider_mirrors) {
      return main_body_distance;
    }
    const double left_mirror_distance = LeftMirrorDistanceTo(target);
    if (left_mirror_distance < kEps) {
      return 0.0;
    }
    const double right_mirror_distance = RightMirrorDistanceTo(target);
    return std::min(
        {main_body_distance, left_mirror_distance, right_mirror_distance});
  }
  double MainBodyDistanceTo(const Segment2d& seg) const override {
    return main_body_.DistanceTo(seg);
  }
  double MainBodyDistanceTo(const Polygon2d& polygon) const override {
    return main_body_.DistanceTo(polygon);
  }
  double MainBodyHasOverlapWithBuffer(const Segment2d& seg, double lat_buffer,
                                      double lon_buffer) const override {
    return main_body_.HasOverlapWithBuffer(seg, lat_buffer, lon_buffer);
  }
  double MainBodyHasOverlapWithBuffer(const Polygon2d& polygon,
                                      double lat_buffer,
                                      double lon_buffer) const override {
    return main_body_.HasOverlapWithBuffer(polygon, lat_buffer, lon_buffer);
  }

 private:
  VehicleOctagonModel main_body_;
};
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_VEHICLE_SHAPE_H_
