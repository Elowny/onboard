#ifndef ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_GEOMETRY_PATH_COLLISION_CHECKER_H  // NOLINT
#define ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_GEOMETRY_PATH_COLLISION_CHECKER_H  // NOLINT

#include <cmath>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/circle_path_overlap.h"
#include "onboard/planner/common/vehicle_octagon_model.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// Please refer to
// https://qcraft.feishu.cn/docs/doccnfgFN3egXvI1wKtvC1JMTfb#oB6y5o.
class GeometryPathCollisonChecker {
 public:
  GeometryPathCollisonChecker(
      const VehicleGeometryParamsProto& veh_geo_params,
      const VehicleOctagonModelParamsProto& vehicle_model_params,
      const GeometryMethodPoint& start, const GeometryMethodPoint& end,
      GeometryPathType type, double length, double kappa)
      : start_(start),
        end_(end),
        type_(type),
        length_(length),
        kappa_(kappa),
        vehicle_half_length_(0.5 * veh_geo_params.length()),
        vehicle_half_width_(0.5 * veh_geo_params.width()),
        front_corner_side_length_(
            vehicle_model_params.front_corner_side_length()),
        rear_corner_side_length_(
            vehicle_model_params.rear_corner_side_length()),
        front_edge_to_center_(veh_geo_params.front_edge_to_center()),
        back_edge_to_center_(veh_geo_params.back_edge_to_center()),
        mirror_offset_x_(vehicle_model_params.mirror_offset_x()),
        mirror_offset_y_(vehicle_model_params.mirror_offset_y()),
        mirror_radius_(vehicle_model_params.mirror_radius()) {}

  virtual ~GeometryPathCollisonChecker() = default;

  const GeometryMethodPoint& start() const { return start_; }

  const GeometryMethodPoint& end() const { return end_; }

  GeometryPathType type() const { return type_; }

  double length() { return length_; }

  double kappa() { return kappa_; }

  virtual bool HasOverlapWithBuffer(const Vec2d& point, double lateral_buffer,
                                    double longitudinal_buffer,
                                    bool consider_mirror) const = 0;

  virtual bool HasOverlapWithBuffer(const Segment2d& segment,
                                    double lateral_buffer,
                                    double longitudinal_buffer,
                                    bool consider_mirror) const = 0;

  virtual bool HasOverlapWithBuffer(const Polygon2d& polygon,
                                    double lateral_buffer,
                                    double longitudinal_buffer,
                                    bool consider_mirror) const = 0;

 protected:
  // Path info.
  GeometryMethodPoint start_;
  GeometryMethodPoint end_;
  GeometryPathType type_;
  double length_;
  double kappa_;

  // Vehicle model params with buffer.
  double vehicle_half_length_;
  double vehicle_half_width_;
  double front_corner_side_length_;
  double rear_corner_side_length_;
  double front_edge_to_center_;
  double back_edge_to_center_;
  double mirror_offset_x_;
  double mirror_offset_y_;
  double mirror_radius_;
};

class LinePathCollisionChecker : public GeometryPathCollisonChecker {
 public:
  LinePathCollisionChecker(
      const VehicleGeometryParamsProto& veh_geo_params,
      const VehicleOctagonModelParamsProto& vehicle_model_params,
      const GeometryMethodPoint& start, const GeometryMethodPoint& end,
      GeometryPathType type, double length, double kappa)
      : GeometryPathCollisonChecker(veh_geo_params, vehicle_model_params, start,
                                    end, type, length, kappa) {
    const double offset = 0.5 * (front_edge_to_center_ - back_edge_to_center_);
    const Vec2d center = start_.pos + (0.5 * length_ + offset) * start_.tangent;
    path_octagon_ = VehicleOctagonModel(
        vehicle_half_length_ + 0.5 * std::abs(length_), vehicle_half_width_,
        center, start_.theta, start_.tangent, front_corner_side_length_,
        rear_corner_side_length_);
    const Vec2d mirror_box_center =
        0.5 * (start_.pos + end_.pos) + start_.tangent * mirror_offset_x_;
    mirror_path_box_ = Box2d(0.5 * std::abs(length_) + mirror_radius_,
                             mirror_offset_y_ + mirror_radius_,
                             mirror_box_center, start_.theta, start_.tangent);
  }

  bool HasOverlapWithBuffer(const Vec2d& point, double lateral_buffer,
                            double longitudinal_buffer,
                            bool consider_mirror) const override;

  bool HasOverlapWithBuffer(const Segment2d& segment, double lateral_buffer,
                            double longitudinal_buffer,
                            bool consider_mirror) const override;

  bool HasOverlapWithBuffer(const Polygon2d& polygon, double lateral_buffer,
                            double longitudinal_buffer,
                            bool consider_mirror) const override;

 private:
  VehicleOctagonModel path_octagon_;
  Box2d mirror_path_box_;
};

class CirclePathCollisionChecker : public GeometryPathCollisonChecker {
 public:
  CirclePathCollisionChecker(
      const VehicleGeometryParamsProto& veh_geo_params,
      const VehicleOctagonModelParamsProto& vehicle_model_params,
      const GeometryMethodPoint& start, const GeometryMethodPoint& end,
      GeometryPathType type, double length, double kappa, bool use_fast_overlap)
      : GeometryPathCollisonChecker(veh_geo_params, vehicle_model_params, start,
                                    end, type, length, kappa),
        use_fast_overlap_(use_fast_overlap) {
    QCHECK_GT(kappa, 0.0);
    ccw_ = (type_ == GeometryPathType::LEFT) ^ (length_ < 0.0);
    const double angle = std::abs(length_) * kappa_;
    monotonic_cosine_ = MonotonicCosine(angle, /*use_fast_math=*/true);
    const double factor =
        (type_ == GeometryPathType::LEFT ? 1.0 / kappa_ : -1.0 / kappa_);
    center_ = start_.pos + factor * start_.tangent.Perp();

    // Start octagon and end octagon.
    const double offset = 0.5 * (front_edge_to_center_ - back_edge_to_center_);
    start_octagon_ = VehicleOctagonModel(
        vehicle_half_length_, vehicle_half_width_,
        start_.pos + offset * start_.tangent, start_.theta, start_.tangent,
        front_corner_side_length_, rear_corner_side_length_);
    end_octagon_ = VehicleOctagonModel(
        vehicle_half_length_, vehicle_half_width_,
        end_.pos + offset * end_.tangent, end_.theta, end_.tangent,
        front_corner_side_length_, rear_corner_side_length_);
  }

  bool HasOverlapWithBuffer(const Vec2d& point, double lateral_buffer,
                            double longitudinal_buffer,
                            bool consider_mirror) const override;

  bool HasOverlapWithBuffer(const Segment2d& segment, double lateral_buffer,
                            double longitudinal_buffer,
                            bool consider_mirror) const override;

  bool HasOverlapWithBuffer(const Polygon2d& polygon, double lateral_buffer,
                            double longitudinal_buffer,
                            bool consider_mirror) const override;

 private:
  bool ccw_;
  double monotonic_cosine_;
  Vec2d center_;
  VehicleOctagonModel start_octagon_;
  VehicleOctagonModel end_octagon_;

  bool use_fast_overlap_;

  // When a segment of vehicle rotating with the path, it will go through an
  // area, check if the area has overlap with boundary.
  bool HasOverlap(const Segment2d& vehicle_segment,
                  const Segment2d& boundary) const;
};
}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_GEOMETRY_PATH_COLLISION_CHECKER_H
