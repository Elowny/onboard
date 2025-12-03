#ifndef ONBOARD_PLANNER_COMMON_VEHICLE_OCTAGON_MODEL_H
#define ONBOARD_PLANNER_COMMON_VEHICLE_OCTAGON_MODEL_H
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <cmath>
#include <vector>

#include "onboard/math/eigen.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {

/**
 * @class VehicleOctagonModel
 * @brief The class of VehicleOctagonModel in 2-D, it's a box with four corners
 * that have been cut off, please refer to
 * https://qcraft.feishu.cn/docs/doccnfgFN3egXvI1wKtvC1JMTfb#UQ3Wk7.
 */

class VehicleOctagonModel {
 public:
  static constexpr double kCornerBufferFactor = 2.0 - M_SQRT2;

  VehicleOctagonModel() = default;

  VehicleOctagonModel(const Box2d& box, double front_corner_side_length,
                      double rear_corner_side_length)
      : front_corner_side_length_(front_corner_side_length),
        rear_corner_side_length_(rear_corner_side_length),
        box_(box),
        rotation_matrix_((Mat2d() << box_.cos_heading(), -box_.sin_heading(),
                          box_.sin_heading(), box_.cos_heading())
                             .finished()),
        rotation_matrix_inv_(rotation_matrix_.transpose()) {
    BuildFromBox();
  }

  VehicleOctagonModel(double half_length, double half_width,
                      const Vec2d& center, double heading, const Vec2d& tangent,
                      double front_corner_side_length,
                      double rear_corner_side_length)
      : front_corner_side_length_(front_corner_side_length),
        rear_corner_side_length_(rear_corner_side_length),
        box_(half_length, half_width, center, heading, tangent),
        rotation_matrix_(
            (Mat2d() << tangent.x(), -tangent.y(), tangent.y(), tangent.x())
                .finished()),
        rotation_matrix_inv_(rotation_matrix_.transpose()) {
    BuildFromBox();
  }

  VehicleOctagonModel(const Vec2d& center, double heading, double length,
                      double width, double front_corner_side_length,
                      double rear_corner_side_length)
      : front_corner_side_length_(front_corner_side_length),
        rear_corner_side_length_(rear_corner_side_length),
        box_(center, heading, length, width),
        rotation_matrix_((Mat2d() << box_.cos_heading(), -box_.sin_heading(),
                          box_.sin_heading(), box_.cos_heading())
                             .finished()),
        rotation_matrix_inv_(rotation_matrix_.transpose()) {
    BuildFromBox();
  }

  [[nodiscard]] VehicleOctagonModel GetModelWithBuffer(
      double lateral_buffer, double longitudinal_buffer) const {
    const double corner_buffer =
        0.5 * kCornerBufferFactor * (lateral_buffer + longitudinal_buffer);
    return VehicleOctagonModel(box().half_length() + longitudinal_buffer,
                               box().half_width() + lateral_buffer,
                               box().center(), box().heading(), box().tangent(),
                               front_corner_side_length() + corner_buffer,
                               rear_corner_side_length() + corner_buffer);
  }

  std::vector<Vec2d> GetCornersWithBufferCounterClockwise(
      double lat_buffer, double lon_buffer) const;

  inline bool IsPointIn(const Vec2d& point) const;

  inline bool IsPointInWithBuffer(const Vec2d& point, double lat_buffer,
                                  double lon_buffer) const;

  bool HasOverlap(const Segment2d& line_segment) const;

  bool HasOverlapWithBuffer(const Segment2d& line_segment,
                            double lateral_buffer,
                            double longitudinal_buffer) const;

  bool HasOverlap(const Box2d& box) const;

  bool HasOverlap(const Polygon2d& polygon) const;

  bool HasOverlapWithBuffer(const Polygon2d& polygon, double lateral_buffer,
                            double longitudinal_buffer) const;

  bool HasOverlap(const VehicleOctagonModel& octagon) const;

  double DistanceTo(const Vec2d& point) const;

  double DistanceTo(const Segment2d& line_segment) const;

  double DistanceToWithBuffer(const Segment2d& line_segment,
                              double lateral_buffer,
                              double longitudinal_buffer) const;

  double DistanceTo(const Box2d& box) const;

  double DistanceTo(const Polygon2d& polygon) const;

  double DistanceToWithBuffer(const Polygon2d& polygon, double lateral_buffer,
                              double longitudinal_buffer) const;

  double DistanceTo(const VehicleOctagonModel& octagon) const;

  int ExtremePoint(const Vec2d& direction_vec) const;

  const Box2d& box() const { return box_; }
  double front_corner_side_length() const { return front_corner_side_length_; }
  double rear_corner_side_length() const { return rear_corner_side_length_; }
  const std::vector<Vec2d>& points() const { return points_; }
  const std::vector<Segment2d>& line_segments() const { return line_segments_; }
  double min_x() const { return aabox_.min_x(); }
  double max_x() const { return aabox_.max_x(); }
  double min_y() const { return aabox_.min_y(); }
  double max_y() const { return aabox_.max_y(); }
  const Vec2d& centroid() const { return box_.center(); }

 private:
  void BuildFromBox();

  double front_corner_side_length_;
  double rear_corner_side_length_;
  int num_points_ = 0;
  std::vector<Vec2d> points_;
  std::vector<Segment2d> line_segments_;
  double area_ = 0.0;
  AABox2d aabox_;
  Box2d box_;
  Mat2d rotation_matrix_;
  Mat2d rotation_matrix_inv_;
};

inline bool VehicleOctagonModel::IsPointIn(const Vec2d& point) const {
  if (!aabox_.IsPointIn(point)) return false;
  const auto rotated_pt =
      (point - box_.center()).Rotate(box_.cos_heading(), -box_.sin_heading());
  const double x = rotated_pt.x();
  const double abs_y = std::fabs(rotated_pt.y());
  const double half_length = box_.half_length();
  const double half_width = box_.half_width();
  // Outside box.
  if (x > half_length || x < -half_length || abs_y > half_width) {
    return false;
  }
  // Whether in four corner triangles.
  if (x > 0.0) {
    return x + abs_y <= half_length + half_width - front_corner_side_length_;
  }
  return -x + abs_y <= half_length + half_width - rear_corner_side_length_;
}

inline bool VehicleOctagonModel::IsPointInWithBuffer(const Vec2d& point,
                                                     double lat_buffer,
                                                     double lon_buffer) const {
  const auto rotated_pt =
      (point - box_.center()).Rotate(box_.cos_heading(), -box_.sin_heading());
  const double x = rotated_pt.x();
  const double abs_y = std::fabs(rotated_pt.y());
  const double corner_buffer =
      0.5 * kCornerBufferFactor * (lat_buffer + lon_buffer);
  const double half_length = box_.half_length() + lon_buffer;
  const double half_width = box_.half_width() + lat_buffer;
  const double front_corner_side_length =
      front_corner_side_length_ + corner_buffer;
  const double rear_corner_side_length =
      rear_corner_side_length_ + corner_buffer;
  // Outside box.
  if (x > half_length || x < -half_length || abs_y > half_width) {
    return false;
  }
  // Whether in four corner triangles.
  if (x > 0.0) {
    return x + abs_y <= half_length + half_width - front_corner_side_length;
  }
  return -x + abs_y <= half_length + half_width - rear_corner_side_length;
}
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMMON_VEHICLE_OCTAGON_MODEL_H
