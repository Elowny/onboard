// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include "onboard/planner/common/vehicle_octagon_model.h"

#include <algorithm>
#include <limits>

#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/gjk2d.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace planner {

std::vector<Vec2d> VehicleOctagonModel::GetCornersWithBufferCounterClockwise(
    double lat_buffer, double lon_buffer) const {
  const double corner_buffer =
      0.5 * kCornerBufferFactor * (lat_buffer + lon_buffer);
  const Vec2d dir = box_.tangent();
  const Vec2d normal = dir.Perp();
  const double half_length = box_.half_length() + lon_buffer;
  const double half_width = box_.half_width() + lat_buffer;
  const double front_corner_side_length =
      front_corner_side_length_ + corner_buffer;
  const double rear_corner_side_length =
      rear_corner_side_length_ + corner_buffer;
  const Vec2d h = dir * half_length;
  const Vec2d h_f = h - dir * front_corner_side_length;
  const Vec2d h_r = h - dir * rear_corner_side_length;
  const Vec2d w = normal * half_width;
  const Vec2d w_f = w - normal * front_corner_side_length;
  const Vec2d w_r = w - normal * rear_corner_side_length;
  return {
      centroid() + h_f - w, centroid() + h - w_f, centroid() + h + w_f,
      centroid() + h_f + w, centroid() - h_r + w, centroid() - h + w_r,
      centroid() - h - w_r, centroid() - h_r - w,
  };
}

bool VehicleOctagonModel::HasOverlap(const Segment2d& line_segment) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;

  const Vec2d line_start = (line_segment.start() - centroid())
                               .Rotate(box_.cos_heading(), -box_.sin_heading());
  const Vec2d line_end = (line_segment.end() - centroid())
                             .Rotate(box_.cos_heading(), -box_.sin_heading());
  if (std::min(line_start.x(), line_end.x()) > box_.half_length() ||
      std::max(line_start.x(), line_end.x()) < -box_.half_length() ||
      std::min(line_start.y(), line_end.y()) > box_.half_width() ||
      std::max(line_start.y(), line_end.y()) < -box_.half_width()) {
    return false;
  }

  if (line_segment.length() <= kEpsilon) {
    return IsPointIn(line_segment.start());
  }

  if (IsPointIn(line_segment.start())) {
    return true;
  }
  if (IsPointIn(line_segment.end())) {
    return true;
  }
  const auto calc_extreme_point =
      [this, &line_segment](const Vec2d& dir_vec) -> Vec2d {
    const auto& extreme_point_of_segment =
        dir_vec.dot(line_segment.unit_direction()) > 0.0 ? line_segment.start()
                                                         : line_segment.end();
    return points_[ExtremePoint(dir_vec)] - extreme_point_of_segment;
  };
  const auto init_point = centroid() - line_segment.center();
  return Gjk2d::HasOverlap(init_point, calc_extreme_point, kEpsilon,
                           kMaxIterations);
}

bool VehicleOctagonModel::HasOverlapWithBuffer(
    const Segment2d& line_segment, double lateral_buffer,
    double longitudinal_buffer) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;

  const Vec2d line_start = (line_segment.start() - centroid())
                               .Rotate(box_.cos_heading(), -box_.sin_heading());
  const Vec2d line_end = (line_segment.end() - centroid())
                             .Rotate(box_.cos_heading(), -box_.sin_heading());
  const double half_length_with_buffer =
      box_.half_length() + longitudinal_buffer;
  const double half_width_with_buffer = box_.half_width() + lateral_buffer;
  if (std::min(line_start.x(), line_end.x()) > half_length_with_buffer ||
      std::max(line_start.x(), line_end.x()) < -half_length_with_buffer ||
      std::min(line_start.y(), line_end.y()) > half_width_with_buffer ||
      std::max(line_start.y(), line_end.y()) < -half_width_with_buffer) {
    return false;
  }

  if (UNLIKELY(lateral_buffer < 0.0 || longitudinal_buffer < 0.0)) {
    return GetModelWithBuffer(lateral_buffer, longitudinal_buffer)
        .HasOverlap(line_segment);
  }
  // IsPointIn is cheap, so we can still use it as early exit condition.
  if (IsPointIn(line_segment.start())) {
    return true;
  }
  if (IsPointIn(line_segment.end())) {
    return true;
  }
  lateral_buffer = std::max(lateral_buffer, kEpsilon);
  longitudinal_buffer = std::max(longitudinal_buffer, kEpsilon);
  // NOLINTNEXTLINE(readability-identifier-naming)
  const Mat2d C =
      (Mat2d() << longitudinal_buffer, 0.0, 0.0, lateral_buffer).finished();
  // NOLINTNEXTLINE(readability-identifier-naming)
  const Mat2d rotated_C = rotation_matrix_ * C * rotation_matrix_inv_;
  // NOLINTNEXTLINE(readability-identifier-naming)
  const Mat2d C_inv =
      (Mat2d() << 1.0 / longitudinal_buffer, 0.0, 0.0, 1.0 / lateral_buffer)
          .finished();
  // NOLINTNEXTLINE(readability-identifier-naming)
  const Mat2d rotated_C_inv = rotation_matrix_ * C_inv * rotation_matrix_inv_;
  const auto calc_extreme_point =
      [this, &line_segment, &rotated_C,
       &rotated_C_inv](const Vec2d& dir_vec) -> Vec2d {
    const auto& extreme_point_of_segment =
        dir_vec.dot(line_segment.unit_direction()) > 0.0 ? line_segment.start()
                                                         : line_segment.end();
    const Vec2d perp = rotated_C_inv * dir_vec.Perp();
    const Vec2d vec = -perp.Perp();
    const Vec2d ellipse_point = rotated_C * vec.Unit();
    return points_[ExtremePoint(dir_vec)] + ellipse_point -
           extreme_point_of_segment;
  };
  const auto init_point = centroid() - line_segment.center();
  return Gjk2d::HasOverlap(init_point, calc_extreme_point, kEpsilon,
                           kMaxIterations);
}

bool VehicleOctagonModel::HasOverlap(const Box2d& box) const {
  QCHECK_GE(points_.size(), 3);
  const auto corners = box.GetCornersCounterClockwise();
  double min_x = corners[0].x();
  double max_x = corners[0].x();
  double min_y = corners[0].y();
  double max_y = corners[0].y();
  for (int i = 1; i < corners.size(); ++i) {
    min_x = std::min(min_x, corners[i].x());
    max_x = std::max(max_x, corners[i].x());
    min_y = std::min(min_y, corners[i].y());
    max_y = std::max(max_y, corners[i].y());
  }
  if (min_x > this->max_x() || max_x < this->min_x() || min_y > this->max_y() ||
      max_y < this->min_y()) {
    return false;
  }
  if (IsPointIn(corners[0])) {
    return true;
  }
  for (const auto& segment : line_segments()) {
    if (box.HasOverlap(segment)) {
      return true;
    }
  }
  return false;
}

bool VehicleOctagonModel::HasOverlap(const Polygon2d& polygon) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;

  if (polygon.max_x() < min_x() || polygon.min_x() > max_x() ||
      polygon.max_y() < min_y() || polygon.min_y() > max_y()) {
    return false;
  }

  if (IsPointIn(polygon.points()[0])) {
    return true;
  }

  if (polygon.is_convex()) {
    const auto calc_extreme_point = [this,
                                     &polygon](const Vec2d& dir_vec) -> Vec2d {
      return points_[ExtremePoint(dir_vec)] -
             polygon.points()[polygon.ExtremePoint(-dir_vec)];
    };
    const auto init_point = centroid() - polygon.centroid();
    return Gjk2d::HasOverlap(init_point, calc_extreme_point, kEpsilon,
                             kMaxIterations);
  }

  for (const auto& segment : polygon.line_segments()) {
    if (HasOverlap(segment)) {
      return true;
    }
  }
  return false;
}

bool VehicleOctagonModel::HasOverlapWithBuffer(
    const Polygon2d& polygon, double lateral_buffer,
    double longitudinal_buffer) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;

  if (polygon.is_convex() && lateral_buffer >= 0.0 &&
      longitudinal_buffer >= 0.0) {
    lateral_buffer = std::max(lateral_buffer, kEpsilon);
    longitudinal_buffer = std::max(longitudinal_buffer, kEpsilon);
    const double max_buffer = std::max(longitudinal_buffer, lateral_buffer);
    if (polygon.max_x() < min_x() - max_buffer ||
        polygon.min_x() > max_x() + max_buffer ||
        polygon.max_y() < min_y() - max_buffer ||
        polygon.min_y() > max_y() + max_buffer) {
      return false;
    }
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d C =
        (Mat2d() << longitudinal_buffer, 0.0, 0.0, lateral_buffer).finished();
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d rotated_C = rotation_matrix_ * C * rotation_matrix_inv_;
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d C_inv =
        (Mat2d() << 1.0 / longitudinal_buffer, 0.0, 0.0, 1.0 / lateral_buffer)
            .finished();
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d rotated_C_inv = rotation_matrix_ * C_inv * rotation_matrix_inv_;

    const auto calc_extreme_point =
        [this, &polygon, &rotated_C,
         &rotated_C_inv](const Vec2d& dir_vec) -> Vec2d {
      const Vec2d perp = rotated_C_inv * dir_vec.Perp();
      const Vec2d vec = -perp.Perp();
      const Vec2d ellipse_point = rotated_C * vec.Unit();
      return points_[ExtremePoint(dir_vec)] + ellipse_point -
             polygon.points()[polygon.ExtremePoint(-dir_vec)];
    };
    const auto init_point = centroid() - polygon.centroid();
    return Gjk2d::HasOverlap(init_point, calc_extreme_point, kEpsilon,
                             kMaxIterations);
  }
  return GetModelWithBuffer(lateral_buffer, longitudinal_buffer)
      .HasOverlap(polygon);
}

bool VehicleOctagonModel::HasOverlap(const VehicleOctagonModel& octagon) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;

  if (octagon.max_x() < min_x() || octagon.min_x() > max_x() ||
      octagon.max_y() < min_y() || octagon.min_y() > max_y()) {
    return false;
  }

  if (IsPointIn(octagon.points()[0])) {
    return true;
  }

  const auto calc_extreme_point = [this,
                                   &octagon](const Vec2d& dir_vec) -> Vec2d {
    return points_[ExtremePoint(dir_vec)] -
           octagon.points()[octagon.ExtremePoint(-dir_vec)];
  };
  const auto init_point = centroid() - octagon.centroid();
  return Gjk2d::HasOverlap(init_point, calc_extreme_point, kEpsilon,
                           kMaxIterations);
}

double VehicleOctagonModel::DistanceTo(const Vec2d& point) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;

  if (IsPointIn(point)) {
    return 0.0;
  }

  const auto calc_extreme_point = [this,
                                   &point](const Vec2d& dir_vec) -> Vec2d {
    return points_[ExtremePoint(dir_vec)] - point;
  };
  const auto init_point = centroid() - point;
  return Gjk2d::DistanceTo(init_point, calc_extreme_point, kEpsilon,
                           kMaxIterations);
}

double VehicleOctagonModel::DistanceTo(const Segment2d& line_segment) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;
  if (IsPointIn(line_segment.center())) {
    return 0.0;
  }
  if (line_segment.length() <= kEpsilon) {
    return DistanceTo(line_segment.start());
  }
  const auto calc_extreme_point =
      [this, &line_segment](const Vec2d& dir_vec) -> Vec2d {
    const auto& extreme_point_of_segment =
        dir_vec.dot(line_segment.unit_direction()) > 0.0 ? line_segment.start()
                                                         : line_segment.end();
    return points_[ExtremePoint(dir_vec)] - extreme_point_of_segment;
  };
  const auto init_point = centroid() - line_segment.center();
  return Gjk2d::DistanceTo(init_point, calc_extreme_point, kEpsilon,
                           kMaxIterations);
}

double VehicleOctagonModel::DistanceToWithBuffer(
    const Segment2d& line_segment, double lateral_buffer,
    double longitudinal_buffer) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;
  // still can use this to pre-check
  if (IsPointIn(line_segment.center())) {
    return 0.0;
  }
  if (lateral_buffer >= 0.0 && longitudinal_buffer >= 0.0) {
    lateral_buffer = std::max(lateral_buffer, kEpsilon);
    longitudinal_buffer = std::max(longitudinal_buffer, kEpsilon);
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d C =
        (Mat2d() << longitudinal_buffer, 0.0, 0.0, lateral_buffer).finished();
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d rotated_C = rotation_matrix_ * C * rotation_matrix_inv_;
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d C_inv =
        (Mat2d() << 1.0 / longitudinal_buffer, 0.0, 0.0, 1.0 / lateral_buffer)
            .finished();
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d rotated_C_inv = rotation_matrix_ * C_inv * rotation_matrix_inv_;

    const auto calc_extreme_point =
        [this, &line_segment, &rotated_C,
         &rotated_C_inv](const Vec2d& dir_vec) -> Vec2d {
      const auto& extreme_point_of_segment =
          dir_vec.dot(line_segment.unit_direction()) > 0.0
              ? line_segment.start()
              : line_segment.end();
      const Vec2d perp = rotated_C_inv * dir_vec.Perp();
      const Vec2d vec = -perp.Perp();
      const Vec2d ellipse_point = rotated_C * vec.Unit();
      return points_[ExtremePoint(dir_vec)] + ellipse_point -
             extreme_point_of_segment;
    };
    const auto init_point = centroid() - line_segment.center();
    return Gjk2d::DistanceTo(init_point, calc_extreme_point, kEpsilon,
                             kMaxIterations);
  }
  return GetModelWithBuffer(lateral_buffer, longitudinal_buffer)
      .DistanceTo(line_segment);
}

double VehicleOctagonModel::DistanceTo(const Box2d& box) const {
  if (IsPointIn(box.center())) {
    return 0.0;
  }
  if (box.IsPointIn(points_[0])) {
    return 0.0;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, box.DistanceTo(line_segments_[i]));
  }
  return distance;
}

double VehicleOctagonModel::DistanceTo(const Polygon2d& polygon) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;

  if (polygon.is_convex()) {
    const auto calc_extreme_point = [this,
                                     &polygon](const Vec2d& dir_vec) -> Vec2d {
      return points_[ExtremePoint(dir_vec)] -
             polygon.points()[polygon.ExtremePoint(-dir_vec)];
    };
    const auto init_point = centroid() - polygon.centroid();
    return Gjk2d::DistanceTo(init_point, calc_extreme_point, kEpsilon,
                             kMaxIterations);
  }

  if (IsPointIn(polygon.points()[0])) {
    return 0.0;
  }
  if (polygon.IsPointIn(points_[0])) {
    return 0.0;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, polygon.DistanceTo(line_segments_[i]));
  }
  return distance;
}

double VehicleOctagonModel::DistanceToWithBuffer(
    const Polygon2d& polygon, double lateral_buffer,
    double longitudinal_buffer) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;

  if (polygon.is_convex() && longitudinal_buffer >= 0.0 &&
      lateral_buffer >= 0.0) {
    lateral_buffer = std::max(lateral_buffer, kEpsilon);
    longitudinal_buffer = std::max(longitudinal_buffer, kEpsilon);
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d C =
        (Mat2d() << longitudinal_buffer, 0.0, 0.0, lateral_buffer).finished();
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d rotated_C = rotation_matrix_ * C * rotation_matrix_inv_;
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d C_inv =
        (Mat2d() << 1.0 / longitudinal_buffer, 0.0, 0.0, 1.0 / lateral_buffer)
            .finished();
    // NOLINTNEXTLINE(readability-identifier-naming)
    const Mat2d rotated_C_inv = rotation_matrix_ * C_inv * rotation_matrix_inv_;

    const auto calc_extreme_point =
        [this, &polygon, &rotated_C,
         &rotated_C_inv](const Vec2d& dir_vec) -> Vec2d {
      const Vec2d perp = rotated_C_inv * dir_vec.Perp();
      const Vec2d vec = -perp.Perp();
      const Vec2d ellipse_point = rotated_C * vec.Unit();
      return points_[ExtremePoint(dir_vec)] + ellipse_point -
             polygon.points()[polygon.ExtremePoint(-dir_vec)];
    };
    const auto init_point = centroid() - polygon.centroid();
    return Gjk2d::DistanceTo(init_point, calc_extreme_point, kEpsilon,
                             kMaxIterations);
  }
  return GetModelWithBuffer(lateral_buffer, longitudinal_buffer)
      .DistanceTo(polygon);
}

double VehicleOctagonModel::DistanceTo(
    const VehicleOctagonModel& octagon) const {
  constexpr double kEpsilon = 1e-8;
  constexpr int kMaxIterations = 10;

  const auto calc_extreme_point = [this,
                                   &octagon](const Vec2d& dir_vec) -> Vec2d {
    return points_[ExtremePoint(dir_vec)] -
           octagon.points()[octagon.ExtremePoint(-dir_vec)];
  };
  const auto init_point = centroid() - octagon.centroid();
  return Gjk2d::DistanceTo(init_point, calc_extreme_point, kEpsilon,
                           kMaxIterations);
}

void VehicleOctagonModel::BuildFromBox() {
  constexpr int kOctagonPointsNum = 8;
  num_points_ = kOctagonPointsNum;
  area_ = box_.area() - Sqr(front_corner_side_length_) -
          Sqr(rear_corner_side_length_);

  // Construct points and line_segments in ccw order.
  points_.reserve(num_points_);
  line_segments_.reserve(num_points_);
  const Vec2d dir = box_.tangent();
  const Vec2d normal = dir.Perp();
  // Direction of corners.
  const Vec2d dir1 = (dir + normal) * M_SQRT1_2;
  const Vec2d dir2 = dir1.Perp();
  // Length of corners.
  const double front_corner_length = front_corner_side_length_ * M_SQRT2;
  const double rear_corner_length = rear_corner_side_length_ * M_SQRT2;
  // Firstly we construct the point of front right corner(point on the right
  // edge).
  points_.push_back(centroid() +
                    (box_.half_length() - front_corner_side_length_) * dir -
                    box_.half_width() * normal);
  // Then we construct all the line segments.
  line_segments_.emplace_back(front_corner_length, points_[0], dir1);
  line_segments_.emplace_back(box_.width() - 2.0 * front_corner_side_length_,
                              line_segments_[0].end(), normal);
  line_segments_.emplace_back(front_corner_length, line_segments_[1].end(),
                              dir2);
  line_segments_.emplace_back(
      box_.length() - front_corner_side_length_ - rear_corner_side_length_,
      line_segments_[2].end(), -dir);
  line_segments_.emplace_back(rear_corner_length, line_segments_[3].end(),
                              -dir1);
  line_segments_.emplace_back(box_.width() - 2.0 * rear_corner_side_length_,
                              line_segments_[4].end(), -normal);
  line_segments_.emplace_back(rear_corner_length, line_segments_[5].end(),
                              -dir2);
  line_segments_.emplace_back(line_segments_[3].length(),
                              line_segments_[6].end(), dir);
  // Finally we fill the rest points.
  for (int i = 0; i + 1 < num_points_; ++i) {
    points_.push_back(line_segments_[i].end());
  }

  aabox_ = AABox2d(points_);
}

int VehicleOctagonModel::ExtremePoint(const Vec2d& direction_vec) const {
  enum Corner {
    kFrontRightRight = 0,
    kFrontRightFront = 1,
    kFrontLeftFront = 2,
    kFrontLeftLeft = 3,
    kRearLeftLeft = 4,
    kRearLeftRear = 5,
    kRearRightRear = 6,
    kRearRightRight = 7,
  };

  if (direction_vec.x() == 0.0 && direction_vec.y() == 0.0) {
    return kFrontRightRight;
  }
  const auto rotated_dir =
      direction_vec.Rotate(box_.cos_heading(), -box_.sin_heading());
  const auto x = rotated_dir.x();
  const auto y = rotated_dir.y();
  if (x > 0.0) {
    if (x > std::fabs(y)) {
      return y > 0.0 ? kFrontLeftFront : kFrontRightFront;
    }
    return y > 0.0 ? kFrontLeftLeft : kFrontRightRight;
  }
  if (-x > std::fabs(y)) {
    return y > 0.0 ? kRearLeftRear : kRearRightRear;
  }
  return y > 0.0 ? kRearLeftLeft : kRearRightRight;
}
}  // namespace planner
}  // namespace qcraft
