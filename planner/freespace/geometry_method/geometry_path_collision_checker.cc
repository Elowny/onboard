#include "onboard/planner/freespace/geometry_method/geometry_path_collision_checker.h"

#include <vector>

#include "onboard/math/geometry/box2d.h"

namespace qcraft {
namespace planner {

bool LinePathCollisionChecker::HasOverlapWithBuffer(
    const Vec2d& point, double lateral_buffer, double longitudinal_buffer,
    bool consider_mirror) const {
  if (path_octagon_.IsPointInWithBuffer(point, lateral_buffer,
                                        longitudinal_buffer)) {
    return true;
  }
  if (!consider_mirror) return false;
  return mirror_path_box_.IsPointInWithBuffer(point, lateral_buffer,
                                              longitudinal_buffer);
}

bool LinePathCollisionChecker::HasOverlapWithBuffer(
    const Segment2d& segment, double lateral_buffer, double longitudinal_buffer,
    bool consider_mirror) const {
  if (path_octagon_.HasOverlapWithBuffer(segment, lateral_buffer,
                                         longitudinal_buffer)) {
    return true;
  }
  if (!consider_mirror) return false;
  return mirror_path_box_.HasOverlapWithBuffer(segment, lateral_buffer,
                                               longitudinal_buffer);
}

bool LinePathCollisionChecker::HasOverlapWithBuffer(
    const Polygon2d& polygon, double lateral_buffer, double longitudinal_buffer,
    bool consider_mirror) const {
  if (path_octagon_.HasOverlapWithBuffer(polygon, lateral_buffer,
                                         longitudinal_buffer)) {
    return true;
  }
  if (!consider_mirror) return false;
  return polygon.HasOverlapWithBuffer(mirror_path_box_, lateral_buffer,
                                      longitudinal_buffer);
}

bool CirclePathCollisionChecker::HasOverlap(const Segment2d& vehicle_segment,
                                            const Segment2d& boundary) const {
  if (monotonic_cosine_ <=
      MonotonicCosineOfPointOverlapWithSegment(vehicle_segment.start(), center_,
                                               ccw_, boundary)) {
    return true;
  }
  if (monotonic_cosine_ <=
      MonotonicCosineOfPointOverlapWithSegment(vehicle_segment.end(), center_,
                                               ccw_, boundary)) {
    return true;
  }
  if (monotonic_cosine_ <=
      MonotonicCosineOfPointOverlapWithSegment(boundary.start(), center_, !ccw_,
                                               vehicle_segment)) {
    return true;
  }
  if (monotonic_cosine_ <=
      MonotonicCosineOfPointOverlapWithSegment(boundary.end(), center_, !ccw_,
                                               vehicle_segment)) {
    return true;
  }
  return false;
}

bool CirclePathCollisionChecker::HasOverlapWithBuffer(
    const Vec2d& point, double lateral_buffer, double longitudinal_buffer,
    bool consider_mirror) const {
  if (start_octagon_.IsPointInWithBuffer(point, lateral_buffer,
                                         longitudinal_buffer)) {
    return true;
  }
  if (end_octagon_.IsPointInWithBuffer(point, lateral_buffer,
                                       longitudinal_buffer)) {
    return true;
  }

  if (consider_mirror) {
    const double half_length = mirror_radius_ + lateral_buffer;
    const double half_width =
        mirror_offset_y_ + mirror_radius_ + lateral_buffer;
    const Box2d start_mirror_box(half_length, half_width,
                                 start_.pos + mirror_offset_x_ * start_.tangent,
                                 start_.theta, start_.tangent);
    if (start_mirror_box.IsPointIn(point)) return true;
    const Box2d end_mirror_box(half_length, half_width,
                               end_.pos + mirror_offset_x_ * end_.tangent,
                               end_.theta, end_.tangent);
    if (end_mirror_box.IsPointIn(point)) return true;
  }

  const Vec2d vec =
      type_ == GeometryPathType::LEFT
          ? -start_.tangent.Perp() * (vehicle_half_width_ + lateral_buffer)
          : start_.tangent.Perp() * (vehicle_half_width_ + lateral_buffer);
  const double corner_buffer = 0.5 * VehicleOctagonModel::kCornerBufferFactor *
                               (lateral_buffer + longitudinal_buffer);
  // Check segment before rac.
  const double front_corner_to_rac =
      front_edge_to_center_ + longitudinal_buffer -
      (front_corner_side_length_ + corner_buffer);
  const Segment2d segment_before_rac(
      start_.pos - vec,
      start_.pos + vec + start_.tangent * front_corner_to_rac);
  if (monotonic_cosine_ <= MonotonicCosineOfPointOverlapWithSegment(
                               point, center_, !ccw_, segment_before_rac)) {
    return true;
  }

  // If we use fast method, we ignore the segment behind rac.
  if (!use_fast_overlap_) {
    // Check segment behind rac.
    const double rear_corner_to_rac =
        back_edge_to_center_ + longitudinal_buffer -
        (rear_corner_side_length_ + corner_buffer);
    const Segment2d segment_behind_rac(
        start_.pos - vec,
        start_.pos + vec - start_.tangent * rear_corner_to_rac);
    if (monotonic_cosine_ <= MonotonicCosineOfPointOverlapWithSegment(
                                 point, center_, !ccw_, segment_behind_rac)) {
      return true;
    }
  }

  if (consider_mirror) {
    const Vec2d mirror_mid = start_.pos + mirror_offset_x_ * start_.tangent;
    const Vec2d vec = (mirror_offset_y_ + mirror_radius_ + lateral_buffer) *
                      start_.tangent.Perp();
    const Segment2d mirror_segment(mirror_mid + vec, mirror_mid - vec);
    if (monotonic_cosine_ <= MonotonicCosineOfPointOverlapWithSegment(
                                 point, center_, !ccw_, mirror_segment)) {
      return true;
    }
  }
  return false;
}

bool CirclePathCollisionChecker::HasOverlapWithBuffer(
    const Segment2d& segment, double lateral_buffer, double longitudinal_buffer,
    bool consider_mirror) const {
  if (start_octagon_.HasOverlapWithBuffer(segment, lateral_buffer,
                                          longitudinal_buffer)) {
    return true;
  }
  if (end_octagon_.HasOverlapWithBuffer(segment, lateral_buffer,
                                        longitudinal_buffer)) {
    return true;
  }

  if (consider_mirror) {
    const double half_length = mirror_radius_ + lateral_buffer;
    const double half_width =
        mirror_offset_y_ + mirror_radius_ + lateral_buffer;
    const Box2d start_mirror_box(half_length, half_width,
                                 start_.pos + mirror_offset_x_ * start_.tangent,
                                 start_.theta, start_.tangent);
    if (start_mirror_box.HasOverlap(segment)) return true;
    const Box2d end_mirror_box(half_length, half_width,
                               end_.pos + mirror_offset_x_ * end_.tangent,
                               end_.theta, end_.tangent);
    if (end_mirror_box.HasOverlap(segment)) return true;
  }

  const Vec2d vec =
      type_ == GeometryPathType::LEFT
          ? -start_.tangent.Perp() * (vehicle_half_width_ + lateral_buffer)
          : start_.tangent.Perp() * (vehicle_half_width_ + lateral_buffer);
  const double corner_buffer = 0.5 * VehicleOctagonModel::kCornerBufferFactor *
                               (lateral_buffer + longitudinal_buffer);
  // Check segment before rac.
  const double front_corner_to_rac =
      front_edge_to_center_ + longitudinal_buffer -
      (front_corner_side_length_ + corner_buffer);
  const Segment2d segment_before_rac(
      start_.pos - vec,
      start_.pos + vec + start_.tangent * front_corner_to_rac);
  if (HasOverlap(segment_before_rac, segment)) return true;

  // If we use fast method, we ignore the segment behind rac.
  if (!use_fast_overlap_) {
    // Check segment behind rac.
    const double rear_corner_to_rac =
        back_edge_to_center_ + longitudinal_buffer -
        (rear_corner_side_length_ + corner_buffer);
    const Segment2d segment_behind_rac(
        start_.pos - vec,
        start_.pos + vec - start_.tangent * rear_corner_to_rac);
    if (HasOverlap(segment_behind_rac, segment)) return true;
  }

  if (consider_mirror) {
    const Vec2d mirror_mid = start_.pos + mirror_offset_x_ * start_.tangent;
    const Vec2d vec = (mirror_offset_y_ + mirror_radius_ + lateral_buffer) *
                      start_.tangent.Perp();
    const Segment2d mirror_segment(mirror_mid + vec, mirror_mid - vec);
    if (HasOverlap(mirror_segment, segment)) return true;
  }

  return false;
}

bool CirclePathCollisionChecker::HasOverlapWithBuffer(
    const Polygon2d& polygon, double lateral_buffer, double longitudinal_buffer,
    bool consider_mirror) const {
  if (start_octagon_.HasOverlapWithBuffer(polygon, lateral_buffer,
                                          longitudinal_buffer)) {
    return true;
  }
  if (end_octagon_.HasOverlapWithBuffer(polygon, lateral_buffer,
                                        longitudinal_buffer)) {
    return true;
  }

  if (consider_mirror) {
    const double half_length = mirror_radius_ + lateral_buffer;
    const double half_width =
        mirror_offset_y_ + mirror_radius_ + lateral_buffer;
    const Box2d start_mirror_box(half_length, half_width,
                                 start_.pos + mirror_offset_x_ * start_.tangent,
                                 start_.theta, start_.tangent);
    if (polygon.HasOverlap(start_mirror_box)) return true;
    const Box2d end_mirror_box(half_length, half_width,
                               end_.pos + mirror_offset_x_ * end_.tangent,
                               end_.theta, end_.tangent);
    if (polygon.HasOverlap(end_mirror_box)) return true;
  }

  const Vec2d vec =
      type_ == GeometryPathType::LEFT
          ? -start_.tangent.Perp() * (vehicle_half_width_ + lateral_buffer)
          : start_.tangent.Perp() * (vehicle_half_width_ + lateral_buffer);
  const double corner_buffer = 0.5 * VehicleOctagonModel::kCornerBufferFactor *
                               (lateral_buffer + longitudinal_buffer);
  // Segment before rac.
  const double front_corner_to_rac =
      front_edge_to_center_ + longitudinal_buffer -
      (front_corner_side_length_ + corner_buffer);
  const Segment2d segment_before_rac(
      start_.pos - vec,
      start_.pos + vec + start_.tangent * front_corner_to_rac);
  // Segment behind rac.
  Segment2d segment_behind_rac;
  if (!use_fast_overlap_) {
    const double rear_corner_to_rac =
        back_edge_to_center_ + longitudinal_buffer -
        (rear_corner_side_length_ + corner_buffer);
    segment_behind_rac =
        Segment2d(start_.pos - vec,
                  start_.pos + vec - start_.tangent * rear_corner_to_rac);
  }
  // Mirror segment.
  Segment2d mirror_segment;
  if (consider_mirror) {
    const Vec2d mirror_mid = start_.pos + mirror_offset_x_ * start_.tangent;
    const Vec2d vec = (mirror_offset_y_ + mirror_radius_ + lateral_buffer) *
                      start_.tangent.Perp();
    mirror_segment = Segment2d(mirror_mid + vec, mirror_mid - vec);
  }

  for (const auto& line_segment : polygon.line_segments()) {
    if (HasOverlap(segment_before_rac, line_segment)) return true;
    if (!use_fast_overlap_ && HasOverlap(segment_behind_rac, line_segment)) {
      return true;
    }
    if (consider_mirror && HasOverlap(mirror_segment, line_segment)) {
      return true;
    }
  }
  return false;
}

}  // namespace planner
}  // namespace qcraft
