#include "onboard/planner/common/geometry_path_util.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/circle_path_overlap.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kEps = 1e-3;

std::optional<double> ComputeStraightPathFirstOverlapDistanceWithSegment(
    const Box2d& ego_box, bool forward, const Segment2d& segment) {
  const double heading = ego_box.heading();
  const Vec2d& center = ego_box.center();
  Segment2d local_segment{(segment.start() - center).Rotate(-heading),
                          (segment.end() - center).Rotate(-heading)};
  const double half_width = ego_box.half_width();
  const double half_length = ego_box.half_length();

  if (local_segment.min_y() > half_width ||
      local_segment.max_y() < -half_width) {
    return std::nullopt;
  }
  local_segment.ClampByYMax(half_width);
  local_segment.ClampByYMin(-half_width);

  if (!forward) {
    // Driving reversely.
    if (local_segment.min_x() > half_length) {
      return std::nullopt;
    }
    return std::max(-half_length - local_segment.max_x(), 0.0);
  }

  if (local_segment.max_x() < -half_length) {
    return std::nullopt;
  }
  return std::max(local_segment.min_x() - half_length, 0.0);
}

std::optional<double> ComputeCurvePathFirstOverlapDistanceWithSegment(
    const Vec2d& pos, const Box2d& ego_box, bool forward, double kappa,
    const Segment2d& segment) {
  QCHECK_GE(std::abs(kappa), kEps);
  const Vec2d heading = Vec2d::FastUnitFromAngle(ego_box.heading());
  const double radius = 1.0 / kappa;
  const Vec2d center = pos + radius * heading.Perp();
  const double ccw = (kappa < 0.0) ^ forward;
  const double first_overlap_angle =
      BoxFirstOverlapAngle(ego_box, center, ccw, segment);
  if (!std::isinf(first_overlap_angle)) {
    return std::abs(first_overlap_angle * radius);
  }
  return std::nullopt;
}

inline std::optional<double> GetMinimumSignedDistanceAlongXAxis(
    const Vec2d& point, const Segment2d& segment) {
  if (point.y() < segment.min_y() || point.y() > segment.max_y()) {
    return std::nullopt;
  }
  const double lerp_factor =
      LerpFactor(segment.start().y(), segment.end().y(), point.y());
  const double lerp_x =
      Lerp(segment.start().x(), segment.end().x(), lerp_factor);
  return point.x() - lerp_x;
}

std::optional<double> ComputeStraightPathFirstOverlapDistanceWithSegment(
    const Vec2d& tangent, const Polygon2d& ego_polygon, bool forward,
    const Segment2d& segment) {
  const double cos_yaw = forward ? tangent.x() : -tangent.x();
  const double sin_yaw = forward ? -tangent.y() : tangent.y();
  const Segment2d local_segment{segment.start().Rotate(cos_yaw, sin_yaw),
                                segment.end().Rotate(cos_yaw, sin_yaw)};
  std::optional<double> res = std::nullopt;
  const auto update_min_dist =
      [](const std::optional<double>& dist_opt, bool use_opposite_num,
         std::optional<double>* min_dist, bool* exist_negative_dist) {
        if (!dist_opt.has_value()) return;
        const double dist = use_opposite_num ? -*dist_opt : *dist_opt;
        if (dist >= 0.0) {
          *min_dist = min_dist->has_value() ? std::min(dist, **min_dist) : dist;
        } else {
          *exist_negative_dist = true;
        }
      };
  for (const auto& ego_segment : ego_polygon.line_segments()) {
    bool exist_negative_dist = false;
    std::optional<double> min_dist = std::nullopt;
    const Segment2d ego_local_segment{
        ego_segment.start().Rotate(cos_yaw, sin_yaw),
        ego_segment.end().Rotate(cos_yaw, sin_yaw)};
    if (local_segment.min_y() > ego_local_segment.max_y() ||
        local_segment.max_y() < ego_local_segment.min_y()) {
      continue;
    }
    update_min_dist(GetMinimumSignedDistanceAlongXAxis(local_segment.start(),
                                                       ego_local_segment),
                    /*use_opposite_num=*/false, &min_dist,
                    &exist_negative_dist);
    update_min_dist(GetMinimumSignedDistanceAlongXAxis(local_segment.end(),
                                                       ego_local_segment),
                    /*use_opposite_num=*/false, &min_dist,
                    &exist_negative_dist);
    update_min_dist(GetMinimumSignedDistanceAlongXAxis(
                        ego_local_segment.start(), local_segment),
                    /*use_opposite_num=*/true, &min_dist, &exist_negative_dist);
    update_min_dist(GetMinimumSignedDistanceAlongXAxis(ego_local_segment.end(),
                                                       local_segment),
                    /*use_opposite_num=*/true, &min_dist, &exist_negative_dist);
    if (min_dist.has_value()) {
      if (*min_dist < kEps || exist_negative_dist) return 0.0;
      res = res.has_value() ? std::min(*min_dist, *res) : *min_dist;
    }
  }
  return res;
}

inline std::optional<double> ComputeCurvePathFirstOverlapDistanceWithSegment(
    const Vec2d& pos, const Vec2d& tangent, const Polygon2d& ego_polygon,
    bool forward, double kappa, const Segment2d& segment) {
  QCHECK_GE(std::abs(kappa), kEps);
  const double radius = 1.0 / kappa;
  const Vec2d center = pos + radius * tangent.Perp();
  const double ccw = (kappa < 0.0) ^ forward;
  const double first_overlap_angle =
      PolygonFirstOverlapAngle(ego_polygon, center, ccw, segment);
  if (!std::isinf(first_overlap_angle)) {
    return std::abs(first_overlap_angle * radius);
  }
  return std::nullopt;
}

}  // namespace

std::optional<double> ComputeFirstOverlapDistanceWithSegment(
    const Vec2d& pos, const Box2d& ego_box, bool forward, double kappa,
    const Segment2d& segment) {
  return std::abs(kappa) < kEps
             ? ComputeStraightPathFirstOverlapDistanceWithSegment(
                   ego_box, forward, segment)
             : ComputeCurvePathFirstOverlapDistanceWithSegment(
                   pos, ego_box, forward, kappa, segment);
}

std::optional<double> ComputeFirstOverlapDistanceWithBox(const Vec2d& pos,
                                                         const Box2d& ego_box,
                                                         bool forward,
                                                         double kappa,
                                                         const Box2d& box) {
  std::optional<double> res = std::nullopt;
  for (const auto& segment : box.GetEdgesCounterClockwise()) {
    const auto first_overlap_dist = ComputeFirstOverlapDistanceWithSegment(
        pos, ego_box, forward, kappa, segment);
    if (first_overlap_dist.has_value()) {
      res = res.has_value() ? std::min(*res, *first_overlap_dist)
                            : first_overlap_dist;
    }
  }
  return res;
}

std::optional<double> ComputeFirstOverlapDistanceWithPolygon(
    const Vec2d& pos, const Box2d& ego_box, bool forward, double kappa,
    const Polygon2d& polygon) {
  std::optional<double> res = std::nullopt;
  for (const auto& segment : polygon.line_segments()) {
    const auto first_overlap_dist = ComputeFirstOverlapDistanceWithSegment(
        pos, ego_box, forward, kappa, segment);
    if (first_overlap_dist.has_value()) {
      res = res.has_value() ? std::min(*res, *first_overlap_dist)
                            : first_overlap_dist;
    }
  }
  return res;
}

std::optional<double> ComputeFirstOverlapDistanceWithSegment(
    const Vec2d& pos, const Vec2d& tangent, const Polygon2d& ego_polygon,
    bool forward, double kappa, const Segment2d& segment) {
  return std::abs(kappa) < kEps
             ? ComputeStraightPathFirstOverlapDistanceWithSegment(
                   tangent, ego_polygon, forward, segment)
             : ComputeCurvePathFirstOverlapDistanceWithSegment(
                   pos, tangent, ego_polygon, forward, kappa, segment);
}

std::optional<double> ComputeFirstOverlapDistanceWithBox(
    const Vec2d& pos, const Vec2d& tangent, const Polygon2d& ego_polygon,
    bool forward, double kappa, const Box2d& box) {
  std::optional<double> res = std::nullopt;
  for (const auto& segment : box.GetEdgesCounterClockwise()) {
    const auto first_overlap_dist = ComputeFirstOverlapDistanceWithSegment(
        pos, tangent, ego_polygon, forward, kappa, segment);
    if (first_overlap_dist.has_value()) {
      res = res.has_value() ? std::min(*res, *first_overlap_dist)
                            : first_overlap_dist;
    }
  }
  return res;
}

std::optional<double> ComputeFirstOverlapDistanceWithPolygon(
    const Vec2d& pos, const Vec2d& tangent, const Polygon2d& ego_polygon,
    bool forward, double kappa, const Polygon2d& polygon) {
  std::optional<double> res = std::nullopt;
  for (const auto& segment : polygon.line_segments()) {
    const auto first_overlap_dist = ComputeFirstOverlapDistanceWithSegment(
        pos, tangent, ego_polygon, forward, kappa, segment);
    if (first_overlap_dist.has_value()) {
      res = res.has_value() ? std::min(*res, *first_overlap_dist)
                            : first_overlap_dist;
    }
  }
  return res;
}

}  // namespace planner
}  // namespace qcraft
