#include "onboard/planner/util/min_segment_distance_problem.h"

#include "glog/logging.h"

#include "onboard/math/util.h"
// IWYU pragma: no_include "Eigen/Core"

namespace qcraft::planner {

double MinSegmentDistanceProblem::Evaluate(const Vec2d& point) const {
  const Segment2d* segment = GetNearestSegment(point);
  if (segment == nullptr) {
    return cutoff_distance_;
  }
  return segment->DistanceTo(point);
}

double MinSegmentDistanceProblem::EvaluateWithNearestSegmentId(
    const Vec2d& point, std::string* segment_id) const {
  DCHECK_NOTNULL(segment_id);
  DCHECK(qtfm_segment_matcher_);
  int index = -1;
  if (qtfm_segment_matcher_->GetNearestSegmentIndex(point.x(), point.y(),
                                                    &index)) {
    *segment_id = named_segments_[index].first;
    return named_segments_[index].second.DistanceTo(point);
  } else {
    *segment_id = "";
    return cutoff_distance_;
  }
}

const Segment2d* MinSegmentDistanceProblem::GetNearestSegment(
    const Vec2d& point) const {
  DCHECK(qtfm_segment_matcher_);
  int index = -1;
  if (qtfm_segment_matcher_->GetNearestSegmentIndex(point.x(), point.y(),
                                                    &index)) {
    return &named_segments_[index].second;
  } else {
    return nullptr;
  }
}

double MinSegmentDistanceProblem::EvaluateWithSecondOrderDerivatives(
    const Vec2d& point, SecondOrderDerivativeType* derivative) const {
  DCHECK_NOTNULL(derivative);
  const Segment2d* segment = GetNearestSegment(point);
  if (segment == nullptr) {
    // All zero.
    *derivative = SecondOrderDerivativeType{};
    return cutoff_distance_;
  }

  const double projection = segment->ProjectOntoUnit(point);
  if (projection <= 0.0 || segment->length() <= kEpsilon) {
    return DistanceToCenterWithDerivative(point, segment->start(), derivative);
  }
  if (projection >= segment->length()) {
    return DistanceToCenterWithDerivative(point, segment->end(), derivative);
  }
  return DistanceToLineWithDerivative(point, *segment, derivative);
}

double MinSegmentDistanceProblem::DistanceToCenterWithDerivative(
    const Vec2d& point, const Vec2d& center,
    SecondOrderDerivativeType* derivative) const {
  DCHECK_NOTNULL(derivative);
  const double x = point.x();
  const double y = point.y();
  const double x0 = center.x();
  const double y0 = center.y();
  const double dx = x - x0;
  const double dy = y - y0;
  const double dist = Hypot(dx, dy);

  if (dist < kEpsilon) {
    derivative->df_dx = 1.0;
    derivative->df_dy = 0.0;
    derivative->d2f_dy_dy = 0.0;
    derivative->d2f_dx_dx = 0.0;
    derivative->d2f_dx_dy = 0.0;
    return 0.0;
  } else {
    const double dist_inv = 1. / dist;
    const double dist_inv_cubic = dist_inv * dist_inv * dist_inv;
    derivative->df_dx = dx * dist_inv;
    derivative->df_dy = dy * dist_inv;
    derivative->d2f_dx_dx = -(dx * dx) * dist_inv_cubic + dist_inv;
    derivative->d2f_dy_dy = -(dy * dy) * dist_inv_cubic + dist_inv;
    derivative->d2f_dx_dy = -(dx * dy) * dist_inv_cubic;
    return dist;
  }
}

double MinSegmentDistanceProblem::DistanceToLineWithDerivative(
    const Vec2d& point, const Segment2d& segment_on_line,
    SecondOrderDerivativeType* derivative) const {
  DCHECK_NOTNULL(derivative);
  DCHECK_GE(segment_on_line.length(), kEpsilon);
  const Vec2d start_to_point = point - segment_on_line.start();
  const Vec2d norm = segment_on_line.unit_direction().Perp();
  const double signed_dist = norm.Dot(start_to_point);

  derivative->d2f_dx_dx = 0.0;
  derivative->d2f_dy_dy = 0.0;
  derivative->d2f_dx_dy = 0.0;
  if (signed_dist > 0.0) {
    derivative->df_dx = norm.x();
    derivative->df_dy = norm.y();
    return signed_dist;
  } else {
    derivative->df_dx = -norm.x();
    derivative->df_dy = -norm.y();
    return -signed_dist;
  }
}

}  // namespace qcraft::planner
