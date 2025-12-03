#include "onboard/planner/util/lane_path_preprocess_util.h"

// IWYU pragma: no_include "onboard/lite/qissue_trans.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/gradient_points_smoother.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/range1d.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/vis/common/color.h"

DEFINE_bool(planner_enable_lane_path_preprocess_debug, false,
            "Whether to send lane path preprocess result to canvas.");

namespace qcraft::planner {
namespace {

// {right, left}
std::pair<Vec2d, Vec2d> CollectNearestBoundaries(
    const PlannerSemanticMapManager& psmm, const Vec2d& xy,
    const Vec2d& tangent, double max_lateral_offset, bool is_virtual_or_merge) {
  Vec2d left_point = xy + tangent.Perp() * max_lateral_offset;
  Vec2d right_point = xy - tangent.Perp() * max_lateral_offset;

  const Segment2d normal_line(right_point, left_point);

  const auto candidate_boundaries = psmm.GetLaneBoundariesInfoAtLevel(
      psmm.GetLevel(), xy, max_lateral_offset);

  double left_offset = max_lateral_offset;
  double right_offset = -max_lateral_offset;

  if (is_virtual_or_merge) {
    return {right_point, left_point};
  }

  constexpr double kMinBoundaryOffset = 0.5;  // m.
  for (const auto& boundary : candidate_boundaries) {
    // Compute cross point of boundary and normal line
    Vec2d closest_intersection;
    double min_sqr_dis = std::numeric_limits<double>::max();
    for (int k = 0; k + 1 < boundary->points_smooth.size(); ++k) {
      const Vec2d& p0 = boundary->points_smooth[k];
      const Vec2d& p1 = boundary->points_smooth[k + 1];
      const Segment2d boundary_segment(p0, p1);

      Vec2d intersection;
      if (!normal_line.GetIntersect(boundary_segment, &intersection)) continue;
      const double sqr_dis = xy.DistanceSquareTo(intersection);
      if (sqr_dis < min_sqr_dis) {
        min_sqr_dis = sqr_dis;
        closest_intersection = intersection;
      }
    }
    if (min_sqr_dis < std::numeric_limits<double>::max()) {
      const double lat_offset = tangent.CrossProd(closest_intersection - xy);
      if (lat_offset <= -kMinBoundaryOffset && lat_offset > right_offset) {
        right_offset = lat_offset;
        right_point = closest_intersection;
      }
      if (lat_offset >= kMinBoundaryOffset && lat_offset < left_offset) {
        left_offset = lat_offset;
        left_point = closest_intersection;
      }
    }
  }

  return {right_point, left_point};
}

void ClampCenterByBoundary(absl::Span<const Vec2d> left_boundary,
                           absl::Span<const Vec2d> right_boundary,
                           double sample_step,
                           std::vector<Vec2d>* center_points) {
  constexpr double kMinLaneWidthForTwoLane = 4.5;  // m.
  constexpr double kMaxAllowableLaneWidth = 9.0;   // m.
  constexpr double kMaxPossibleLaneWidth = 12.0;   // m.
  constexpr double kSameBoundLatThres = 1.0;       // m.
  constexpr double kBoundaryDistWeight = 1.0;
  constexpr double kHeadingDiffWeight = 1.5 / M_PI_2;

  const double same_bound_dist_sqr = Sqr(sample_step) + Sqr(kSameBoundLatThres);
  const int n = left_boundary.size();
  for (int i = n - 3; i >= 0; --i) {
    const auto& right_pt = right_boundary[i];
    const auto& left_pt = left_boundary[i];
    const double lane_width = left_pt.DistanceTo(right_pt);

    if (lane_width > kMaxAllowableLaneWidth) continue;

    if (lane_width < kMinLaneWidthForTwoLane) {
      (*center_points)[i] = (left_pt + right_pt) * 0.5;
      continue;
    }

    const auto& prev_right_pt = right_boundary[i + 1];
    const auto& prev_left_pt = left_boundary[i + 1];
    const auto& prev_center_pt = (*center_points)[i + 1];
    const double prev_left_dist = prev_left_pt.DistanceTo(prev_center_pt);
    const double prev_right_dist = prev_right_pt.DistanceTo(prev_center_pt);
    const double prev_lane_width = prev_left_dist + prev_right_dist;
    const double prev_heading =
        ((*center_points)[i + 2] - prev_center_pt).FastAngle();

    const double sample_frac = kDefaultHalfLaneWidth / lane_width;
    const double origin_frac =
        right_pt.DistanceTo((*center_points)[i]) / lane_width;
    const std::vector<double> candidate_fractions = {
        sample_frac, 0.5, 1.0 - sample_frac, origin_frac};

    const bool is_same_left_bound =
        prev_left_pt.DistanceSquareTo(left_pt) <= same_bound_dist_sqr;
    const bool is_same_right_bound =
        prev_right_pt.DistanceSquareTo(right_pt) <= same_bound_dist_sqr;

    Vec2d best_pt;
    double min_cost = std::numeric_limits<double>::infinity();
    for (const double frac : candidate_fractions) {
      const auto pt = Lerp(right_pt, left_pt, frac);

      const double left_dist = left_pt.DistanceTo(pt);
      const double right_dist = right_pt.DistanceTo(pt);
      const double left_dist_diff = std::fabs(left_dist - prev_left_dist);
      const double right_dist_diff = std::fabs(right_dist - prev_right_dist);

      double bound_dist_diff = 0.0;
      if (is_same_left_bound && is_same_right_bound) {
        bound_dist_diff = std::min(left_dist_diff, right_dist_diff);
      } else if (is_same_left_bound || is_same_right_bound) {
        bound_dist_diff = is_same_left_bound ? left_dist_diff : right_dist_diff;
      }

      const double bound_dist_cost = prev_lane_width <= kMaxPossibleLaneWidth
                                         ? bound_dist_diff * kBoundaryDistWeight
                                         : 0.0;

      const double heading = (prev_center_pt - pt).FastAngle();
      const double heading_diff =
          std::fabs(NormalizeAngle(heading - prev_heading));
      const double heading_diff_cost = heading_diff * kHeadingDiffWeight;

      const double cost = bound_dist_cost + heading_diff_cost;
      if (cost < min_cost) {
        min_cost = cost;
        best_pt = pt;
      }
      VLOG(1) << absl::StrFormat(
          "Idx %d, frac %f: dist diff %f, cost %f; angle diff %f, cost %f; "
          "total %f",
          i, frac, bound_dist_diff, bound_dist_cost, heading_diff,
          heading_diff_cost, cost);
    }

    (*center_points)[i] = best_pt;
  }
}

std::vector<Vec2d> SmoothCenterPointsOrReturnOriginal(
    const std::vector<Vec2d>& points) {
  FUNC_QTRACE();

  constexpr double kTolerance = 1.5;
  auto smooth_result = ResampleAndSmoothPoints(points, kTolerance);

  if (!smooth_result.ok()) return points;
  return std::move(smooth_result).value();
}

Range1d<double> FindFirstMergeProtectRange(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  constexpr double kProtectionLength = 20.0;  // m.
  constexpr double kEpsilon = 0.1;            // m.

  for (int i = 1; i < lane_path.size(); ++i) {
    const auto prev_lane_id = lane_path.lane_id(i - 1);
    const auto lane_id = lane_path.lane_id(i);

    const auto* prev_lane_proto = psmm.FindLaneByIdOrNull(prev_lane_id);
    const auto* lane_proto = psmm.FindLaneByIdOrNull(lane_id);

    if (prev_lane_proto == nullptr || lane_proto == nullptr) continue;

    if (prev_lane_proto->is_merging() ||
        (prev_lane_proto->outgoing_lanes_size() == 1 &&
         lane_proto->incoming_lanes_size() > 1)) {
      const double s = lane_path.start_s(i);
      const double start_s = std::max(s - kProtectionLength, 0.0);
      return Range1d<double>(start_s, s + kEpsilon);
    }
  }

  return Range1d<double>();
}

}  // namespace

std::vector<Vec2d> StraightenedLanePathPoints(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double start_s, double end_s, double step) {
  FUNC_QTRACE();

  const auto merge_range = FindFirstMergeProtectRange(psmm, lane_path);

  const int n = CeilToInt((end_s - start_s) / step);
  std::vector<Vec2d> center_points, left_points, right_points;
  center_points.reserve(n);
  left_points.reserve(n);
  right_points.reserve(n);

  for (double sample_s = start_s; sample_s <= end_s; sample_s += step) {
    const auto sample_lane_point = lane_path.ArclengthToLanePoint(sample_s);

    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm,
                                      sample_lane_point.lane_id());

    const auto xy = ComputeLanePointPos(psmm, sample_lane_point);
    const auto tangent = ComputeLanePointTangent(psmm, sample_lane_point);
    const bool is_virtual_or_merge =
        lane_info.IsVirtual() || merge_range.Contains(sample_s);

    const auto [right_point, left_point] = CollectNearestBoundaries(
        psmm, xy, tangent, kMaxLateralOffset, is_virtual_or_merge);

    center_points.push_back(xy);
    left_points.push_back(left_point);
    right_points.push_back(right_point);
  }

  if (FLAGS_planner_enable_lane_path_preprocess_debug) {
    SendPointsToCanvas(left_points, "left_boundary", vis::Color::kSeaGreen);
    SendPointsToCanvas(right_points, "right_boundary", vis::Color::kSeaGreen);
    SendPointsToCanvas(center_points, "raw center_points",
                       vis::Color::kMiddleBlueGreen);
  }

  ClampCenterByBoundary(left_points, right_points, step, &center_points);

  const auto smoothed_center_points =
      SmoothCenterPointsOrReturnOriginal(center_points);

  if (FLAGS_planner_enable_lane_path_preprocess_debug) {
    SendPointsToCanvas(center_points, "straightened center_points",
                       vis::Color::kMiddleBlueGreen);
    SendPointsToCanvas(smoothed_center_points, "smoothed center_points",
                       vis::Color::kMiddleBlueGreen);
  }

  std::vector<double> s_vec(smoothed_center_points.size(), 0.0);
  for (int i = 1; i < s_vec.size(); ++i) {
    s_vec[i] = s_vec[i - 1] + smoothed_center_points[i].DistanceTo(
                                  smoothed_center_points[i - 1]);
  }

  const PiecewiseLinearFunction<Vec2d> plf(s_vec, smoothed_center_points);
  const int resampled_n = CeilToInt(s_vec.back() / step);
  std::vector<Vec2d> resampled_points;
  resampled_points.reserve(resampled_n);
  for (double sample_s = 0.0; sample_s <= s_vec.back(); sample_s += step) {
    resampled_points.emplace_back(plf(sample_s));
  }

  return resampled_points;
}

}  // namespace qcraft::planner
