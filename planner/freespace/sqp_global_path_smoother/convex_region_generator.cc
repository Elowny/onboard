#include "onboard/planner/freespace/sqp_global_path_smoother/convex_region_generator.h"

#include <cmath>
#include <utility>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/math/eigen.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/util.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {
namespace {
constexpr double kEpsilon = 1e-4;
// Calculate rotation matrix from a vector (aligned with x-axis).
inline Mat2d Vec2dToRotation(const Vec2d& v) {
  const Vec2d tangent = v.Unit();
  return (Mat2d() << tangent(0), -tangent(1), tangent(1), tangent(0))
      .finished();
}

bool ClipPolygon(const Hyperplane& v,
                 std::pair<double, Polygon2d>* const polygon_info) {
  std::vector<Vec2d> points = polygon_info->second.points();
  const Hyperplane shifted_plane = v.Shift(v.n() * polygon_info->first);
  if (!shifted_plane.ClipConvexHull(&points)) {
    return false;
  }
  polygon_info->second = Polygon2d(points, /*is_convex=*/true);
  return true;
}

bool ClipLineSegment(const Hyperplane& v,
                     std::pair<double, Segment2d>* const line_info) {
  std::vector<Vec2d> points{line_info->second.start(), line_info->second.end()};
  const Hyperplane shifted_plane = v.Shift(v.n() * line_info->first);
  if (!shifted_plane.ClipConvexHull(&points)) {
    return false;
  }
  line_info->second = Segment2d(points[0], points[1]);
  return true;
}

void RemoveOrClipLinesNotInside(
    const Hyperplane& v,
    std::vector<std::pair<double, Segment2d>>* lines_info) {
  int cur = 0;
  // Num of elements that should be removed.
  int num = 0;
  int size = lines_info->size();
  while (cur < size - num) {
    if (!ClipLineSegment(v, &(*lines_info)[cur])) {
      std::swap((*lines_info)[cur], (*lines_info)[size - 1 - num]);
      ++num;
    } else {
      ++cur;
    }
  }
  lines_info->resize(size - num);
}

void RemoveOrClipPolygonsNotInside(
    const Hyperplane& v,
    std::vector<std::pair<double, Polygon2d>>* polygons_info) {
  int cur = 0;
  // Num of elements that should be removed.
  int num = 0;
  int size = polygons_info->size();
  while (cur < size - num) {
    if (!ClipPolygon(v, &(*polygons_info)[cur])) {
      std::swap((*polygons_info)[cur], (*polygons_info)[size - 1 - num]);
      ++num;
    } else {
      ++cur;
    }
  }
  polygons_info->resize(size - num);
}
}  // namespace
void LineSegmentDecomp::FindConvexRegion() {
  SCOPED_QTRACE("Line Segment Decomp: FindConvexRegion");
  // Find half-space
  std::vector<Hyperplane> vs;
  std::vector<std::pair<double, Segment2d>> boundaries_info_remain =
      boundaries_info_;
  std::vector<std::pair<double, Polygon2d>> objects_info_remain = objects_info_;
  while (!(boundaries_info_remain.empty() && objects_info_remain.empty())) {
    Hyperplane v({1.0, 0.0}, {1.0, 0.0});
    if (objects_info_remain.empty()) {
      v = ellipse_->GetClosestHyperplane(boundaries_info_remain);
    } else if (boundaries_info_remain.empty()) {
      v = ellipse_->GetClosestHyperplane(objects_info_remain);
    } else {
      const auto boundaries_v =
          ellipse_->GetClosestHyperplane(boundaries_info_remain);
      const auto objects_v =
          ellipse_->GetClosestHyperplane(objects_info_remain);
      v = ellipse_->InvDistanceSquareToCenter(boundaries_v.p()) <
                  ellipse_->InvDistanceSquareToCenter(objects_v.p())
              ? boundaries_v
              : objects_v;
    }
    v = v.Shift(-v.n() * kEpsilon);
    RemoveOrClipLinesNotInside(v, &boundaries_info_remain);
    RemoveOrClipPolygonsNotInside(v, &objects_info_remain);
    vs.push_back(std::move(v));
  }
  convex_region_ = std::make_unique<ConvexRegion>(std::move(vs));
}
void LineSegmentDecomp::AddLocalObb(double local_obb_length,
                                    double local_obb_width,
                                    ConvexRegion* region) {
  SCOPED_QTRACE("Line Segment Decomp: AddLocalObb");
  // Virtual walls parallel to path p1 p2.
  Vec2d dir(0.0, 1.0);
  if (p1_ != p2_) {
    dir = (p2_ - p1_).normalized();
  }
  Vec2d dir_h = dir.Perp();

  // Along x.
  Vec2d pp1 = p1_ + dir_h * local_obb_length;
  Vec2d pp2 = p1_ - dir_h * local_obb_length;
  region->AddHyperplane(Hyperplane(pp1, dir_h));
  region->AddHyperplane(Hyperplane(pp2, -dir_h));
  // Along y.
  Vec2d pp3 = p2_ + dir * local_obb_width;
  Vec2d pp4 = p1_ - dir * local_obb_width;
  region->AddHyperplane(Hyperplane(pp3, dir));
  region->AddHyperplane(Hyperplane(pp4, -dir));
}

void LineSegmentDecomp::FindEllipse() {
  // TODO(ziyi) Check if cannot find the ellipse
  SCOPED_QTRACE("Line Segment Decomp: FindEllipse");
  const double f = line_.length() / 2.0;
  // NOLINTNEXTLINE(readability-identifier-naming)
  Mat2d C = f * Mat2d::Identity();
  Vec2d axes = Vec2d::Constant(f);

  if (axes(0) > 0.0) {
    double ratio = axes(1) / axes(0);
    axes *= ratio;
    C *= ratio;
  }

  // NOLINTNEXTLINE(readability-identifier-naming)
  const auto Ri = Vec2dToRotation(p2_ - p1_);
  // NOLINTNEXTLINE(readability-identifier-naming)
  const Mat2d Ri_transpose = Ri.transpose();
  C = Ri * C * Ri_transpose;
  GeneralEllipse ellipse(C, (p1_ + p2_) / 2.0);
  // NOLINTNEXTLINE(readability-identifier-naming)
  const Vec2d Ri_transpose_dot_d = Ri_transpose * ellipse.d();
  auto boundaries_info_not_outside = boundaries_info_;
  auto objects_info_not_outside = objects_info_;
  ellipse.RemoveLinesOut(&boundaries_info_not_outside);
  ellipse.RemovePolygonsOut(&objects_info_not_outside);
  // Decide short axes.
  while (!(boundaries_info_not_outside.empty() &&
           objects_info_not_outside.empty())) {
    Vec2d closest_pt;
    if (objects_info_not_outside.empty()) {
      closest_pt = ellipse.GetClosestPoint(boundaries_info_not_outside);
    } else if (boundaries_info_not_outside.empty()) {
      closest_pt = ellipse.GetClosestPoint(objects_info_not_outside);
    } else {
      const auto boundaries_closest_pt =
          ellipse.GetClosestPoint(boundaries_info_not_outside);
      const auto objects_closest_pt =
          ellipse.GetClosestPoint(objects_info_not_outside);
      closest_pt = ellipse.InvDistanceSquareToCenter(boundaries_closest_pt) <
                           ellipse.InvDistanceSquareToCenter(objects_closest_pt)
                       ? boundaries_closest_pt
                       : objects_closest_pt;
    }
    // p = R^{T}(pt - d)
    const Vec2d p =
        Ri_transpose * closest_pt - Ri_transpose_dot_d;  // To ellipse frame.
    if (p(0) < axes(0)) {
      // The new ellipse should be smaller in order not to touch closet_pt.
      axes(1) =
          std::abs(p(1)) / std::sqrt(1.0 - Sqr(p(0) / axes(0))) - kEpsilon;
    }
    // NOLINTNEXTLINE(readability-identifier-naming)
    Mat2d new_C = Mat2d::Identity();
    new_C(0, 0) = axes(0);
    new_C(1, 1) = axes(1);
    ellipse.set_C(Ri * new_C * Ri_transpose);
    ellipse.RemoveLinesOut(&boundaries_info_not_outside);
    ellipse.RemovePolygonsOut(&objects_info_not_outside);
  }

  ellipse_ = std::make_unique<GeneralEllipse>(std::move(ellipse));
}

void ConvexRegionsGenerator::Dilate(
    const DiscretizedPath& path, const std::vector<bool>& forwards,
    const std::vector<std::pair<double, Segment2d>>& boundaries_info,
    const std::vector<std::pair<double, Polygon2d>>& objects_info,
    double local_obb_length, double local_obb_width, double front_edge_to_pt,
    double back_edge_to_pt, bool set_start_to_origin) {
  SCOPED_QTRACE("Convex Regions Generator: Dilate");
  const int num = path.size();
  lines_.clear();
  lines_.reserve(num);
  linear_constraints_.clear();
  linear_constraints_.reserve(num);
  const Vec2d start_point(path[0].x(), path[0].y());
  std::vector<Hyperplane> shifted_hyperplanes;
  for (int i = 0; i < num; ++i) {
    const int next_i = std::min(i + 1, num - 1);
    const int prev_i = std::max(i - 1, 0);
    double next_theta_cos_sin[2];
    fast_math::CosAndSin<7>(path[next_i].theta(), next_theta_cos_sin);
    double prev_theta_cos_sin[2];
    fast_math::CosAndSin<7>(path[prev_i].theta(), prev_theta_cos_sin);
    const double next_sin_theta = next_theta_cos_sin[1];
    const double next_cos_theta = next_theta_cos_sin[0];
    const double prev_sin_theta = prev_theta_cos_sin[1];
    const double prev_cos_theta = prev_theta_cos_sin[0];
    Vec2d rear, front;
    if (forwards[i]) {
      rear = Vec2d(path[prev_i].x() - prev_cos_theta * back_edge_to_pt,
                   path[prev_i].y() - prev_sin_theta * back_edge_to_pt);
      front = Vec2d(path[next_i].x() + next_cos_theta * front_edge_to_pt,
                    path[next_i].y() + next_sin_theta * front_edge_to_pt);
    } else {
      rear = Vec2d(path[next_i].x() - next_cos_theta * back_edge_to_pt,
                   path[next_i].y() - next_sin_theta * back_edge_to_pt);
      front = Vec2d(path[prev_i].x() + prev_cos_theta * front_edge_to_pt,
                    path[prev_i].y() + prev_sin_theta * front_edge_to_pt);
    }
    lines_.emplace_back(rear, front);
    lines_.back().Dilate(boundaries_info, objects_info, local_obb_length,
                         local_obb_width, global_constraints_);
    const Vec2d path_point(path[i].x(), path[i].y());
    const auto& hyperplanes = lines_.back().convex_region().hyperplanes();
    if (set_start_to_origin) {
      shifted_hyperplanes.clear();
      shifted_hyperplanes.reserve(hyperplanes.size());
      for (auto& hyperplane : hyperplanes) {
        shifted_hyperplanes.push_back(hyperplane.Shift(-start_point));
      }
      linear_constraints_.emplace_back(path_point - start_point,
                                       shifted_hyperplanes);
    } else {
      linear_constraints_.emplace_back(path_point, hyperplanes);
    }
  }
}
}  // namespace planner
}  // namespace qcraft
