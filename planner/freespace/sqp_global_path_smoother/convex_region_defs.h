#ifndef ONBOARD_PLANNER_FREESPACE_CONVEX_REGION_DEFS_H
#define ONBOARD_PLANNER_FREESPACE_CONVEX_REGION_DEFS_H
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "onboard/math/eigen.h"
#include "onboard/math/geometry/gjk2d.h"
#include "onboard/math/geometry/hyperplane.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
namespace qcraft {
namespace planner {

class ConvexRegion {
 public:
  explicit ConvexRegion(std::vector<Hyperplane> vs) : vs_(std::move(vs)) {}

  /**
   * @brief A HyperPlane is represented by a point p and a unit normal vector n.
   * A ConvexRegion is represented by a series of HyperPlane. The "inside" /
   * "outside" means "inside/outside the ConvexRegion".
   */
  void AddHyperplane(Hyperplane v) { vs_.push_back(std::move(v)); }

  // Check if the point is inside convex region or at the boundary.
  bool IsPointIn(const Vec2d& pt) const {
    if (vs_.empty()) {
      return false;
    }
    constexpr double kEpsilon = 1e-10;
    for (const auto& v : vs_) {
      if (v.SignedDistanceTo(pt) > kEpsilon) {
        return false;
      }
    }
    return true;
  }

  // Check if the point is strictly inside convex region
  bool IsPointStrictlyIn(const Vec2d& pt) const {
    if (vs_.empty()) {
      return false;
    }
    constexpr double kEpsilon = 1e-10;
    for (const auto& v : vs_) {
      if (v.SignedDistanceTo(pt) > -kEpsilon) {
        return false;
      }
    }
    return true;
  }

  // Remove lines outside and clip lines intersecting with the region, the
  // elements' order may be changed.
  void RemoveOrClipLinesNotInside(
      std::vector<std::pair<double, Segment2d>>* lines_info) const {
    int cur = 0;
    // Num of elements that should be removed.
    int num = 0;
    int size = lines_info->size();
    while (cur < size - num) {
      if (!ClipLineSegment(&(*lines_info)[cur])) {
        std::swap((*lines_info)[cur], (*lines_info)[size - 1 - num]);
        ++num;
      } else {
        ++cur;
      }
    }
    lines_info->resize(size - num);
  }

  void RemoveOrClipPolygonsNotInside(
      std::vector<std::pair<double, Polygon2d>>* polygon_info) const {
    int cur = 0;
    // Num of elements that should be removed.
    int num = 0;
    int size = polygon_info->size();
    while (cur < size - num) {
      if (!ClipPolygon(&(*polygon_info)[cur])) {
        std::swap((*polygon_info)[cur], (*polygon_info)[size - 1 - num]);
        ++num;
      } else {
        ++cur;
      }
    }
    polygon_info->resize(size - num);
  }

  // Remove points strictly outside ellipse, the elements' order may
  // be changed.
  void RemovePointsNotInside(std::vector<Vec2d>* pts) const {
    pts->erase(std::remove_if(pts->begin(), pts->end(),
                              [this](const Vec2d& v) { return !IsPointIn(v); }),
               pts->end());
  }

  /**
   * @brief  Clip a line using the convex region
   *
   * @param line_segment
   * @return false if the line is outside the convex region
   */
  bool ClipLineSegment(std::pair<double, Segment2d>* const line_info) const {
    std::vector<Vec2d> points{line_info->second.start(),
                              line_info->second.end()};
    for (const auto& v : vs_) {
      const Hyperplane shifted_plane = v.Shift(v.n() * line_info->first);
      if (!shifted_plane.ClipConvexHull(&points)) {
        return false;
      }
    }
    line_info->second = Segment2d(points[0], points[1]);
    return true;
  }

  bool ClipPolygon(std::pair<double, Polygon2d>* polygon_info) const {
    std::vector<Vec2d> points = polygon_info->second.points();
    for (const auto& v : vs_) {
      const Hyperplane shifted_plane = v.Shift(v.n() * polygon_info->first);
      if (!shifted_plane.ClipConvexHull(&points)) {
        return false;
      }
    }
    polygon_info->second = Polygon2d(points, /*is_convex=*/true);
    return true;
  }

  // Get the hyperplane vector.
  const std::vector<Hyperplane>& hyperplanes() const { return vs_; }

 private:
  // Hyperplane array.
  std::vector<Hyperplane> vs_;  // Normal must go outside.
};

class LinearConstraintConvexRegion {
 public:
  /**
   * @brief Construct from an inside point and hyperplane array.
   * @param pt Point that is inside.
   * @param vs Hyperplane array, normal should go outside.
   */
  LinearConstraintConvexRegion(const Vec2d& pt,
                               const std::vector<Hyperplane>& vs) {
    const int size = vs.size();
    A_.setZero(size, 2);
    b_.setZero(size);
    for (int i = 0; i < size; ++i) {
      const auto& n = vs[i].n();
      const double c = vs[i].p().dot(n);
      if (n.dot(pt) > c) {
        A_.row(i) = -n;
        b_(i) = -c;
      } else {
        A_.row(i) = n;
        b_(i) = c;
      }
    }
  }

  const MatX2d& A() const { return A_; }

  const VecXd& b() const { return b_; }

 private:
  MatX2d A_;
  VecXd b_;
};

class GeneralEllipse {
 public:
  // An ellispse is {Cx+d | ||x||_2 <= 1}.
  GeneralEllipse(const Mat2d& C, const Vec2d& d)
      : C_(C),
        C_inverse_(C_.inverse()),
        C_inv_sqr_(C_inverse_ * C_inverse_.transpose()),
        d_(d) {}

  // Calculate distance to the center in inverse image.
  double InvDistanceSquareToCenter(const Vec2d& pt) const {
    return (C_inverse_ * (pt - d_)).squaredNorm();
  }

  // Check if the point is inside.
  bool IsPointIn(const Vec2d& pt) const {
    return InvDistanceSquareToCenter(pt) <= 1.0;
  }

  // Check if the point is strictly inside.
  bool IsPointStrictlyIn(const Vec2d& pt) const {
    constexpr double kEpsilon = 1e-10;
    return InvDistanceSquareToCenter(pt) < 1.0 - kEpsilon;
  }

  bool IsLineOut(const Segment2d& line) const {
    const Segment2d inv_line(C_inverse_ * (line.start() - d_),
                             C_inverse_ * (line.end() - d_));
    const Vec2d origin(0.0, 0.0);
    return inv_line.DistanceSquareTo(origin) >= 1.0;
  }

  bool IsPolygonOut(const Polygon2d& object) const {
    const Polygon2d inv_object =
        object.AffineTransform(C_inverse_, -C_inverse_ * d_);
    const Vec2d origin(0.0, 0.0);
    return inv_object.DistanceSquareTo(origin) >= 1.0;
  }

  // Remove lines outside ellipse, the elements' order may be changed.
  void RemoveLinesOut(
      std::vector<std::pair<double, Segment2d>>* lines_info) const {
    lines_info->erase(
        std::remove_if(lines_info->begin(), lines_info->end(),
                       [this](const std::pair<double, Segment2d>& v) {
                         return IsLineOut(v.second);
                       }),
        lines_info->end());
  }

  void RemovePolygonsOut(
      std::vector<std::pair<double, Polygon2d>>* objects_info) const {
    objects_info->erase(
        std::remove_if(objects_info->begin(), objects_info->end(),
                       [this](const std::pair<double, Polygon2d>& v) {
                         return IsPolygonOut(v.second);
                       }),
        objects_info->end());
  }

  // Find the closest point to origin in inverse image.

  Vec2d GetClosestPoint(
      const std::vector<std::pair<double, Polygon2d>>& polygons_info) const {
    constexpr double kEpsilon = 1e-8;
    constexpr int kMaxIterations = 10;

    const Vec2d C_inv_dot_d = C_inverse_ * d_;
    Vec2d closest_point;
    double min_dis = std::numeric_limits<double>::infinity();
    for (const auto& polygon_info : polygons_info) {
      const auto inv_polygon =
          polygon_info.second.AffineTransform(C_inverse_, -C_inv_dot_d);
      const auto calc_extreme_point = [&inv_polygon,
                                       buffer = polygon_info.first,
                                       this](const Vec2d& dir_vec) -> Vec2d {
        const Vec2d perp = C_ * dir_vec.Perp();
        const Vec2d vec = -perp.Perp();
        const Vec2d inv_circle_point = C_inverse_ * vec.normalized() * buffer;
        return inv_polygon.points()[inv_polygon.ExtremePoint(dir_vec)] +
               inv_circle_point;
      };
      Vec2d cur_closest_point;
      const double distance =
          Gjk2d::DistanceSquareTo(inv_polygon.centroid(), calc_extreme_point,
                                  kEpsilon, kMaxIterations, &cur_closest_point);
      if (distance < min_dis) {
        min_dis = distance;
        closest_point = cur_closest_point;
      }
    }
    return C_ * closest_point + d_;
  }

  Vec2d GetClosestPoint(
      const std::vector<std::pair<double, Segment2d>>& lines_info) const {
    constexpr double kEpsilon = 1e-8;
    constexpr int kMaxIterations = 10;

    const Vec2d C_inv_dot_d = C_inverse_ * d_;
    Vec2d closest_point;
    double min_dis = std::numeric_limits<double>::infinity();
    for (const auto& line_info : lines_info) {
      const auto inv_line =
          line_info.second.AffineTransform(C_inverse_, -C_inv_dot_d);
      const auto calc_extreme_point = [&inv_line, buffer = line_info.first,
                                       this](const Vec2d& dir_vec) -> Vec2d {
        const auto line_point = dir_vec.dot(inv_line.unit_direction()) < 0.0
                                    ? inv_line.start()
                                    : inv_line.end();
        const Vec2d perp = C_ * dir_vec.Perp();
        const Vec2d vec = -perp.Perp();
        const Vec2d inv_circle_point = C_inverse_ * vec.normalized() * buffer;
        return line_point + inv_circle_point;
      };
      Vec2d cur_closest_point;
      const double distance =
          Gjk2d::DistanceSquareTo(inv_line.center(), calc_extreme_point,
                                  kEpsilon, kMaxIterations, &cur_closest_point);
      if (distance < min_dis) {
        min_dis = distance;
        closest_point = cur_closest_point;
      }
    }
    return C_ * closest_point + d_;
  }

  // Find the closest hyperplane from the closest point.
  template <typename T>
  Hyperplane GetClosestHyperplane(const std::vector<T>& objs) const {
    const Vec2d closest_pt = GetClosestPoint(objs);
    const Vec2d n = C_inv_sqr_ * (closest_pt - d_);
    return Hyperplane(closest_pt, n.normalized());
  }

  const Mat2d& C() const { return C_; }

  // Get center
  const Vec2d& d() const { return d_; }

  void set_C(Mat2d input_C) {
    C_ = std::move(input_C);
    C_inverse_ = C().inverse();
    C_inv_sqr_ = C_inverse_ * C_inverse_.transpose();
  }

 private:
  Mat2d C_;
  Mat2d C_inverse_;
  Mat2d C_inv_sqr_;
  Vec2d d_;
};
}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_FREESPACE_CONVEX_REGION_DEFS_H
