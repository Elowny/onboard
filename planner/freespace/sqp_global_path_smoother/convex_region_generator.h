#ifndef ONBOARD_PLANNER_FREESPACE_CONVEX_REGION_GENERATOR_H
#define ONBOARD_PLANNER_FREESPACE_CONVEX_REGION_GENERATOR_H
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "onboard/lite/check.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/hyperplane.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/freespace/sqp_global_path_smoother/convex_region_defs.h"

namespace qcraft {
namespace planner {
/**
 * @brief Given a line segment represented by p1 & p2, an obb and some obstacle
 * points. Construct an ellipse whose center is the middle of p1 & p2, long semi
 * axis is parallel to p1-p2. Iteratively shrink the short semi axis such that
 * the ellipse contains no obstacle point in obb. Finally, use the ellipse to
 * get the convex region.
 */
class LineSegmentDecomp {
 public:
  LineSegmentDecomp(const Vec2d& p1, const Vec2d& p2)
      : p1_(p1), p2_(p2), line_(Segment2d(p1, p2)) {}

  /**
   * @brief Main function. Given a series of obstacle points and a local obb,
   * calculate the largest ellipse that contains no obstacle and then generate a
   * collision-free convex region.
   * @param obs A series of Obstacle Points.
   * @param local_obb_length local obb's extra half length, parallel to the line
   * @param local_obb_width local obb's extra half width, perpendicular to the
   * line.
   * @param extra_hyperplanes Extra Hyperplanes added to generated ConvexRegion.
   */
  void Dilate(std::vector<std::pair<double, Segment2d>> boundaries_info,
              std::vector<std::pair<double, Polygon2d>> objects_info,
              double local_obb_length, double local_obb_width,
              std::vector<Hyperplane> extra_hyperplanes) {
    SetBoundariesAndObjectsInfo(std::move(boundaries_info),
                                std::move(objects_info), local_obb_length,
                                local_obb_width);
    FindEllipse();
    FindConvexRegion();
    AddLocalObb(local_obb_length, local_obb_width, convex_region_.get());
    AddExtraHyperplanes(std::move(extra_hyperplanes));
  }

  const ConvexRegion& convex_region() const {
    return *QCHECK_NOTNULL(convex_region_);
  }

 private:
  void FindConvexRegion();
  /**
   * @brief Adding local bounding box around line seg.
   * This virtual bounding box is parallel to the line segment, x-axis is
   * parallel to the line, y-axis is perpendicular to the line.
   */
  void AddLocalObb(double local_obb_length, double local_obb_width,
                   ConvexRegion* region);
  void AddExtraHyperplanes(std::vector<Hyperplane> extra_hyperplanes) {
    for (auto& hyperplane : extra_hyperplanes) {
      convex_region_->AddHyperplane(std::move(hyperplane));
    }
  }

  /**
   * @brief Start from a circle and iteratively shrink short axis until the
   * ellipse contains no obstacle.
   */
  void FindEllipse();
  /**
   * @brief Import obstacle points that are in the local obb.
   * @param obs A series of Obstacle Points.
   * @param local_obb_length local obb's extra half length, parallel to the line
   * @param local_obb_width local obb's extra half width, perpendicular to the
   * line.
   */
  void SetBoundariesAndObjectsInfo(
      std::vector<std::pair<double, Segment2d>> boundaries_info,
      std::vector<std::pair<double, Polygon2d>> objects_info,
      double local_obb_length, double local_obb_width) {
    // Only consider points inside local obb.
    boundaries_info_ = std::move(boundaries_info);
    objects_info_ = std::move(objects_info);
    std::vector<Hyperplane> vs;
    ConvexRegion convex_region(vs);
    AddLocalObb(local_obb_length, local_obb_width, &convex_region);
    convex_region.RemoveOrClipLinesNotInside(&boundaries_info_);
    convex_region.RemoveOrClipPolygonsNotInside(&objects_info_);
  }

  Vec2d p1_;
  Vec2d p2_;
  // line_ must be declared after p1_ and p2_.
  Segment2d line_;

  std::vector<std::pair<double, Segment2d>> boundaries_info_;
  std::vector<std::pair<double, Polygon2d>> objects_info_;
  std::unique_ptr<GeneralEllipse> ellipse_;
  std::unique_ptr<ConvexRegion> convex_region_;
};

class ConvexRegionsGenerator {
 public:
  ConvexRegionsGenerator() = default;
  explicit ConvexRegionsGenerator(const AABox2d& global_aabox) {
    global_constraints_.reserve(4);
    // x.
    global_constraints_.emplace_back(Vec2d(global_aabox.max_x(), 0),
                                     Vec2d(1, 0));
    global_constraints_.emplace_back(Vec2d(global_aabox.min_x(), 0),
                                     Vec2d(-1, 0));
    // y.
    global_constraints_.emplace_back(Vec2d(0, global_aabox.max_y()),
                                     Vec2d(0, 1));
    global_constraints_.emplace_back(Vec2d(0, global_aabox.min_y()),
                                     Vec2d(0, -1));
  }

  const std::vector<LineSegmentDecomp>& line_segment_decomps() const {
    return lines_;
  }
  // Get the constraints as Ax <= b.
  const std::vector<LinearConstraintConvexRegion>& linear_constraints() const {
    return linear_constraints_;
  }

  /**
   * @brief Compute convex region and linear constraint on each path point.
   * @param path The path to Dilate.
   * @param obs A series of obstacle points.
   * @param local_obb_length local obb's extra half length, parallel to the line
   * @param local_obb_width local obb's extra half width, perpendicular to the
   * line.
   * @param front_edge_to_pt Distance between ellipse extreme point along the
   * path point's dir to path point.
   * @param back_edge_to_pt Distance between ellipse extreme point
   * along the opposite path point's dir to path point.
   * @param set_start_to_origin Whether to move the whole path such that the
   * first point is in origin ONLY when generating linear constraints.
   */
  void Dilate(const DiscretizedPath& path, const std::vector<bool>& forwards,
              const std::vector<std::pair<double, Segment2d>>& boundaries_info,
              const std::vector<std::pair<double, Polygon2d>>& objects_info,
              double local_obb_length, double local_obb_width,
              double front_edge_to_pt, double back_edge_to_pt,
              bool set_start_to_origin);

 private:
  std::vector<Vec2d> obs_;

  std::vector<LineSegmentDecomp> lines_;
  std::vector<LinearConstraintConvexRegion> linear_constraints_;
  std::vector<Hyperplane> global_constraints_;
};
}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_FREESPACE_CONVEX_REGION_GENERATOR_H
