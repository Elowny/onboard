#ifndef ONBOARD_PLANNER_SPEED_PATH_APPROX_H_
#define ONBOARD_PLANNER_SPEED_PATH_APPROX_H_

#include <utility>
#include <vector>

#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/offset_rect.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/math/vec.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

// Models a straight piece of segment on path. It is modeled as a box.
class PathSegment : public Box2d {
 public:
  PathSegment(int first_index, int last_index, const Vec2d& first_ra,
              const Vec2d& last_ra, double first_s, double last_s, Box2d box)
      : Box2d(std::move(box)),
        first_index_(first_index),
        last_index_(last_index),
        first_ra_(first_ra),
        last_ra_(last_ra),
        first_s_(first_s),
        last_s_(last_s) {
    radius_ = Box2d::radius();
  }

  int first_index() const { return first_index_; }
  int last_index() const { return last_index_; }

  Vec2d first_ra() const { return first_ra_; }
  Vec2d last_ra() const { return last_ra_; }

  double first_s() const { return first_s_; }
  double last_s() const { return last_s_; }

  double radius() const { return radius_; }

 private:
  int first_index_;
  int last_index_;
  Vec2d first_ra_;
  Vec2d last_ra_;
  double first_s_;
  double last_s_;
  double radius_;
};

// Approximated path that allows a given lateral error.
class PathApprox {
 public:
  PathApprox(std::vector<PathSegment> segments,
             const SegmentMatcherKdtree* path_kd_tree);

  absl::Span<const PathSegment> segments() const { return segments_; }
  const PathSegment& segment(int i) const { return segments_[i]; }

  // Find the segment index corresponding to an index point.
  int PointToSegmentIndex(int index) const {
    QCHECK_GE(index, 0);
    QCHECK_LE(index, segments_.back().last_index());
    return point_to_segment_index_[index];
  }
  absl::Span<const int> point_to_segment_index() const {
    return point_to_segment_index_;
  }
  const SegmentMatcherKdtree* path_kd_tree() const { return path_kd_tree_; }

 private:
  std::vector<PathSegment> segments_;
  // point_to_segment_index_[i] returns the PathSegment that the original path
  // point i is on.
  std::vector<int> point_to_segment_index_;
  const SegmentMatcherKdtree* path_kd_tree_;
};

// The step length is the traveled distance between two neighboring path points.
PathApprox BuildPathApprox(const absl::Span<const PathPoint> path_points,
                           const OffsetRect& rect, double tolerance,
                           const SegmentMatcherKdtree* path_kd_tree);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_PATH_APPROX_H_
