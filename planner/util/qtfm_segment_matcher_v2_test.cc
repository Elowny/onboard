#include "onboard/planner/util/qtfm_segment_matcher_v2.h"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <unordered_map>

// IWYU pragma: no_include "Eigen/Core"

#include "gtest/gtest.h"
#include "onboard/math/util.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner {
namespace {

TEST(QtfmSegmentMatcherV2, GetNearestSegmentIndexSet1) {
  // Equal length segments many polyline.
  unsigned int seed = 0;
  const double buffer = 10.0;
  const double cutoff_distance = 3.0;
  const double length = 1.0;

  auto random_segment_end_point = [&seed, &length](const Vec2d& start) {
    const double angle = (rand_r(&seed) * M_PI * 2.0) / RAND_MAX;
    return start + Vec2d::UnitFromAngle(angle) * length;
  };

  // Generate random segments.
  const int num_segments = 50;
  std::vector<Segment2d> segments;

  std::vector<int> start_ids = {0, 11, 33, 44};
  std::unordered_map<int, Vec2d> start_id_to_pos;
  start_id_to_pos[0] = Vec2d{5.0, 5.0};
  start_id_to_pos[11] = Vec2d{2.0, 5.0};
  start_id_to_pos[33] = Vec2d{5.0, 7.0};
  start_id_to_pos[44] = Vec2d{8.0, 9.0};

  Vec2d prev_end;

  double x_min = 5.0, x_max = 5.0, y_min = 5.0, y_max = 5.0;
  for (int i = 0; i < num_segments; ++i) {
    if (ContainsKey(start_id_to_pos, i)) {
      prev_end = start_id_to_pos[i];
      x_min = std::min(x_min, prev_end.x());
      x_max = std::max(x_max, prev_end.x());
      y_min = std::min(y_min, prev_end.y());
      y_max = std::max(y_max, prev_end.y());
    }

    const Vec2d curr_end = random_segment_end_point(prev_end);
    segments.emplace_back(prev_end, curr_end);
    prev_end = curr_end;

    x_min = std::min(x_min, prev_end.x());
    x_max = std::max(x_max, prev_end.x());
    y_min = std::min(y_min, prev_end.y());
    y_max = std::max(y_max, prev_end.y());
  }

  auto random_sample = [&seed, &x_min, &x_max, &y_min, &y_max, &buffer]() {
    const double rx = (rand_r(&seed) * 1.0) / RAND_MAX;
    const double ry = (rand_r(&seed) * 1.0) / RAND_MAX;
    return Vec2d{rx * (x_max - x_min + 2 * buffer) + x_min - buffer,
                 ry * (y_max - y_min + 2 * buffer) + y_min - buffer};
  };

  auto brutal_force_nearest_segment_id =
      [&segments, &cutoff_distance](const Vec2d& point) {
        int nearest_id = -1;
        double nearest_dist_sqr = std::numeric_limits<double>::infinity();
        for (int i = 0; i < segments.size(); ++i) {
          double dist_sqr = segments[i].DistanceSquareTo(point);

          if (dist_sqr < Sqr(cutoff_distance) && dist_sqr < nearest_dist_sqr) {
            nearest_id = i;
            nearest_dist_sqr = dist_sqr;
          }
        }
        return nearest_id;
      };

  // Construct a segment matcher
  QtfmSegmentMatcherV2::Config config{
      .x_min = x_min - cutoff_distance,
      .x_max = x_max + cutoff_distance,
      .y_min = y_min - cutoff_distance,
      .y_max = y_max + cutoff_distance,
      .resolution = 0.25,
      .qtfm_depth = 5,
      .cutoff_num_range = 1,
      .cutoff_max_num_candidate = 16,
      .cutoff_distance = cutoff_distance,
  };
  QtfmSegmentMatcherV2 matcher(config, segments, start_ids);

  const int num_sample = 1000;
  for (int i = 0; i < num_sample; ++i) {
    const Vec2d sample = random_sample();
    int nearest_seg_id_from_matcher = -1;
    matcher.GetNearestSegmentIndex(sample.x(), sample.y(),
                                   &nearest_seg_id_from_matcher);
    int nearest_seg_id_from_brutal_force =
        brutal_force_nearest_segment_id(sample);
    if (nearest_seg_id_from_brutal_force != -1) {
      CHECK_GE(nearest_seg_id_from_matcher, 0);
      const double brute_force_dist =
          segments[nearest_seg_id_from_brutal_force].DistanceTo(sample);
      const double matcher_dist =
          segments[nearest_seg_id_from_matcher].DistanceTo(sample);
      EXPECT_NEAR(matcher_dist, brute_force_dist, 1e-9)
          << "sample:" << sample.DebugString() << "\n"
          << "brute forceseg:"
          << segments[nearest_seg_id_from_brutal_force].DebugString() << "\n"
          << "Dist:" << nearest_seg_id_from_brutal_force << "\n"
          << "matcher seg:"
          << segments[nearest_seg_id_from_matcher].DebugString() << "\n"
          << "Dist:" << nearest_seg_id_from_matcher;
    }
  }
}

TEST(QtfmSegmentMatcherV2, GetNearestSegmentIndexSet2) {
  // Unequal length segments, one polyline.
  unsigned int seed = 0;
  const double width = 10.0;
  const double buffer = 10.0;
  const double cutoff_distance = 3.0;
  auto random_segment_end_point = [&seed, &width]() {
    const double ex = (rand_r(&seed) * 1.0) / RAND_MAX;
    const double ey = (rand_r(&seed) * 1.0) / RAND_MAX;
    return Vec2d{width * ex, width * ey};
  };

  auto random_sample = [&seed, &width, &buffer]() {
    const double rx = (rand_r(&seed) * 1.0) / RAND_MAX;
    const double ry = (rand_r(&seed) * 1.0) / RAND_MAX;
    return Vec2d{rx * (width + 2 * buffer) - buffer,
                 ry * (width + 2 * buffer) - buffer};
  };

  // Generate random segments connected one by one.
  const int num_segments = 50;
  std::vector<Segment2d> segments;
  Vec2d prev_end{5.0, 5.0};
  for (int i = 0; i < num_segments; ++i) {
    const Vec2d curr_end = random_segment_end_point();
    segments.emplace_back(prev_end, curr_end);
    prev_end = curr_end;
  }

  auto brutal_force_nearest_segment_id =
      [&segments, &cutoff_distance](const Vec2d& point) {
        int nearest_id = -1;
        double nearest_dist_sqr = std::numeric_limits<double>::infinity();
        for (int i = 0; i < segments.size(); ++i) {
          double dist_sqr = segments[i].DistanceSquareTo(point);

          if (dist_sqr < Sqr(cutoff_distance) && dist_sqr < nearest_dist_sqr) {
            nearest_id = i;
            nearest_dist_sqr = dist_sqr;
          }
        }
        return nearest_id;
      };

  // Construct a segment matcher
  QtfmSegmentMatcherV2::Config config{
      .x_min = -buffer,
      .x_max = width + buffer,
      .y_min = -buffer,
      .y_max = width + buffer,
      .resolution = 0.25,
      .cutoff_num_range = 1,
      .cutoff_max_num_candidate = 16,
      .cutoff_distance = cutoff_distance,
  };

  QtfmSegmentMatcherV2 matcher(config, segments, std::vector<int>{0});

  const int num_sample = 1000;
  for (int i = 0; i < num_sample; ++i) {
    const Vec2d sample = random_sample();
    int nearest_seg_id_from_matcher = -1;
    matcher.GetNearestSegmentIndex(sample.x(), sample.y(),
                                   &nearest_seg_id_from_matcher);
    int nearest_seg_id_from_brutal_force =
        brutal_force_nearest_segment_id(sample);
    if (nearest_seg_id_from_brutal_force != -1) {
      CHECK_GE(nearest_seg_id_from_matcher, 0);
      const double brute_force_dist =
          segments[nearest_seg_id_from_brutal_force].DistanceTo(sample);
      const double matcher_dist =
          segments[nearest_seg_id_from_matcher].DistanceTo(sample);
      EXPECT_NEAR(matcher_dist, brute_force_dist, 1e-9)
          << "sample:" << sample.DebugString() << "\n"
          << "brute forceseg:"
          << segments[nearest_seg_id_from_brutal_force].DebugString() << "\n"
          << "Dist:" << nearest_seg_id_from_brutal_force << "\n"
          << "matcher seg:"
          << segments[nearest_seg_id_from_matcher].DebugString() << "\n"
          << "Dist:" << nearest_seg_id_from_matcher;
    }
  }
}

}  // namespace
}  // namespace qcraft::planner
