#include "onboard/planner/util/qtfm_segment_matcher_util.h"

#include <memory>

#include "gtest/gtest.h"

namespace qcraft::planner::qtfm_util {
namespace {

TEST(QtfmSegmentMatcherUtil, SegmentRatioRangeInHalfPlane) {
  double start, end;
  Vec2d p0(0.0, 0.0), p1(5.0, 5.0);
  Segment2d hp(Vec2d{2.0, 2.0}, Vec2d{0.0, 2.0});

  EXPECT_TRUE(SegmentRatioRangeInHalfPlane(p0, p1, hp, false, &start, &end));
  EXPECT_NEAR(end, 0.4, 1e-9);
  EXPECT_NEAR(start, 0.0, 1e-9);

  EXPECT_TRUE(SegmentRatioRangeInHalfPlane(p0, p1, hp, true, &start, &end));
  EXPECT_NEAR(end, 1.0, 1e-9);
  EXPECT_NEAR(start, 0.4, 1e-9);
}

TEST(QtfmSegmentMatcherUtil, SegmentIntersectsFanRegion) {
  Vec2d p0(3.0, -10.0), p1(3.0, 10.0);

  Segment2d right_bound(Vec2d{0.0, 0.0}, Vec2d{1.0, -1.0});
  Segment2d left_bound(Vec2d{0.0, 0.0}, Vec2d{1.0, 1.0});

  EXPECT_TRUE(
      qtfm_util::SegmentIntersectsFanRegion(p0, p1, right_bound, left_bound));

  p0.x() = -3.0;
  p1.x() = -3.0;

  EXPECT_FALSE(
      qtfm_util::SegmentIntersectsFanRegion(p0, p1, right_bound, left_bound));
}

}  // namespace
}  // namespace qcraft::planner::qtfm_util
