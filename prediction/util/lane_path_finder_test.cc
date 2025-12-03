#include "onboard/prediction/util/lane_path_finder.h"

#include <stddef.h>
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <utility>

#include "absl/strings/str_join.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::prediction {
namespace {
template <class T>
std::string VectorDebugString(const std::vector<T>& set) {
  return absl::StrJoin(set, " ");
}
template <class T>
void ExpectVectorEqual(const std::vector<T>& a, const std::vector<T>& b) {
  EXPECT_EQ(a.size(), b.size());
  for (int i = 0, n = a.size(); i < n; ++i) {
    EXPECT_EQ(a[i], b[i]);
  }
}

planner::DiscretizedPath ComputeDiscretizedPath(
    const std::vector<Vec2d>& path_pt) {
  QCHECK_GE(path_pt.size(), 3);
  planner::DiscretizedPath path;
  for (size_t i = 0; i < path_pt.size(); ++i) {
    const auto& cur_pt = path_pt[i];
    PathPoint pt;
    pt.set_x(cur_pt.x());
    pt.set_y(cur_pt.y());
    pt.set_z(0.0);
    if (i + 1 < path_pt.size()) {
      const auto& next_pt = path_pt[i + 1];
      const auto seg = next_pt - cur_pt;
      pt.set_theta(seg.Angle());
    } else {
      // Last path point, set heading equal to the heading of the previous
      // point.
      pt.set_theta(path.back().theta());
    }
    if (i == 0) {
      pt.set_s(0.0);  // Path length at front = 0.0.
    } else {
      const auto& prev_pt = path_pt[i - 1];
      const double ds = (cur_pt - prev_pt).Length();
      pt.set_s(path.back().s() + ds);  // Accumulating.
    }
    path.push_back(std::move(pt));
  }
  return path;
}

TEST(NearestLaneTest, NearestLaneTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const auto lane1 =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          psmm, /*pt=*/Vec2d(9.4, 17.6), /*heading=*/-M_PI,
          /*boundary_distance_limit=*/0.0,
          /*max_heading_diff=*/M_PI / 3.0);
  EXPECT_EQ(lane1, std::nullopt);
  const auto lane2 =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          psmm, /*pt=*/Vec2d(5.4, 14.1), /*heading=*/-M_PI,
          /*boundary_distance_limit=*/0.0,
          /*max_heading_diff=*/M_PI / 3.0);
  EXPECT_EQ(*lane2, mapping::ElementId(6));
  const auto lane3 =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          psmm, /*pt=*/Vec2d(5.4, 14.1), /*heading=*/0,
          /*boundary_distance_limit=*/0.0,
          /*max_heading_diff=*/M_PI / 3.0);
  EXPECT_EQ(lane3, std::nullopt);
  const auto lane4 =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          psmm, /*pt=*/Vec2d(2.8, 16.3), /*heading=*/-M_PI,
          /*boundary_distance_limit=*/1.0,
          /*max_heading_diff=*/M_PI / 3.0);
  EXPECT_EQ(*lane4, mapping::ElementId(6));
  const auto lane5 = FindNearestLaneIdWithBoundaryDistanceLimit(
      psmm, /*pt=*/Vec2d(5.4, 14.1),
      /*boundary_distance_limit=*/0.0);
  EXPECT_EQ(*lane5, mapping::ElementId(6));
}

TEST(LaneIdFilter, LaneIdFilterTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  std::set<mapping::ElementId> input = {
      mapping::ElementId(5), mapping::ElementId(8), mapping::ElementId(100)};
  const auto out = FilterLanesByConsecutiveRelationship(
      psmm, input, /*filter_virtual=*/false);
  EXPECT_EQ(out.size(), 1);
  std::set<mapping::ElementId> input2 = {
      mapping::ElementId(5), mapping::ElementId(8), mapping::ElementId(11),
      mapping::ElementId(100)};
  const auto out2 = FilterLanesByConsecutiveRelationship(
      psmm, input, /*filter_virtual=*/false);
  EXPECT_EQ(out2.size(), 1);
}

TEST(SearchMostStraightLanePath, SearchMostStraightLanePathTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const auto lp = SearchMostStraightLanePath(Vec2d(4.5, 10.8), psmm,
                                             /*lane_id=*/mapping::ElementId(5),
                                             /*forward_len=*/50.0,
                                             /*is_reverse_driving=*/false);
  const std::vector<mapping::ElementId> answer = {
      mapping::ElementId(5), mapping::ElementId(100), mapping::ElementId(94)};
  LOG(INFO) << "seq :" << VectorDebugString(lp.lane_ids());
  ExpectVectorEqual(lp.lane_ids(), answer);

  const auto lp2 = SearchMostStraightLanePath(Vec2d(4.5, 10.8), psmm,
                                              /*lane_id=*/mapping::ElementId(4),
                                              /*forward_len=*/50.0,
                                              /*is_reverse_driving=*/true);
  LOG(INFO) << "seq :" << VectorDebugString(lp2.lane_ids());

  const std::vector<mapping::ElementId> answer2 = {mapping::ElementId(4)};
  ExpectVectorEqual(lp2.lane_ids(), answer2);
}

TEST(SearchLanePath, SearchLanePathTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const auto res = SearchLanePath(Vec2d(4.5, 10.8), psmm, mapping::ElementId(5),
                                  /*forward_len=*/50.0,
                                  /*is_reverse_driving=*/false);
  EXPECT_EQ(res.size(), 2);

  const auto res2 =
      SearchLanePath(Vec2d(4.5, 10.8), psmm, mapping::ElementId(4),
                     /*forward_len=*/50.0,
                     /*is_reverse_driving=*/true);
  EXPECT_EQ(res2.size(), 2);
}

TEST(FindConsecutiveLanesForDiscretizedPath, RightTurn) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  // Create fake right turning path.
  std::vector<Vec2d> path_pos;
  path_pos.push_back(Vec2d(-31.6, -70.0));
  path_pos.push_back(Vec2d(-27.1, -70.3));
  path_pos.push_back(Vec2d(-24.4, -71.3));
  path_pos.push_back(Vec2d(-22.2, -73.2));
  path_pos.push_back(Vec2d(-21.1, -76.1));
  path_pos.push_back(Vec2d(-20.8, -79.5));
  path_pos.push_back(Vec2d(-20.9, -82.6));
  path_pos.push_back(Vec2d(-20.3, -87.0));

  const auto path = ComputeDiscretizedPath(path_pos);
  const auto lanes_or =
      FindConsecutiveLanesForDiscretizedPath(psmm, path, /*is_reversed=*/false);

  std::vector<mapping::ElementId> correct_ans;
  correct_ans.push_back(mapping::ElementId(645));
  correct_ans.push_back(mapping::ElementId(609));
  correct_ans.push_back(mapping::ElementId(610));

  EXPECT_OK(lanes_or);
  EXPECT_EQ(*lanes_or, correct_ans);
}

TEST(FindConsecutiveLanesForDiscretizedPath, Straight) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  // Create fake straight path. (Same first two points as the right turn case).
  std::vector<Vec2d> path_pos;
  path_pos.push_back(Vec2d(-31.6, -70.0));
  path_pos.push_back(Vec2d(-27.1, -70.3));
  path_pos.push_back(Vec2d(-18.4, -70.0));
  path_pos.push_back(Vec2d(-11.4, -70.0));
  const auto path = ComputeDiscretizedPath(path_pos);
  const auto lanes_or =
      FindConsecutiveLanesForDiscretizedPath(psmm, path, /*is_reversed=*/false);

  std::vector<mapping::ElementId> correct_ans;
  correct_ans.push_back(mapping::ElementId(645));
  correct_ans.push_back(mapping::ElementId(654));

  EXPECT_OK(lanes_or);
  EXPECT_EQ(*lanes_or, correct_ans);
}

// TODO(changqing): Add crazy moving path in intersection.

}  // namespace
}  // namespace qcraft::prediction
