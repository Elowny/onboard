#include "onboard/planner/util/scene_util.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_point.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft::planner {
namespace {
TEST(SceneUtilTest, IsInHighWay) {
  const auto& smm = LoadDojoMap();
  const RouteSections route_sections(
      0.0, 0.6,
      {mapping::SectionId(12437), mapping::SectionId(12439),
       mapping::SectionId(12438)},
      mapping::LanePoint(mapping::ElementId(237), 0.6));
  const RouteSectionsInfo sections_info(smm, &route_sections);
  EXPECT_FALSE(IsInHighWay(smm, sections_info, /*preview_distance=*/100.0));
}

TEST(SceneUtilTest, IsRightMostDrivableLane) {
  const auto& psmm = CreateDojoTestPSMM();
  EXPECT_TRUE(IsRightMostDrivableLane(psmm, mapping::ElementId(3)));
  EXPECT_FALSE(IsRightMostDrivableLane(psmm, mapping::ElementId(2448)));
}

TEST(SceneUtilTest, CountSectionDrivableLanesFromRight) {
  const auto& psmm = CreateDojoTestPSMM();
  const RouteSections route_sections(
      0.0, 0.6,
      {mapping::SectionId(12437), mapping::SectionId(12439),
       mapping::SectionId(12438)},
      mapping::LanePoint(mapping::ElementId(237), 0.6));
  const RouteSectionsInfo sections_info(psmm, &route_sections);
  EXPECT_EQ(CountSectionDrivableLanesFromRight(
                psmm, sections_info.section_segment(0), /*preview_dist=*/100.0),
            3);
}

TEST(SceneUtilTest, PreviewMinDrivableLanes) {
  const auto& psmm = CreateDojoTestPSMM();
  const RouteSections route_sections(
      0.0, 0.6,
      {mapping::SectionId(12437), mapping::SectionId(12439),
       mapping::SectionId(12438)},
      mapping::LanePoint(mapping::ElementId(237), 0.6));
  const RouteSectionsInfo sections_info(psmm, &route_sections);
  EXPECT_EQ(PreviewMinDrivableLanes(psmm, sections_info, /*preview_dist=*/40.0,
                                    /*start_index=*/0),
            3);
  EXPECT_EQ(PreviewMinDrivableLanes(psmm, sections_info, /*preview_dist=*/40.0,
                                    /*start_index=*/1),
            1);
}
}  // namespace
}  // namespace qcraft::planner
