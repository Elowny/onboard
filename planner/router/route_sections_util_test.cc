#include "onboard/planner/router/route_sections_util.h"

#include <string>

#include "absl/hash/hash.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/smm_proto_util.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft::planner {

// NOLINTNEXTLINE(readability-function-size)
TEST(RouteSectionsUtilTest, AlignSectionsTest) {
  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.0, 1.0,
        {mapping::SectionId(1), mapping::SectionId(2), mapping::SectionId(3),
         mapping::SectionId(4), mapping::SectionId(5)},
        destination);
    const RouteSections local_sections(0.2, 1.0, {mapping::SectionId(1)},
                                       destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);
    EXPECT_EQ(aligned_sections->size(), 5);
    EXPECT_NEAR(aligned_sections->start_fraction(), 0.2, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 1.0, 1e-10);
    EXPECT_EQ(aligned_sections->section_ids().front().value(), 1);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.0, 1.0,
        {mapping::SectionId(1), mapping::SectionId(2), mapping::SectionId(3),
         mapping::SectionId(4), mapping::SectionId(5)},
        destination);
    const RouteSections local_sections(0.2, 1.0, {mapping::SectionId(6)},
                                       destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_TRUE(!aligned_sections.ok());
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.0, 1.0,
        {mapping::SectionId(1), mapping::SectionId(2), mapping::SectionId(3),
         mapping::SectionId(4), mapping::SectionId(5)},
        destination);
    const RouteSections local_sections(
        0.2, 1.0, {mapping::SectionId(0), mapping::SectionId(1)}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 6);
    EXPECT_NEAR(aligned_sections->start_fraction(), 0.2, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 1.0, 1e-10);

    for (int i = 0; i < 6; ++i)
      EXPECT_EQ(aligned_sections->section_ids()[i].value(), i);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.0, 1.0,
        {mapping::SectionId(1), mapping::SectionId(2), mapping::SectionId(3)},
        destination);
    const RouteSections local_sections(
        0.0, 0.9, {mapping::SectionId(1), mapping::SectionId(2)}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 3);
    EXPECT_NEAR(aligned_sections->start_fraction(), 0.0, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 1.0, 1e-10);

    for (int i = 0; i < 3; ++i)
      EXPECT_EQ(aligned_sections->section_ids()[i].value(), i + 1);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(0.0, 0.9, {mapping::SectionId(1)},
                                        destination);
    const RouteSections local_sections(0.1, 1.0, {mapping::SectionId(1)},
                                       destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 1);
    EXPECT_EQ(aligned_sections->front().id.value(), 1);

    EXPECT_NEAR(aligned_sections->start_fraction(), 0.1, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 0.9, 1e-10);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(0.2, 0.9, {mapping::SectionId(1)},
                                        destination);
    const RouteSections local_sections(0.1, 1.0, {mapping::SectionId(1)},
                                       destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 1);
    EXPECT_EQ(aligned_sections->front().id.value(), 1);

    EXPECT_NEAR(aligned_sections->start_fraction(), 0.1, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 0.9, 1e-10);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.2, 0.9, {mapping::SectionId(1), mapping::SectionId(2)}, destination);
    const RouteSections local_sections(
        0.1, 1.0,
        {mapping::SectionId(0), mapping::SectionId(1), mapping::SectionId(2)},
        destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 3);
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(aligned_sections->section_ids()[i].value(), i);
    }

    EXPECT_NEAR(aligned_sections->start_fraction(), 0.1, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 0.9, 1e-10);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(0.2, 0.9, {mapping::SectionId(1)},
                                        destination);
    const RouteSections local_sections(0.1, 0.8, {mapping::SectionId(1)},
                                       destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 1);
    EXPECT_EQ(aligned_sections->front().id.value(), 1);

    EXPECT_NEAR(aligned_sections->start_fraction(), 0.1, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 0.9, 1e-10);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.2, 0.9, {mapping::SectionId(1), mapping::SectionId(2)}, destination);
    const RouteSections local_sections(
        0.1, 0.7,
        {mapping::SectionId(0), mapping::SectionId(1), mapping::SectionId(2)},
        destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 3);
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(aligned_sections->section_ids()[i].value(), i);
    }

    EXPECT_NEAR(aligned_sections->start_fraction(), 0.1, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 0.9, 1e-10);
  }
}

TEST(RouteSectionUtilTest, SpliceRouteSectionsTest) {
  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections target_sections(
        0.0, 1.0,
        {mapping::SectionId(1), mapping::SectionId(2), mapping::SectionId(3),
         mapping::SectionId(4), mapping::SectionId(5)},
        destination);
    const RouteSections origin_sections(0.2, 1.0, {mapping::SectionId(1)},
                                        destination);

    const auto splice_sections =
        SpliceRouteSections(origin_sections, target_sections);
    EXPECT_OK(splice_sections);
    EXPECT_EQ(splice_sections->size(), 5);
    EXPECT_NEAR(splice_sections->start_fraction(), 0.2, 1e-10);
    EXPECT_NEAR(splice_sections->end_fraction(), 1.0, 1e-10);
    EXPECT_EQ(splice_sections->section_ids().front().value(), 1);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections target_sections(
        0.0, 1.0,
        {mapping::SectionId(1), mapping::SectionId(2), mapping::SectionId(3),
         mapping::SectionId(4), mapping::SectionId(5)},
        destination);
    const RouteSections origin_sections(
        0.2, 1.0, {mapping::SectionId(1), mapping::SectionId(2)}, destination);

    const auto splice_sections =
        SpliceRouteSections(origin_sections, target_sections);
    EXPECT_OK(splice_sections);
    EXPECT_EQ(splice_sections->size(), 5);
    EXPECT_NEAR(splice_sections->start_fraction(), 0.2, 1e-10);
    EXPECT_NEAR(splice_sections->end_fraction(), 1.0, 1e-10);
    EXPECT_EQ(splice_sections->section_ids().front().value(), 1);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections target_sections(
        0.0, 0.8,
        {mapping::SectionId(1), mapping::SectionId(2), mapping::SectionId(3)},
        destination);
    const RouteSections origin_sections(
        0.2, 1.0, {mapping::SectionId(2), mapping::SectionId(3)}, destination);

    const auto splice_sections =
        SpliceRouteSections(origin_sections, target_sections);
    EXPECT_OK(splice_sections);
    EXPECT_EQ(splice_sections->size(), 2);
    EXPECT_NEAR(splice_sections->start_fraction(), 0.2, 1e-10);
    EXPECT_NEAR(splice_sections->end_fraction(), 0.8, 1e-10);
    EXPECT_EQ(splice_sections->section_ids().front().value(), 2);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections target_sections(0.0, 0.8, {mapping::SectionId(2)},
                                        destination);
    const RouteSections origin_sections(
        0.2, 1.0, {mapping::SectionId(2), mapping::SectionId(3)}, destination);

    const auto splice_sections =
        SpliceRouteSections(origin_sections, target_sections);
    EXPECT_OK(splice_sections);
    EXPECT_EQ(splice_sections->size(), 1);
    EXPECT_NEAR(splice_sections->start_fraction(), 0.2, 1e-10);
    EXPECT_NEAR(splice_sections->end_fraction(), 0.8, 1e-10);
    EXPECT_EQ(splice_sections->section_ids().front().value(), 2);
  }

  {
    const mapping::LanePoint destination(mapping::ElementId(10), 1.0);

    const RouteSections target_sections(
        0.0, 0.8,
        {mapping::SectionId(1), mapping::SectionId(2), mapping::SectionId(3)},
        destination);
    const RouteSections origin_sections(
        0.2, 1.0, {mapping::SectionId(0), mapping::SectionId(1)}, destination);

    const auto splice_sections =
        SpliceRouteSections(origin_sections, target_sections);
    EXPECT_OK(splice_sections);
    EXPECT_EQ(splice_sections->size(), 4);
    EXPECT_NEAR(splice_sections->start_fraction(), 0.2, 1e-10);
    EXPECT_NEAR(splice_sections->end_fraction(), 0.8, 1e-10);
    EXPECT_EQ(splice_sections->section_ids().front().value(), 0);
  }
}

TEST(RouteSectionUtilTest, AppendSectionsToTailTest) {
  {
    const mapping::LanePoint origin_destination(mapping::ElementId(111), 1.0);
    const mapping::LanePoint global_destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.0, 1.0,
        {mapping::SectionId(1), mapping::SectionId(2), mapping::SectionId(3),
         mapping::SectionId(4), mapping::SectionId(5)},
        global_destination);
    const RouteSections origin_sections(0.2, 1.0, {mapping::SectionId(1)},
                                        origin_destination);

    const auto res_sections =
        AppendRouteSectionsToTail(origin_sections, global_sections);
    EXPECT_OK(res_sections);
    EXPECT_DOUBLE_EQ(res_sections->start_fraction(), 0.2);
    EXPECT_DOUBLE_EQ(res_sections->end_fraction(), 1.0);
    EXPECT_EQ(res_sections->destination().lane_id().value(), 10);
    EXPECT_THAT(
        res_sections->section_ids(),
        testing::ElementsAre(mapping::SectionId(1), mapping::SectionId(2),
                             mapping::SectionId(3), mapping::SectionId(4),
                             mapping::SectionId(5)));
  }

  {
    const mapping::LanePoint origin_destination(mapping::ElementId(111), 1.0);
    const mapping::LanePoint global_destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.0, 1.0,
        {mapping::SectionId(1), mapping::SectionId(2), mapping::SectionId(3),
         mapping::SectionId(1), mapping::SectionId(2)},
        global_destination);
    const RouteSections origin_sections(0.2, 1.0, {mapping::SectionId(2)},
                                        origin_destination);

    const auto res_sections =
        AppendRouteSectionsToTail(origin_sections, global_sections);
    EXPECT_OK(res_sections);
    EXPECT_DOUBLE_EQ(res_sections->start_fraction(), 0.2);
    EXPECT_DOUBLE_EQ(res_sections->end_fraction(), 1.0);
    EXPECT_EQ(res_sections->destination().lane_id().value(), 10);
    EXPECT_THAT(
        res_sections->section_ids(),
        testing::ElementsAre(mapping::SectionId(2), mapping::SectionId(3),
                             mapping::SectionId(1), mapping::SectionId(2)));
  }

  {
    const mapping::LanePoint origin_destination(mapping::ElementId(111), 1.0);
    const mapping::LanePoint global_destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.0, 1.0,
        {mapping::SectionId(2), mapping::SectionId(3), mapping::SectionId(4),
         mapping::SectionId(5)},
        global_destination);
    const RouteSections origin_sections(0.2, 1.0, {mapping::SectionId(3)},
                                        origin_destination);

    const auto res_sections =
        AppendRouteSectionsToTail(origin_sections, global_sections);
    EXPECT_OK(res_sections);
    EXPECT_DOUBLE_EQ(res_sections->start_fraction(), 0.2);
    EXPECT_DOUBLE_EQ(res_sections->end_fraction(), 1.0);
    EXPECT_EQ(res_sections->destination().lane_id().value(), 10);
    EXPECT_THAT(
        res_sections->section_ids(),
        testing::ElementsAre(mapping::SectionId(3), mapping::SectionId(4),
                             mapping::SectionId(5)));
  }

  {
    const mapping::LanePoint origin_destination(mapping::ElementId(111), 1.0);
    const mapping::LanePoint global_destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.0, 1.0,
        {mapping::SectionId(2), mapping::SectionId(3), mapping::SectionId(4),
         mapping::SectionId(5)},
        global_destination);
    const RouteSections origin_sections(
        0.2, 1.0, {mapping::SectionId(1), mapping::SectionId(2)},
        origin_destination);

    const auto res_sections =
        AppendRouteSectionsToTail(origin_sections, global_sections);
    EXPECT_OK(res_sections);
    EXPECT_DOUBLE_EQ(res_sections->start_fraction(), 0.2);
    EXPECT_DOUBLE_EQ(res_sections->end_fraction(), 1.0);
    EXPECT_EQ(res_sections->destination().lane_id().value(), 10);
    EXPECT_THAT(
        res_sections->section_ids(),
        testing::ElementsAre(mapping::SectionId(1), mapping::SectionId(2),
                             mapping::SectionId(3), mapping::SectionId(4),
                             mapping::SectionId(5)));
  }

  {
    const mapping::LanePoint origin_destination(mapping::ElementId(111), 1.0);
    const mapping::LanePoint global_destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.0, 1.0,
        {mapping::SectionId(2), mapping::SectionId(3), mapping::SectionId(4),
         mapping::SectionId(5)},
        global_destination);
    const RouteSections origin_sections(
        0.2, 1.0,
        {mapping::SectionId(2), mapping::SectionId(3), mapping::SectionId(4),
         mapping::SectionId(5)},
        origin_destination);

    const auto res_sections =
        AppendRouteSectionsToTail(origin_sections, global_sections);
    EXPECT_OK(res_sections);
    EXPECT_DOUBLE_EQ(res_sections->start_fraction(), 0.2);
    EXPECT_DOUBLE_EQ(res_sections->end_fraction(), 1.0);
    EXPECT_EQ(res_sections->destination().lane_id().value(), 10);
    EXPECT_THAT(
        res_sections->section_ids(),
        testing::ElementsAre(mapping::SectionId(2), mapping::SectionId(3),
                             mapping::SectionId(4), mapping::SectionId(5)));
  }

  {
    const mapping::LanePoint origin_destination(mapping::ElementId(111), 1.0);
    const mapping::LanePoint global_destination(mapping::ElementId(10), 1.0);

    const RouteSections global_sections(
        0.0, 1.0,
        {mapping::SectionId(2), mapping::SectionId(3), mapping::SectionId(4),
         mapping::SectionId(5)},
        global_destination);
    const RouteSections origin_sections(
        0.2, 1.0, {mapping::SectionId(2), mapping::SectionId(6)},
        origin_destination);

    const auto res_sections =
        AppendRouteSectionsToTail(origin_sections, global_sections);
    EXPECT_NOT_OK(res_sections);
  }
}

TEST(RouteSectionsUtilTest, FindClosestLanePathOnRouteSectionsToSmoothPoint) {
  SetMap("dojo");

  const auto& psmm = CreateDojoTestPSMM();
  const mapping::LanePoint destination(mapping::ElementId(2555), 0.9);
  const RouteSections route_sections(
      0.2, 0.9, {mapping::SectionId(12442), mapping::SectionId(12443)},
      destination);

  double proj_s;
  const auto closest_lane_path_or =
      FindClosestLanePathOnRouteSectionsToSmoothPoint(
          psmm, route_sections, Vec2d(345.0, 12.5), &proj_s);
  ASSERT_OK(closest_lane_path_or);

  EXPECT_EQ(closest_lane_path_or->front().lane_id().value(), 230);
  EXPECT_NEAR(closest_lane_path_or->start_fraction(), 0.2, 1e-5);

  EXPECT_EQ(closest_lane_path_or->back().lane_id().value(), 2554);
  EXPECT_NEAR(closest_lane_path_or->end_fraction(), 0.9, 1e-5);

  EXPECT_NEAR(proj_s, 19.0, 0.1);
}

TEST(ClampRouteSectionsBeforeArcLengthTest, ClampRouteSections) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const mapping::LanePoint dest(mapping::ElementId(10), 1.0);

  {
    RouteSections raw_sections(0.0, 0.9, {mapping::SectionId(12415)}, dest);

    const auto clamped_sections =
        ClampRouteSectionsBeforeArcLength(psmm, raw_sections, 10.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 1);
    EXPECT_NEAR(clamped_sections->end_fraction(), 0.286, 0.01);
  }

  {
    RouteSections raw_sections(
        0.1, 0.9,
        {mapping::SectionId(12318), mapping::SectionId(12412),
         mapping::SectionId(12414)},
        dest);

    const auto clamped_sections =
        ClampRouteSectionsBeforeArcLength(psmm, raw_sections, 30.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 2);
  }

  {
    RouteSections raw_sections(0.0, 1.0, {mapping::SectionId(12004)}, dest);

    const auto clamped_sections =
        ClampRouteSectionsBeforeArcLength(psmm, raw_sections, 70.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 1);
    EXPECT_NEAR(clamped_sections->end_fraction(), 1.0, 0.01);
  }
}

TEST(ClampRouteSectionsAfterArcLength, ClampRouteSections) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const mapping::LanePoint dest(mapping::ElementId(10), 1.0);

  {
    RouteSections raw_sections(0.0, 0.9, {mapping::SectionId(12357)}, dest);

    const auto clamped_sections =
        ClampRouteSectionsAfterArcLength(psmm, raw_sections, 10.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 1);
    EXPECT_NEAR(clamped_sections->start_fraction(), 0.1, 0.01);
  }

  {
    RouteSections raw_sections(
        0.0, 0.9, {mapping::SectionId(12357), mapping::SectionId(12356)}, dest);

    const auto clamped_sections =
        ClampRouteSectionsAfterArcLength(psmm, raw_sections, 105.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 1);
    EXPECT_NEAR(clamped_sections->start_fraction(), 0.11, 0.01);
  }

  {
    RouteSections raw_sections(
        0.0, 0.99,
        {mapping::SectionId(12357), mapping::SectionId(12356),
         mapping::SectionId(12355)},
        dest);

    const auto clamped_sections =
        ClampRouteSectionsAfterArcLength(psmm, raw_sections, 135.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 1);
    EXPECT_NEAR(clamped_sections->start_fraction(), 0.11, 0.01);
  }

  {
    RouteSections raw_sections(
        0.0, 0.99,
        {mapping::SectionId(12357), mapping::SectionId(12356),
         mapping::SectionId(12355)},
        dest);

    const auto clamped_sections =
        ClampRouteSectionsAfterArcLength(psmm, raw_sections, 50.0);
    EXPECT_OK(clamped_sections);
    EXPECT_EQ(clamped_sections->size(), 3);
    EXPECT_NEAR(clamped_sections->start_fraction(), 0.5, 0.01);
  }

  {
    RouteSections raw_sections(
        0.0, 0.99,
        {mapping::SectionId(12357), mapping::SectionId(12356),
         mapping::SectionId(12355)},
        dest);

    const auto clamped_sections =
        ClampRouteSectionsAfterArcLength(psmm, raw_sections, 160.0);

    EXPECT_FALSE(clamped_sections.ok());
  }
}

TEST(BackwardExtendRouteSections, BackExtend) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  mapping::LanePoint destination(mapping::ElementId(10), 1.0);

  {
    RouteSections raw_sections(0.0, 1.0, {mapping::SectionId(12335)},
                               destination);

    const auto extend_sections =
        BackwardExtendRouteSections(psmm, raw_sections, 50.0);

    EXPECT_EQ(extend_sections.size(), 3);
  }

  {
    RouteSections raw_sections(0.1, 1.0, {mapping::SectionId(12335)},
                               destination);

    const auto extend_sections =
        BackwardExtendRouteSections(psmm, raw_sections, 50.0);
    LOG(INFO) << raw_sections.DebugString();
    EXPECT_EQ(extend_sections.size(), 3);
  }
}

TEST(BackwardExtendRouteSectionsFromPos, BackExtend) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto& cc = psmm.coordinate_converter();

  mapping::LanePoint destination(mapping::ElementId(10), 1.0);

  {
    RouteSections raw_sections(0.0, 1.0, {mapping::SectionId(12401)},
                               destination);

    const auto extend_sections_or = BackwardExtendRouteSectionsFromPos(
        psmm, raw_sections, Vec2d(1.0, -3.5), 5.0);

    ASSERT_OK(extend_sections_or);

    EXPECT_EQ(extend_sections_or->size(), 2);
    EXPECT_TRUE(extend_sections_or->section_ids().front().value() == 12310 ||
                extend_sections_or->section_ids().front().value() == 12201);
  }

  {
    RouteSections raw_sections(0.01, 1.0, {mapping::SectionId(12415)},
                               destination);
    const auto extend_sections_or = BackwardExtendRouteSectionsFromPos(
        psmm, raw_sections, Vec2d(118.615, 68.89), 10.0);
    ASSERT_OK(extend_sections_or);

    EXPECT_EQ(extend_sections_or->size(), 2);
    EXPECT_EQ(extend_sections_or->section_ids().front().value(), 12298);
  }

  {
    const auto section_id = mapping::SectionId(12415);
    RouteSections raw_sections(0, 0, {section_id}, destination);
    const auto section_info = mapping::FindSectionProto(psmm, section_id);
    ASSERT_NE(section_info, nullptr);
    const auto lane_info = mapping::FindLaneProto(
        psmm, mapping::ElementId(section_info->lanes()[0]));
    const auto& geo_point = lane_info->polyline().points(0);
    const auto smooth =
        cc.GlobalToSmooth({geo_point.longitude(), geo_point.latitude()});
    const auto extend_sections_or =
        BackwardExtendRouteSectionsFromPos(psmm, raw_sections, smooth, 10.0);
    ASSERT_OK(extend_sections_or);
  }
}

TEST(RouteSectionsUtilTest, FindClosestTargetLanePathOnReset) {
  const auto& psmm = CreateDojoTestPSMM();

  const mapping::LanePoint destination(mapping::ElementId(46), 0.9);

  const RouteSections route_sections(
      0.0, 0.9,
      {mapping::SectionId(12401), mapping::SectionId(12400),
       mapping::SectionId(12408), mapping::SectionId(12412),
       mapping::SectionId(12414), mapping::SectionId(12285),
       mapping::SectionId(12433), mapping::SectionId(12445),
       mapping::SectionId(12219), mapping::SectionId(12088),
       mapping::SectionId(12078), mapping::SectionId(12087),
       mapping::SectionId(12218), mapping::SectionId(12442),
       mapping::SectionId(12443), mapping::SectionId(12441),
       mapping::SectionId(12287), mapping::SectionId(12410)},
      destination);
  const double behind_length = 10.0;
  const double local_horizon = route_sections.planning_horizon(psmm);
  const RouteSectionsInfo sections_info(psmm, &route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*psmm.semantic_map_manager(), sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  {
    const auto target_lane_path_or = FindClosestTargetLanePathOnReset(
        psmm, route_sections, Vec2d(behind_length, 0.9), route_navi_info);
    ASSERT_OK(target_lane_path_or);

    EXPECT_EQ(target_lane_path_or->front().lane_id(), mapping::ElementId(2448));
    EXPECT_NEAR(target_lane_path_or->start_fraction(), 0.0, 1e-5);

    EXPECT_NEAR(target_lane_path_or->length(), local_horizon + behind_length,
                0.2);

    const auto last_lane_pt = target_lane_path_or->ArclengthToLanePoint(
        local_horizon + behind_length);
    EXPECT_EQ(target_lane_path_or->back().lane_id(), last_lane_pt.lane_id());
    EXPECT_NEAR(target_lane_path_or->end_fraction(), last_lane_pt.fraction(),
                1e-2);
  }

  {
    const auto target_lane_path_or = FindClosestTargetLanePathOnReset(
        psmm, route_sections, Vec2d(behind_length, 2.9), route_navi_info);
    ASSERT_OK(target_lane_path_or);

    EXPECT_EQ(target_lane_path_or->front().lane_id(), mapping::ElementId(2));
    EXPECT_NEAR(target_lane_path_or->start_fraction(), 0.0, 1e-5);

    EXPECT_EQ(target_lane_path_or->back().lane_id(), mapping::ElementId(51));
    EXPECT_NEAR(target_lane_path_or->end_fraction(), 1.0, 1e-5);

    EXPECT_NEAR(target_lane_path_or->length(), 255.5, 0.1);
  }
}

TEST(RouteSectionsUtilTest, CollectAllLanePathOnRouteSections) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const mapping::LanePoint destination(mapping::ElementId(7740), 0.9);

  const RouteSections route_sections(
      0.0, 0.9,
      {mapping::SectionId(12540), mapping::SectionId(12509),
       mapping::SectionId(12507), mapping::SectionId(12637),
       mapping::SectionId(12506)},
      destination);

  {
    const auto lane_path_or =
        CollectAllLanePathOnRouteSections(psmm, route_sections);
    EXPECT_OK(lane_path_or);
    EXPECT_EQ(lane_path_or->size(), 5);
  }

  {
    const auto lane_path_or =
        CollectAllLanePathOnRouteSectionsFromStart(psmm, route_sections);
    EXPECT_OK(lane_path_or);
    EXPECT_EQ(lane_path_or->size(), 2);
  }
}

TEST(BackwardExtendLanePathOnRouteSections, BackwardExtendLanePathTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const mapping::LanePath lane_path(smm, {mapping::ElementId(2448)},
                                    /*start_fraction*/ 0.0,
                                    /*end_fraction*/ 1.0);
  const mapping::LanePoint destination(mapping::ElementId(1), 1.0);

  {
    const RouteSections route_sections(
        0.0, 1.0,
        {mapping::SectionId(12311), mapping::SectionId(12401),
         mapping::SectionId(12400)},
        destination);
    const auto extended_lane_path_or = BackwardExtendLanePathOnRouteSections(
        psmm, route_sections, lane_path, /*extend_len*/ 10.0);

    EXPECT_OK(extended_lane_path_or);
    EXPECT_EQ(extended_lane_path_or->size(), 2);
    EXPECT_EQ(extended_lane_path_or->front().lane_id(), mapping::ElementId(56));
  }

  {
    const RouteSections route_sections(
        0.0, 1.0, {mapping::SectionId(12401), mapping::SectionId(12400)},
        destination);
    const auto extended_lane_path_or = BackwardExtendLanePathOnRouteSections(
        psmm, route_sections, lane_path, /*extend_len*/ 10.0);

    EXPECT_OK(extended_lane_path_or);
    EXPECT_EQ(extended_lane_path_or->size(), 2);
  }

  {
    const RouteSections route_sections(
        0.0, 1.0,
        {mapping::SectionId(12310), mapping::SectionId(12401),
         mapping::SectionId(12400)},
        destination);
    const auto extended_lane_path_or = BackwardExtendLanePathOnRouteSections(
        psmm, route_sections, lane_path, /*extend_len*/ 10.0);

    EXPECT_OK(extended_lane_path_or);
    EXPECT_EQ(extended_lane_path_or->size(), 2);
  }
}

TEST(RouteSectionsUtilTest, RouteSectionsFromCompositeLanePathTest) {
  SetMap("dojo");

  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const mapping::LanePath lp1(
      smm, {mapping::ElementId(2448), mapping::ElementId(1)}, 0.0, 0.6);
  const mapping::LanePath lp2(smm, {mapping::ElementId(950)}, 0.6, 1.0);

  const CompositeLanePath::TransitionInfo trans = {
      .lc_left = false,
      .lc_section_id = mapping::SectionId(12400),
  };

  const CompositeLanePath clp({lp1, lp2}, {trans});

  const auto route_sections = RouteSectionsFromCompositeLanePath(psmm, clp);
  EXPECT_EQ(route_sections.size(), 2);
  EXPECT_EQ(route_sections.back().id, mapping::SectionId(12400));
}

TEST(RouteSectionsUtilTest, ForwardExtendLanePathOnRouteSections) {
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const mapping::LanePath lane_path(smm, {mapping::ElementId(155)},
                                    /*start_fraction*/ 0.0,
                                    /*end_fraction*/ 0.7);
  const mapping::LanePoint destination(mapping::ElementId(238), 1.0);

  {
    const RouteSections route_sections(
        0.0, 1.0,
        {mapping::SectionId(12437), mapping::SectionId(12439),
         mapping::SectionId(12438)},
        destination);
    const RouteSectionsInfo sections_info(psmm, &route_sections);
    const auto route_navi_info =
        *CalcNaviInfoByLaneGraph(*psmm.semantic_map_manager(), sections_info,
                                 /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);
    const auto extended_lane_path_or = ForwardExtendLanePathOnRouteSections(
        psmm, route_sections, lane_path, /*extend_len=*/20.0, route_navi_info);

    EXPECT_OK(extended_lane_path_or);
    EXPECT_EQ(extended_lane_path_or->size(), 2);
    EXPECT_THAT(std::vector<mapping::ElementId>(
                    {mapping::ElementId(155), mapping::ElementId(233)}),
                testing::ElementsAreArray(extended_lane_path_or->lane_ids()));
  }
}

}  // namespace qcraft::planner
