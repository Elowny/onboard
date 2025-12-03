#include "onboard/planner/router/route_speed_limit_util.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <memory>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_test_util.h"

namespace qcraft::planner {
TEST(RouteSpeedLimitUtilTest, GetLaneFractionSpeedLimit) {
  mapping::LaneProto lane_proto;
  lane_proto.set_speed_limit_kph(50.0);
  EXPECT_DOUBLE_EQ(GetLaneFractionSpeedLimit(lane_proto, /*fraction=*/0.5),
                   50.0);
  auto* mutable_speed_limits_kph = lane_proto.mutable_speed_limits_kph();
  mutable_speed_limits_kph->Reserve(2);
  auto* speed_limits_kph_1 = mutable_speed_limits_kph->Add();
  speed_limits_kph_1->set_start_fraction(0.0);
  speed_limits_kph_1->set_speed_limit_kph(70.0);

  auto* speed_limits_kph_2 = mutable_speed_limits_kph->Add();
  speed_limits_kph_2->set_start_fraction(0.5);
  speed_limits_kph_2->set_speed_limit_kph(100.0);

  EXPECT_DOUBLE_EQ(GetLaneFractionSpeedLimit(lane_proto, /*fraction=*/0.4),
                   70.0);
  EXPECT_DOUBLE_EQ(GetLaneFractionSpeedLimit(lane_proto, /*fraction=*/0.8),
                   100.0);
}

TEST(RouteSpeedLimitUtilTest, FindFrontOverwrittenSpeedLimit) {
  const auto& smm = LoadDojoMap();
  const RouteSections route_sections(
      /*start_fraction=*/0.5,
      /*end_fraction=*/0.8,
      {mapping::SectionId(12401), mapping::SectionId(12400),
       mapping::SectionId(12317), mapping::SectionId(12399)},
      mapping::LanePoint(mapping::ElementId(88), 0.8));

  RouteSectionSequenceProto section_proto;
  route_sections.ToProto(&section_proto);
  const auto recommend_max_speed_limit = FindFrontOverwrittenSpeedLimit(
      smm, section_proto, /*look_ahead_dist=*/3.0 * 15.0);
  EXPECT_TRUE(recommend_max_speed_limit.has_value());
  EXPECT_NEAR(*recommend_max_speed_limit, 11.11, 0.01);
}

TEST(RouteSpeedLimitUtilTest, MapSpeedLimit) {
  const auto& smm = LoadDojoMap();
  const double res = GetLaneFractionSpeedLimit(
      smm.FindLane(mapping::ElementId(2448))->proto(), /*fraction=*/0.3);
  const double modify_res = GetOverwrittenLaneFractionSpeedLimit(
      smm, mapping::ElementId(2448), /*fraction=*/0.3);
  EXPECT_DOUBLE_EQ(res, 40);
  EXPECT_DOUBLE_EQ(res, modify_res);

  const double res1 =
      GetLaneAverageSpeedLimit(smm.FindLane(mapping::ElementId(2448))->proto());
  const double modify_res1 =
      GetOverwrittenLaneAverageSpeedLimit(smm, mapping::ElementId(2448));
  EXPECT_DOUBLE_EQ(res1, 40);
  EXPECT_DOUBLE_EQ(res1, modify_res1);

  const double res2 =
      GetLaneMaxSpeedLimit(smm.FindLane(mapping::ElementId(2448))->proto());
  const double modify_res2 =
      GetOverwrittenLaneMaxSpeedLimit(smm, mapping::ElementId(2448));
  EXPECT_DOUBLE_EQ(res2, 40);
  EXPECT_DOUBLE_EQ(res2, modify_res2);

  const double res3 =
      GetLaneMinSpeedLimit(smm.FindLane(mapping::ElementId(2448))->proto());
  const double modify_res3 =
      GetOverwrittenLaneMinSpeedLimit(smm, mapping::ElementId(2448));
  EXPECT_DOUBLE_EQ(res3, 40);
  EXPECT_DOUBLE_EQ(res3, modify_res3);
}

}  // namespace qcraft::planner
