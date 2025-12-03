#include "onboard/planner/router/alternate/alternative_route_util.h"

#include <algorithm>
#include <memory>

#include "gtest/gtest.h"

#include "common/proto/lane_point.pb.h"

#include "onboard/planner/router/route_test_util.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {
namespace {
TEST(AlternativeRouteUtilTest, GenerateExtraCostAlongRouteTest) {
  const auto& smm = LoadDojoMap();

  RouteSectionSequenceProto section_seq_proto;

  section_seq_proto.set_start_fraction(0.2);
  section_seq_proto.set_end_fraction(0.9);

  mapping::LanePointProto dest;
  dest.set_lane_id(154);
  dest.set_fraction(0.9);

  *section_seq_proto.mutable_destination() = dest;

  section_seq_proto.add_section_id(12395);
  section_seq_proto.add_section_id(12308);
  section_seq_proto.add_section_id(12415);
  section_seq_proto.add_section_id(12417);
  section_seq_proto.add_section_id(12416);
  section_seq_proto.add_section_id(12418);
  section_seq_proto.add_section_id(12275);
  section_seq_proto.add_section_id(12437);

  const auto extra_cost = GenerateDecreasingCostSeq(smm, section_seq_proto,
                                                    /*look_ahead_dist=*/500.0,
                                                    /*cost_per_meter=*/3.0);

  EXPECT_EQ(extra_cost.size(), 2);
  EXPECT_TRUE(extra_cost.contains(mapping::SectionId(12308)));
  EXPECT_TRUE(extra_cost.contains(mapping::SectionId(12275)));
}

TEST(AlternativeRouteUtilTest, IsAlternateRouteDiffPrimaryTest) {
  RouteSectionSequenceProto section_seq_proto;

  section_seq_proto.set_start_fraction(0.2);
  section_seq_proto.set_end_fraction(0.9);

  mapping::LanePointProto dest;
  dest.set_lane_id(2475);
  dest.set_fraction(0.9);

  *section_seq_proto.mutable_destination() = dest;

  section_seq_proto.add_section_id(12395);
  section_seq_proto.add_section_id(12308);
  section_seq_proto.add_section_id(12415);

  const auto section_seq_proto_1 = section_seq_proto;
  const bool result1 =
      IsAlternateRouteDiffPrimary(section_seq_proto, section_seq_proto_1);
  EXPECT_FALSE(result1);

  auto section_seq_proto_2 = section_seq_proto;
  section_seq_proto_2.mutable_section_id()->erase(
      section_seq_proto_2.section_id().begin());
  const bool result2 =
      IsAlternateRouteDiffPrimary(section_seq_proto, section_seq_proto_2);
  EXPECT_TRUE(result2);

  auto section_seq_proto_3 = section_seq_proto_2;
  section_seq_proto_3.add_section_id(1);
  const bool result3 =
      IsAlternateRouteDiffPrimary(section_seq_proto, section_seq_proto_3);
  EXPECT_TRUE(result3);
}

}  // namespace
}  // namespace qcraft::planner::route
