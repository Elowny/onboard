#include "onboard/planner/planner_state_util.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_test_util.h"

namespace qcraft::planner {
namespace {
TEST(PlannerStateUtilTest, ObtainHdMapState) {
  const auto& smm = LoadDojoMap();
  const double length12401 =
      smm.FindSection(mapping::SectionId(12401))->proto().average_length() *
      0.8;
  const double length12400 =
      smm.FindSection(mapping::SectionId(12400))->proto().average_length();
  const double length12408 =
      smm.FindSection(mapping::SectionId(12408))->proto().average_length() *
      0.8;
  {
    const RouteSections route_sections(
        /*start_fraction=*/0.2, /*end_fraction=*/0.8,
        {mapping::SectionId(12401), mapping::SectionId(12400),
         mapping::SectionId(12408)},
        mapping::LanePoint(mapping::ElementId(34), 0.8));
    RouteSectionSequenceProto sections_proto;
    route_sections.ToProto(&sections_proto);
    const auto hd_map_state = ObtainHdMapState(smm, sections_proto);
    EXPECT_DOUBLE_EQ(hd_map_state.load_distance,
                     length12401 + length12400 + length12408);
    EXPECT_TRUE(hd_map_state.has_destination);
  }

  {
    const RouteSections route_sections(
        /*start_fraction=*/0.2, /*end_fraction=*/0.8,
        {mapping::SectionId(12401), mapping::SectionId(124000000),
         mapping::SectionId(1240008)},
        mapping::LanePoint(mapping::ElementId(3000004), 0.8));
    RouteSectionSequenceProto sections_proto;
    route_sections.ToProto(&sections_proto);
    const auto hd_map_state = ObtainHdMapState(smm, sections_proto);
    EXPECT_DOUBLE_EQ(hd_map_state.load_distance, length12401);
    EXPECT_FALSE(hd_map_state.has_destination);
  }
}
}  // namespace
}  // namespace qcraft::planner
