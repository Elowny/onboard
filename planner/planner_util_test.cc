#include "onboard/planner/planner_util.h"

#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/maps/v2/semantic_map_loader.h"

namespace qcraft::planner {
namespace {
TEST(PlannerUtilTest, CreateSemanticMapModificationV2Test) {
  SetMap("dojo");
  std::shared_ptr<mapping::v2::SemanticMapLoader> loader_v2 =
      mapping::v2::SemanticMapLoader::MakeShared();
  std::shared_ptr<mapping::v2::SemanticMapManager> smm_v2 =
      loader_v2->PreloadAsync(0.0, 0.0, 3).Get();

  mapping::SemanticMapModifierProto mod;
  auto* speed_limit_mod = mod.mutable_speed_limit_modifier();
  speed_limit_mod->set_max_speed_limit(118.0);

  mapping::SemanticMapModifierProto::SpeedLimitModifierProto::LaneIdModifier
      lane_id_modifier;
  lane_id_modifier.set_lane_id(213);
  lane_id_modifier.set_override_speed_limit(100.0);
  speed_limit_mod->mutable_lane_id_modifier()->Add(std::move(lane_id_modifier));

  mapping::SemanticMapModifierProto::SpeedLimitModifierProto::RegionModifier
      region_modifier;
  mapping::GeoPolygonProto polygon;

  mapping::GeoPointProto point1;
  point1.set_longitude(-1.6408368797483624e-06);
  point1.set_latitude(-2.7504920173141589e-06);
  point1.set_altitude(0.0);
  *polygon.mutable_points()->Add() = point1;

  mapping::GeoPointProto point2;
  point2.set_longitude(-1.6344704359329477e-06);
  point2.set_latitude(3.2862357356570919e-06);
  point2.set_altitude(0.0);
  *polygon.mutable_points()->Add() = point2;

  mapping::GeoPointProto point3;
  point3.set_longitude(1.3190199163651294e-05);
  point3.set_latitude(4.3968250465334372e-06);
  point3.set_altitude(0.0);
  *polygon.mutable_points()->Add() = point3;

  mapping::GeoPointProto point4;
  point4.set_longitude(1.3180379595103556e-05);
  point4.set_latitude(-2.7614211329166759e-06);
  point4.set_altitude(0.0);
  *polygon.mutable_points()->Add() = point4;

  *region_modifier.mutable_region() = std::move(polygon);
  region_modifier.set_override_speed_limit(77.0);
  speed_limit_mod->mutable_region_modifier()->Add(std::move(region_modifier));

  const auto planner_sm_mod = CreateSemanticMapModification(*smm_v2, mod);
  EXPECT_EQ(planner_sm_mod.max_speed_limit, 118.0);
  EXPECT_EQ(planner_sm_mod.lane_speed_limit_map.at(mapping::ElementId(213)),
            100.0);
  EXPECT_EQ(planner_sm_mod.lane_speed_limit_map.at(mapping::ElementId(2448)),
            77.0);
}
}  // namespace
}  // namespace qcraft::planner
