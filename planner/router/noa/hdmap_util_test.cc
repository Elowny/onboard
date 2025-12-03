#include "onboard/planner/router/noa/hdmap_util.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/async/future.h"
#include "onboard/maps/ehr/proto/extension.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"

namespace qcraft::planner::route::noa {

namespace {

TEST(HdMapUtil, HasRouteGuideAttrBySection) {
  mapping::SectionProto proto;
  ASSERT_FALSE(HasRouteGuideAttrBySection(proto));
  proto.mutable_third_party_section_extend()->set_is_part_of_calculated_route(
      true);
  ASSERT_TRUE(HasRouteGuideAttrBySection(proto));
}

TEST(HdMapUtil, PointToNearLanesWithHeading) {
  auto loader =
      mapping::v2::SemanticMapLoader::MakeShared({.map_name = "dojo"});
  auto map_fut = loader->PreloadWholeMap();
  std::shared_ptr<mapping::v2::SemanticMapManager> v2smm = map_fut.Get();

  constexpr double kMaxDist = 10.0;
  std::shared_ptr<mapping::v2::SemanticMapSpatialIndex> smm_index =
      mapping::v2::SemanticMapSpatialIndex::MakeShared(v2smm);
  auto matched_lanes = PointToNearLanesWithHeading(
      *smm_index, /*global=*/{0.0, 0.0}, /*z=*/0.0, kMaxDist, 0.0, 0.7);
  ASSERT_GT(matched_lanes.size(), 0);
  for (const auto& matched : matched_lanes) {
    ASSERT_LE(matched.dist, kMaxDist);
  }
  ASSERT_FALSE(matched_lanes.front().DebugString().empty());
  {
    auto ignore_heading_lanes =
        PointToNearLanes(*smm_index, /*global=*/{0.0, 0.0}, kMaxDist);
    ASSERT_LE(matched_lanes.size(), ignore_heading_lanes.size());
    ASSERT_GT(ignore_heading_lanes.size(), 0);
    for (const auto& matched : ignore_heading_lanes) {
      ASSERT_LE(matched.dist, kMaxDist);
    }
  }
}

}  // namespace
}  // namespace qcraft::planner::route::noa
