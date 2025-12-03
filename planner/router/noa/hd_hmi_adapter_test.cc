#include "onboard/planner/router/noa/hd_hmi_adapter.h"

#include <memory>

#include "gtest/gtest.h"

#include "common/proto/lane_point.pb.h"

#include "onboard/async/future.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/planner/router/route_util.h"
namespace qcraft::planner::route::noa {

namespace {

TEST(HdHmiAdapter, CaclDistanceToEndTest) {
  // CaclDistanceToEnd(const mapping::v2::SemanticMapSpatialIndex &smm_index,
  // const int &route_sections, const Vec2d &sd_destination, int dist_to_sd_end)
  auto loader =
      mapping::v2::SemanticMapLoader::MakeShared({.map_name = "dojo"});
  auto map_fut = loader->PreloadWholeMap();
  std::shared_ptr<mapping::v2::SemanticMapManager> v2smm = map_fut.Get();
  std::shared_ptr<mapping::v2::SemanticMapSpatialIndex> smm_index =
      mapping::v2::SemanticMapSpatialIndex::MakeShared(v2smm);

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

  const auto len = CalcRouteSectionsLength(*smm_index->semantic_manager(),
                                           section_seq_proto);
  {
    auto dist_op = CaclDistanceToEnd(
        *smm_index, section_seq_proto,
        /*sd_destination =*/{4.8616738364575844e-05, 1.0443328122141306e-05},
        /*dist_to_sd_end=*/len, /*dist_to_map_boundary=*/len);
    ASSERT_TRUE(dist_op.has_value());
    EXPECT_NEAR(*dist_op, len, 50.0);
  }

  {
    auto dist_op = CaclDistanceToEnd(
        *smm_index, section_seq_proto,
        /*sd_destination =*/{4.8616738364575844e-05, 1.0443328122141306e-05},
        /*dist_to_sd_end=*/5000, /*dist_to_map_boundary=*/len);
    ASSERT_FALSE(dist_op.has_value());
  }

  {
    auto dist_op = CaclDistanceToEnd(
        *smm_index, section_seq_proto,
        /*sd_destination =*/{4.8616738364575844e-05, 1.0443328122141306e-05},
        /*dist_to_sd_end=*/300, /*dist_to_map_boundary=*/20);
    ASSERT_FALSE(dist_op.has_value());
  }

  {
    auto dist_op =
        CaclDistanceToEnd(*smm_index, section_seq_proto,
                          /*sd_destination =*/{0, 0},
                          /*dist_to_sd_end=*/5000, /*dist_to_map_boundary=*/10);
    ASSERT_FALSE(dist_op.has_value());
  }
}

}  // namespace
}  // namespace qcraft::planner::route::noa
