#include "onboard/planner/router/road_conditions_process.h"

#include <algorithm>
#include <initializer_list>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/route_test_util.h"

// #include "onboard/vis/canvas/canvas.h"
namespace qcraft::planner::route {
namespace {
TEST(RoadConditionsProcessTest, FindRestrictSectionsFromRestrictRegionsTest) {
  const auto& smm = LoadDojoMap();
  CoordinateConverter cc = CoordinateConverter::FromMap("dojo");

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

  google::protobuf::RepeatedPtrField<mapping::GeoPolygonProto> r_polygon;
  r_polygon.Add(std::move(polygon));

  const auto sections_in_region =
      FindRestrictSectionsFromRestrictRegions(smm, r_polygon);
  EXPECT_TRUE(!sections_in_region.empty());
  EXPECT_GT(sections_in_region.size(), 8);
  ASSERT_TRUE(sections_in_region.contains(mapping::SectionId(12398)));
  ASSERT_TRUE(sections_in_region.contains(mapping::SectionId(12400)));
  ASSERT_TRUE(sections_in_region.contains(mapping::SectionId(12401)));

  google::protobuf::RepeatedPtrField<mapping::GeoPointProto> rp;
  for (auto& pt : {point1, point2, point3, point4}) {
    *rp.Add() = pt;
  }
  auto points = mapping::ConverterGeoPoints(rp);
  std::vector<Vec3d> points3d;
  for (const auto& pt : points) {
    const auto t = cc.GlobalToSmooth(Vec3d(pt));
    points3d.push_back(t);
  }
  EXPECT_EQ(points3d.size(), 4);
  // sections
  // {12082,12086,12199,12200,12201,12317,12319,12322,12323,12398,12400,12401,12405}

  // vis::Canvas& canvas = vantage_client_man::GetCanvas("debug");
  // canvas.DrawPolygon(points3d, vis::Color::kRed, 7.0);
  // vantage_client_man::FlushAll();
}

TEST(RoadConditionsProcessTest, ParseRestrictRoadsFromRoutingRequestTest) {
  const auto& smm = LoadDojoMap();

  RoutingRequestProto routing_request;

  routing_request.mutable_avoid_lanes()->Add(1);
  routing_request.mutable_avoid_lanes()->Add(2);
  routing_request.mutable_avoid_lanes()->Add(57);

  auto* forbidden = routing_request.mutable_forbidden();
  forbidden->mutable_sections()->Add(12400);
  forbidden->mutable_sections()->Add(12401);

  auto limit_district =
      ParseRestrictRoadsFromRoutingRequest(smm, routing_request);

  EXPECT_OK(limit_district);
  EXPECT_EQ(limit_district->avoid_lanes.size(), 3);
  EXPECT_EQ(limit_district->restrict_sections.size(), 2);

  limit_district->Clear();
  EXPECT_EQ(limit_district->avoid_lanes.size(), 0);
  EXPECT_EQ(limit_district->restrict_sections.size(), 0);
}

TEST(RoadConditionsProcessTest, UpdateAvoidLanesTest) {
  const auto& smm = LoadDojoMap();

  google::protobuf::RepeatedField<int64_t> section_ids;
  section_ids.Add(12401);
  section_ids.Add(12400);
  section_ids.Add(12408);

  google::protobuf::RepeatedField<int64_t> avoid_lanes;
  avoid_lanes.Add(2);
  UpdateAvoidLanes(smm, section_ids, &avoid_lanes);
  EXPECT_EQ(avoid_lanes.size(), 2);
  EXPECT_TRUE(std::find(avoid_lanes.begin(), avoid_lanes.end(), 3) !=
              avoid_lanes.end());
}

TEST(RoadConditionsProcessTest, UpdateDynamicOddAvoidLanes) {
  const auto& v2smm = LoadDojoMap();

  google::protobuf::RepeatedField<int64_t> section_ids;
  section_ids.Add(12401);
  section_ids.Add(12400);
  section_ids.Add(12408);

  RouteSectionSequenceProto section_seq;
  *section_seq.mutable_section_id() = section_ids;
  section_seq.set_start_fraction(0.3);

  google::protobuf::RepeatedField<int64_t> avoid_lanes;
  avoid_lanes.Add(2);

  AvoidLaneSegmentMap avoid_seg_map;
  auto& lane_seg_map = avoid_seg_map.lane_segments_map;
  lane_seg_map[mapping::ElementId(2)].start_fraction = 0.1;
  lane_seg_map[mapping::ElementId(2)].end_fraction = 0.5;

  lane_seg_map[mapping::ElementId(3)].start_fraction = 0.8;
  lane_seg_map[mapping::ElementId(3)].end_fraction = 1.0;

  const auto results1 = UpdateDynamicOddAvoidLanes(v2smm, avoid_seg_map,
                                                   section_seq, avoid_lanes);
  EXPECT_EQ(results1.size(), 2);
  EXPECT_TRUE(std::find(results1.begin(), results1.end(), 3) != results1.end());
  EXPECT_TRUE(std::find(results1.begin(), results1.end(), 2) != results1.end());

  section_seq.set_start_fraction(0.9);
  const auto results2 =
      UpdateDynamicOddAvoidLanes(v2smm, avoid_seg_map, section_seq, results1);
  EXPECT_EQ(results2.size(), 1);
  EXPECT_TRUE(std::find(results2.begin(), results2.end(), 3) != results2.end());
  EXPECT_TRUE(std::find(results2.begin(), results2.end(), 2) == results2.end());
}

}  // namespace
}  // namespace qcraft::planner::route
