#include "onboard/planner/router/route_util.h"

#include <float.h>

#include <cmath>
#include <memory>

#include "absl/hash/hash.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "common/proto/lane_point.pb.h"

#include "onboard/base/macros.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(RouteUtilTest, GlobalToSmoothPointsTest) {
  const auto& smm = LoadDojoMap();
  auto cc = CoordinateConverter::FromMap("dojo");
  SMM_ASSIGN_LANE_OR_RETURN(lane, smm, 3, void());
  auto points = GlobalToSmoothPoints(cc, lane.Proto().polyline().points());
  ASSERT_EQ(points.size(), lane.Proto().polyline().points().size());
}

TEST(RouteUtilTest, FindLanePathFromLaneAlongRouteSections) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  // const auto& cc = psmm.coordinate_converter();

  {
    const RouteSections route_sections(
        0.0, 1.0,
        {mapping::SectionId(12437), mapping::SectionId(12439),
         mapping::SectionId(12438), mapping::SectionId(12215),
         mapping::SectionId(12211)},
        mapping::LanePoint(mapping::ElementId(246), 1.0));
    const RouteSectionsInfo sections_info(psmm, &route_sections);
    const auto route_navi_info =
        *CalcNaviInfoByLaneGraph(*smm, sections_info,
                                 /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

    const auto lane_path = FindLanePathFromLaneAlongRouteSections(
        psmm, sections_info, route_navi_info, mapping::ElementId(155),
        sections_info.start_fraction(), /*extend_len=*/100.0);
    EXPECT_NEAR(lane_path.length(), 100.0, 1e-3);
    EXPECT_EQ(lane_path.back().lane_id().value(), 246);
  }
  {
    const RouteSections route_sections(
        0.0, 0.6,
        {mapping::SectionId(12437), mapping::SectionId(12439),
         mapping::SectionId(12438), mapping::SectionId(12440)},
        mapping::LanePoint(mapping::ElementId(241), 0.6));
    const RouteSectionsInfo sections_info(psmm, &route_sections);
    const auto route_navi_info =
        *CalcNaviInfoByLaneGraph(*smm, sections_info,
                                 /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

    const auto lane_path = FindLanePathFromLaneAlongRouteSections(
        psmm, sections_info, route_navi_info, mapping::ElementId(155),
        sections_info.start_fraction(), /*extend_len=*/100.0);
    EXPECT_EQ(lane_path.back().lane_id().value(), 241);
    EXPECT_EQ(lane_path.back().fraction(), 0.6);
  }
}

TEST(RouteUtilTest, TravelDistanceBetweenTest) {
  const auto& smm = LoadDojoMap();

  mapping::LanePath lane_path0(
      &smm, {mapping::ElementId(169), mapping::ElementId(130)}, 0.0, 0.5);
  mapping::LanePath lane_path1(&smm, {mapping::ElementId(129)}, 0.5, 0.9);
  CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0,
      .lc_left = false,
      .lc_section_id = mapping::SectionId(3047),
  };
  const CompositeLanePath clp({lane_path0, lane_path1}, {trans1});

  SMM_ASSIGN_LANE_OR_RETURN(lane_info169, smm, 169, void());
  SMM_ASSIGN_LANE_OR_RETURN(lane_info130, smm, 130, void());
  SMM_ASSIGN_LANE_OR_RETURN(lane_info129, smm, 129, void());

  {
    const double cal_distance = TravelDistanceBetween(
        smm, clp, CompositeLanePath::CompositeIndex(0, 0), 0.0,
        CompositeLanePath::CompositeIndex(1, 0), 0.8);

    const double expected_length = lane_info169.length() +
                                   lane_info130.length() * 0.5 +
                                   lane_info129.length() * 0.3;
    EXPECT_TRUE(std::fabs(cal_distance - expected_length) < 0.1);
  }

  {
    const double cal_distance = TravelDistanceBetween(
        smm, clp, CompositeLanePath::CompositeIndex(0, 0), 0.3,
        CompositeLanePath::CompositeIndex(0, 1), 0.3);

    const double expected_length =
        lane_info169.length() * 0.7 + lane_info130.length() * 0.3;
    EXPECT_TRUE(std::fabs(cal_distance - expected_length) < 0.1);
  }
}

TEST(RouteUtilTest, IsTravelMatchedValid) {
  SetMap("dojo");
  const auto& smm = LoadDojoMap();
  mapping::LanePath lane_path0(
      &smm, {mapping::ElementId(169), mapping::ElementId(130)}, 0.0, 0.5);
  mapping::LanePath lane_path1(&smm, {mapping::ElementId(129)}, 0.5, 0.9);
  EXPECT_TRUE(IsTravelMatchedValid(CompositeLanePath::CompositeIndex(0, 0), 0.0,
                                   CompositeLanePath::CompositeIndex(1, 0), 0.5,
                                   1000000, 6000000, 63.2644, 23, 200, 20));
}

TEST(RouteUtilTest, SearchConnectLaneInSection) {
  SetMap("dojo");
  const auto& smm = LoadDojoMap();
  const auto connect_lanes_in_section_or = SearchConnectLaneInSection(
      smm, mapping::ElementId(2), mapping::ElementId(3),
      /*search_direction_left=*/false);
  std::vector<mapping::ElementId> res;
  if (!connect_lanes_in_section_or.ok()) {
    EXPECT_TRUE(false);
  } else {
    for (const auto& item : connect_lanes_in_section_or.value()) {
      res.push_back(item.id);
    }
  }
  EXPECT_THAT(std::vector<mapping::ElementId>({mapping::ElementId(3),
                                               mapping::ElementId(2448),
                                               mapping::ElementId(2)}),
              testing::ElementsAreArray(res));
}

TEST(RouteUtilTest, IsTwoLaneConnectedWithBrokenWhites) {
  SetMap("dojo");
  const auto& smm = LoadDojoMap();
  EXPECT_TRUE(IsTwoLaneConnectedWithBrokenWhites(smm, mapping::ElementId(2),
                                                 mapping::ElementId(3)));
}

TEST(RouteUtilTest, PointToNearLanesAtLevelTest) {
  // Because of the missing of multi-level map， it can only test 0 level.
  SetMap("dojo");
  const auto* map_index = LoadDojoIndex();
  const auto cc = CoordinateConverter::FromMap("dojo");
  constexpr double kMaxDistance = 7.0;
  const auto pt_lanes = PointToNearLanesAtLevel(
      *map_index, cc.GetLevel(), /*global=*/{0.0, 0.0}, kMaxDistance);
  ASSERT_TRUE(!pt_lanes.empty());
  for (const auto& pt_lane : pt_lanes) {
    EXPECT_LT(pt_lane.dist, kMaxDistance + 0.1);
  }
}

TEST(RouteUtilTest, PointToNearLanesWithHeadingAtLevelTest) {
  SetMap("dojo");

  const auto* map_index = LoadDojoIndex();
  const auto cc = CoordinateConverter::FromMap("dojo");
  const auto lanes1 = PointToNearLanesWithHeadingAtLevel(
      *map_index, cc.GetLevel(), /*global=*/{0.0, 0.0}, /*max_distance=*/7.0,
      /*global heading*/ 0.0, /*max_heading_diff=*/M_PI / 8);
  // auto fn = [](decltype(lanes1)& pt_lanes) -> void {
  //   for (const auto& pt_lane : pt_lanes) {
  //     std::cout << "id: " << pt_lane.lane_info->id << ", dist:" <<
  //     pt_lane.dist
  //               << " heading:" << pt_lane.segment.heading()
  //               << " match:" << pt_lane.match_point
  //               << " fraction: " << pt_lane.fraction << std::endl;
  //   }
  // };
  // std::cout << " ====lanes1=== " << std::endl;
  // fn(lanes1);
  // std::cout << " ====lanes2=== " << std::endl;
  // fn(lanes2);
  ASSERT_TRUE(!lanes1.empty());
}

TEST(RouteUtilTest, AccumRouteSectionsLengthBetweenTest) {
  const auto& smm = LoadDojoMap();

  RouteSectionSequenceProto sections_proto;
  sections_proto.set_start_fraction(0.1);
  sections_proto.set_end_fraction(0.8);
  auto* section_ids_ptr = sections_proto.mutable_section_id();
  section_ids_ptr->Add(12401);
  section_ids_ptr->Add(12400);
  section_ids_ptr->Add(12408);

  SMM_SECTION_PROTO_OR_RETURN(section_proto_12401, smm,
                              mapping::SectionId(12401), void());
  SMM_SECTION_PROTO_OR_RETURN(section_proto_12400, smm,
                              mapping::SectionId(12400), void());
  SMM_SECTION_PROTO_OR_RETURN(section_proto_12408, smm,
                              mapping::SectionId(12408), void());
  const double len_1 =
      AccumRouteSectionsLengthBetween(smm, sections_proto, 0, 3);
  const double expect_len_1 = section_proto_12401->average_length() * 0.9 +
                              section_proto_12400->average_length() +
                              section_proto_12408->average_length() * 0.8;
  EXPECT_NEAR(len_1, expect_len_1, 0.1);
  const double len_2 =
      AccumRouteSectionsLengthBetween(smm, sections_proto, 1, 3);
  const double expect_len_2 = section_proto_12400->average_length() +
                              section_proto_12408->average_length() * 0.8;
  EXPECT_NEAR(len_2, expect_len_2, 0.1);
}

TEST(RouteUtilTest, ComputeEtaToRouteEndTest) {
  const auto& smm = LoadDojoMap();

  mapping::LanePath lane_path0(
      &smm, {mapping::ElementId(169), mapping::ElementId(130)}, 0.0, 0.5);
  mapping::LanePath lane_path1(&smm, {mapping::ElementId(129)}, 0.5, 0.9);
  CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0,
      .lc_left = false,
      .lc_section_id = mapping::SectionId(3047),
  };
  const CompositeLanePath clp({lane_path0, lane_path1}, {trans1});

  const double eta = ComputeEtaToRouteEnd(smm, clp);

  SMM_ASSIGN_LANE_OR_RETURN(lane_info169, smm, 169, void());
  SMM_ASSIGN_LANE_OR_RETURN(lane_info130, smm, 130, void());
  SMM_ASSIGN_LANE_OR_RETURN(lane_info129, smm, 129, void());
  const double expected_eta =
      lane_info169.length() * 1.0 /
          Kph2Mps(lane_info169.Proto().speed_limit_kph()) +
      lane_info130.length() * 0.5 /
          Kph2Mps(lane_info130.Proto().speed_limit_kph()) +
      lane_info129.length() * 0.4 /
          Kph2Mps(lane_info129.Proto().speed_limit_kph());
  EXPECT_NEAR(eta, expected_eta, 0.1);
}

TEST(RouteUtilTest, HasOnlyOneOffRoadRequest) {
  RoutingRequestProto routing_request;
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(129);
  dest1.mutable_lane_point()->set_fraction(0.5);
  routing_request.mutable_destinations()->Add(std::move(dest1));
  ASSERT_FALSE(HasOnlyOneOffRoadRequest(routing_request));
}

TEST(RouteUtilTest, ConvertToGlobal) {
  const auto& smm = LoadDojoMap();
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(129);
  dest1.mutable_lane_point()->set_fraction(0.5);
  auto lane_pt_global_or = internal::ConvertToGlobal(smm, dest1);
  ASSERT_TRUE(lane_pt_global_or.ok());

  RoutingDestinationProto dest2;
  *dest2.mutable_named_spot() = "8c_s2";
  ASSERT_TRUE(internal::ConvertToGlobal(smm, dest2).ok());

  RoutingDestinationProto dest2_1;
  *dest2_1.mutable_named_spot() = "wrong_name";
  ASSERT_FALSE(internal::ConvertToGlobal(smm, dest2_1).ok());

  RoutingDestinationProto dest3;
  dest3.mutable_global_point()->set_longitude(2.0);
  dest3.mutable_global_point()->set_latitude(0.5);
  auto global_or = internal::ConvertToGlobal(smm, dest3);
  ASSERT_TRUE(global_or.ok());
  ASSERT_NEAR(global_or->longitude(), 2.0, 1E-8);
  ASSERT_NEAR(global_or->latitude(), 0.5, 1E-8);
}

TEST(RouteUtilTest, ConvertRoutingRequestToGlobalPoint) {
  const auto& smm = LoadDojoMap();
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(129);
  dest1.mutable_lane_point()->set_fraction(0.5);
  auto lane_pt_global_or = internal::ConvertToGlobal(smm, dest1);

  RoutingDestinationProto dest2;
  *dest2.mutable_named_spot() = "8c_s2";

  RoutingDestinationProto dest3;
  dest3.mutable_global_point()->set_longitude(2.0);
  dest3.mutable_global_point()->set_latitude(0.5);

  RoutingRequestProto proto;
  *proto.add_destinations() = dest1;
  *proto.add_destinations() = dest1;
  *proto.add_destinations() = dest3;
  auto request_or = ConvertRoutingRequestToGlobalPoint(smm, proto);
  ASSERT_TRUE(request_or.ok());
  ASSERT_EQ(request_or->destinations_size(), 3);
  ASSERT_TRUE(request_or->destinations(0).has_global_point());
  ASSERT_TRUE(request_or->destinations(1).has_global_point());
  ASSERT_TRUE(request_or->destinations(2).has_global_point());
}

TEST(RouteUtilTest, CalcRouteSectionsLengthTest) {
  const auto& smm = LoadDojoMap();

  const RouteSections route_sections(
      /*start_fraction=*/0.2, /*end_fraction=*/0.8,
      {mapping::SectionId(12401), mapping::SectionId(12400),
       mapping::SectionId(12408)},
      mapping::LanePoint(mapping::ElementId(34), 0.8));

  const double expect_length =
      FindSectionProto(smm, mapping::SectionId(12401))->average_length() * 0.8 +
      FindSectionProto(smm, mapping::SectionId(12400))->average_length() +
      FindSectionProto(smm, mapping::SectionId(12408))->average_length() * 0.8;

  const double calc_length = CalcRouteSectionsLength(smm, route_sections);
  EXPECT_NEAR(calc_length, expect_length, 0.01);

  RouteSectionSequenceProto sections_proto;
  route_sections.ToProto(&sections_proto);
  const double calc_proto_length = CalcRouteSectionsLength(smm, sections_proto);
  EXPECT_NEAR(calc_proto_length, expect_length, 0.01);
}

TEST(RouteUtilTest, BuildRouteSectionsInfoFromDataTest) {
  const auto& smm = LoadDojoMap();

  const RouteSections route_sections(
      /*start_fraction=*/0.2, /*end_fraction=*/0.8,
      {mapping::SectionId(12401), mapping::SectionId(12400),
       mapping::SectionId(12408)},
      mapping::LanePoint(mapping::ElementId(34), 0.8));

  const auto sections_info_or = BuildRouteSectionsInfoFromData(
      smm, &route_sections, /*planning_horizon=*/0.0);
  EXPECT_OK(sections_info_or);
  EXPECT_EQ(sections_info_or->size(), 3);
  EXPECT_EQ(sections_info_or->start_fraction(), 0.2);
  EXPECT_EQ(sections_info_or->end_fraction(), 0.8);
  const auto section_proto_ptr12401 =
      FindSectionProto(smm, mapping::SectionId(12401));
  EXPECT_TRUE(section_proto_ptr12401 != nullptr);

  const auto section_proto_ptr12400 =
      FindSectionProto(smm, mapping::SectionId(12400));
  EXPECT_TRUE(section_proto_ptr12400 != nullptr);

  const auto section_proto_ptr12408 =
      FindSectionProto(smm, mapping::SectionId(12408));
  EXPECT_TRUE(section_proto_ptr12408 != nullptr);

  const double exp_length = section_proto_ptr12401->average_length() * 0.8 +
                            section_proto_ptr12400->average_length() +
                            section_proto_ptr12408->average_length() * 0.8;
  EXPECT_NEAR(sections_info_or->length(), exp_length, 0.1);
}

TEST(RouteUtilTest, ComputePointOnLaneTest) {
  mapping::LaneProto proto;
  auto* proto1 = proto.mutable_polyline()->mutable_points()->Add();
  proto1->set_longitude(1.0);
  proto1->set_latitude(1.0);
  proto1->set_altitude(1.0);
  auto* proto2 = proto.mutable_polyline()->mutable_points()->Add();
  proto2->set_longitude(2.0);
  proto2->set_latitude(2.0);
  proto2->set_altitude(2.0);
  const auto lerp = ComputePointOnLane(proto.polyline().points(), 0.5);
  EXPECT_NEAR(lerp.x(), 1.5, 1E-6);
  EXPECT_NEAR(lerp.y(), 1.5, 1E-6);
  EXPECT_NEAR(lerp.z(), 1.5, 1E-6);
}

TEST(RouteUtilTest, SampleRouteSectionsPolygons) {
  const auto& psmm = CreateDojoTestPSMM();
  const RouteSections route_sections(
      /*start_fraction=*/0.2, /*end_fraction=*/0.8,
      {mapping::SectionId(12401), mapping::SectionId(12400),
       mapping::SectionId(12408)},
      mapping::LanePoint(mapping::ElementId(34), 0.8));
  const auto polygons = SampleRouteSectionsPolygons(psmm, route_sections);
  EXPECT_EQ(polygons.size(), 9);
  EXPECT_NEAR(polygons[0].min_x(), 11.25, 0.01);
  EXPECT_NEAR(polygons[0].min_y(), -5.24, 0.01);
}

TEST(RouteUtilTest, SimplifyPathPoints) {
  const auto& smm = LoadDojoMap();
  CompositeLanePathProto clp_proto;
  auto* lane_path_proto = clp_proto.add_lane_paths();
  lane_path_proto->add_lane_ids(3);
  lane_path_proto->add_lane_ids(950);
  lane_path_proto->add_lane_ids(35);
  lane_path_proto->add_lane_ids(2472);
  lane_path_proto->set_start_fraction(0);
  lane_path_proto->set_end_fraction(0.5);
  auto res = SimplifyPathPoints(smm, clp_proto, /*max_distance_meters*/ 1.5);
  ASSERT_OK(res);
}

TEST(RouteUtilTest, ValidRouteManagerOutput) {
  RouteManagerOutputProto route_proto;
  route_proto.set_route_status(RouteManagerOutputProto::NORMAL);
  EXPECT_TRUE(IsValidRouteOutputProto(route_proto));
  EXPECT_FALSE(IsResetRouteOutputProto(route_proto));
  RouteManagerOutput route_mgr_out;
  route_mgr_out.FromProto(route_proto);
  EXPECT_FALSE(HasValidRouteResults(route_mgr_out));
  route_proto.set_route_status(RouteManagerOutputProto::INVALID);
  EXPECT_FALSE(IsValidRouteOutputProto(route_proto));
  route_proto.set_route_status(RouteManagerOutputProto::RESET);
  EXPECT_TRUE(IsResetRouteOutputProto(route_proto));
}

TEST(RouteUtilTest, IsValidLocalization) {
  auto lp_proto = std::make_shared<LocalizationTransformProto>();
  lp_proto->set_loc_status(LocalizationTransformProto::kInit);
  EXPECT_FALSE(IsValidLocalization(lp_proto));
  lp_proto->set_loc_status(LocalizationTransformProto::kNormal);
  EXPECT_TRUE(IsValidLocalization(lp_proto));
}

TEST(RouteUtilTest, FindMostReasonableOutgoingLaneIdOrInvalid) {
  const auto& psmm = CreateDojoTestPSMM();
  {
    const RouteSections route_sections(
        0.0, 1.0,
        {mapping::SectionId(12401), mapping::SectionId(12400),
         mapping::SectionId(12408)},
        mapping::LanePoint(mapping::ElementId(34), 1.0));
    const RouteSectionsInfo sections_info(psmm, &route_sections);
    const auto route_navi_info =
        *CalcNaviInfoByLaneGraph(*psmm.semantic_map_manager(), sections_info,
                                 /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);
    const auto result_1 = FindMostReasonableOutgoingLaneIdOrInvalid(
        psmm, *psmm.FindLaneInfoOrNull(mapping::ElementId(2)), route_navi_info,
        sections_info.section_segment(1));
    EXPECT_EQ(result_1.value(), 938);

    const auto result_2 = FindMostReasonableOutgoingLaneIdOrInvalid(
        psmm, *psmm.FindLaneInfoOrNull(mapping::ElementId(2)), route_navi_info,
        RouteSectionsInfo::RouteSectionSegmentInfo());
    EXPECT_EQ(result_2, mapping::kInvalidElementId);
  }

  {
    const RouteSections route_sections(
        0.0, 1.0, {mapping::SectionId(12437), mapping::SectionId(12439)},
        mapping::LanePoint(mapping::ElementId(2543), 1.0));
    const RouteSectionsInfo sections_info(psmm, &route_sections);
    const auto route_navi_info =
        *CalcNaviInfoByLaneGraph(*psmm.semantic_map_manager(), sections_info,
                                 /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);
    const auto result_1 = FindMostReasonableOutgoingLaneIdOrInvalid(
        psmm, *psmm.FindLaneInfoOrNull(mapping::ElementId(155)),
        route_navi_info, sections_info.section_segment(1));
    EXPECT_EQ(result_1.value(), 233);
  }
}

TEST(RouteUtilTest, CurvatureSpeedLimit) {
  const auto& psmm = CreateDojoTestPSMM();
  {
    // Straight.
    const auto& points =
        psmm.FindLaneInfoOrNull(mapping::ElementId(34))->points_smooth;
    const double turning_radius = EstimateNaviSegmentTurningRadius(points);
    EXPECT_DOUBLE_EQ(turning_radius, DBL_MAX);
    const double speed_limit = EstimateNaviSegmentCurvatureSpeedLimit(points);
    EXPECT_DOUBLE_EQ(speed_limit, 60.0);
  }
  {
    // Turn right.
    const auto& points =
        psmm.FindLaneInfoOrNull(mapping::ElementId(42))->points_smooth;
    const double turning_radius = EstimateNaviSegmentTurningRadius(points);
    EXPECT_NEAR(turning_radius, 15.0, 1.0);
    const double speed_limit = EstimateNaviSegmentCurvatureSpeedLimit(points);
    EXPECT_DOUBLE_EQ(speed_limit, 20.0);
  }
  {
    // Turn left.
    const auto& points =
        psmm.FindLaneInfoOrNull(mapping::ElementId(50))->points_smooth;
    const double turning_radius = EstimateNaviSegmentTurningRadius(points);
    EXPECT_NEAR(turning_radius, 24.0, 1.0);
    const double speed_limit = EstimateNaviSegmentCurvatureSpeedLimit(points);
    EXPECT_DOUBLE_EQ(speed_limit, 40.0);
  }
  {
    // Roundabout.
    const auto& points =
        psmm.FindLaneInfoOrNull(mapping::ElementId(1856))->points_smooth;
    const double turning_radius = EstimateNaviSegmentTurningRadius(points);
    EXPECT_NEAR(turning_radius, 17.0, 1.0);
    const double speed_limit = EstimateNaviSegmentCurvatureSpeedLimit(points);
    EXPECT_DOUBLE_EQ(speed_limit, 20.0);
  }
  {
    // Sd route, ramp.
    std::vector<Vec2d> global_points = {
        {1.9770827083222933, 0.400920773941622},
        {1.9770818585585785, 0.400921042531788},
        {1.9770793539845888, 0.400921323723723},
        {1.9770753358355426, 0.40092086410709671},
        {1.9770729020708628, 0.40092002537942834},
        {1.9770711228046536, 0.40091899272628756},
        {1.9770695326157794, 0.40091768372934866},
        {1.9770681266561045, 0.40091599657773841},
        {1.9770668176591653, 0.40091318950652471},
        {1.9770662552752953, 0.40091075574184554}};

    std::vector<Vec2d> smooth_points;
    smooth_points.reserve(global_points.size());
    CoordinateConverter cc;
    cc.SetSmoothCoordinateOrigin({1.9770827083222933, 0.400920773941622});
    std::for_each(global_points.begin(), global_points.end(),
                  [&smooth_points, &cc](const auto& global_point) {
                    smooth_points.push_back(cc.GlobalToSmooth(global_point));
                  });
    const double turning_radius =
        EstimateNaviSegmentTurningRadius(smooth_points);
    EXPECT_NEAR(turning_radius, 73.0, 1.0);
    const double speed_limit =
        EstimateNaviSegmentCurvatureSpeedLimit(smooth_points);
    EXPECT_DOUBLE_EQ(speed_limit, 60.0);
  }
}

TEST(RouteUtilTest, CalcOutgoingLaneCrossProd) {
  const auto& psmm = CreateDojoTestPSMM();
  {
    const auto res_or = CalcOutgoingLaneCrossProd(psmm, mapping::ElementId(54),
                                                  mapping::ElementId(122));
    EXPECT_OK(res_or);
    EXPECT_NEAR(*res_or, 0.0, 1e-3);
  }
  {
    const auto res_or = CalcOutgoingLaneCrossProd(psmm, mapping::ElementId(54),
                                                  mapping::ElementId(188));
    EXPECT_OK(res_or);
    EXPECT_LT(*res_or, 0.0);
  }
  {
    const auto res_or = CalcOutgoingLaneCrossProd(psmm, mapping::ElementId(54),
                                                  mapping::ElementId(168));
    EXPECT_OK(res_or);
    EXPECT_GT(*res_or, 0.0);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
