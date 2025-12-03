#include "onboard/planner/router/route_core.h"

#include "absl/container/flat_hash_set.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/maps/smm_proto_util.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/utils/file_util.h"

namespace qcraft::planner::route {
namespace {
TEST(SearchForRoutePathFromLanePointTest, OneLane) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.5,
      .fraction = 0.1};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.5,
      .fraction = 0.9};

  EXPECT_TRUE(route_core.state_code() ==
              RouteCore::SearchStateCode::kDefaultState);
  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});
  EXPECT_TRUE(route_core.state_code() ==
              RouteCore::SearchStateCode::kSearchSuccess);
  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 1);
}

TEST(SearchForRoutePathFromLanePointTest, ReverseOneLane) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.5,
      .fraction = 0.9};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.5,
      .fraction = 0.1};

  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 15);
}

TEST(SearchForRoutePathFromLanePointTest, ContinuousLaneChange1) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(3)),
      .dist = 0.5,
      .fraction = 0.8};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(25)),
      .dist = 0.5,
      .fraction = 0.5};

  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 9);
}

TEST(SearchForRoutePathFromLanePointTest, ContinuousLaneChange2) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2)),
      .dist = 0.5,
      .fraction = 0.3};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(3)),
      .dist = 0.5,
      .fraction = 0.6};

  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 10);
}

TEST(SearchForRoutePathFromLanePointTest, NormalCase) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(15)),
      .dist = 0.5,
      .fraction = 0.3};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2290)),
      .dist = 0.5,
      .fraction = 0.6};

  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 43);
}

TEST(SearchForRoutePathFromLanePointTest, AvoidCrashCase) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(4)),
      .dist = 0.5,
      .fraction = 0.1};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(4)),
      .dist = 0.5,
      .fraction = 0.9};

  mapping::PointToLane dest2{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(5)),
      .dist = 0.5,
      .fraction = 0.9};

  mapping::PointToLane dest3{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(6)),
      .dist = 0.5,
      .fraction = 0.9};

  const auto route_result_or = route_core.SearchForRoutePathFromLanePoint(
      route_pre, {origin1}, {dest1, dest2, dest3});

  EXPECT_OK(route_result_or);
}

TEST(SearchForRoutePathFromLanePointTest, BusOnlyTest) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = false;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(950)),
      .dist = 0.5,
      .fraction = 0.3};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(159)),
      .dist = 0.5,
      .fraction = 0.6};

  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 6);
}

TEST(SearchForRoutePathFromLanePointTest, AvoidLanesTest) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;
  absl::flat_hash_set<mapping::ElementId> avoid_lanes{mapping::ElementId(1)};
  route_pre.route_restrict_district.avoid_lanes = std::move(avoid_lanes);

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.5,
      .fraction = 0.3};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(160)),
      .dist = 0.5,
      .fraction = 0.6};

  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 7);
}

TEST(SearchForRoutePathFromLanePointTest, StartOnBusLane) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = false;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(3)),
      .dist = 0.5,
      .fraction = 0.3};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(159)),
      .dist = 0.5,
      .fraction = 0.6};

  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->second.lane_paths().front().end_fraction(), 1.0);
}

TEST(SearchForRoutePathFromLanePointTest, EndOnBusLane) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = false;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(3)),
      .dist = 0.5,
      .fraction = 0.3};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2472)),
      .dist = 0.5,
      .fraction = 0.6};

  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->second.lane_paths().front().end_fraction(), 1.0);
  EXPECT_EQ(route_result_or->second.lane_paths().back().start_fraction(), 0.6);
}

TEST(SearchForRoutePathFromLanePointTest, NormalCase2) {
  const auto& smm = LoadDojoMap();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.5,
      .fraction = 0.5};

  mapping::PointToLane origin2{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(3)),
      .dist = 0.5,
      .fraction = 0.5};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(129)),
      .dist = 0.5,
      .fraction = 0.9};

  const auto route_result_or = route_core.SearchForRoutePathFromLanePoint(
      route_pre, {origin1, origin2}, {dest1});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 7);
  EXPECT_NEAR(route_result_or->second.length(), 281.664, 0.1);
}

TEST(SearchForRoutePathAlongLanePointsTest, ValidTest) {
  const auto& smm = LoadDojoMap();
  FLAGS_router_send_route_core_state_to_canvas = true;
  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.5,
      .fraction = 0.3};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2470)),
      .dist = 0.5,
      .fraction = 0.6};

  mapping::PointToLane dest2{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(65)),
      .dist = 0.5,
      .fraction = 0.6};

  const auto route_result_or = route_core.SearchForRoutePathAlongLanePoints(
      route_pre, {origin1}, {{dest1}, {dest2}});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 9);
  EXPECT_NEAR(route_result_or->second.length(), 423.57, 0.1);
}

TEST(SearchForRoutePathAlongLanePointsTest, ViaPointsTest) {
  const auto& smm = LoadDojoMap();
  FLAGS_router_send_route_core_state_to_canvas = true;
  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2)),
      .dist = 0.0,
      .fraction = 0.1};

  mapping::PointToLane dest11{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2)),
      .dist = 0.0,
      .fraction = 0.9};

  mapping::PointToLane dest12{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.0,
      .fraction = 0.9};

  mapping::PointToLane dest13{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(3)),
      .dist = 0.0,
      .fraction = 0.9};

  mapping::PointToLane dest2{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(88)),
      .dist = 0.0,
      .fraction = 0.6};

  const auto route_result_or = route_core.SearchForRoutePathAlongLanePoints(
      route_pre, {origin1}, {{dest11, dest12, dest13}, {dest2}});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 4);
  EXPECT_EQ(route_result_or->second.lane_paths().size(), 3);
}

TEST(SearchForRoutePathFromLanePointTest, ExtraSectionCostTest) {
  const auto& smm = LoadDojoMap();

  const auto route_params = CreateDefaultRouteParam();

  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;
  route_pre.route_restrict_district
      .extra_sections_cost[mapping::SectionId(12408)] = 1000.0;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.5,
      .fraction = 0.5};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2471)),
      .dist = 0.5,
      .fraction = 0.9};

  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});

  EXPECT_OK(route_result_or);
  EXPECT_EQ(route_result_or->first.size(), 9);
  EXPECT_EQ(route_result_or->first.section_id(3), mapping::SectionId(12399));
}

}  // namespace
}  // namespace qcraft::planner::route
