#include "onboard/planner/router/noa/sd_route_util.h"

#include <utility>
#include <vector>

#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/base/macros.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route::noa {

namespace {

SDRouteProto CreateSdRouteProto() {
  SDRouteProto sd_route;

  sd_route.set_route_id(100);
  sd_route.set_length(100);
  sd_route.set_status(SDRouteProto::CREATE);

  auto* section = sd_route.add_sd_sections();
  section->set_length(100);
  {
    auto* link1 = section->add_sd_links();
    link1->set_length(50);
    link1->set_link_id(1);
    auto* geo_pt_1 = link1->mutable_points()->Add();
    geo_pt_1->set_longitude(2.0);
    geo_pt_1->set_latitude(0.501);

    auto* geo_pt_2 = link1->mutable_points()->Add();
    geo_pt_2->set_longitude(2.0);
    geo_pt_2->set_latitude(0.5010001);
  }
  {
    auto* link2 = section->add_sd_links();
    link2->set_length(50);
    link2->set_link_id(2);
    auto* geo_pt_1 = link2->mutable_points()->Add();
    geo_pt_1->set_longitude(2.0);
    geo_pt_1->set_latitude(0.5010001);

    auto* geo_pt_2 = link2->mutable_points()->Add();
    geo_pt_2->set_longitude(2.0);
    geo_pt_2->set_latitude(0.5010002);
  }
  return sd_route;
}

TEST(SdRouteUtil, FillRoutingResultBySdRoute) {
  auto sd_route = CreateSdRouteProto();
  RoutingResultProto routing_result;
  const auto s = FillRoutingResultBySdRoute(sd_route, &routing_result);
  ASSERT_TRUE(s.ok());
  ASSERT_TRUE(routing_result.success());
  ASSERT_EQ(100, routing_result.update_id());
  ASSERT_EQ(4, routing_result.path_points_from_current().size());
  EXPECT_NEAR(100, routing_result.distance(), 1E-6);

  auto sd_match_state_or = BuildSdRouteMatchState(
      std::make_shared<const SDRouteProto>(std::move(sd_route)), nullptr);
  ASSERT_OK(sd_match_state_or);
  const auto sd_match_state = std::move(sd_match_state_or).value();
  ASSERT_EQ(sd_match_state.cur_coord_index, 0);
  ASSERT_EQ(sd_match_state.segment_dists.size(),
            routing_result.path_points_from_current().size());
  EXPECT_NEAR(sd_match_state.segment_dists[0], 0.0, 1E-6);
}

TEST(SdRouteUtil, FillRouteManagerState) {
  auto sd_route = CreateSdRouteProto();
  RouteManagerState rms;
  auto sd_route_shared = std::make_shared<const SDRouteProto>(sd_route);
  auto s = FillRouteManagerState(sd_route_shared, nullptr, &rms);
  ASSERT_OK(s);
  EXPECT_NEAR(0, rms.sd_route_match_state.s, 1E-6);
  ASSERT_EQ(sd_route.length(), rms.sd_route_match_state.sd_route_length);
  ASSERT_EQ(rms.sd_route_match_state.coords.size(),
            rms.routing_result_proto.path_points_from_current().size());
  {
    SdRouteMatchResultProto sd_match;
    sd_match.set_route_id(100);
    auto* segment = sd_match.add_segments();
    segment->mutable_start()->set_link_index(0);
    segment->mutable_start()->set_link_point_index(1);
    auto* start_pt = segment->mutable_start()->mutable_point();
    start_pt->set_longitude(2.0);
    start_pt->set_latitude(0.501001);

    segment->mutable_end()->set_link_index(1);
    segment->mutable_end()->set_link_point_index(0);
    auto* end_pt = segment->mutable_end()->mutable_point();
    end_pt->set_longitude(2.0);
    end_pt->set_latitude(0.501002);
    auto update_status = InitSdRouteMatchResult(
        std::make_shared<const SdRouteMatchResultProto>(std::move(sd_match)),
        &rms.sd_route_match_state);
    ASSERT_OK(update_status);
    ASSERT_EQ(
        rms.sd_route_match_state.match_dist_ranges_from_cur.size(),
        rms.sd_route_match_state.sd_route_match_result_proto->segments_size());
    ASSERT_GT(rms.sd_route_match_state.match_dist_ranges_from_cur[0].start, 0);
  }
}

}  // namespace
}  // namespace qcraft::planner::route::noa
