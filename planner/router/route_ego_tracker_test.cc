#include "onboard/planner/router/route_ego_tracker.h"

#include <string>

#include "absl/status/statusor.h"
#include "absl/time/time.h"

#include "gtest/gtest.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/clock.h"
#include "onboard/global/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/proto/route.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner::route {

namespace {
TEST(RouteEgoTracker, FindLaneFromLastCompositeIndexTest) {
  const auto clp_str = R"(
    lane_paths {
    lane_ids: 3
    lane_ids: 950
    lane_ids: 35
    lane_ids: 2472
    lane_ids: 54
    lane_ids: 168
    lane_ids: 129
    start_fraction: 0.5
    end_fraction: 0.5
    lane_path_in_forward_direction: true
    })";
  CompositeLanePathProto clp;
  file_util::StringToProto(clp_str, &clp);
  {
    absl::StatusOr<PointToCompositeLanePath> point_clp =
        FindLaneFromLastCompositeIndex(clp, /*last_composite_index=*/{0, 1},
                                       {2470, 2471, 2472},
                                       mapping::ElementId(2472));
    EXPECT_TRUE(point_clp.ok());
    EXPECT_TRUE(point_clp->point_on_route_lane);
    EXPECT_EQ(point_clp->route_lane_id, mapping::ElementId(2472));
  }

  {
    absl::StatusOr<PointToCompositeLanePath> point_clp =
        FindLaneFromLastCompositeIndex(clp, /*last_composite_index=*/{0, 1},
                                       {2470, 2471, 2472},
                                       mapping::ElementId(2471));
    EXPECT_TRUE(point_clp.ok());
    EXPECT_FALSE(point_clp->point_on_route_lane);
    EXPECT_EQ(point_clp->route_lane_id, mapping::ElementId(2472));
  }

  {
    // Failed if `last_composite_index` is out of range
    absl::StatusOr<PointToCompositeLanePath> point_clp =
        FindLaneFromLastCompositeIndex(clp, /*last_composite_index=*/{0, 4},
                                       {2470, 2471, 472},
                                       mapping::ElementId(2471));
    EXPECT_FALSE(point_clp.ok());
  }

  {
    // Failed if the lane need to find is not in the given section.
    absl::StatusOr<PointToCompositeLanePath> point_clp =
        FindLaneFromLastCompositeIndex(clp, /*last_composite_index=*/{0, 3},
                                       {46, 47, 48}, mapping::ElementId(46));
    EXPECT_FALSE(point_clp.ok());
  }
}

TEST(RouteEgoTracker, GetTrackerInfoOnRouteTest) {
  const auto& smm = LoadDojoMap();
  const route::MapIndex* map_index = LoadDojoIndex();
  auto cc = CoordinateConverter::FromMap("dojo");
  RouteManagerState rm_state;
  const auto clp_str = R"(
    lane_paths {
    lane_ids: 3
    lane_ids: 950
    lane_ids: 35
    lane_ids: 2472
    lane_ids: 54
    lane_ids: 168
    lane_ids: 129
    start_fraction: 0.5
    end_fraction: 0.5
    lane_path_in_forward_direction: true
    })";

  file_util::StringToProto(clp_str, rm_state.route.mutable_lane_path());
  rm_state.composite_index = {0, 1};  //   lane_ids: 950
  double heading = 0.0;
  SMM_LANE_PROTO_OR_RETURN(lane_proto, smm, 950, void());
  SMM_SECTION_PROTO_OR_RETURN(section_proto, smm, lane_proto->section_id(),
                              void());
  Vec3d global = ComputePointOnLane(lane_proto->polyline().points(), 0.3);
  rm_state.last_lane_point = {mapping::ElementId(950), 0.2};

  const auto cur_time_micros = absl::ToUnixMicros(Clock::Now());
  rm_state.rms_debug.set_last_update_time(cur_time_micros -
                                          SecondsToMicroSeconds(1));
  RouteTrackerDebugProto tracker = GetTrackerInfoOnRoute(
      smm, map_index, cc.GetLevel(), rm_state, {global.x(), global.y()},
      heading, /*av_speed_x=*/10.0);
  EXPECT_TRUE(tracker.success());
  EXPECT_TRUE(tracker.point_on_route_lane());
  EXPECT_NEAR(tracker.travel_distance(),
              section_proto->average_length() * (0.3 - 0.2), 1.0);
}

}  // namespace

}  // namespace qcraft::planner::route
