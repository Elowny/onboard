#include "onboard/planner/router/route_manager.h"

#include <algorithm>
#include <tuple>
#include <vector>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_ego_tracker.h"
#include "onboard/planner/router/router_flags.h"
namespace qcraft {
namespace planner {
namespace {

TEST(RouteManager, RouteSTest) {
  CompositeLanePath composite_lane_path;
  mapping::LanePath lane_path1;
  lane_path1.lane_path_data_.lane_ids_ = {
      mapping::ElementId(1), mapping::ElementId(2), mapping::ElementId(3)};
  mapping::LanePath lane_path2;
  lane_path2.lane_path_data_.lane_ids_ = {mapping::ElementId(4),
                                          mapping::ElementId(5)};
  mapping::LanePath lane_path3;
  lane_path3.lane_path_data_.lane_ids_ = {mapping::ElementId(6)};
  composite_lane_path.lane_paths_ = {lane_path1, lane_path2, lane_path3};
  composite_lane_path.transitions_ = {};
  composite_lane_path.lane_path_end_s_ = {0.0, 10.0, 20.0, 30.0};

  CompositeLanePath::CompositeIndex composite_index = {0, 1};  // lane_id=2
  std::vector<std::int64_t> section_lane_ids = {3, 30, 300};
  mapping::ElementId to_find_lane_id = mapping::ElementId(3);
  CompositeLanePathProto composite_lane_path_proto;
  composite_lane_path.ToProto(&composite_lane_path_proto);
  absl::StatusOr<PointToCompositeLanePath> found_index_or =
      FindLaneFromLastCompositeIndex(composite_lane_path_proto, composite_index,
                                     absl::MakeSpan(section_lane_ids),
                                     to_find_lane_id);
  CHECK(found_index_or.ok());
  CHECK(found_index_or->point_on_route_lane);
  CompositeLanePath::CompositeIndex expected_composite_index(0, 2);
  EXPECT_EQ(found_index_or->composite_index,
            expected_composite_index);  // perfect match

  to_find_lane_id = mapping::ElementId(30);
  found_index_or = FindLaneFromLastCompositeIndex(
      composite_lane_path_proto, composite_index,
      absl::MakeSpan(section_lane_ids), to_find_lane_id);

  CHECK(found_index_or.ok());
  EXPECT_FALSE(found_index_or->point_on_route_lane);
  EXPECT_EQ(found_index_or->composite_index, expected_composite_index);

  found_index_or = FindLaneFromLastCompositeIndex(
      composite_lane_path_proto, composite_index,
      std::vector<std::int64_t>({5, 50, 500}), mapping::ElementId(50));
  CHECK(found_index_or.ok());
  EXPECT_EQ(found_index_or->composite_index,
            CompositeLanePath::CompositeIndex(1, 1));
}

TEST(RouteManager, UpdateStationsInfoTest) {
  RouteManager route_manager(/* semantic_map_manager= */ nullptr,
                             /* map_index= */ nullptr,
                             /* route_param_proto= */ nullptr);
  CompositeLanePathProto composite_lane_path_proto;
  auto* lane_path = composite_lane_path_proto.add_lane_paths();
  lane_path->set_start_fraction(0.5);
  lane_path->set_end_fraction(1.0);
  lane_path->add_lane_ids(1);
  {
    //  Stop arrived
    RouteProto* route_proto =
        &route_manager.mutable_route_manager_state()->route_from_current;
    auto* dest1 = route_proto->mutable_routing_request()->add_destinations();
    dest1->set_id("test1");
    auto* point = dest1->mutable_global_point();
    point->set_longitude(2.0);
    point->set_latitude(1.0);
    *route_proto->mutable_lane_path() = composite_lane_path_proto;

    auto& state = route_manager.mutable_route_manager_state()->rms_debug;
    state.set_distance_to_next_stop(FLAGS_route_arrive_distance - 1.0);
    state.set_distance_to_next_viapoint(FLAGS_route_arrive_distance - 1.0);
    state.set_reached_destination_index(0);
    state.set_pending_next_destination_index(1);
    state.set_reached_dest_cnt(0);

    std::ignore = route_manager.UpdateStationsInfo(composite_lane_path_proto);
    EXPECT_EQ(state.pending_next_destination_index(), 1);
    EXPECT_EQ(state.reached_destination_index(), 1);
    EXPECT_EQ(state.reached_dest_cnt(), 1);

    // Duplicate, never update any thing
    std::ignore = route_manager.UpdateStationsInfo(composite_lane_path_proto);
    EXPECT_EQ(state.pending_next_destination_index(), 1);
    EXPECT_EQ(state.reached_destination_index(), 1);
    EXPECT_EQ(state.reached_dest_cnt(), 1);
    EXPECT_EQ(route_manager.route_manager_state()
                  .route_from_current.routing_request()
                  .destinations_size(),
              1);
  }

  {
    //  Via-point arrived
    route_manager.mutable_route_manager_state()->route_from_current.Clear();
    RouteProto* route_proto =
        &route_manager.mutable_route_manager_state()->route_from_current;
    auto* dest1 = route_proto->mutable_routing_request()->add_destinations();
    dest1->set_id("test1");
    auto* point1 = dest1->mutable_global_point();
    point1->set_longitude(2.0);
    point1->set_latitude(1.0);

    auto* dest2 = route_proto->mutable_routing_request()->add_destinations();
    dest2->set_id("test2");
    auto* point2 = dest2->mutable_global_point();
    point2->set_longitude(2.000005);
    point2->set_latitude(1.000005);
    *route_proto->mutable_lane_path() = composite_lane_path_proto;

    auto& state = route_manager.mutable_route_manager_state()->rms_debug;
    state.set_distance_to_next_stop(2 * FLAGS_route_arrive_distance + 1);
    state.set_distance_to_next_viapoint(FLAGS_route_arrive_distance - 1.0);
    state.set_reached_destination_index(0);
    state.set_pending_next_destination_index(1);
    state.set_reached_dest_cnt(0);
    EXPECT_EQ(route_manager.route_manager_state()
                  .route_from_current.routing_request()
                  .destinations_size(),
              2);

    std::ignore = route_manager.UpdateStationsInfo(composite_lane_path_proto);
    EXPECT_EQ(state.pending_next_destination_index(), 2);
    EXPECT_EQ(state.reached_destination_index(), 1);
    EXPECT_EQ(state.reached_dest_cnt(), 1);
    EXPECT_EQ(route_manager.route_manager_state()
                  .route_from_current.routing_request()
                  .destinations_size(),
              1);

    state.set_distance_to_next_stop(FLAGS_route_arrive_distance - 1.0);
    std::ignore = route_manager.UpdateStationsInfo(composite_lane_path_proto);
    EXPECT_EQ(state.pending_next_destination_index(), 2);
    EXPECT_EQ(state.reached_destination_index(), 2);
    EXPECT_EQ(state.reached_dest_cnt(), 2);
    EXPECT_EQ(route_manager.route_manager_state()
                  .route_from_current.routing_request()
                  .destinations_size(),
              1);
  }
}

TEST(RouteManager, IsStopArrived) {
  FLAGS_route_arrive_distance = 10.0;
  EXPECT_TRUE(internal::IsStopArrived(10.0));
  EXPECT_FALSE(internal::IsStopArrived(100.0));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
