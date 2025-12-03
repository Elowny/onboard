#include "onboard/planner/router/sim/snapshot_sim_state.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/utils/proto_util.h"
namespace qcraft::planner::route {
namespace {

TEST(SnapshotSimState, RecoverPartialSnapshotStateTest) {
  const auto& smm = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();
  RoutingResultProto routing_result_proto;
  routing_result_proto.set_update_id(1234L);
  routing_result_proto.set_success(true);

  auto* lane_path = routing_result_proto.mutable_lane_path()->add_lane_paths();
  lane_path->set_start_fraction(0.5);
  lane_path->set_end_fraction(1.0);
  lane_path->add_lane_ids(1);

  RouteManagerStateDebugProto rms_debug;
  rms_debug.set_distance_to_next_stop(100);
  rms_debug.set_last_update_time(0L);

  RouteSnapshotSimInput sim_input = {
      .routing_result_proto = &routing_result_proto, .rms_debug = &rms_debug};
  auto state_or = RecoverPartialSnapshotState(smm, map_index, sim_input);
  EXPECT_TRUE(state_or.ok());
  EXPECT_TRUE(
      ProtoEquals(state_or->routing_result_proto, routing_result_proto));
  EXPECT_EQ(state_or->rms_debug.distance_to_next_stop(),
            rms_debug.distance_to_next_stop());
  EXPECT_EQ(state_or->rms_debug.last_update_time(),
            rms_debug.last_update_time());
}

}  // namespace

}  // namespace qcraft::planner::route
