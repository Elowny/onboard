#include "onboard/planner/planner_state.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {
namespace {

using ::testing::Pair;
using ::testing::UnorderedElementsAre;

absl::Time MakeTime(double seconds) { return absl::FromUnixSeconds(seconds); }

TEST(PlannerState, Proto) {
  PlannerState state;
  state.planner_frame_seq_num = 9;
  state.header.set_timestamp(2);
  state.header.set_domain("planner");
  state.header.set_seq_number(10);
  state.header.set_channel("planner_state");

  {
    ApolloTrajectoryPointProto point;
    point.mutable_path_point()->set_x(1.0);
    point.mutable_path_point()->set_y(2.0);
    point.mutable_path_point()->set_theta(3.0);
    point.set_v(4.0);
    *state.previous_trajectory.add_trajectory_point() = point;
  }
  state.last_audio_alert_time = MakeTime(42.0);
  state.parking_brake_release_time = MakeTime(40.0);

  {
    LaneChangeStateProto lc_state;
    lc_state.set_stage(LaneChangeStage::LCS_WAITING);
    state.lane_change_state = lc_state;
  }

  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  state.previous_autonomy_state = AutonomyStateProto();
  state.selector_state = SelectorState();

  state.input_seq_num.set_pose(1);
  state.input_seq_num.set_autonomy_state(4);
  state.input_seq_num.set_traffic_light_states(5);
  state.input_seq_num.set_driver_action(6);
  state.input_seq_num.set_remote_assist_to_car(8);
  state.input_seq_num.set_chassis(10);
  state.input_seq_num.set_prediction(11);
  state.input_seq_num.set_localization_transform(12);
  state.input_seq_num.set_av_objects(14);
  state.input_seq_num.set_real_objects(15);
  state.input_seq_num.set_virtual_objects(16);

  state.last_door_override_time = 3.0;
  state.current_time = absl::FromUnixMicros(1637127272874026L);

  PlannerState other_state = state;
  EXPECT_EQ(state, other_state);

  PlannerStateProto proto;
  state.ToProto(&proto);

  PlannerState from;
  from.FromProto(proto);

  EXPECT_EQ(from, state);
}

}  // namespace
}  // namespace qcraft::planner
