#include "onboard/prediction/container/traffic_light_manager.h"

#include "gtest/gtest.h"

#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(TrafficLightManagerTest, CheckTrafficLightManager) {
  TrafficLightManager traffic_light_manager;
  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightStatesProto light_states;

  TrafficLightStateProto light_state_red;
  light_state_red.set_color(qcraft::TrafficLightColor::TL_RED);
  light_state_red.set_traffic_light_id(920);
  light_state_red.set_flashing(true);
  *light_states.add_states() = light_state_red;

  TrafficLightStateProto light_state_green;
  light_state_green.set_color(qcraft::TrafficLightColor::TL_GREEN);
  light_state_green.set_traffic_light_id(902);
  light_state_green.set_flashing(false);
  *light_states.add_states() = light_state_green;

  TrafficLightStateProto light_state_yellow;
  light_state_yellow.set_color(qcraft::TrafficLightColor::TL_YELLOW);
  light_state_yellow.set_traffic_light_id(904);
  light_state_yellow.set_flashing(false);
  *light_states.add_states() = light_state_yellow;

  TrafficLightStateProto light_state_yellow_2;
  light_state_yellow_2.set_color(qcraft::TrafficLightColor::TL_YELLOW);
  light_state_yellow_2.set_traffic_light_id(987);
  light_state_yellow_2.set_flashing(false);
  *light_states.add_states() = light_state_yellow_2;

  TrafficLightStateProto light_state_yellow_3;
  light_state_yellow_3.set_color(qcraft::TrafficLightColor::TL_YELLOW);
  light_state_yellow_3.set_traffic_light_id(200);
  light_state_yellow_3.set_flashing(false);
  *light_states.add_states() = light_state_yellow_3;

  traffic_light_manager.UpdateTlStateMap(psmm, light_states);

  EXPECT_EQ(traffic_light_manager.GetInferedTlStateMap().size(), 12);
  EXPECT_EQ(traffic_light_manager.GetOriginalTlStateMap().size(), 5);
  EXPECT_EQ(traffic_light_manager.GetOriginalTlStateMap().contains(
                mapping::ElementId(920)),
            true);
  EXPECT_EQ(traffic_light_manager.GetOriginalTlStateMap().contains(
                mapping::ElementId(902)),
            true);
  EXPECT_EQ(traffic_light_manager.GetOriginalTlStateMap().contains(
                mapping::ElementId(904)),
            true);
  EXPECT_EQ(traffic_light_manager.GetOriginalTlStateMap().contains(
                mapping::ElementId(987)),
            true);
  EXPECT_EQ(traffic_light_manager.GetOriginalTlStateMap().contains(
                mapping::ElementId(200)),
            true);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
