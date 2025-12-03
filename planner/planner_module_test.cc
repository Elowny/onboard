#include "onboard/planner/planner_module.h"

#include "gtest/gtest.h"

#include "onboard/lite/unittest/lite2_unittest_helper.h"

DECLARE_bool(non_trigger_channel);

namespace qcraft {
namespace planner {
namespace {

class PlannerModuleTest : public ::testing::Test {
 protected:
  void SetUp() override { module_.Init("PlannerModule", PLANNER_MODULE); }

  void TearDown() override { module_.Destroy(); }

  PlannerModule* planner_module() {
    return dynamic_cast<PlannerModule*>(module_.GetLiteModule());
  }

 private:
  Lite2UnitTestHelper module_;
};

TEST_F(PlannerModuleTest, ModuleTest) {
  auto* planner = planner_module();
  const auto& input = planner->GetPlannerInput();
  EXPECT_NE(input.traffic_light_states, nullptr);
  EXPECT_EQ(input.semantic_map_manager, nullptr);
  EXPECT_EQ(input.planner_state_proto, nullptr);
  EXPECT_EQ(input.pose, nullptr);
  EXPECT_EQ(input.real_objects, nullptr);
  EXPECT_EQ(input.localization_transform, nullptr);
  EXPECT_EQ(input.online_semantic_map, nullptr);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
