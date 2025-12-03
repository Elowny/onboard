#include "onboard/control/controllers/predict_vehicle_pose.h"

#include "gtest/gtest.h"

#include "onboard/lite/logging.h"

namespace qcraft::control {
namespace {

TEST(VehPoseTest, AdvancedByTest) {
  // TODO(Yangyu): add a unit test for AdvancedBy();
}

TEST(PredictedVehPoseTest, PredictLatTest) {
  // TODO(Yangyu): add a unit test for PredictLatControlInitPose();
}

TEST(PredictedVehPoseTest, PredictLonTest) {
  // TODO(Yangyu): add a unit test for PredictLonControlInitPose();
}

TEST(PredictedVehPoseTest, OperatorTest) {
  VehPose vehpose_base, vehpose_comp;
  vehpose_base.x = 100.0;
  vehpose_base.y = 110.0;
  vehpose_base.heading = 0.5;
  vehpose_base.lateral_velocity = 0.4;
  vehpose_base.moving_direction = 0.3;

  vehpose_comp.x = 101.0;
  vehpose_comp.y = 111.0;
  vehpose_comp.heading = 0.6;
  vehpose_comp.lateral_velocity = 0.5;
  vehpose_comp.moving_direction = 0.4;

  const VehPose veh_pose_diff = vehpose_base - vehpose_comp;
  QLOG(INFO) << veh_pose_diff.DebugString();
  EXPECT_DOUBLE_EQ(veh_pose_diff.x, vehpose_base.x - vehpose_comp.x);
  EXPECT_DOUBLE_EQ(veh_pose_diff.y, vehpose_base.y - vehpose_comp.y);
  EXPECT_DOUBLE_EQ(veh_pose_diff.heading,
                   vehpose_base.heading - vehpose_comp.heading);
  EXPECT_DOUBLE_EQ(
      veh_pose_diff.lateral_velocity,
      vehpose_base.lateral_velocity - vehpose_comp.lateral_velocity);
  EXPECT_DOUBLE_EQ(
      veh_pose_diff.moving_direction,
      vehpose_base.moving_direction - vehpose_comp.moving_direction);
}

}  // namespace
}  // namespace qcraft::control
