#include "onboard/prediction/predictor/kinematic_predictor.h"

#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(KinematicPredictorTest, MakeStationaryPrediction) {
  const auto object_motion_history = BuildVehicleMotionHistoryByConstVel(
      "stationary_obj", 10, Vec2d::Zero(), 0.0);
  const auto pred_traj = MakeStationaryPrediction(object_motion_history, 5.0);
  EXPECT_EQ(pred_traj.probability(), 1.0);
  EXPECT_FALSE(pred_traj.is_reversed());
  EXPECT_EQ(pred_traj.points().size(), 50);
  EXPECT_NEAR(pred_traj.points().back().s(), 0.0, 1e-3);
  EXPECT_EQ(pred_traj.annotation(), "Stationary Prediction");
  EXPECT_EQ(pred_traj.type(), PredictionType::PT_STATIONARY);
}

TEST(KinematicPredictorTest, MakeCYCVPrediction) {
  const auto object_motion_history =
      BuildVehicleMotionHistoryByConstVel("cycv_obj", 10, Vec2d::Zero(), 5.0);
  const auto pred_traj = MakeCYCVPrediction(object_motion_history, 5.0);
  EXPECT_EQ(pred_traj.probability(), 1.0);
  EXPECT_FALSE(pred_traj.is_reversed());
  EXPECT_EQ(pred_traj.points().size(), 50);
  EXPECT_NEAR(pred_traj.points().back().s(), 24.5, 1e-3);
  EXPECT_EQ(pred_traj.type(), PredictionType::PT_CYCV);
}

TEST(KinematicPredictorTest, MakeReverseCYCVPrediction) {
  const auto object_motion_history = BuildVehicleMotionHistoryByConstVel(
      "reversed_cycv_obj", 10, Vec2d::Zero(), -1.0);
  const auto pred_traj = MakeReverseCYCVPrediction(object_motion_history, 5.0);
  EXPECT_EQ(pred_traj.probability(), 1.0);
  EXPECT_TRUE(pred_traj.is_reversed());
  EXPECT_EQ(pred_traj.points().size(), 50);
  EXPECT_NEAR(pred_traj.points().back().s(), 4.9, 1e-3);
  EXPECT_EQ(pred_traj.type(), PredictionType::PT_REVERSE_CYCV);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
