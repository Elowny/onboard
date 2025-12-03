#include "onboard/prediction/predictor/void_predictor.h"

#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(VoidPredictorTest, MakeVoidPrediction) {
  const auto object_motion_history =
      BuildVehicleMotionHistoryByConstVel("obj", 10, Vec2d::Zero(), 5.0);
  const auto pred_traj = MakeVoidPrediction(object_motion_history);
  EXPECT_EQ(pred_traj.probability(), 1.0);
  EXPECT_FALSE(pred_traj.is_reversed());
  EXPECT_EQ(pred_traj.points().size(), 1);
  EXPECT_NEAR(pred_traj.points().back().s(), 0.0, 1e-3);
  EXPECT_EQ(pred_traj.annotation(), "Void Prediction");
  EXPECT_EQ(pred_traj.type(), PredictionType::PT_VOID);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
