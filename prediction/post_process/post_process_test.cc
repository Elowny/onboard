#include "onboard/prediction/post_process/post_process.h"

#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {
namespace {

inline ObjectPredictionResult ObjectPredictionToObjectPredictionResult(
    const ObjectPrediction& obj_pred) {
  return ObjectPredictionResult{
      .id = obj_pred.id(),
      .priority = ObjectPredictionPriority::OPP_P1,
      .perception_object = obj_pred.perception_object(),
      .trajectories = obj_pred.trajectories(),
  };
}

std::map<ObjectIDType, ObjectPredictionResult>
BuildPredResultsMapForPostProcess() {
  const auto obj_1 = planner::PerceptionObjectBuilder().set_id("1").Build();
  planner::ObjectPredictionBuilder builder_1;
  builder_1.set_object(obj_1)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(45.0, 0.0),
                         /*init_v=*/5.0, /*last_v=*/4.0);
  builder_1.add_predicted_trajectory()->set_probability(0.7).set_straight_line(
      /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(50.0, 0.0),
      /*init_v=*/5.0, /*last_v=*/5.0);
  const auto obj_prediction_1 = builder_1.Build();
  const auto obj_2 = planner::PerceptionObjectBuilder()
                         .set_pos(Vec2d(30.0, 0.0))
                         .set_speed(Vec2d::Zero())
                         .set_accel(Vec2d::Zero())
                         .set_id("2")
                         .Build();
  planner::ObjectPredictionBuilder builder_2;
  const auto obj_prediction_2 = builder_2.set_object(obj_2).Build();
  std::map<ObjectIDType, ObjectPredictionResult> obj_pred_result_map = {
      {obj_1.id(), ObjectPredictionToObjectPredictionResult(obj_prediction_1)},
      {obj_2.id(), ObjectPredictionToObjectPredictionResult(obj_prediction_2)}};
  return obj_pred_result_map;
}

std::vector<ObjectPrediction> BuildObjectsPredictionForPostProcess() {
  const auto obj_1 = planner::PerceptionObjectBuilder().set_id("1").Build();
  planner::ObjectPredictionBuilder builder_1;
  builder_1.set_object(obj_1)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(45.0, 0.0),
                         /*init_v=*/5.0, /*last_v=*/4.0);
  builder_1.add_predicted_trajectory()->set_probability(0.7).set_straight_line(
      /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(50.0, 0.0),
      /*init_v=*/5.0, /*last_v=*/5.0);
  auto obj_prediction_1 = builder_1.Build();
  const auto obj_2 = planner::PerceptionObjectBuilder()
                         .set_pos(Vec2d(30.0, 0.0))
                         .set_speed(Vec2d::Zero())
                         .set_accel(Vec2d::Zero())
                         .set_id("2")
                         .Build();
  planner::ObjectPredictionBuilder builder_2;
  auto obj_prediction_2 = builder_2.set_object(obj_2).Build();
  return {std::move(obj_prediction_1), std::move(obj_prediction_2)};
}

TEST(PredictionPostProcessTest,
     RunPredictionPostProcessFromObjectPredictionResult) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  ConflictResolverParams params;
  params.LoadParams();
  ConflictResolverDebugProto debug_proto;
  auto pred_result_map = BuildPredResultsMapForPostProcess();
  const auto obj_pred_pp = FromObjectPredictionResults(&pred_result_map);
  const auto status_or = RunPredictionPostProcess(psmm, /*red_tls=*/{}, params,
                                                  obj_pred_pp, &debug_proto,
                                                  /*thread_pool=*/nullptr);
  EXPECT_TRUE(status_or.ok());
}

TEST(PredictionPostProcessTest, RunPredictionPostProcessFromObjectPrediction) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  ConflictResolverParams params;
  params.LoadParams();
  ConflictResolverDebugProto debug_proto;
  auto obj_predictions = BuildObjectsPredictionForPostProcess();
  const auto obj_pred_pp =
      FromObjectPredictions(absl::MakeSpan(obj_predictions));
  const auto status_or = RunPredictionPostProcess(psmm, /*red_tls=*/{}, params,
                                                  obj_pred_pp, &debug_proto,
                                                  /*thread_pool=*/nullptr);
  EXPECT_TRUE(status_or.ok());
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
