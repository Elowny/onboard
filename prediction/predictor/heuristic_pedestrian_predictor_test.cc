#include "onboard/prediction/predictor/heuristic_pedestrian_predictor.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::prediction {
namespace {
const double kTimeStep = 0.1;
const int kHistoryNum = 10;
TEST(HeuristicPedestrianPredictorTest, OneStateTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  FLAGS_prediction_use_tracker_history = false;
  const Vec2d pos(8.0, 19.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_PEDESTRIAN)
                 .set_pos(pos)
                 .set_velocity(0.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  FLAGS_prediction_use_tracker_history = false;
  *objects_proto.add_objects() = std::move(obj);
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  const auto& hist = input.objects_history->at(id);
  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist}, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);

  const auto traj = MakeHeuristicPedestrianPrediction(
      obj_sampler.GetResampledMotionHistoryById(id));
  EXPECT_EQ(traj.type(), PT_CYCV);
}

TEST(HeuristicPedestrianPredictorTest, NormalTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(15.0, 38.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_PEDESTRIAN)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  FLAGS_prediction_use_tracker_history = false;
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  const Vec2d pos2(14.9, 38.0);
  auto obj2 = planner::PerceptionObjectBuilder()
                  .set_id(id)
                  .set_type(OT_PEDESTRIAN)
                  .set_pos(pos2)
                  .set_velocity(1.0)
                  .set_timestamp(0.1)
                  .set_box_center(pos2)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  ObjectsProto objects_proto2;
  *objects_proto2.add_objects() = std::move(obj2);
  input.objects_history->Update(objects_proto2, *input.localization_transform,
                                /*thread_pool=*/nullptr);

  const auto& hist = input.objects_history->at(id);
  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist}, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);

  const auto traj = MakeHeuristicPedestrianPrediction(
      obj_sampler.GetResampledMotionHistoryById(id));
  EXPECT_EQ(traj.type(), PT_PED_KINEMATIC);
  EXPECT_EQ(traj.points().size(),
            static_cast<int>(kComfortableHorizon / kPredictionTimeStep));
}

TEST(HeuristicPedestrianPredictorTest, CurbCutoffTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  FLAGS_prediction_use_tracker_history = false;
  const Vec2d pos(-27.0, 23.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_PEDESTRIAN)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);

  const Vec2d pos2(-26.9, 23.0);
  auto obj2 = planner::PerceptionObjectBuilder()
                  .set_id(id)
                  .set_type(OT_PEDESTRIAN)
                  .set_pos(pos2)
                  .set_velocity(1.0)
                  .set_timestamp(0.1)
                  .set_box_center(pos2)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  ObjectsProto objects_proto2;
  *objects_proto2.add_objects() = std::move(obj2);
  input.objects_history->Update(objects_proto2, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  const auto& hist = input.objects_history->at(id);
  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist}, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);

  const auto traj = MakeHeuristicPedestrianPrediction(
      obj_sampler.GetResampledMotionHistoryById(id));
  EXPECT_EQ(traj.type(), PT_PED_KINEMATIC);
  // Now assume ped can cross all curbs.
  EXPECT_GE(traj.points().size(),
            static_cast<int>(kSafeHorizon / kPredictionTimeStep));
}

}  // namespace
}  // namespace qcraft::prediction
