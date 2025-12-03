#include "onboard/prediction/scheduler/scheduler_utils.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/vehicle.pb.h"

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"

namespace qcraft::prediction {
namespace {
const double kTimeStep = 0.1;
const int kHistoryNum = 10;
TEST(SchedulerUtilsTest, IsHighOrEqualPriorityTest) {
  const PredictionType type1 = PredictionType::PT_ACTNET;
  const PredictionType type2 = PredictionType::PT_CTRA;
  const bool out1 = IsHighOrEqualPriority(type1, type2);
  const bool out2 = IsHighOrEqualPriority(type2, type1);
  EXPECT_TRUE(out1);
  EXPECT_FALSE(out2);
}

TEST(SchedulerUtilsTest, TryInsertTrajectoriesTest) {
  std::map<std::string, std::vector<PredictedTrajectory>> obj_traj_map;
  std::vector<PredictedTrajectory> trajs;

  const Vec2d start(0.0, 0.0);
  const Vec2d end(10.0, 10.0);
  const double init_v = 10.0;  // m/s
  const double last_v = 20.0;  // m/s
  const ObjectIDType obj_id = "1";

  planner::PredictedTrajectoryBuilder builder;
  auto traj1 = builder.set_annotation("123")
                   .set_type(PredictionType::PT_PED_KINEMATIC)
                   .set_probability(0.6)
                   .set_straight_line(start, end, init_v, last_v)
                   .Build();
  trajs.push_back(std::move(traj1));
  auto traj2 = builder.set_annotation("123")
                   .set_type(PredictionType::PT_PED_KINEMATIC)
                   .set_probability(0.4)
                   .set_straight_line(start, end, init_v, last_v)
                   .Build();
  trajs.push_back(std::move(traj2));
  TryInsertTrajectories(&obj_traj_map, trajs, obj_id);

  EXPECT_EQ(obj_traj_map.size(), 1);
  EXPECT_EQ(obj_traj_map[obj_id].size(), 2);
  EXPECT_FLOAT_EQ(obj_traj_map[obj_id][0].probability(), 0.6);
  EXPECT_FLOAT_EQ(obj_traj_map[obj_id][1].probability(), 0.4);
}

TEST(SchedulerUtilsTest, RecordPredSpeeGapQeventTest) {
  const Vec2d start(0.0, 0.0);
  const Vec2d end(10.0, 10.0);
  const double init_v = 10.0;  // m/s
  const double last_v = 20.0;  // m/s
  const ObjectIDType obj_id = "1";

  planner::PredictedTrajectoryBuilder builder;
  const auto traj1 = builder.set_annotation("123")
                         .set_type(PredictionType::PT_PED_KINEMATIC)
                         .set_probability(0.6)
                         .set_straight_line(start, end, init_v, last_v)
                         .Build();
  std::vector<PredictedTrajectory> obj_trajs;
  obj_trajs.push_back(traj1);
  RecordPredSpeeGapQevent("1", 10.0, obj_trajs);
}

TEST(SchedulerUtilsTest, PredictStationaryAndReversedTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

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
  const Vec2d pos2(8.0, 10.0);
  std::string id2("2");
  const double velocity2 = 5.0;
  auto obj2 = planner::PerceptionObjectBuilder()
                  .set_id(id2)
                  .set_type(OT_PEDESTRIAN)
                  .set_pos(pos2)
                  .set_velocity(velocity2)
                  .set_timestamp(0.0)
                  .set_box_center(pos2)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  const Vec2d v2 = Vec2d::FastUnitFromAngle(M_PI) * velocity2;
  v2.ToProto(obj2.mutable_vel());
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  *objects_proto.add_objects() = std::move(obj2);
  FLAGS_prediction_use_tracker_history = false;
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);

  auto& hist = input.objects_history->at(id);
  auto& hist2 = input.objects_history->at(id2);
  std::vector<ObjectHistory*> objs_to_predict;
  objs_to_predict.push_back(&hist);
  objs_to_predict.push_back(&hist2);

  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  scenarios[id2] = scenario;
  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist, &hist2}, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);
  std::map<ObjectIDType, std::vector<PredictedTrajectory>> object_trajs;
  PredictStationaryAndReversed(objs_to_predict, obj_sampler, &object_trajs,
                               scenarios,
                               /*ignore_off_road=*/false);
  EXPECT_EQ(object_trajs[id].size(), 1);
  EXPECT_EQ(object_trajs[id][0].type(), PredictionType::PT_STATIONARY);
  EXPECT_EQ(object_trajs[id2].size(), 1);
  EXPECT_EQ(object_trajs[id2][0].type(), PredictionType::PT_REVERSE_CYCV);
}

TEST(SchedulerUtilsTest, PredictIgnoredObjectsTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

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
  const Vec2d pos2(8.0, 10.0);
  std::string id2("2");
  const double velocity2 = 5.0;
  auto obj2 = planner::PerceptionObjectBuilder()
                  .set_id(id2)
                  .set_type(OT_PEDESTRIAN)
                  .set_pos(pos2)
                  .set_velocity(velocity2)
                  .set_timestamp(0.0)
                  .set_box_center(pos2)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  const Vec2d v2 = Vec2d::FastUnitFromAngle(M_PI) * velocity2;
  v2.ToProto(obj2.mutable_vel());
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  *objects_proto.add_objects() = std::move(obj2);
  FLAGS_prediction_use_tracker_history = false;
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  auto& hist = input.objects_history->at(id);
  auto& hist2 = input.objects_history->at(id2);
  std::vector<ObjectHistory*> objs_to_predict;
  objs_to_predict.push_back(&hist);
  objs_to_predict.push_back(&hist2);

  std::map<ObjectIDType, ObjectPredictionPriorityInfo> priorities;
  priorities[id].priority = OPP_P1;
  priorities[id2].priority = OPP_P3;
  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist, &hist2}, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);
  std::map<ObjectIDType, std::vector<PredictedTrajectory>> object_trajs;
  PredictIgnoredObjects(objs_to_predict, obj_sampler, priorities,
                        &object_trajs);
  EXPECT_EQ(object_trajs[id].size(), 0);
  EXPECT_EQ(object_trajs[id2].size(), 1);
  EXPECT_EQ(object_trajs[id2][0].type(), PredictionType::PT_VOID);
}

TEST(SchedulerUtilsTest, PredictObjectsByHeuristicTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos1(8.0, 19.0);
  std::string id1("1");
  auto obj1 = planner::PerceptionObjectBuilder()
                  .set_id(id1)
                  .set_type(OT_PEDESTRIAN)
                  .set_pos(pos1)
                  .set_velocity(1.0)
                  .set_timestamp(0.0)
                  .set_box_center(pos1)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj1);
  FLAGS_prediction_use_tracker_history = false;
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);

  std::map<ObjectIDType, std::vector<PredictedTrajectory>> object_trajs;

  const Vec2d pos2(3.0, 7.0);
  std::string id2("2");
  auto obj2 = planner::PerceptionObjectBuilder()
                  .set_id(id2)
                  .set_type(OT_VEHICLE)
                  .set_pos(pos2)
                  .set_velocity(5.0)
                  .set_timestamp(0.0)
                  .set_box_center(pos2)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(M_PI)
                  .Build();
  const Vec2d pos3(4.0, 10.0);
  std::string id3("3");
  auto obj3 = planner::PerceptionObjectBuilder()
                  .set_id(id3)
                  .set_type(OT_CYCLIST)
                  .set_pos(pos3)
                  .set_velocity(0.0)
                  .set_timestamp(0.0)
                  .set_box_center(pos3)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  const Vec2d pos4(8.0, 10.0);
  std::string id4("4");
  const double velocity4 = 5.0;
  auto obj4 = planner::PerceptionObjectBuilder()
                  .set_id(id4)
                  .set_type(OT_BARRIER)
                  .set_pos(pos4)
                  .set_velocity(velocity4)
                  .set_timestamp(0.0)
                  .set_box_center(pos4)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  *objects_proto.add_objects() = std::move(obj2);
  *objects_proto.add_objects() = std::move(obj3);
  *objects_proto.add_objects() = std::move(obj4);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  auto& hist1 = input.objects_history->at(id1);
  auto& hist2 = input.objects_history->at(id2);
  auto& hist3 = input.objects_history->at(id3);
  auto& hist4 = input.objects_history->at(id4);
  std::vector<ObjectHistory*> objs_to_predict;
  objs_to_predict.push_back(&hist1);
  objs_to_predict.push_back(&hist2);
  objs_to_predict.push_back(&hist3);
  objs_to_predict.push_back(&hist4);
  const double current_ts = hist1.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist1, &hist2, &hist3, &hist4}, hist1, current_ts, kTimeStep,
      kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id1] = scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_ON_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  scenarios[id2] = scenario;
  scenarios[id3] = scenario;
  scenarios[id4] = scenario;

  std::map<ObjectIDType, ObjectPredictionPriorityInfo> priorities;
  priorities[id1].priority = ObjectPredictionPriority::OPP_P1;
  priorities[id2].priority = ObjectPredictionPriority::OPP_P1;
  priorities[id3].priority = ObjectPredictionPriority::OPP_P1;
  priorities[id4].priority = ObjectPredictionPriority::OPP_P1;
  PredictionContext context(input);
  vantage_client_man::CreateVantageClientMan();
  int i = 0;
  for (const auto& dp : context.drive_passages()) {
    SendDrivePassageLineToCanvas(*dp, "dp_" + std::to_string(i));
    i++;
  }
  PredictObjectsByHeuristic(context, objs_to_predict, scenarios, obj_sampler,
                            /*thread_pool=*/nullptr, &object_trajs,
                            /*ignore_off_road=*/false);
  EXPECT_EQ(object_trajs[id1].size(), 1);
  EXPECT_EQ(object_trajs[id1][0].type(), PredictionType::PT_CYCV);
  EXPECT_EQ(object_trajs[id2].size(), 2);
  EXPECT_EQ(object_trajs[id2][0].type(),
            PredictionType::PT_VEHICLE_LANE_FOLLOW);
  EXPECT_EQ(object_trajs[id3].size(), 1);
  EXPECT_EQ(object_trajs[id3][0].type(), PredictionType::PT_L2_LANE_FOLLOW);
  EXPECT_EQ(object_trajs[id4].size(), 1);
  EXPECT_EQ(object_trajs[id4][0].type(), PredictionType::PT_CYCV);

  const auto trajs_1 =
      FilterOutStaticAndLowProbTrajectory(object_trajs[id1], 0.5);
  std::map<ObjectIDType, ObjectPredictionResult> results;
  AssemblePredictionResult(context, objs_to_predict, obj_sampler, scenarios,
                           priorities, &object_trajs, &results);
  EXPECT_EQ(results[id1].id, id1);
  EXPECT_EQ(results[id1].trajectories.size(), 1);
  EXPECT_EQ(results[id2].id, id2);
  EXPECT_EQ(results[id2].trajectories.size(), 2);
}

TEST(SchedulerUtilsTest, PredictUnderParkingScenario) {
  const auto& psmm = planner::CreateDojoTestPSMM();

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

  const Vec2d pos2(8.0, 10.0);
  std::string id2("2");
  const double velocity2 = 3.0;
  auto obj2 = planner::PerceptionObjectBuilder()
                  .set_id(id2)
                  .set_type(OT_VEHICLE)
                  .set_pos(pos2)
                  .set_velocity(velocity2)
                  .set_timestamp(0.0)
                  .set_box_center(pos2)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  const Vec2d v2 = Vec2d::FastUnitFromAngle(M_PI) * velocity2;
  v2.ToProto(obj2.mutable_vel());

  const Vec2d pos3(40.0, 10.0);
  std::string id3("3");
  auto obj3 = planner::PerceptionObjectBuilder()
                  .set_id(id3)
                  .set_type(OT_CYCLIST)
                  .set_pos(pos3)
                  .set_velocity(3.0)
                  .set_timestamp(0.0)
                  .set_box_center(pos3)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  *objects_proto.add_objects() = std::move(obj2);
  *objects_proto.add_objects() = std::move(obj3);

  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);

  auto& hist = input.objects_history->at(id);
  auto& hist2 = input.objects_history->at(id2);
  auto& hist3 = input.objects_history->at(id3);
  std::vector<ObjectHistory*> objs_to_predict = {&hist, &hist2, &hist3};
  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      objs_to_predict, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);

  std::map<ObjectIDType, std::vector<PredictedTrajectory>> object_trajs;
  PredictUnderParkingScenario(objs_to_predict, obj_sampler,
                              /*thread_pool=*/nullptr, &object_trajs);
  EXPECT_EQ(object_trajs.size(), objs_to_predict.size());
  EXPECT_EQ(object_trajs[id][0].type(), PT_STATIONARY);
  EXPECT_EQ(object_trajs[id2][0].type(), PT_REVERSE_CYCV);
  EXPECT_EQ(object_trajs[id3][0].type(), PT_CYCV);
  EXPECT_TRUE(object_trajs[id2][0].is_reversed());

  const int points_num = kParkingPredictionDuration / kTimeStep;
  EXPECT_EQ(object_trajs[id][0].points().size(), points_num);
  EXPECT_EQ(object_trajs[id2][0].points().size(), points_num);
  EXPECT_EQ(object_trajs[id3][0].points().size(), points_num);
}

}  // namespace
}  // namespace qcraft::prediction
