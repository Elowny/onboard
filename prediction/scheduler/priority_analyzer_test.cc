#include "onboard/prediction/scheduler/priority_analyzer.h"

#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::prediction {
namespace {
TEST(PriorityAnalyzerTest, IgnoreTest1) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(13.0, -31.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(0.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  cparams.LoadParams();
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  input.av_context->Update(*input.pose, *input.localization_transform,
                           *input.veh_geom_params);
  const auto& hist = input.objects_history->at(id);
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  PredictionContext context(input);
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id).priority, ObjectPredictionPriority::OPP_P3);
}

TEST(PriorityAnalyzerTest, IgnoreTest2) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const Vec2d pos(13.0, -31.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  cparams.LoadParams();
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  input.av_context->Update(*input.pose, *input.localization_transform,
                           *input.veh_geom_params);
  const auto& hist = input.objects_history->at(id);
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  PredictionContext context(input);
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id).priority, ObjectPredictionPriority::OPP_P3);
}

TEST(PriorityAnalyzerTest, ScanBoxTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const Vec2d pos(13.0, -6.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  cparams.LoadParams();
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  input.av_context->Update(*input.pose, *input.localization_transform,
                           *input.veh_geom_params);
  const auto& hist = input.objects_history->at(id);
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  PredictionContext context(input);
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id).priority, ObjectPredictionPriority::OPP_P1);
}

TEST(PriorityAnalyzerTest, OnRoadTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const Vec2d pos(13.0, -31.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  cparams.LoadParams();
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  input.av_context->Update(*input.pose, *input.localization_transform,
                           *input.veh_geom_params);
  const auto& hist = input.objects_history->at(id);
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_ON_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  PredictionContext context(input);
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id).priority, ObjectPredictionPriority::OPP_P3);
}

TEST(PriorityAnalyzerTest, InIntersectionTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(13.0, -31.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  cparams.LoadParams();
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  input.av_context->Update(*input.pose, *input.localization_transform,
                           *input.veh_geom_params);
  const auto& hist = input.objects_history->at(id);

  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_IN_INTERSECTION);

  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  PredictionContext context(input);
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id).priority, ObjectPredictionPriority::OPP_P1);
}

TEST(PriorityAnalyzerTest, PedTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(100.0, 0.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_CYCLIST)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  cparams.LoadParams();
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  input.av_context->Update(*input.pose, *input.localization_transform,
                           *input.veh_geom_params);
  const auto& hist = input.objects_history->at(id);

  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_abs_dist_to_nearest_lane(1.0);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  PredictionContext context(input);
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id).priority, ObjectPredictionPriority::OPP_P1);
}

TEST(PriorityAnalyzerTest, NoPedTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(100.0, 0.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  cparams.LoadParams();
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  input.av_context->Update(*input.pose, *input.localization_transform,
                           *input.veh_geom_params);
  const auto& hist = input.objects_history->at(id);

  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_abs_dist_to_nearest_lane(1.0);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  PredictionContext context(input);
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id).priority, ObjectPredictionPriority::OPP_P1);
}
}  // namespace
}  // namespace qcraft::prediction
