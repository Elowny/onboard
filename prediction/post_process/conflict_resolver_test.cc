#include "onboard/prediction/post_process/conflict_resolver.h"

#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/post_process/post_process_defs.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
namespace {

std::map<ObjectIDType, ObjectPredictionPostProcess>
ObjectPredMapToObjectPredPPMap(
    std::map<ObjectIDType, ObjectPredictionResult>* object_pred_map) {
  std::map<ObjectIDType, ObjectPredictionPostProcess> obj_preds_post_process;
  for (auto& [id, result] : *object_pred_map) {
    ObjectPredictionPostProcess obj_pred_pp;
    obj_pred_pp.object_proto = &result.perception_object;
    for (auto& traj : result.trajectories) {
      obj_pred_pp.ptrs_mutable_trajs.push_back(&traj);
    }
    obj_preds_post_process.emplace(id, std::move(obj_pred_pp));
  }
  QCHECK_EQ(object_pred_map->size(), obj_preds_post_process.size());
  return obj_preds_post_process;
}

std::map<ObjectIDType, ObjectPredictionResult> BuildPredResultsForResolver() {
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
      {obj_1.id(),
       ObjectPredictionResult{.id = obj_prediction_1.id(),
                              .priority = ObjectPredictionPriority::OPP_P1,
                              .perception_object = obj_1,
                              .trajectories = obj_prediction_1.trajectories()}},
      {obj_2.id(), ObjectPredictionResult{
                       .id = obj_prediction_2.id(),
                       .priority = ObjectPredictionPriority::OPP_P1,
                       .perception_object = obj_2,
                       .trajectories = obj_prediction_2.trajectories()}}};
  return obj_pred_result_map;
}

std::map<ObjectIDType, ObjectPredictionResult> BuildPredResultsForResolver2() {
  const auto obj_1 = planner::PerceptionObjectBuilder()
                         .set_id("1")
                         .set_type(OT_VEHICLE)
                         .set_pos(Vec2d(0.0, 0.0))
                         .set_velocity(5.0)
                         .set_timestamp(0.0)
                         .set_box_center(Vec2d(0.0, 0.0))
                         .set_length_width(/*length=*/5.0, /*width=*/1.95)
                         .set_yaw(0.0)
                         .Build();
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
                         .set_id("2")
                         .set_type(OT_VEHICLE)
                         .set_pos(Vec2d(10.0, 0.0))
                         .set_velocity(5.0)
                         .set_timestamp(0.0)
                         .set_box_center(Vec2d(10.0, 0.0))
                         .set_length_width(/*length=*/5.0, /*width=*/1.95)
                         .set_yaw(0.0)
                         .Build();
  planner::ObjectPredictionBuilder builder_2;
  builder_2.set_object(obj_2)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(10.0, 0.0), /*end=*/Vec2d(55.0, 0.0),
                         /*init_v=*/5.0, /*last_v=*/4.0);
  builder_2.add_predicted_trajectory()->set_probability(0.7).set_straight_line(
      /*start=*/Vec2d(10.0, 0.0), /*end=*/Vec2d(60.0, 0.0),
      /*init_v=*/5.0, /*last_v=*/5.0);
  const auto obj_prediction_2 = builder_2.Build();
  std::map<ObjectIDType, ObjectPredictionResult> obj_pred_result_map = {
      {obj_1.id(),
       ObjectPredictionResult{.id = obj_prediction_1.id(),
                              .priority = ObjectPredictionPriority::OPP_P1,
                              .perception_object = obj_1,
                              .trajectories = obj_prediction_1.trajectories()}},
      {obj_2.id(), ObjectPredictionResult{
                       .id = obj_prediction_2.id(),
                       .priority = ObjectPredictionPriority::OPP_P1,
                       .perception_object = obj_2,
                       .trajectories = obj_prediction_2.trajectories()}}};
  return obj_pred_result_map;
}

TEST(ConflictResolverTest, ResolveNone) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  std::string object_id("object_0");
  ConflictResolutionConfigProto config_proto;
  config_proto.set_resolution_level(CRL_NONE);
  ConflictResolverParams params(config_proto);
  ConflictResolverDebugProto debug_proto;
  std::map<ObjectIDType, ObjectPredictionResult> obj_pred_result_map = {
      {object_id, ObjectPredictionResult{.id = object_id}}};

  const auto obj_preds_pp =
      ObjectPredMapToObjectPredPPMap(&obj_pred_result_map);
  ResolveConflict(psmm, /*red_tls=*/{}, params, obj_preds_pp, &debug_proto,
                  /*thread_pool=*/nullptr);
}

TEST(ConflictResolverTest, ResolveStationary) {
  FLAGS_prediction_conflict_resolver_visual_on = true;
  const auto& psmm = planner::CreateDojoTestPSMM();
  ConflictResolverParams params;
  params.LoadParams();
  ConflictResolverDebugProto debug_proto;
  auto pred_result_map = BuildPredResultsForResolver();
  const auto obj_preds_pp = ObjectPredMapToObjectPredPPMap(&pred_result_map);
  ResolveConflict(psmm, /*red_tls=*/{}, params, obj_preds_pp, &debug_proto,
                  /*thread_pool=*/nullptr);

  for (const auto& [id, result] : pred_result_map) {
    LOG(INFO) << result.trajectories.front().annotation();
  }
  // Two trajectories.
  EXPECT_EQ(debug_proto.svt_graphs_size(), 2);
}

TEST(ConflictResolverTest, ResolveStationaryMultiple) {
  FLAGS_prediction_conflict_resolver_visual_on = true;
  const auto& psmm = planner::CreateDojoTestPSMM();
  ConflictResolverParams params;
  params.LoadParams();
  ConflictResolverDebugProto debug_proto;
  auto pred_result_map = BuildPredResultsForResolver2();
  const auto obj_preds_pp = ObjectPredMapToObjectPredPPMap(&pred_result_map);
  ResolveConflict(psmm, /*red_tls=*/{}, params, obj_preds_pp, &debug_proto,
                  /*thread_pool=*/nullptr);
  EXPECT_EQ(debug_proto.svt_graphs_size(), 0);  // Both dynamic, do not resolve.
}

TEST(ConflictResolverTest, ResolveRedLight) {
  // Build pred result of object avoid red light scene.
  FLAGS_prediction_conflict_resolver_visual_on = true;
  const Vec2d start(50.354, 0.278);
  const Vec2d end(92.034, 0.067);
  const double init_v = 10;
  const double last_v = 10;
  const auto obj_1 = planner::PerceptionObjectBuilder()
                         .set_id("object_1")
                         .set_type(ObjectType::OT_VEHICLE)
                         .set_pos(start)
                         .set_timestamp(0.0)
                         .set_box_center(start)
                         .set_length_width(5.0, 1.7)
                         .set_yaw(0.01)
                         .Build();
  planner::ObjectPredictionBuilder pred_builder;
  pred_builder.set_object(obj_1)
      .add_predicted_trajectory()
      ->set_probability(1.0)
      .set_index(0)
      .set_type(PredictionType::PT_CYCV)
      .set_straight_line(start, end, init_v, last_v);
  const auto obj_pred = pred_builder.Build();

  std::map<ObjectIDType, ObjectPredictionResult> obj_pred_result_map = {
      {obj_1.id(),
       ObjectPredictionResult{.id = obj_pred.id(),
                              .priority = ObjectPredictionPriority::OPP_P1,
                              .perception_object = obj_1,
                              .trajectories = obj_pred.trajectories()}}};

  // Create traffic red light scene. light 889 controlling lane 34.
  const auto& psmm = planner::CreateDojoTestPSMM();
  ConflictResolverParams params;
  params.LoadParams();
  ConflictResolverDebugProto debug_proto;

  const auto obj_preds_post_process =
      ObjectPredMapToObjectPredPPMap(&obj_pred_result_map);
  ResolveConflict(psmm, {mapping::ElementId(889)}, params,
                  obj_preds_post_process, &debug_proto,
                  /*thread_pool=*/nullptr);
  // Should resolve for object on lane 1 to lane 34 ignore red
  // light 889.
  EXPECT_EQ(debug_proto.svt_graphs_size(), 1);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
