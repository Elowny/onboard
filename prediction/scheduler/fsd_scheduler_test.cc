#include "onboard/prediction/scheduler/fsd_scheduler.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_finder.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/model_pool.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::prediction {
namespace {
const double kTimeStep = 0.1;
const int kHistoryNum = 10;
TEST(SchedulerTest, SchedulePredictionTest) {
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
  input.av_context->Update(*input.pose, *input.localization_transform,
                           *input.veh_geom_params);

  std::map<ObjectIDType, std::vector<PredictedTrajectory>> object_trajs;

  const Vec2d pos2(-24.2, 49.8);
  std::string id2("2");
  auto obj2 = planner::PerceptionObjectBuilder()
                  .set_id(id2)
                  .set_type(OT_VEHICLE)
                  .set_pos(pos2)
                  .set_velocity(5.0)
                  .set_timestamp(0.0)
                  .set_box_center(pos2)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(-0.5 * M_PI)
                  .Build();
  *objects_proto.add_objects() = std::move(obj2);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  PredictionContext context(input);
  auto& hist1 = context.object_history_manager().at(id1);
  auto& hist2 = context.object_history_manager().at(id2);
  std::vector<ObjectHistory*> objs_to_predict;
  objs_to_predict.push_back(&hist1);
  objs_to_predict.push_back(&hist2);
  const double current_ts = hist1.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist1, &hist2}, hist1, current_ts, kTimeStep, kHistoryNum,
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

  auto param_manager = CreateParamManagerFromCarId("Q8001");
  QCHECK(param_manager != nullptr);
  auto param_finder = CreateParamFinderWithCarId("Q8001");
  QCHECK(param_finder != nullptr);
  ModelPool model_pool(*param_manager, *param_finder, {});
  ConflictResolverParams prediction_conflict_resolver_params;
  prediction_conflict_resolver_params.LoadParams();
  PredictionDebugProto debug_proto;
  auto object_results =
      ScheduleFsdPrediction(context, model_pool, obj_sampler, objs_to_predict,
                            /*thread_pool=*/nullptr, &debug_proto);

  EXPECT_EQ(object_results[id1].id, id1);
  EXPECT_GE(object_results[id1].trajectories.size(), 1);
  EXPECT_EQ(object_results[id2].id, id2);
  EXPECT_GE(object_results[id2].trajectories.size(), 1);
}

}  // namespace
}  // namespace qcraft::prediction
