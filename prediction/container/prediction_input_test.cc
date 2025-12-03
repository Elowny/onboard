#include "onboard/prediction/container/prediction_input.h"

#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/global/clock.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(PredictionInputTest, CheckPredictionInput) {
  PredictionInput prediction_input(/*history_size=*/100, /*history_len=*/1,
                                   /*min_dt=*/0.01,
                                   /*long_term_history_size=*/100,
                                   /*long_term_history_len=*/30,
                                   /*long_term_min_dt=*/0.5);
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
  auto objects_proto = std::make_shared<ObjectsProto>();
  *objects_proto->add_objects() = std::move(obj);
  VehicleGeometryParamsProto params;
  params.set_width(1.0);
  auto av_pose = std::make_shared<PoseProto>();
  Vec3dProto av_pos;
  Vec3dToProto(Vec3d(0.0, 0.0, 0.0), &av_pos);
  *av_pose->mutable_pos_smooth() = av_pos;
  auto loc_transform = std::make_shared<LocalizationTransformProto>();
  absl::Time time = Clock::Now();
  prediction_input.prediction_init_time = time;
  prediction_input.pose = av_pose;
  prediction_input.traffic_light_states =
      std::make_shared<TrafficLightStatesProto>(TrafficLightStatesProto());
  prediction_input.localization_transform = std::move(loc_transform);

  prediction_input.semantic_map_manager = &psmm;
  prediction_input.veh_geom_params = &params;

  prediction_input.av_context->Update(*av_pose,
                                      *prediction_input.localization_transform,
                                      *prediction_input.veh_geom_params);
  prediction_input.objects_history->Update(
      *objects_proto, *prediction_input.localization_transform,
      /*thread_pool=*/nullptr);
  prediction_input.long_term_objects_history->Update(
      *objects_proto, *prediction_input.localization_transform,
      /*thread_pool=*/nullptr);
  EXPECT_EQ(prediction_input.pose->pos_smooth().x(), 0.0);
  EXPECT_EQ(prediction_input.av_context->GetAvObjectHistory().size(), 1);
  EXPECT_NE(prediction_input.semantic_map_manager, nullptr);
  EXPECT_EQ(prediction_input.prediction_init_time, time);
  EXPECT_EQ(prediction_input.objects_history->Contains("1"), true);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
