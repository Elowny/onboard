#include "onboard/prediction/prediction_util.h"

#include <math.h>

#include <string>
#include <utility>

#include "absl/strings/str_split.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/utils/proto_util.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace prediction {
namespace {

using planner::PerceptionObjectBuilder;

TEST(FindTrajCurbCollisionIndex, FindLaneBoundaryByIdTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const auto level_id = psmm.GetLevel();
  std::string boundary_info;
  Segment2d nda_segment;
  const Vec2d check_pos(14.0, 4.0);
  ASSERT_TRUE(psmm.GetNearestNamedImpassableBoundaryAtLevel(
      level_id, check_pos, &nda_segment, &boundary_info));

  std::vector<std::string> boundaries_info = absl::StrSplit(boundary_info, '|');
  ASSERT_GE(boundaries_info.size(), 2);
  EXPECT_EQ(boundaries_info[0], "CURB");
  EXPECT_EQ(boundaries_info[1], "382");

  const auto* lane_boundary = psmm.FindLaneBoundaryByIdOrNull(
      mapping::ElementId(std::stoi(boundaries_info[1])));
  EXPECT_NE(lane_boundary, nullptr);
  const auto* lane_boundary_proto = lane_boundary->proto;
  EXPECT_EQ(lane_boundary_proto->type(),
            mapping::LaneBoundaryProto_Type::LaneBoundaryProto_Type_CURB);
}

TEST(PredictionUtilTest, LerpObjectProto) {
  const auto obj_a = PerceptionObjectBuilder()
                         .set_id("obj")
                         .set_pos(Vec2d(1.0, 2.0))
                         .set_timestamp(10.0)
                         .set_length_width(4.0, 2.0)
                         .set_yaw(0.0)
                         .set_yaw_rate(0.1)
                         .set_speed(Vec2d(1.0, 2.0))
                         .set_accel(Vec2d(1.0, -1.0))
                         .Build();
  const auto obj_b = PerceptionObjectBuilder()
                         .set_id("obj")
                         .set_pos(Vec2d(2.0, 4.0))
                         .set_timestamp(20.0)
                         .set_length_width(4.0, 2.0)
                         .set_yaw(0.0)
                         .set_yaw_rate(0.1)
                         .set_speed(Vec2d(1.0, 2.0))
                         .set_accel(Vec2d(1.0, -1.0))
                         .Build();
  const auto lerped_obj = LerpObjectProto(obj_a, obj_b, 0.5);
  EXPECT_THAT(lerped_obj, ProtoPartialApproximatelyMatchesText(R"(
                                      pos { x: 1.5 y: 3.0 }
                                      vel { x: 1.0 y: 2.0 }
                                      accel { x: 1.0 y: -1.0 }
                                      timestamp: 15.0
                                      id: "obj"
                                      yaw: 0.0
                                      yaw_rate: 0.1
                                      parked: false
                                      offroad: false)",
                                                               1e-6));
}

TEST(PredictionUtilTest, AllLanesFoundInSemanticMap) {
  mapping::LanePathProto lane_path_proto;
  const auto& psmm = planner::CreateDojoTestPSMM();

  EXPECT_FALSE(AllLanesFoundInSemanticMap(lane_path_proto, psmm));

  TextToProto(R"(
      start_fraction: 0.0
      end_fraction: 1.0
      lane_path_in_forward_direction: true     
      lane_ids: 2
      lane_ids: 939
      lane_ids: 940
        )",
              &lane_path_proto);
  EXPECT_TRUE(AllLanesFoundInSemanticMap(lane_path_proto, psmm));
}

TEST(PredictionUtilTest, MaybeVRU) {
  EXPECT_FALSE(MaybeVRU(OT_VEHICLE));
  EXPECT_TRUE(MaybeVRU(OT_PEDESTRIAN));
  EXPECT_TRUE(MaybeVRU(OT_UNKNOWN_MOVABLE));
}

TEST(PredictionUtilTest, GuessType) {
  const std::string id = "object_0";
  const int history_num = 10;
  const double vel = 5.0;
  const auto object_history =
      BuildVehicleHistoryByConstVel(id, history_num, Vec2d::Zero(), vel);
  EXPECT_EQ(GuessType(object_history), OT_VEHICLE);
}

TEST(PredictionUtilTest, IsStationary) {
  PredictedTrajectoryProto traj_proto;
  traj_proto.set_type(PredictionType::PT_STATIONARY);
  auto* pt = traj_proto.add_points();
  pt->mutable_pos()->set_x(0.0);
  pt->mutable_pos()->set_y(0.0);
  pt->set_s(0.0);
  pt->set_t(0.0);
  EXPECT_TRUE(IsStationaryTrajectory(traj_proto));
  PredictedTrajectory traj(traj_proto);
  EXPECT_TRUE(IsStationaryTrajectory(traj));
  ObjectPredictionProto pred_proto;
  *pred_proto.add_trajectories() = traj_proto;
  EXPECT_TRUE(IsStationaryPrediction(pred_proto));

  *pred_proto.mutable_perception_object() = PerceptionObjectBuilder().Build();
  ObjectPrediction pred(pred_proto);
  EXPECT_TRUE(IsStationaryPrediction(pred));
}

TEST(PredictionUtilTest, ResampleObjectProtos) {
  std::vector<ObjectProto> objects;
  objects.reserve(2);
  EXPECT_TRUE(
      ResampleObjectProtos(objects, 0.0, kPredictionTimeStep, 20).empty());
  auto object_1 = PerceptionObjectBuilder().Build();
  objects.push_back(std::move(object_1));
  const auto resample_one =
      ResampleObjectProtos(objects, 2.0, kPredictionTimeStep, 10);
  EXPECT_EQ(resample_one.size(), 1);
  EXPECT_EQ(resample_one[0].timestamp(), 2.0);

  auto object_2 = PerceptionObjectBuilder().set_timestamp(2.0).Build();
  objects.push_back(object_2);
  const auto resample_two =
      ResampleObjectProtos(objects, 1.0, kPredictionTimeStep, 10);
  EXPECT_EQ(resample_two.size(), 10);
}
TEST(PredictionUtilTest, InstantPredictionForNewObject) {
  ObjectProto obj_a;
  const auto status_a = InstantPredictionForNewObject(obj_a, 3.0);
  EXPECT_FALSE(status_a.ok());
  const auto obj_b =
      PerceptionObjectBuilder().set_type(OT_UNKNOWN_STATIC).Build();
  const auto status_b = InstantPredictionForNewObject(obj_b, 3.0);
  EXPECT_TRUE(status_b.ok());
  EXPECT_EQ(status_b.value().trajectories()[0].type(),
            PredictionType::PT_STATIONARY);
  const auto obj_c = PerceptionObjectBuilder().Build();
  const auto status_c = InstantPredictionForNewObject(obj_c, 3.0);
  EXPECT_TRUE(status_c.ok());
  EXPECT_EQ(status_c.value().trajectories()[0].points_size(), 30);
}

TEST(PredictionUtilTest, InstantObjectPredictionForNewObject) {
  ObjectProto obj_a;
  const auto status_a =
      InstantObjectPredictionForNewObject(obj_a, /*prediction_time=*/3.0);
  EXPECT_FALSE(status_a.ok());
  const auto obj_b =
      PerceptionObjectBuilder().set_type(OT_UNKNOWN_STATIC).Build();
  const auto status_b =
      InstantObjectPredictionForNewObject(obj_b, /*prediction_time=*/3.0);
  EXPECT_TRUE(status_b.ok());
  EXPECT_EQ(status_b.value().trajectories()[0].type(),
            PredictionType::PT_STATIONARY);
  const auto obj_c = PerceptionObjectBuilder().Build();
  const auto status_c =
      InstantObjectPredictionForNewObject(obj_c, /*prediction_time=*/3.0);
  EXPECT_TRUE(status_c.ok());
  EXPECT_EQ(status_c.value().trajectories()[0].points().size(), 30);
}

TEST(PredictionUtilTest, MustReceiveHDMapForPrediction) {
  AutonomyStateProto autonomy_state;
  EXPECT_TRUE(MustReceiveHDMapForPrediction(autonomy_state));
  auto* assist_drive_state = autonomy_state.mutable_assist_state();
  assist_drive_state->set_assist_drive_system_state(
      AssistStateProto::ASSIST_NOA_ACTIVE);
  EXPECT_TRUE(MustReceiveHDMapForPrediction(autonomy_state));
  assist_drive_state->set_assist_drive_system_state(
      AssistStateProto::ASSIST_LCC_READY);
  EXPECT_FALSE(MustReceiveHDMapForPrediction(autonomy_state));
}

TEST(PredictionUtilTest, RegionBox) {
  Vec2d center(0.0, 0.0);
  const auto box = GetRegionBox(center, /*heading=*/0.0,
                                /*detection_region_front=*/2.0,
                                /*detection_region_behind=*/1.0,
                                /*detection_half_width=*/1.0);
  EXPECT_NEAR(box.radius(), sqrt(1.5 * 1.5 + 1.0), 1e-3);
  const auto front_center_pt = box.FrontCenterPoint();
  EXPECT_NEAR(front_center_pt.x(), 2.0, 1e-3);
  EXPECT_NEAR(front_center_pt.y(), 0.0, 1e-3);
}

TEST(PredictionUtilTest, BuildModelScaleParamMap) {
  ModelScaleParam model_scale;
  TextToProto(R"(
        model_name: "model_name"
        feature_scales {
          feature_name: "feature1"
          scale: 10.0
        }
        feature_scales {
          feature_name: "feature2"
          scale: 20.0
          scale: 40.0
          zero: 5.0
          zero: 2.0
        }
        feature_scales {
          feature_name: "feature3"
          scale: 10.0
          zero: 5.0
          clamp: true
          min: -1.0
          max: 1.0
        })",
              &model_scale);
  auto scale_map = BuildModelScaleParamMap(model_scale);
  EXPECT_EQ(scale_map.size(), 3);
  const auto& feature1 = scale_map["feature1"];
  const auto& feature2 = scale_map["feature2"];
  EXPECT_NEAR(feature1.inv_scale[0], 0.1, 1e-6);
  EXPECT_NEAR(feature2.inv_scale[0], 1.0 / 20.0, 1e-6);
  EXPECT_NEAR(feature2.inv_scale[1], 1.0 / 40.0, 1e-6);
  EXPECT_NEAR(feature2.zero[0], 5.0, 1e-6);
  EXPECT_NEAR(feature2.zero[1], 2.0, 1e-6);
  EXPECT_FALSE(feature1.clamp);
  EXPECT_FALSE(feature2.clamp);

  const auto& feature3 = scale_map["feature3"];
  const auto scaled_value = feature3.ScaleOnly(30.0f, 0);
  EXPECT_EQ(scaled_value, 3.0f);
  const auto scaled_clamp_value = feature3.ScaleWithClamp(30.0f, 0);
  EXPECT_EQ(scaled_clamp_value, 1.0f);
  const auto scaled_clamp_zero_value = feature3.ScaleWithZeroAndClamp(13.0f, 0);
  EXPECT_EQ(scaled_clamp_zero_value, static_cast<float>(0.8));
  const auto scale_zero_value = feature3.ScaleWithZero(13.0f, 0);
  EXPECT_EQ(scale_zero_value, 0.8f);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
