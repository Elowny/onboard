#include "onboard/prediction/feature_extractor/act_net_feature_extractor.h"

#include <string>
#include <vector>

#include "absl/strings/str_join.h"

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft::prediction {
namespace {
constexpr double kEps = 1e-5;
constexpr double kEgoInitPosX = 10.0;
constexpr double kEgoVel = 2.0;
constexpr double kObject1InitPosY = 10.0;
constexpr double kObject1Vel = 1.0;
constexpr char kObject1Id[] = "999";
constexpr char kEgoId[] = "9999";

constexpr int kNumHistory = 10;
constexpr double kUpdateTimeStep = 0.1;
template <typename T>
std::string VecDebugString(const std::vector<T>& vec) {
  return absl::StrJoin(vec, ",");
}

ObjectHistorySampler MakeObjectHistorySampler(const int history_num) {
  auto agent_history = BuildVehicleHistoryByConstVel(
      kEgoId, history_num, Vec2d(kEgoInitPosX, 0.0), kEgoVel);
  auto obj_history = BuildVehicleHistoryByConstVel(
      kObject1Id, history_num, Vec2d(0.0, kObject1InitPosY), kObject1Vel);
  auto av_history = BuildVehicleHistoryByConstVel(kAvObjectId, history_num,
                                                  Vec2d::Zero(), kEgoVel);
  return ObjectHistorySampler({&agent_history, &obj_history}, av_history,
                              kUpdateTimeStep * (history_num - 1),
                              kUpdateTimeStep, history_num,
                              /*enable_smoothing=*/false,
                              /*use_tracker_history=*/false);
}

ActNetFeature GetActNetFeature() {
  const auto obj_sampler = MakeObjectHistorySampler(kNumHistory);
  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightManager::TLStateHashMap tl_state;
  MapSampler map_sampler(psmm, tl_state, kFeatureV2MaxMapSampleLen,
                         kFeatureV2MapSegmentNum,
                         MapSampler::SampleType::ADAPTIVE);
  const StopTimeInfo info;
  AvMapCache av_map_cache;
  const auto* av_history = obj_sampler.GetResampledAVMotionHistory();
  const auto& av_pos = av_history->states.back().pos;
  av_map_cache.lane_centers =
      map_sampler.GetLaneCentersWithRadius(av_pos, kActNetConfig.av_map_radius);
  av_map_cache.solid_lane_boundarys =
      map_sampler.GetSolidLaneBoundariesWithRadius(av_pos,
                                                   kActNetConfig.av_map_radius);

  // Get region box
  const auto agent_history = obj_sampler.GetResampledMotionHistoryById(kEgoId);
  const double heading = agent_history.states.back().heading;
  const auto ref_position = agent_history.states.back().pos;
  const auto region_box =
      GetDynamicRegionBox(ref_position, agent_history, heading);

  return ExtractActNetFeature(kEgoId, obj_sampler, info,
                              /*drive_passage_ptr=*/nullptr, &map_sampler,
                              av_map_cache, region_box);
}

TEST(ActNetFeatureExtractorTest, TypeConvertCheck) {
  EXPECT_EQ(ObjectTypeToActNetType(OT_UNKNOWN_STATIC),
            ActNetType::AT_UNKNOWN_STATIC);
  EXPECT_EQ(ObjectTypeToActNetType(OT_VEHICLE), ActNetType::AT_VEHICLE);
  EXPECT_EQ(ObjectTypeToActNetType(OT_MOTORCYCLIST), ActNetType::AT_CYCLIST);
  EXPECT_EQ(ObjectTypeToActNetType(OT_CYCLIST), ActNetType::AT_CYCLIST);
  EXPECT_EQ(ObjectTypeToActNetType(OT_TRICYCLIST), ActNetType::AT_CYCLIST);
  EXPECT_EQ(ObjectTypeToActNetType(OT_PEDESTRIAN), ActNetType::AT_PEDESTRIAN);
  EXPECT_EQ(ObjectTypeToActNetType(OT_FOD), ActNetType::AT_FOD);
  EXPECT_EQ(ObjectTypeToActNetType(OT_UNKNOWN_MOVABLE),
            ActNetType::AT_UNKNOWN_MOVABLE);
  EXPECT_EQ(ObjectTypeToActNetType(OT_VEGETATION), ActNetType::AT_VEGETATION);
  EXPECT_EQ(ObjectTypeToActNetType(OT_BARRIER), ActNetType::AT_BARRIER);
  EXPECT_EQ(ObjectTypeToActNetType(OT_CONE), ActNetType::AT_CONE);
  EXPECT_EQ(ObjectTypeToActNetType(OT_WARNING_TRIANGLE),
            ActNetType::AT_WARNING_TRIANGLE);
}

TEST(ActNetFeatureExtractorTest, ExtractActNetFeatureTest) {
  const auto feature = GetActNetFeature();
  const auto& agent_pos = feature.agent_feature.pos;
  const auto& agent_yaw = feature.agent_feature.yaw;
  const auto& agent_pos_diff = feature.agent_feature.pos_diff;
  // last point always (0,0)
  EXPECT_NEAR(agent_pos.back(), 0.0, kEps);
  EXPECT_NEAR(agent_pos[agent_pos.size() - 2], 0.0, kEps);
  // We have kNumHistory+1 valid points.
  EXPECT_NEAR(
      feature.agent_feature.mask[kFeatureV2HistoryStepNum - kNumHistory], 1.0,
      kEps);
  for (int i = 0; i < kFeatureV2HistoryStepNum; ++i) {
    if (static_cast<bool>(feature.agent_feature.mask[i])) {
      const auto& dx = agent_pos_diff[i * kActNetConfig.coord_num];
      const auto& dy = agent_pos_diff[i * kActNetConfig.coord_num + 1];
      EXPECT_NEAR(dx, 0.2, kEps);
      EXPECT_NEAR(dy, 0.0, kEps);
    }
  }
  EXPECT_EQ(feature.context_obj_features.size(), 2);
  const auto& av = feature.context_obj_features[0];
  const auto& context = feature.context_obj_features[1];

  for (int i = 0; i < kFeatureV2HistoryStepNum; ++i) {
    if (static_cast<bool>(av.rel_feat.rel_mask[i])) {
      const auto& x = agent_pos[i * kActNetConfig.coord_num];
      const auto& y = agent_pos[i * kActNetConfig.coord_num + 1];
      const auto& av_x = av.abs_feat.pos[i * kActNetConfig.coord_num];
      const auto& av_y = av.abs_feat.pos[i * kActNetConfig.coord_num + 1];
      EXPECT_NEAR(av_x - x, av.rel_feat.rel_pos[i * kActNetConfig.coord_num],
                  kEps);
      EXPECT_NEAR(av_y - y,
                  av.rel_feat.rel_pos[i * kActNetConfig.coord_num + 1], kEps);
      EXPECT_NEAR(Vec2d(av_x - x, av_y - y).norm(), av.rel_feat.rel_dist[i],
                  kEps);
    }
    if (static_cast<bool>(context.rel_feat.rel_mask[i])) {
      const auto& x = agent_pos[i * kActNetConfig.coord_num];
      const auto& y = agent_pos[i * kActNetConfig.coord_num + 1];
      const auto& obj_x = context.abs_feat.pos[i * kActNetConfig.coord_num];
      const auto& obj_y = context.abs_feat.pos[i * kActNetConfig.coord_num + 1];
      EXPECT_NEAR(obj_x - x,
                  context.rel_feat.rel_pos[i * kActNetConfig.coord_num], kEps);
      EXPECT_NEAR(obj_y - y,
                  context.rel_feat.rel_pos[i * kActNetConfig.coord_num + 1],
                  kEps);
      EXPECT_NEAR(Vec2d(obj_x - x, obj_y - y).norm(),
                  context.rel_feat.rel_dist[i], kEps);
      const auto unit = Vec2d(obj_x - x, obj_y - y).Unit();
      EXPECT_NEAR(unit.x(),
                  context.rel_feat.rel_yaw[i * kActNetConfig.coord_num], kEps);
      EXPECT_NEAR(unit.y(),
                  context.rel_feat.rel_yaw[i * kActNetConfig.coord_num + 1],
                  kEps);

      const auto& yx = agent_yaw[i * kActNetConfig.coord_num];
      const auto& yy = agent_yaw[i * kActNetConfig.coord_num + 1];
      const auto& obj_yx = context.abs_feat.yaw[i * kActNetConfig.coord_num];
      const auto& obj_yy =
          context.abs_feat.yaw[i * kActNetConfig.coord_num + 1];
      const auto yaw_diff = Vec2d::UnitFromAngle(
          (Vec2d(obj_yx, obj_yy).Angle() - Vec2d(yx, yy).Angle()));
      EXPECT_NEAR(yaw_diff.x(),
                  context.rel_feat.yaw_diff[i * kActNetConfig.coord_num], kEps);
      EXPECT_NEAR(yaw_diff.y(),
                  context.rel_feat.yaw_diff[i * kActNetConfig.coord_num + 1],
                  kEps);

      const auto& vx = feature.agent_feature.speed[i * kActNetConfig.coord_num];
      const auto& vy =
          feature.agent_feature.speed[i * kActNetConfig.coord_num + 1];
      const auto& obj_vx = context.abs_feat.speed[i * kActNetConfig.coord_num];
      const auto& obj_vy =
          context.abs_feat.speed[i * kActNetConfig.coord_num + 1];
      context.abs_feat.yaw[i * kActNetConfig.coord_num + 1];
      const auto speed_diff = Vec2d(obj_vx, obj_vy) - Vec2d(vx, vy);
      EXPECT_NEAR(speed_diff.x(),
                  context.rel_feat.rel_speed[i * kActNetConfig.coord_num],
                  kEps);
      EXPECT_NEAR(speed_diff.y(),
                  context.rel_feat.rel_speed[i * kActNetConfig.coord_num + 1],
                  kEps);

      constexpr int kShapeNum = 4;
      for (int k = 0; k < kShapeNum; ++k) {
        const auto& pt_x =
            context.abs_feat.shape[i * kActNetConfig.coord_num * kShapeNum +
                                   k * kActNetConfig.coord_num];
        const auto& pt_y =
            context.abs_feat.shape[i * kActNetConfig.coord_num * kShapeNum +
                                   k * kActNetConfig.coord_num + 1];
        EXPECT_NEAR(
            pt_x - x,
            context.rel_feat.rel_shape[i * kActNetConfig.coord_num * kShapeNum +
                                       k * kActNetConfig.coord_num],
            kEps);
        EXPECT_NEAR(
            pt_y - y,
            context.rel_feat.rel_shape[i * kActNetConfig.coord_num * kShapeNum +
                                       k * kActNetConfig.coord_num + 1],
            kEps);
      }
    }
  }
  EXPECT_NEAR(context.abs_feat.type, 2.0, kEps);
}

TEST(ActNetFeatureExtractorTest, ToDumpedProto) {
  const double ts = (kNumHistory - 1) * kUpdateTimeStep;
  const Vec2d ref_pos(kEgoInitPosX + kEgoVel * ts, 0.0);
  const double rot_rad = 0.0;
  const Vec2d av_pos(kEgoVel * ts, 0.0);
  const double av_angle = 0.0;
  const auto dumped_proto = ToActNetDumpedFeatureProto(
      GetActNetFeature(), ts, ref_pos, rot_rad, av_pos, av_angle);
  EXPECT_EQ(dumped_proto.timestamp(), ts);
  EXPECT_GT(dumped_proto.agent_feature().pos().size(), kNumHistory);
  EXPECT_GT(dumped_proto.objects_feature().pos().size(), 2 * kNumHistory);
}
}  // namespace
}  // namespace qcraft::prediction
