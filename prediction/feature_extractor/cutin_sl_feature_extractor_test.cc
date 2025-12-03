#include "onboard/prediction/feature_extractor/cutin_sl_feature_extractor.h"

#include <vector>

#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/math/frenet_common.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/cutin_sl_feature.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::prediction {
namespace {
constexpr double kEps = 1e-5;
constexpr double kEgoInitPosX = 10.0;
constexpr double kEgoVel = 2.0;
constexpr double kObject1InitPosY = 10.0;
constexpr double kObject1Vel = 1.0;
constexpr double kMaxHeadingDiff = M_PI / 6.0;
constexpr double kForwardLane = 50.0;          // m.
constexpr double kForwardExtendLength = 10.0;  // m.
constexpr double kDrivePassageStepS = 4.0;
constexpr double kDrivePassageBackwardLength = 40.0;  // m.
constexpr char kObject1Id[] = "999";
constexpr char kEgoId[] = "9999";
constexpr double kSLSpeedMax = 50;  // m/s
constexpr double kSLCorMax = 500;   // m
constexpr double kSstep = 0.5;      // m

constexpr int kNumHistory = 10;
constexpr double kUpdateTimeStep = 0.1;
constexpr double kAvMapRadius = 210.0;

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

std::optional<CutinSLFeature> GetCutinSLNetFeature() {
  CutinSLFeature feature;
  const auto obj_sampler = MakeObjectHistorySampler(kNumHistory);
  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightManager::TLStateHashMap tl_state;
  MapSampler map_sampler(psmm, tl_state, kFeatureV2MaxMapSampleLen,
                         kFeatureV2MapSegmentNum,
                         MapSampler::SampleType::ADAPTIVE);
  const auto* av_motion_history = obj_sampler.GetResampledAVMotionHistory();
  const auto& av_pos = av_motion_history->states.back().pos;
  const auto lane_centers =
      map_sampler.GetLaneCentersWithRadius(av_pos, kAvMapRadius);
  const auto lane_boundaries =
      map_sampler.GetSolidLaneBoundariesWithRadius(av_pos, kAvMapRadius);
  const auto crosswalks =
      map_sampler.GetCrosswalksWithRadius(av_pos, kAvMapRadius);
  const auto av_resampled_histories = obj_sampler.GetResampledAVMotionHistory();
  const auto av_last_state = av_resampled_histories->states.back();

  const auto lane_id_or =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          psmm, av_last_state.pos, av_last_state.heading,
          /*boundary_distance_limit=*/0.0, kMaxHeadingDiff);
  if (!lane_id_or.has_value()) {
    return feature;
  }
  const auto lps =
      SearchLanePath(av_last_state.pos, psmm, *lane_id_or, kForwardLane,
                     /*is_reverse_driving=*/false);
  auto extended_lp =
      ExtendMostStraightLanePath(lps[0], psmm, kForwardExtendLength,
                                 /*is_reversed=*/false);
  const auto drive_passage = planner::BuildDrivePassageForPrediction(
      psmm, extended_lp, kDrivePassageStepS,
      /*avoid_loop=*/true,
      /*backward_extend_len=*/kDrivePassageBackwardLength,
      kMaxLateralBoundaryForPredictionDp);
  return ExtractCutinSLFeature(kEgoId, obj_sampler, *drive_passage,
                               lane_centers, lane_boundaries, crosswalks,
                               &map_sampler);
}

TEST(CutinSLNetFeatureExtractorTest, TypeConvertCheck) {
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_UNKNOWN_STATIC),
            CutinSLNetType::kAtUnknownStatic);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_VEHICLE), CutinSLNetType::kAtVehicle);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_MOTORCYCLIST),
            CutinSLNetType::kAtCyclist);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_CYCLIST), CutinSLNetType::kAtCyclist);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_TRICYCLIST),
            CutinSLNetType::kAtCyclist);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_PEDESTRIAN),
            CutinSLNetType::kAtPedestrian);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_FOD), CutinSLNetType::kAtFod);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_UNKNOWN_MOVABLE),
            CutinSLNetType::kAtUnknownMovable);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_VEGETATION),
            CutinSLNetType::kAtVegetation);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_BARRIER), CutinSLNetType::kAtBarrier);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_CONE), CutinSLNetType::kAtCone);
  EXPECT_EQ(ObjectTypeToCutinSLNetType(OT_WARNING_TRIANGLE),
            CutinSLNetType::kAtWarningTriangle);
}

TEST(CutinSLNetFeatureExtractorTest, ExtractCutinSLNetFeatureTest) {
  const auto feature_opt = GetCutinSLNetFeature();
  EXPECT_NE(feature_opt, std::nullopt);
  const auto& feature = *feature_opt;
  // At least has one object AV
  EXPECT_GE(feature.objs_sl_features.size(), 1);

  // Check rel_sl_pos_agent
  for (int i = 0; i < kFeatureV2HistoryStepNum; ++i) {
    if (static_cast<bool>(feature.agent_sl_feature.mask[i])) {
      const auto& agent_s =
          feature.agent_sl_feature.sl_pos[i * kCutinSLConfig.coord_num];
      const auto& agent_l =
          feature.agent_sl_feature.sl_pos[i * kCutinSLConfig.coord_num + 1];
      for (int j = 0; j < feature.objs_sl_features.size(); ++j) {
        const auto& obj_s =
            feature.objs_sl_features[j].sl_pos[i * kCutinSLConfig.coord_num];
        const auto& obj_l = feature.objs_sl_features[j]
                                .sl_pos[i * kCutinSLConfig.coord_num + 1];
        if (!(obj_s == 0 && obj_l == 0)) {
          EXPECT_NEAR(obj_s - agent_s,
                      feature.objs_sl_features[j]
                          .rel_sl_pos_agent[i * kCutinSLConfig.coord_num],
                      kEps);
          EXPECT_NEAR(obj_l - agent_l,
                      feature.objs_sl_features[j]
                          .rel_sl_pos_agent[i * kCutinSLConfig.coord_num + 1],
                      kEps);
        }
      }
    }
  }

  // Check rel_sl_speed_agent
  for (int i = 0; i < kFeatureV2HistoryStepNum; ++i) {
    if (static_cast<bool>(feature.agent_sl_feature.mask[i])) {
      const auto& agent_speed_s =
          feature.agent_sl_feature.sl_speed[i * kCutinSLConfig.coord_num];
      const auto& agent_speed_l =
          feature.agent_sl_feature.sl_speed[i * kCutinSLConfig.coord_num + 1];
      for (int j = 0; j < feature.objs_sl_features.size(); ++j) {
        const auto& obj_speed_s =
            feature.objs_sl_features[j].sl_speed[i * kCutinSLConfig.coord_num];
        const auto& obj_speed_l =
            feature.objs_sl_features[j]
                .sl_speed[i * kCutinSLConfig.coord_num + 1];
        if (!(obj_speed_s == 0 && obj_speed_l == 0)) {
          EXPECT_NEAR(obj_speed_s - agent_speed_s,
                      feature.objs_sl_features[j]
                          .rel_sl_speed_agent[i * kCutinSLConfig.coord_num],
                      kEps);
          EXPECT_NEAR(obj_speed_l - agent_speed_l,
                      feature.objs_sl_features[j]
                          .rel_sl_speed_agent[i * kCutinSLConfig.coord_num + 1],
                      kEps);
        }
      }
    }
  }

  // Speed less than threshold
  for (int i = 0; i < kFeatureV2HistoryStepNum; ++i) {
    const auto& agent_speed_s =
        feature.agent_sl_feature.sl_speed[i * kCutinSLConfig.coord_num];
    const auto& agent_speed_l =
        feature.agent_sl_feature.sl_speed[i * kCutinSLConfig.coord_num + 1];
    EXPECT_LE(std::abs(agent_speed_s), kSLSpeedMax);
    EXPECT_LE(std::abs(agent_speed_l), kSLSpeedMax);
    for (int j = 0; j < feature.objs_sl_features.size(); ++j) {
      const auto& obj_speed_s =
          feature.objs_sl_features[j].sl_speed[i * kCutinSLConfig.coord_num];
      const auto& obj_speed_l = feature.objs_sl_features[j]
                                    .sl_speed[i * kCutinSLConfig.coord_num + 1];
      EXPECT_LE(std::abs(obj_speed_s), kSLSpeedMax);
      EXPECT_LE(std::abs(obj_speed_l), kSLSpeedMax);
    }
  }

  // Pos less than threshold
  for (int i = 0; i < kFeatureV2HistoryStepNum; ++i) {
    const auto& agent_pos_s =
        feature.agent_sl_feature.sl_pos[i * kCutinSLConfig.coord_num];
    const auto& agent_pos_l =
        feature.agent_sl_feature.sl_pos[i * kCutinSLConfig.coord_num + 1];
    EXPECT_LE(std::abs(agent_pos_s), kSLCorMax);
    EXPECT_LE(std::abs(agent_pos_l), kSLCorMax);
    for (int j = 0; j < feature.objs_sl_features.size(); ++j) {
      const auto& obj_pos_s =
          feature.objs_sl_features[j].sl_pos[i * kCutinSLConfig.coord_num];
      const auto& obj_pos_l =
          feature.objs_sl_features[j].sl_pos[i * kCutinSLConfig.coord_num + 1];
      EXPECT_LE(std::abs(obj_pos_s), kSLCorMax);
      EXPECT_LE(std::abs(obj_pos_l), kSLCorMax);
    }
  }
}

TEST(CutinSLNetFeatureExtractorTest, ToDumpedProto) {
  const auto feature_opt = GetCutinSLNetFeature();
  EXPECT_NE(feature_opt, std::nullopt);
  const auto& feature = *feature_opt;

  const double ts = (kNumHistory - 1) * kUpdateTimeStep;

  std::vector<FrenetCoordinate> agent_gt;
  for (int i = 0; i < kCutinSLConfig.future_num; ++i) {
    FrenetCoordinate frenet_point = {i * kSstep, 0.5};
    agent_gt.push_back(frenet_point);
  }
  const auto gt_sl = agent_gt;
  const std::vector<CrossType> if_cross_gts{
      CrossType::kNotcrossed, CrossType::kNotcrossed, CrossType::kNotcrossed,
      CrossType::kNotcrossed};
  const auto dumped_proto = ToCutinSLDumpedFeatureProto(
      feature, absl::MakeSpan(gt_sl), absl::MakeSpan(if_cross_gts), ts);
  EXPECT_EQ(dumped_proto.timestamp(), ts);
  EXPECT_GT(dumped_proto.agent_sl_feature().sl_pos().size(),
            kCutinSLConfig.coord_num * kNumHistory);
  EXPECT_GT(dumped_proto.objects_sl_feature().sl_pos().size(),
            2 * kCutinSLConfig.coord_num * kNumHistory);
}
}  // namespace
}  // namespace qcraft::prediction
