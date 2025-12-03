#include "onboard/prediction/feature_extractor/lane_selection_feature_extractor.h"

#include <algorithm>  // for max
#include <cmath>      // for abs, M_PI
#include <memory>     // for make_shared, __shared_ptr_access, shared_ptr
#include <vector>     // for vector, allocator

#include "absl/status/statusor.h"  // for StatusOr

#include "gtest/gtest.h"  // for Test, Message, TestPartResult, CmpHelperLE

#include "onboard/maps/map_selector.h"          // for SetMap
#include "onboard/maps/semantic_map_manager.h"  // for SemanticMapManager
#include "onboard/math/vec.h"                   // for Vec2d
#include "onboard/planner/router/drive_passage_builder.h"  // for BuildDrivePassageForPrediction
#include "onboard/planner/test_util/load_psmm_util.h"  // for CreateDojoTestPSMM
#include "onboard/prediction/container/traffic_light_manager.h"  // for TrafficLightManager
#include "onboard/prediction/feature_extractor/lane_selection_feature.h"  // for ObjectTypeToLaneSelectionNetType, LaneSele...
#include "onboard/prediction/prediction_defs.h"  // for ObjectMotionState, kFeatureV2HistoryStepNum
#include "onboard/prediction/test_util/object_history_builder.h"  // for BuildVehicleHistoryByConstVel
#include "onboard/prediction/util/lane_path_finder.h"  // for ExtendMostStraightLanePath, FindNearestLan...
#include "onboard/proto/perception.pb.h"  // for ObjectType

namespace qcraft::prediction {
namespace {
constexpr double kEps = 1e-5;
constexpr double kMaxHeadingDiff = M_PI / 6.0;
constexpr double kForwardLane = 50.0;          // m.
constexpr double kForwardExtendLength = 10.0;  // m.
constexpr double kDrivePassageStepS = 4.0;
constexpr double kDrivePassageBackwardLength = 40.0;  // m.
constexpr char kEgoId[] = "9999";
constexpr int kNumHistory = 10;
constexpr double kEgoInitPosX = 10.0;
constexpr double kEgoVel = 2.0;
constexpr char kObject1Id[] = "999";
constexpr double kObject1InitPosY = 10.0;
constexpr double kObject1Vel = 1.0;
constexpr double kUpdateTimeStep = 0.1;
constexpr double kSLSpeedMax = 50;  // m/s
constexpr double kSLCorMax = 500;   // m
constexpr double kSstep = 0.5;      // m

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

std::optional<LaneSelectionFeature> GetLaneSelectionNetFeature() {
  LaneSelectionFeature feature;
  const auto obj_sampler = MakeObjectHistorySampler(kNumHistory);
  SetMap("dojo");
  auto smm = std::make_shared<SemanticMapManager>();
  smm->LoadWholeMap().Build();
  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightManager::TLStateHashMap tl_state;
  MapSampler map_sampler(psmm, tl_state, kFeatureV2MaxMapSampleLen,
                         kFeatureV2MapSegmentNum,
                         MapSampler::SampleType::ADAPTIVE);
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
  return ExtractLaneSelectionFeature(kEgoId, obj_sampler, *drive_passage,
                                     &map_sampler);
}

TEST(LaneSelectionNetFeatureExtractorTest, TypeConvertCheck) {
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_UNKNOWN_STATIC),
            LaneSelectionNetType::AT_UNKNOWN_STATIC);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_VEHICLE),
            LaneSelectionNetType::AT_VEHICLE);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_MOTORCYCLIST),
            LaneSelectionNetType::AT_CYCLIST);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_CYCLIST),
            LaneSelectionNetType::AT_CYCLIST);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_TRICYCLIST),
            LaneSelectionNetType::AT_CYCLIST);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_PEDESTRIAN),
            LaneSelectionNetType::AT_PEDESTRIAN);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_FOD),
            LaneSelectionNetType::AT_FOD);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_UNKNOWN_MOVABLE),
            LaneSelectionNetType::AT_UNKNOWN_MOVABLE);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_VEGETATION),
            LaneSelectionNetType::AT_VEGETATION);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_BARRIER),
            LaneSelectionNetType::AT_BARRIER);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_CONE),
            LaneSelectionNetType::AT_CONE);
  EXPECT_EQ(ObjectTypeToLaneSelectionNetType(OT_WARNING_TRIANGLE),
            LaneSelectionNetType::AT_WARNING_TRIANGLE);
}

TEST(LaneSelectionNetFeatureExtractorTest, ExtractLaneSelectionNetFeatureTest) {
  const auto feature_opt = GetLaneSelectionNetFeature();
  EXPECT_NE(feature_opt, std::nullopt);
  const auto& feature = *feature_opt;
  // At least has one object AV
  EXPECT_GE(feature.objs_sl_features.size(), 1);

  // Check rel_sl_pos_agent
  for (int i = 0; i < kFeatureV2HistoryStepNum; ++i) {
    if (static_cast<bool>(feature.agent_sl_feature.mask[i])) {
      const auto& agent_s =
          feature.agent_sl_feature.sl_pos[i * kLaneSelectionConfig.coord_num];
      const auto& agent_l = feature.agent_sl_feature
                                .sl_pos[i * kLaneSelectionConfig.coord_num + 1];
      for (int j = 0; j < feature.objs_sl_features.size(); ++j) {
        const auto& obj_s = feature.objs_sl_features[j]
                                .sl_pos[i * kLaneSelectionConfig.coord_num];
        const auto& obj_l = feature.objs_sl_features[j]
                                .sl_pos[i * kLaneSelectionConfig.coord_num + 1];
        if (!(obj_s == 0 && obj_l == 0)) {
          EXPECT_NEAR(obj_s - agent_s,
                      feature.objs_sl_features[j]
                          .rel_sl_pos_agent[i * kLaneSelectionConfig.coord_num],
                      kEps);
          EXPECT_NEAR(
              obj_l - agent_l,
              feature.objs_sl_features[j]
                  .rel_sl_pos_agent[i * kLaneSelectionConfig.coord_num + 1],
              kEps);
        }
      }
    }
  }

  // Speed less than threshold
  for (int i = 0; i < kFeatureV2HistoryStepNum; ++i) {
    const auto& agent_speed_s =
        feature.agent_sl_feature.sl_speed[i * kLaneSelectionConfig.coord_num];
    const auto& agent_speed_l =
        feature.agent_sl_feature
            .sl_speed[i * kLaneSelectionConfig.coord_num + 1];
    EXPECT_LE(std::abs(agent_speed_s), kSLSpeedMax);
    EXPECT_LE(std::abs(agent_speed_l), kSLSpeedMax);
    for (int j = 0; j < feature.objs_sl_features.size(); ++j) {
      const auto& obj_speed_s =
          feature.objs_sl_features[j]
              .sl_speed[i * kLaneSelectionConfig.coord_num];
      const auto& obj_speed_l =
          feature.objs_sl_features[j]
              .sl_speed[i * kLaneSelectionConfig.coord_num + 1];
      EXPECT_LE(std::abs(obj_speed_s), kSLSpeedMax);
      EXPECT_LE(std::abs(obj_speed_l), kSLSpeedMax);
    }
  }

  // Pos less than threshold
  for (int i = 0; i < kFeatureV2HistoryStepNum; ++i) {
    const auto& agent_pos_s =
        feature.agent_sl_feature.sl_pos[i * kLaneSelectionConfig.coord_num];
    const auto& agent_pos_l =
        feature.agent_sl_feature.sl_pos[i * kLaneSelectionConfig.coord_num + 1];
    EXPECT_LE(std::abs(agent_pos_s), kSLCorMax);
    EXPECT_LE(std::abs(agent_pos_l), kSLCorMax);
    for (int j = 0; j < feature.objs_sl_features.size(); ++j) {
      const auto& obj_pos_s = feature.objs_sl_features[j]
                                  .sl_pos[i * kLaneSelectionConfig.coord_num];
      const auto& obj_pos_l =
          feature.objs_sl_features[j]
              .sl_pos[i * kLaneSelectionConfig.coord_num + 1];
      EXPECT_LE(std::abs(obj_pos_s), kSLCorMax);
      EXPECT_LE(std::abs(obj_pos_l), kSLCorMax);
    }
  }
}

TEST(LaneSelectionNetFeatureExtractorTest, ToDumpedProto) {
  const auto feature_opt = GetLaneSelectionNetFeature();
  EXPECT_NE(feature_opt, std::nullopt);
  const auto& feature = *feature_opt;

  const double ts = (kNumHistory - 1) * kUpdateTimeStep;
  std::vector<FrenetCoordinate> agent_gt;
  for (int i = 0; i < kLaneSelectionConfig.future_num; ++i) {
    FrenetCoordinate frenet_point = {i * kSstep, 0.5};
    agent_gt.push_back(frenet_point);
  }
  const auto gt_sl = agent_gt;
  const auto dumped_proto =
      ToLaneSelectionDumpedFeatureProto(feature, gt_sl, ts);
  EXPECT_EQ(dumped_proto.timestamp(), ts);
  EXPECT_GT(dumped_proto.agent_sl_feature().sl_pos().size(),
            kLaneSelectionConfig.coord_num * kNumHistory);
  EXPECT_GT(dumped_proto.objects_sl_feature().sl_pos().size(),
            2 * kLaneSelectionConfig.coord_num * kNumHistory);
}

}  // namespace
}  // namespace qcraft::prediction
