#include "onboard/prediction/predictor/cutin_sl_net_predictor.h"

#include <algorithm>
#include <ostream>  // for operator<<, basic_ostream, basic_ostre...
#include <string>
#include <utility>

#include "absl/status/statusor.h"   // for StatusOr
#include "absl/strings/str_join.h"  // for StrJoin
#include "glog/logging.h"  // for COMPACT_GOOGLE_LOG_INFO, LogMessage, VLOG

#include "onboard/async/parallel_for.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"  // for ScopedTrace, SCOPED_QTRACE, FUNC_QTRACE
#include "onboard/lite/check.h"    // for QCHECK_NOTNULL
#include "onboard/maps/maps_common.h"  // for CrosswalkInfo, LaneBoundaryInfo, LaneInfo
#include "onboard/math/frenet_common.h"               // for FrenetCoordinate
#include "onboard/planner/router/drive_passage.h"     // for DrivePassage
#include "onboard/prediction/container/av_context.h"  // for AvContext
#include "onboard/prediction/container/object_history_span.h"  // for ObjectHistorySpan
#include "onboard/prediction/feature_extractor/cutin_sl_feature.h"
#include "onboard/prediction/feature_extractor/cutin_sl_feature_extractor.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kChannelProbThreshold = 0.2;
constexpr int kCenterChannel = 2;
constexpr double kCenterChannelsThreshold = 0.7;  // probability
const std::vector<int> kCenterChannels = {1, 2, 3};
constexpr double kLateralSpeedLookAheadTime = 3.0;  // s.
constexpr double kLateralSpeedClamp = 2.5;          // m/s.
constexpr double kLineFitTime = 0.5;
constexpr int kMaxPredNum = 8;
constexpr double kAvMapRadius = 210.0;

std::vector<CutinSLFeature> PrepareCutinSLFeature(
    const std::vector<ObjectIDType>& objs_to_predict_ids,
    const ObjectHistorySampler& obj_sampler, MapSampler* const map_sampler,
    ThreadPool* thread_pool, const planner::DrivePassage& drive_passage) {
  SCOPED_QTRACE_ARG1("PrepareCutinSLFeature",
                     "objs_num:", objs_to_predict_ids.size());
  std::vector<std::optional<CutinSLFeature>> features;
  features.resize(objs_to_predict_ids.size());
  const auto* av_history = obj_sampler.GetResampledAVMotionHistory();
  const auto& av_pos = av_history->states.back().pos;
  const auto lane_centers =
      map_sampler->GetLaneCentersWithRadius(av_pos, kAvMapRadius);
  const auto lane_boundaries =
      map_sampler->GetSolidLaneBoundariesWithRadius(av_pos, kAvMapRadius);
  const auto crosswalks =
      map_sampler->GetCrosswalksWithRadius(av_pos, kAvMapRadius);
  ParallelFor(0, objs_to_predict_ids.size(), thread_pool, [&](int i) {
    auto obj_feats = ExtractCutinSLFeature(
        objs_to_predict_ids[i], obj_sampler, drive_passage, lane_centers,
        lane_boundaries, crosswalks, map_sampler);

    if (obj_feats != std::nullopt) {
      features[i] = std::move(*obj_feats);
    }
  });
  std::vector<CutinSLFeature> final_features;
  final_features.reserve(features.size());
  for (auto& feat_opt : features) {
    if (feat_opt.has_value()) {
      final_features.push_back(std::move(feat_opt).value());
    }
  }
  return final_features;
}
std::optional<PredictedTrajectory> MakeCutinLaneFollowTrajectory(
    double target_l, int target_channel, int cur_channel,
    const ObjectMotionState& last_state,
    const planner::DrivePassage& drive_passage,
    absl::Span<const double> channel_probs, int traj_index) {
  const double lateral_offset =
      (target_channel - kCenterChannel) * kChannelWidth;
  VLOG(2) << "Current channel " << cur_channel << " target offset " << target_l
          << " channel center " << lateral_offset;
  if (target_l < 0.0) {
    target_l = std::clamp(target_l, lateral_offset - 3 * kHalfChannelWidth,
                          lateral_offset + kHalfChannelWidth);
  } else {
    target_l = std::clamp(target_l, lateral_offset - kHalfChannelWidth,
                          lateral_offset + 3 * kHalfChannelWidth);
  }
  VLOG(2) << " Clamped offset " << target_l;
  auto generated_points = GeneratePredictedTrajectoryPoints(
      last_state, drive_passage, /*accel=*/0.0, kCutinPredictionDuration,
      target_l);
  if (!generated_points.ok() ||
      generated_points->size() <
          static_cast<int>(kEmergencyGuardHorizon / kPredictionTimeStep)) {
    return std::nullopt;
  }
  auto traj = PredictedTrajectory(
      channel_probs[target_channel],
      "Cutin lane follow: channel " + std::to_string(target_channel),
      PredictionType::PT_CUTIN_SL_NET, traj_index,
      std::move(generated_points.value()),
      /*is_reversed=*/false);
  traj.set_predicted_channel(target_channel);
  traj.set_cur_channel(cur_channel);
  traj.set_channel_probs(channel_probs);
  return traj;
}

std::optional<int> GetTrajChannelIndex(absl::Span<const double> channel_probs,
                                       int cur_channel) {
  struct ChannelIndexProb {
    int index;
    double prob;
  };
  std::vector<ChannelIndexProb> channel_index_probs;
  for (int i = 0; i < 5; ++i) {
    double center_channels_sum = 0.0;
    for (const auto center_channel : kCenterChannels) {
      center_channels_sum += channel_probs[center_channel];
    }
    if (center_channels_sum < kCenterChannelsThreshold) {
      continue;
    }
    if (channel_probs[i] < kChannelProbThreshold) {
      continue;
    }
    // For channel 0/4, Only consider the cases approaching the av drivepassage.
    if ((i == 0 || i == 4) &&
        (cur_channel - kCenterChannel) * (cur_channel - i) <= 0) {
      continue;
    }
    if (i == 2) {
      return i;
    } else {
      channel_index_probs.push_back({.index = i, .prob = channel_probs[i]});
    }
  }
  if (channel_index_probs.size() == 1) {
    return channel_index_probs[0].index;
  } else if (channel_index_probs.size() > 1) {
    std::sort(channel_index_probs.begin(), channel_index_probs.end(),
              [](const auto& left, const auto& right) {
                return left.prob > right.prob;
              });
    return channel_index_probs[0].index;
  } else {
    return std::nullopt;
  }
}

std::vector<PredictedTrajectory> PostProcessing(
    const ObjectMotionHistory& motion_hist,
    absl::Span<const double> channel_probs, int cur_channel,
    const planner::DrivePassage& drive_passage) {
  std::vector<PredictedTrajectory> trajs;
  trajs.reserve(4);
  const auto& last_state = motion_hist.states.back();
  const auto& obj_pos = last_state.pos;
  const auto cur_pos_sl_or =
      drive_passage.QueryUnboundedFrenetCoordinateAt(obj_pos);
  if (!cur_pos_sl_or.ok()) {
    return trajs;
  }
  // Calculate target_l with current lateral speed.
  const auto cur_pos_sl = cur_pos_sl_or.value();
  double cur_lat_speed = LineFitLateralSpeedByMotionHistory(
      motion_hist.states, drive_passage, kLineFitTime,
      /*clamp_by_lane_width=*/true);
  cur_lat_speed =
      std::clamp(cur_lat_speed, -kLateralSpeedClamp, kLateralSpeedClamp);
  double target_l = 0.0;
  if (cur_pos_sl.l > 0) {
    target_l = std::max(
        0.0, cur_pos_sl.l + cur_lat_speed * kLateralSpeedLookAheadTime);
  } else {
    target_l = std::min(
        0.0, cur_pos_sl.l + cur_lat_speed * kLateralSpeedLookAheadTime);
  }
  const auto traj_channel_index =
      GetTrajChannelIndex(channel_probs, cur_channel);
  if (traj_channel_index.has_value()) {
    auto traj = MakeCutinLaneFollowTrajectory(
        target_l, traj_channel_index.value(), cur_channel, last_state,
        drive_passage, channel_probs, trajs.size());
    if (traj.has_value()) {
      trajs.push_back(std::move(traj.value()));
    }
  }
  return trajs;
}
}  // namespace

std::optional<ObjectsCutinSLNetPredMap> MakeCutinSLNetPrediction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const cutin_sl_net::CutinSLNetInferencer& cutin_sl_net_inferencer,
    const ObjectHistorySampler& obj_sampler, MapSampler* map_sampler,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("MakeCutinSLNetPrediction");
  ScopedMultiTimer timer("CutinSLNet predictor::MakeCutinSLNetPrediction");
  const auto& av_context = prediction_context.av_context();
  const auto* drive_passage_ptr = prediction_context.av_drive_passage();
  if (drive_passage_ptr == nullptr) {
    VLOG(2) << "Can not build drive passage!";
    return std::nullopt;
  }

  const ObjectHistorySpan av_history_span =
      av_context.GetAvObjectHistory().GetHistory();
  const auto cutin_sl_candidate_objs_ids = SelectCutinSLNetPredictedObjects(
      av_history_span, kMaxPredNum, objects_history, *drive_passage_ptr,
      obj_sampler);
  timer.Mark("CutinSLNet predictor::Select CutinSLNet objects.");
  if (cutin_sl_candidate_objs_ids.empty()) {
    return std::nullopt;
  }
  VLOG(2) << "Number of candidate CutinSLNet Objects is "
          << cutin_sl_candidate_objs_ids.size();

  const auto cutin_sl_features =
      PrepareCutinSLFeature(cutin_sl_candidate_objs_ids, obj_sampler,
                            map_sampler, thread_pool, *drive_passage_ptr);
  if (cutin_sl_features.empty()) {
    return std::nullopt;
  }
  VLOG(2) << "Number of actually predicted CutinSLNet Objects is "
          << cutin_sl_features.size();
  timer.Mark("CutinSLNet predictor::Prepare cutin sl net feature.");

  auto objs_out = cutin_sl_net_inferencer.PredictForObjects(cutin_sl_features);
  timer.Mark("CutinSLNet predictor::Inference.");

  ObjectsCutinSLNetPredMap res;
  {
    SCOPED_QTRACE("CutinSLNetPredictor::PostProcessing");
    for (const auto& [id, obj_out] : objs_out) {
      const auto& channel_probs = obj_out.channle_probs;
      const auto& cur_channel = obj_out.cur_channel;
      VLOG(2) << "Considered ID " << id;
      VLOG(2) << "Channel prob: " << absl::StrJoin(channel_probs, ",");
      const auto* motion_hist =
          obj_sampler.GetResampledMotionHistoryPtrById(id);
      QCHECK_NOTNULL(motion_hist);
      auto predicted_traj = PostProcessing(*motion_hist, channel_probs,
                                           cur_channel, *drive_passage_ptr);
      if (predicted_traj.size() > 0) {
        res[id] = ObjectCutinSLNetPred{
            .pred_trajs = std::move(predicted_traj),
        };
      }
    }
  }
  return res;
}
}  // namespace prediction
}  // namespace qcraft
