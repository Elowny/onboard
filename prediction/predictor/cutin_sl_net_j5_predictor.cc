#include "onboard/prediction/predictor/cutin_sl_net_j5_predictor.h"

#include <algorithm>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/prediction/util/lane_follow_util.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/perception.pb.h"
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

std::optional<PredictedTrajectory> MakeCutinLaneFollowTrajectory(
    double target_l, int target_channel, int cur_channel,
    const ObjectMotionState& last_state,
    const planner::DrivePassage& drive_passage,
    absl::Span<const double> channel_probs, int traj_index,
    double perception_acc, bool is_slow_cutin) {
  double cutin_pred_horizon = kCutinPredictionDuration;
  if (is_slow_cutin) {
    cutin_pred_horizon = kSlowCutinPredictionDuration;
  }
  double accel = 0.0;
  if (FLAGS_only_use_perception_acc) {
    accel = perception_acc;
  }
  auto generated_points = GeneratePredictedTrajectoryPoints(
      last_state, drive_passage, accel, cutin_pred_horizon, target_l);
  if (!generated_points.ok() ||
      generated_points->size() <
          static_cast<int>(kEmergencyGuardHorizon / kPredictionTimeStep)) {
    return std::nullopt;
  }
  auto traj = PredictedTrajectory(
      channel_probs[target_channel],
      "Cutin lane follow j5: channel " + std::to_string(target_channel),
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
    const planner::DrivePassage& drive_passage, double perception_acc) {
  std::vector<PredictedTrajectory> trajs;
  trajs.reserve(4);
  const auto& last_state = motion_hist.states.back();
  const auto& obj_pos = last_state.pos;
  const auto cur_pos_sl_or =
      drive_passage.QueryUnboundedFrenetCoordinateAt(obj_pos);
  if (!cur_pos_sl_or.ok()) {
    return trajs;
  }

  // Calculate lateral speed with history
  double cur_lat_speed = LineFitLateralSpeedByMotionHistory(
      motion_hist.states, drive_passage, kLineFitTime,
      /*clamp_by_lane_width=*/true);
  cur_lat_speed =
      std::clamp(cur_lat_speed, -kLateralSpeedClamp, kLateralSpeedClamp);

  // Calculate target_l with current lateral speed.
  const auto cur_pos_sl = cur_pos_sl_or.value();
  double target_l = 0.0;
  if (cur_pos_sl.l > 0) {
    target_l = std::max(
        0.0, cur_pos_sl.l + cur_lat_speed * kLateralSpeedLookAheadTime);
  } else {
    target_l = std::min(
        0.0, cur_pos_sl.l + cur_lat_speed * kLateralSpeedLookAheadTime);
  }

  int target_channel = -1;
  // Upadate target l using channel info
  const auto traj_channel_index =
      GetTrajChannelIndex(channel_probs, cur_channel);
  if (traj_channel_index.has_value()) {
    target_channel = traj_channel_index.value();
    const double lateral_offset =
        (traj_channel_index.value() - kCenterChannel) * kChannelWidth;

    VLOG(2) << "Current channel " << cur_channel << " target offset "
            << target_l << " channel center " << lateral_offset;
    if (target_l < 0.0) {
      target_l = std::clamp(target_l, lateral_offset - 6 * kHalfChannelWidth,
                            lateral_offset + kHalfChannelWidth);
    } else {
      target_l = std::clamp(target_l, lateral_offset - kHalfChannelWidth,
                            lateral_offset + 6 * kHalfChannelWidth);
    }
    VLOG(2) << " Clamped offset " << target_l;
  }

  // Update target l if the object is on lane boundary
  const auto bounds_or =
      drive_passage.QueryNearestBoundaryLateralOffset(cur_pos_sl.s);
  double min_l = -kDefaultHalfLaneWidth;
  double max_l = kDefaultHalfLaneWidth;
  if (bounds_or.ok()) {
    min_l = std::max(min_l, bounds_or->first);
    max_l = std::min(max_l, bounds_or->second);
  }

  const bool is_slow_cutin = IsSlowCutinObj(drive_passage, last_state);

  // If cutin net predicts channel 2 and the prob is high, we set target l to 0
  // directly if the object is approaching. Otherwise, the crossing lane logic
  // is enabled.
  constexpr double kCenterChannelProbThreshold = 0.8;
  if (target_channel == kCenterChannel &&
      channel_probs[target_channel] > kCenterChannelProbThreshold &&
      IsObjectApproachingDrivePassage(drive_passage, last_state,
                                      motion_hist.type, min_l, max_l,
                                      cur_lat_speed)) {
    target_l = 0.0;
    target_channel = kCenterChannel;
  } else {
    const auto target_l_or = CalcTargetLForObjectCrossingBoundary(
        last_state, cur_lat_speed, min_l, max_l, drive_passage,
        motion_hist.type, target_l, /*is_hd_map=*/true, is_slow_cutin);
    if (target_l_or.ok()) {
      target_l = target_l_or.value();
      target_channel = kCenterChannel;
    }
  }

  // Generate traj only if one of the two update is valid
  if (target_channel != -1) {
    auto traj = MakeCutinLaneFollowTrajectory(
        target_l, target_channel, cur_channel, last_state, drive_passage,
        channel_probs, trajs.size(), perception_acc, is_slow_cutin);
    if (traj.has_value()) {
      trajs.push_back(std::move(traj.value()));
    }
  }

  return trajs;
}
}  // namespace

ObjectsCutinSLNetPredMap MakeCutinSLNetPrediction(
    const PredictionContext& prediction_context,
    const std::vector<ObjectIDType>& cutin_sl_candidate_objs,
    const cutin_sl_net_j5::CutinNetJ5Inferencer& cutin_sl_net_j5_inferencer,
    const ObjectHistorySampler& obj_sampler, MapSampler* map_sampler) {
  SCOPED_QTRACE("MakeCutinSLNetPrediction");
  ScopedMultiTimer timer("CutinSLNet predictor::MakeCutinSLNetPrediction");
  const auto* drive_passage_ptr = prediction_context.av_drive_passage();
  ObjectsCutinSLNetPredMap res;
  if (drive_passage_ptr == nullptr) {
    VLOG(2) << "Can not build drive passage!";
    return res;
  }

  timer.Mark("CutinSLNet predictor::Select CutinSLNet objects.");
  if (cutin_sl_candidate_objs.empty()) {
    return res;
  }
  auto objs_out = cutin_sl_net_j5_inferencer.PredictForObjects(
      cutin_sl_candidate_objs, obj_sampler, map_sampler, *drive_passage_ptr);
  timer.Mark("CutinSLNet predictor::Inference.");

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

      // Compute perception acc
      double perception_acc = 0.0;
      if (FLAGS_only_use_perception_acc) {
        const auto& obj_proto =
            prediction_context.object_history_manager().at(id).object_proto();
        const auto perception_accel_vec = Vec2d(obj_proto.accel());
        const auto perception_vel_vec = Vec2d(obj_proto.vel());
        if (perception_vel_vec.norm() > 0) {
          perception_acc = perception_accel_vec.dot(perception_vel_vec.Unit());
        }
      }

      auto predicted_traj =
          PostProcessing(*motion_hist, channel_probs, cur_channel,
                         *drive_passage_ptr, perception_acc);
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
