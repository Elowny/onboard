#include "onboard/prediction/predictor/act_net_predictor.h"

#include <algorithm>
#include <array>
#include <map>
#include <numeric>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/global/logging.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/feature_extractor/act_net_feature_extractor.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/predictor/cutin_sl_net_predictor.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/prediction/util/safe_guard_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"  // for ObjectType
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/elements_history.h"
namespace qcraft {
namespace prediction {
namespace {
constexpr double kProbThreshold = 0.15;
using actnet::ActNetInferencer;
std::vector<AgentCentricObjectProbTraj> FilterTrajsByProb(
    absl::Span<const AgentCentricObjectProbTraj> prob_trajs) {
  std::vector<AgentCentricObjectProbTraj> filtered_trajs;
  filtered_trajs.reserve(prob_trajs.size());
  // Trajectory probability is ordered from large to small.
  filtered_trajs.push_back(prob_trajs.front());
  int valid_traj_idx = 1;
  double sum_prob = prob_trajs.front().mode_prob;
  while (valid_traj_idx < prob_trajs.size()) {
    if (prob_trajs[valid_traj_idx].mode_prob < kProbThreshold) {
      break;
    }
    filtered_trajs.push_back(prob_trajs[valid_traj_idx]);
    sum_prob += prob_trajs[valid_traj_idx].mode_prob;
    ++valid_traj_idx;
  }
  QCHECK_GT(sum_prob, 0.0);
  for (auto& prob_traj : filtered_trajs) {
    prob_traj.mode_prob /= sum_prob;
  }
  return filtered_trajs;
}

std::optional<PredictedTrajectory> MakeSafeGuardBrakePrediciton(
    const ObjectProto& obj_proto,
    const ObjectMotionHistory& object_motion_history, double av_v,
    const std::vector<PredictedTrajectory>& pred_trajs) {
  // BANDAID(xiangjun): temporary hack to avoid being too optimistic on object
  // speed.
  if (object_motion_history.type == OT_VEHICLE ||
      object_motion_history.type == OT_LARGE_VEHICLE) {
    // Compute perception acc.
    const auto perception_accel_vec = Vec2d(obj_proto.accel());
    const auto perception_vel_vec = Vec2d(obj_proto.vel());
    double perception_acc = 0.0;
    if (perception_vel_vec.norm() > 0) {
      perception_acc = perception_accel_vec.dot(perception_vel_vec.Unit());
    }
    auto pred_traj_or = CreateSafeGuardBrakeTrajectory(
        object_motion_history, pred_trajs, perception_acc, av_v,
        PredictionType::PT_ACTNET);
    return pred_traj_or;
  }
  return std::nullopt;
}

bool FilterRedundantCutinTraj(
    const planner::DrivePassage& drive_passage,
    const std::vector<PredictedTrajectory>& actnet_trajs,
    const PredictedTrajectory& cutin_traj) {
  // If Actnet trajs cutin faster than cutin traj, then do not use cutin traj.
  QCHECK(!cutin_traj.points().empty());
  const int cutin_index =
      std::min(static_cast<int>(kCutinSLHorizon / kPredictionTimeStep) - 1,
               static_cast<int>(cutin_traj.points().size()) - 1);
  const Vec2d& cur_pos = cutin_traj.points().at(0).pos();
  const auto sl_cur_pos_or =
      drive_passage.QueryUnboundedFrenetCoordinateAt(cur_pos);
  if (!sl_cur_pos_or.ok()) return true;
  const Vec2d& cutin_pos = cutin_traj.points().at(cutin_index).pos();
  const auto sl_cutin_pos_or =
      drive_passage.QueryUnboundedFrenetCoordinateAt(cutin_pos);
  if (!sl_cutin_pos_or.ok()) return true;
  if (actnet_trajs.empty()) return false;
  for (const auto& actnet_traj : actnet_trajs) {
    if (actnet_traj.points().size() < cutin_index + 1) continue;
    const Vec2d& actnet_traj_pos = actnet_traj.points().at(cutin_index).pos();
    const auto sl_actnet_traj_pos_or =
        drive_passage.QueryUnboundedFrenetCoordinateAt(actnet_traj_pos);
    if (!sl_actnet_traj_pos_or.ok()) continue;
    if (sl_cur_pos_or->l > 0.0) {
      if (sl_cutin_pos_or->l > sl_actnet_traj_pos_or->l) {
        // Current pos on right, and actnet traj pos on cutin pos left.
        return true;
      }
    } else {
      if (sl_actnet_traj_pos_or->l > sl_cutin_pos_or->l) {
        // Current pos on left, and actnet traj pos on cutin pos right.
        return true;
      }
    }
  }
  return false;
}

std::optional<PredictedTrajectory> GetCutinTrajectory(
    const planner::DrivePassage& drive_passage,
    const ObjectsCutinSLNetPredMap& cutin_sl_net_out_map, const std::string& id,
    const std::vector<PredictedTrajectory>& actnet_trajs) {
  auto iter = cutin_sl_net_out_map.find(id);
  if (iter != cutin_sl_net_out_map.end()) {
    const auto& cutin_trajs = iter->second;
    if (cutin_trajs.pred_trajs.size() > 0) {
      if (FilterRedundantCutinTraj(drive_passage, actnet_trajs,
                                   cutin_trajs.pred_trajs[0])) {
        return std::nullopt;
      }
      return cutin_trajs.pred_trajs[0];
    }
  }
  return std::nullopt;
}

std::vector<PredictedTrajectory> PostProcessingTrajectories(
    const PredictionContext& prediction_context, const ObjectProto& obj_proto,
    const ObjectMotionHistory& object_motion_history,
    absl::Span<const AgentCentricObjectProbTraj> prob_trajs,
    const std::optional<ObjectsCutinSLNetPredMap>& cutin_sl_net_out_map_or,
    double predict_start_time) {
  const auto cur_motion = object_motion_history.states.back();
  const double t_diff = cur_motion.timestamp - predict_start_time;
  std::vector<PredictedTrajectory> pred_trajs;
  const auto filtered_trajs = FilterTrajsByProb(prob_trajs);
  QCHECK_GT(filtered_trajs.size(), 0);
  pred_trajs.reserve(filtered_trajs.size());
  for (int traj_index = 0; traj_index < filtered_trajs.size(); ++traj_index) {
    const auto& prob_traj = filtered_trajs[traj_index];
    std::vector<Vec2d> traj_points;
    std::vector<double> vec_t;
    // Current pos + predicted points.
    const int traj_points_size =
        FloorToInt(kPredictionDuration / kPredictionTimeStep) + 1;
    traj_points.reserve(traj_points_size);
    vec_t.reserve(traj_points_size);
    // Here assuming that the current pos is the first predicted point.
    traj_points.push_back(cur_motion.pos);
    vec_t.push_back(t_diff);
    for (int i = 0; i < prob_traj.traj_points.size(); ++i) {
      const auto& pt_with_uncer = prob_traj.traj_points[i];
      Vec2d pos(pt_with_uncer[0], pt_with_uncer[1]);
      traj_points.push_back(std::move(pos));
      vec_t.push_back((i + 1) * kPredictionTimeStep);
    }
    auto traj_pts = Vec2dPointsToPredTrajPoints(
        traj_points, vec_t, traj_points.size(), /*use_pos_fitter=*/false,
        kPolyFitDownSampleStep);

    if (IsBicycleModelLike(object_motion_history.type)) {
      // Compute perception acc.
      const auto perception_accel_vec = Vec2d(obj_proto.accel());
      const auto perception_vel_vec = Vec2d(obj_proto.vel());
      double perception_acc = 0.0;
      if (perception_vel_vec.squaredNorm() > 0.0) {
        perception_acc = perception_accel_vec.dot(perception_vel_vec.Unit());
      }
      traj_pts = PostProcessModelOutputTrajPts(
          traj_pts, object_motion_history.states.back(), kPredictionTimeStep,
          kPredictionDuration, perception_acc, /*rectify_speed=*/false);
    }

    // Assign uncertainty.
    for (int i = 0; i < traj_pts.size(); ++i) {
      if (i >= prob_traj.traj_points.size()) {
        break;
      }
      const auto& pt_with_uncer = prob_traj.traj_points[i];
      auto& pred_traj_pt = traj_pts[i];
      pred_traj_pt.set_uncertainty(Vec2d(pt_with_uncer[2], pt_with_uncer[3]),
                                   pt_with_uncer[4] + prob_traj.rot_rad);
    }

    std::string annotation = "Predicted by actnet model";
    PredictedTrajectory pred_traj(prob_traj.mode_prob, annotation,
                                  PredictionType::PT_ACTNET, traj_index,
                                  traj_pts, /*is_reversed=*/false);
    pred_traj.set_rot_rad(prob_traj.rot_rad);  // For calculation
                                               // of uncertatiny ellipse.
    pred_trajs.push_back(std::move(pred_traj));
  }

  std::optional<PredictedTrajectory> cutin_traj;
  const auto* drive_passage_ptr = prediction_context.av_drive_passage();
  if (cutin_sl_net_out_map_or.has_value() && drive_passage_ptr != nullptr) {
    cutin_traj =
        GetCutinTrajectory(*drive_passage_ptr, cutin_sl_net_out_map_or.value(),
                           obj_proto.id(), pred_trajs);
  }
  const auto brake_traj = MakeSafeGuardBrakePrediciton(
      obj_proto, object_motion_history,
      prediction_context.av_context().GetAvCurrentSpeed(), pred_trajs);
  if (cutin_traj.has_value()) {
    if (pred_trajs.size() > 2) {
      pred_trajs.pop_back();
    }
  }
  if (brake_traj.has_value()) {
    pred_trajs.push_back(brake_traj.value());
  }
  if (cutin_traj.has_value()) {
    const double sum_prob =
        std::accumulate(pred_trajs.begin(), pred_trajs.end(), 0.0,
                        [](double prob, const auto& traj2) {
                          return prob + traj2.probability();
                        });
    if (cutin_traj->probability() > sum_prob) {
      cutin_traj->set_probability(sum_prob);
    }
    pred_trajs.push_back(cutin_traj.value());
  }
  // Normalize and Sort
  if (cutin_traj.has_value() || brake_traj.has_value()) {
    NormalizeAndDescSortTrajProbs(absl::MakeSpan(pred_trajs));
  }

  // Reassign idx
  for (int i = 0; i < pred_trajs.size(); ++i) {
    pred_trajs[i].set_index(i);
  }

  return pred_trajs;
}

std::vector<ActNetFeature> PrepareActNetFeature(
    absl::Span<const ObjectHistory* const> objects_history,
    const ObjectHistorySampler& obj_sampler, MapSampler* const map_sampler_ptr,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE_ARG1("PrepareActNetFeature",
                     "objs_num:", objects_history.size());
  AvMapCache av_map_cache;
  const auto* av_history = obj_sampler.GetResampledAVMotionHistory();
  const auto& av_pos = av_history->states.back().pos;
  const Vec2d center =
      Vec2d::FastUnitFromAngle(av_history->states.back().heading) *
          kActNetConfig.av_map_offset +
      av_pos;
  av_map_cache.lane_centers = map_sampler_ptr->GetLaneCentersWithRadius(
      center, kActNetConfig.av_map_radius);
  av_map_cache.solid_lane_boundarys =
      map_sampler_ptr->GetSolidLaneBoundariesWithRadius(
          center, kActNetConfig.av_map_radius);
  av_map_cache.cross_walks = map_sampler_ptr->GetCrosswalksWithRadius(
      center, kActNetConfig.av_map_radius);
  std::vector<std::optional<ActNetFeature>> features;
  features.resize(objects_history.size());
  ParallelFor(0, objects_history.size(), thread_pool, [&](int i) {
    const auto& agent_hist = *objects_history[i];
    const auto& agent_id = agent_hist.id();
    const auto& agent_history =
        obj_sampler.GetResampledMotionHistoryById(agent_id);
    const double heading = agent_history.states.back().heading;
    const auto ref_position = agent_history.states.back().pos;

    const auto region_box = GetRegionBox(
        ref_position, heading, kActNetConfig.scan_box_front,
        kActNetConfig.scan_box_back, kActNetConfig.scan_box_half_width);
    auto obj_feats = ExtractActNetFeature(
        agent_id, obj_sampler, agent_hist.GetStopTimeInfo(),
        /*drive_passage_ptr=*/nullptr, map_sampler_ptr, av_map_cache,
        region_box);
    // If the extracted feature does not contain any lane center
    // polyline, do not make prediction.
    if (obj_feats.lc_features.size() != 0) {
      features[i] = std::move(obj_feats);
    }
  });
  std::vector<ActNetFeature> final_features;
  final_features.reserve(features.size());
  for (auto& feat_opt : features) {
    if (feat_opt.has_value()) {
      final_features.push_back(std::move(feat_opt).value());
    }
  }
  return final_features;
}
}  // namespace

ObjectsActNetPredMap MakeActNetPrediction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const actnet::ActNetInferencer* act_net_inferencer_ptr,
    const cutin_sl_net::CutinSLNetInferencer* cutin_sl_inferencer_ptr,
    const ObjectHistorySampler& obj_sampler, ThreadPool* threadpool) {
  SCOPED_QTRACE("MakeActNetPrediction");
  ScopedMultiTimer timer("ActNet predictor::MakeActNetPrediction");
  QCHECK_NOTNULL(act_net_inferencer_ptr);
  const auto& act_net_inferencer = *act_net_inferencer_ptr;
  const auto& av_context = prediction_context.av_context();
  const ObjectHistorySpan av_history =
      av_context.GetAvObjectHistory().GetHistory();
  const auto& semantic_map_manager = prediction_context.semantic_map_manager();
  MapSampler map_sampler(
      semantic_map_manager,
      prediction_context.traffic_light_manager().GetOriginalTlStateMap(),
      kFeatureV2MaxMapSampleLen, kFeatureV2MapSegmentNum,
      MapSampler::SampleType::ADAPTIVE);
  const TypePrioMap type_prio_map = {
      {PredictTypePrio::HIGH,
       {OT_VEHICLE, OT_LARGE_VEHICLE, OT_CYCLIST, OT_TRICYCLIST,
        OT_MOTORCYCLIST}},
      {PredictTypePrio::MED, {OT_PEDESTRIAN}},
      {PredictTypePrio::LOW, {OT_UNKNOWN_MOVABLE}}};
  // Here holds the assumption that the max num can be divided by 8.
  const int base_quota_num = kActNetConfig.max_pred_num / 8;
  const std::map<PredictTypePrio, int> type_max_num_map = {
      {PredictTypePrio::HIGH, base_quota_num * 5},
      {PredictTypePrio::MED, base_quota_num * 2},
      {PredictTypePrio::LOW, base_quota_num * 1}};
  const auto act_candidate_objs = SelectPredictedObjectsByTypePriority(
      av_history.back().val.bounding_box(), kActNetConfig.max_pred_num,
      objects_history, type_prio_map, type_max_num_map);
  timer.Mark("ActNet predictor::Select ActNet objects.");

  VLOG(2) << "Number of candidate ActNet Objects is "
          << act_candidate_objs.size();
  const double current_ts = obj_sampler.current_time();

  const auto act_net_features = PrepareActNetFeature(
      act_candidate_objs, obj_sampler, &map_sampler, threadpool);

  VLOG(2) << "Number of actually predicted ActNet Objects is "
          << act_net_features.size();
  timer.Mark("ActNet predictor::Prepare act net feature.");

  VLOG(2) << "act_net_features:" << act_net_features.size();
  auto objs_out = act_net_inferencer.PredictForObjects(act_net_features);
  timer.Mark("ActNet predictor::Inference.");
  std::optional<ObjectsCutinSLNetPredMap> cutin_sl_net_out_map = std::nullopt;
  // make cutin prediction
  if (FLAGS_prediction_enable_auxiliary_cutin_sl_net &&
      cutin_sl_inferencer_ptr != nullptr && act_candidate_objs.size() > 0) {
    cutin_sl_net_out_map = MakeCutinSLNetPrediction(
        prediction_context, act_candidate_objs, *cutin_sl_inferencer_ptr,
        obj_sampler, &map_sampler, threadpool);
  }
  ObjectsActNetPredMap res;
  {
    SCOPED_QTRACE("ActNetPredictor::PostProcessingTrajs");
    for (const auto& [id, obj_out] : objs_out) {
      const auto& resampled_motion_history =
          obj_sampler.GetResampledMotionHistoryById(id);
      const auto& obj_proto =
          prediction_context.object_history_manager().at(id).object_proto();
      const auto& actnet_trajs = obj_out.prob_trajs;
      auto trajs = PostProcessingTrajectories(
          prediction_context, obj_proto, resampled_motion_history, actnet_trajs,
          cutin_sl_net_out_map, current_ts);
      res[id] = ObjectActNetPred{
          .pred_trajs = std::move(trajs),
          .startup_prob = obj_out.startup_prob,
      };
    }
  }
  timer.Mark("ActNet predictor::Postprocessing.");
  if (FLAGS_print_prediction_time_stats) {
    planner::PrintMultiTimerReportStat(timer);
  }
  return res;
}
}  // namespace prediction
}  // namespace qcraft
