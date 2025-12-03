#include "onboard/prediction/predictor/act_net_j5_predictor.h"

#include <stddef.h>
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numeric>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/global/logging.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/net/horizon/act_net_j5.h"
#include "onboard/prediction/net/horizon/act_net_local_j5.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/predictor/cutin_sl_net_j5_predictor.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/elements_history.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr int kMaxCutinPreNum = 4;

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

std::optional<std::vector<Vec2d>> RectifyParallelHighRiskObjectTrajectory(
    const ObjectMotionHistory& motion_history,
    const std::vector<Vec2d>& traj_pts, const planner::DrivePassage& dp,
    const ParallelRiskType& parallel_risk_type, std::string* annotation_ptr) {
  constexpr int kLargeVehiclLateralCutOffIndex =
      static_cast<int>(kLargeVehicleCutinHorizon / kPredictionTimeStep) + 1;
  constexpr int kNormalLateralVehicleCutoffIndex =
      static_cast<int>(kNormalVehicleCutinHorizon / kPredictionTimeStep) + 1;
  int lateral_cut_off_horizon = kLargeVehicleCutinHorizon;
  int lateral_cut_off_index = kLargeVehiclLateralCutOffIndex;
  double no_rectify_dist = kLargeVehicleNoRectifyDist;
  if (parallel_risk_type != ParallelRiskType::kNearLargeVehicle) {
    lateral_cut_off_horizon = kNormalVehicleCutinHorizon;
    lateral_cut_off_index = kNormalLateralVehicleCutoffIndex;
    no_rectify_dist = kNormalVehicleNoRectifyDist;
  }
  if (traj_pts.size() <= lateral_cut_off_index + 1) {
    return std::nullopt;
  }

  // Rectify when the traj is over certain time and length threshold.
  double accum_length = 0.0;
  for (int i = 1; i < traj_pts.size(); ++i) {
    accum_length += traj_pts[i].DistanceTo(traj_pts[i - 1]);
    if (accum_length >= no_rectify_dist) {
      if (i > lateral_cut_off_index) {
        lateral_cut_off_index = i;
      }
      break;
    }
  }
  if (accum_length < no_rectify_dist) {
    *annotation_ptr =
        absl::StrFormat("%s, no rectify for short traj", *annotation_ptr);

    return std::nullopt;
  }

  std::vector<Vec2d> new_pts;
  new_pts.reserve(traj_pts.size());
  new_pts.insert(new_pts.end(), traj_pts.begin(),
                 traj_pts.begin() + lateral_cut_off_index + 1);

  const auto target_sl_pos_or =
      dp.QueryUnboundedFrenetCoordinateAt(traj_pts[lateral_cut_off_index]);
  if (!target_sl_pos_or.ok()) {
    *annotation_ptr =
        absl::StrFormat("%s, no rectify when project sl fail", *annotation_ptr);

    return std::nullopt;
  }

  const FrenetCoordinate& target_sl = target_sl_pos_or.value();
  const auto bounds_or = dp.QueryNearestBoundaryLateralOffset(target_sl.s);
  if (!bounds_or.ok()) {
    return std::nullopt;
  }
  // Assuming that the agent will along the target drive passage center in long
  // horizon. Lateral step giant change should be handled by latter process.
  double final_l = 0.0;
  const auto& motion_states = motion_history.states;
  const auto& cur_motion = motion_states.back();
  const double lateral_buffer = cur_motion.bbox.width() * 0.5;  // m

  const auto cur_sl_pos_or =
      dp.QueryUnboundedFrenetCoordinateAt(cur_motion.pos);
  if (!cur_sl_pos_or.ok()) {
    return std::nullopt;
  }

  if (cur_sl_pos_or->l < bounds_or->first + lateral_buffer ||
      cur_sl_pos_or->l > bounds_or->second - lateral_buffer) {
    const auto prev_sl_pose_or =
        dp.QueryUnboundedFrenetCoordinateAt(motion_states.front().pos);
    if (!prev_sl_pose_or.ok()) {
      return std::nullopt;
    }
    const double time_diff = kPredictionTimeStep * motion_states.size();
    const double lateral_v =
        (cur_sl_pos_or->l - prev_sl_pose_or->l) / time_diff;
    if (std::abs(lateral_v) < 0.1) {
      final_l = cur_sl_pos_or->l;
    } else {
      if (lateral_v * cur_sl_pos_or->l > 0.0) {
        // Lateral offset to the other lane center, use the same lane width.
        const double lane_width = bounds_or->second - bounds_or->first;
        if (lane_width < 3.0 || lane_width > 4.0) {
          *annotation_ptr =
              absl::StrFormat("%s, no rectify for %.2f lane width",
                              *annotation_ptr, lane_width);

          return std::nullopt;
        }
        final_l = std::copysign(lane_width, cur_sl_pos_or->l);
      }
    }
  }

  double current_s = target_sl.s;
  const double delta_s = traj_pts[lateral_cut_off_index + 1].DistanceTo(
      traj_pts[lateral_cut_off_index]);

  for (size_t i = lateral_cut_off_index + 1; i < traj_pts.size(); ++i) {
    current_s += delta_s;
    auto xy_or = dp.QueryPointXYAtSL(current_s, final_l);
    if (xy_or.ok()) {
      new_pts.push_back(std::move(xy_or).value());
    } else {
      break;
    }
  }

  if (new_pts.size() < kPredictionMinPointNum) {
    *annotation_ptr = absl::StrFormat("%s, no rectify when points num < %d",
                                      *annotation_ptr, kPredictionMinPointNum);

    return std::nullopt;
  }

  *annotation_ptr =
      absl::StrFormat("%s, rectify with horizon %d s and final lateral %.2f m",
                      *annotation_ptr, lateral_cut_off_horizon, final_l);

  return new_pts;
}

// TODO(yinbao): Optimize for J5.
std::vector<PredictedTrajectory> PostProcessingTrajectories(
    const PredictionContext& prediction_context, const ObjectProto& obj_proto,
    const ObjectMotionHistory& object_motion_history,
    absl::Span<const AgentCentricObjectProbTraj> prob_trajs,
    const ObjectsCutinSLNetPredMap& cutin_sl_net_out_map,
    double predict_start_time, ParallelRiskType parallel_risk_type,
    PredictionType pred_type) {
  const auto cur_motion = object_motion_history.states.back();
  const auto* drive_passage_ptr = FindBestMatchingDrivePassage(
      prediction_context.drive_passages(), cur_motion.pos, cur_motion.heading);
  const auto& filtered_trajs = prob_trajs;
  QCHECK_GT(filtered_trajs.size(), 0);
  std::vector<PredictedTrajectory> pred_trajs;
  pred_trajs.reserve(filtered_trajs.size());
  const double t_diff = cur_motion.timestamp - predict_start_time;
  constexpr int kTrajPointsSize = kPredictionDuration / kPredictionTimeStep + 1;
  for (int traj_index = 0; traj_index < filtered_trajs.size(); ++traj_index) {
    VLOG(2) << "traj index " << traj_index;
    const auto& prob_traj = filtered_trajs[traj_index];
    std::string annotation = absl::StrFormat("Predicted by actnet j5 (%s)",
                                             PredictionType_Name(pred_type));
    std::vector<Vec2d> traj_points;
    traj_points.reserve(kTrajPointsSize);
    std::vector<double> vec_t;
    vec_t.reserve(kTrajPointsSize);
    // Here assuming that the current pos is the first predicted point.
    traj_points.push_back(cur_motion.pos);
    vec_t.push_back(t_diff);
    double time_step = kPredictionTimeStep;
    for (int i = 0; i < prob_traj.traj_points.size(); ++i) {
      const auto& pt_with_uncer = prob_traj.traj_points[i];
      Vec2d pos(pt_with_uncer[0], pt_with_uncer[1]);
      traj_points.push_back(std::move(pos));
      vec_t.push_back(time_step);
      time_step += kPredictionTimeStep;
    }
    // Rectify trajectory lateral offset based on best matching drive passage.
    if (parallel_risk_type != ParallelRiskType::kNone &&
        drive_passage_ptr != nullptr) {
      auto traj_points_or = RectifyParallelHighRiskObjectTrajectory(
          object_motion_history, traj_points, *drive_passage_ptr,
          parallel_risk_type, &annotation);
      if (traj_points_or.has_value()) {
        traj_points = std::move(traj_points_or.value());
      }
      vec_t.resize(traj_points.size());
    }

    // Convert Vec2d trajectory to PredictedTrajectoryPoint by poly fitting.
    auto traj_pts = Vec2dPointsToPredTrajPoints(
        traj_points, vec_t, traj_points.size(), /*use_pos_fitter=*/true,
        kPolyFitDownSampleStep);

    // Refine trajectory kinematic behavior by PolePlacement controller.
    if (IsBicycleModelLike(object_motion_history.type)) {
      // Compute perception acc.
      const auto perception_accel_vec = Vec2d(obj_proto.accel());
      const auto perception_vel_vec = Vec2d(obj_proto.vel());
      double perception_acc = 0.0;
      if (perception_vel_vec.x() != 0.0 || perception_vel_vec.y() != 0.0) {
        perception_acc = perception_accel_vec.dot(perception_vel_vec.Unit());
      }
      traj_pts = PostProcessModelOutputTrajPts(
          traj_pts, object_motion_history.states.back(), kPredictionTimeStep,
          kPredictionDuration, perception_acc, FLAGS_rectify_speed_profile);
    }

    PredictedTrajectory pred_traj(prob_traj.mode_prob, annotation, pred_type,
                                  traj_index, traj_pts,
                                  /*is_reversed=*/false);
    pred_trajs.push_back(std::move(pred_traj));
  }

  std::optional<PredictedTrajectory> cutin_traj;
  const auto* av_drive_passage_ptr = prediction_context.av_drive_passage();
  if (cutin_sl_net_out_map.size() > 0 && av_drive_passage_ptr != nullptr) {
    cutin_traj = GetCutinTrajectory(*av_drive_passage_ptr, cutin_sl_net_out_map,
                                    obj_proto.id(), pred_trajs);
  }

  const double sum_prob =
      std::accumulate(pred_trajs.begin(), pred_trajs.end(), 0.0,
                      [](double prob, const auto& traj2) {
                        return prob + traj2.probability();
                      });

  if (cutin_traj.has_value()) {
    // Make sure cutin prob is less than 0.5
    if (cutin_traj->probability() > sum_prob) {
      cutin_traj->set_probability(sum_prob);
    }

    // Make sure all cutin traj prob is over 0.2.
    const double converted_minimal_prob =
        sum_prob * (kMinimalCutinProb / (1 - kMinimalCutinProb));
    // center_prob / (center_prob + actnet_prob_sum) > kMinimalCutinProb
    // leads to center_prob > converted_minimal_prob
    if (cutin_traj->probability() < converted_minimal_prob) {
      cutin_traj->set_probability(converted_minimal_prob);
    }

    // Make sure all center cutin traj prob is over 0.4.
    const double converted_center_prob =
        sum_prob * (kMinimalCenterCutinProb / (1 - kMinimalCenterCutinProb));
    if (cutin_traj->predicted_channel() == kCenterChannel &&
        cutin_traj->probability() < converted_center_prob) {
      cutin_traj->set_probability(converted_center_prob);
    }

    pred_trajs.push_back(cutin_traj.value());
  }

  NormalizeAndDescSortTrajProbs(absl::MakeSpan(pred_trajs));

  // Reassign idx
  for (int i = 0; i < pred_trajs.size(); ++i) {
    pred_trajs[i].set_index(i);
  }

  return pred_trajs;
}
}  // namespace

ObjectsActNetPredMap MakeActNetJ5Prediction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const actnetj5::ActNetJ5Inferencer& act_net_j5_inferencer,
    const cutin_sl_net_j5::CutinNetJ5Inferencer* cutin_sl_inferencer_ptr,
    const ObjectHistorySampler& obj_sampler, ThreadPool* threadpool) {
  SCOPED_QTRACE("MakeActNetJ5Prediction");
  ScopedMultiTimer timer("ActNetJ5 predictor::MakeActNetJ5Prediction");
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
  const int base_quota_num = actnetj5::kMaxPredObjectsNum / 8;
  const std::map<PredictTypePrio, int> type_max_num_map = {
      {PredictTypePrio::HIGH, base_quota_num * 5},
      {PredictTypePrio::MED, base_quota_num * 2},
      {PredictTypePrio::LOW, base_quota_num * 1}};
  const auto act_candidate_objs = SelectPredictedObjectsByTypePriority(
      av_history.back().val.bounding_box(), actnetj5::kMaxPredObjectsNum,
      objects_history, type_prio_map, type_max_num_map);
  timer.Mark("ActNetJ5 predictor::Select ActNetJ5 objects.");

  VLOG(3) << "Number of candidate ActNetJ5 Objects is "
          << act_candidate_objs.size();
  const double current_ts = obj_sampler.current_time();

  auto objs_out = act_net_j5_inferencer.PredictForObjects(
      act_candidate_objs, obj_sampler, &map_sampler, threadpool);
  timer.Mark("ActNetJ5 predictor::Inference.");

  const ObjectHistorySpan av_history_span =
      prediction_context.av_context().GetAvObjectHistory().GetHistory();

  // make cutin prediction
  ObjectsCutinSLNetPredMap cutin_sl_net_out_map;
  if (const auto* drive_passage_ptr = prediction_context.av_drive_passage();
      drive_passage_ptr != nullptr) {
    std::vector<ObjectIDType> cutin_sl_net_objs_ids =
        SelectCutinSLNetPredictedObjects(av_history_span, kMaxCutinPreNum,
                                         act_candidate_objs, *drive_passage_ptr,
                                         obj_sampler);
    if (FLAGS_prediction_enable_auxiliary_cutin_sl_net_j5 &&
        cutin_sl_inferencer_ptr != nullptr && act_candidate_objs.size() > 0) {
      cutin_sl_net_out_map = MakeCutinSLNetPrediction(
          prediction_context, cutin_sl_net_objs_ids, *cutin_sl_inferencer_ptr,
          obj_sampler, &map_sampler);
    }
  }

  ObjectsActNetPredMap res;
  {
    SCOPED_QTRACE("ActNetJ5 Predictor::PostProcessingTrajs");
    std::vector<const ObjectIDType*> id_vec;
    std::vector<const AgentCentricObjectOut*> out_vec;
    std::vector<ObjectActNetPred> preds;
    id_vec.reserve(objs_out.size());
    out_vec.reserve(objs_out.size());
    preds.resize(objs_out.size());
    for (const auto& [id, obj_out] : objs_out) {
      id_vec.push_back(&id);
      out_vec.push_back(&obj_out);
    }
    std::unordered_map<ObjectIDType, ParallelRiskType>
        parallel_high_risk_objs_map;
    if (const auto* drive_passage_ptr = prediction_context.av_drive_passage();
        drive_passage_ptr != nullptr) {
      parallel_high_risk_objs_map = SelectParallelHighRiskObjects(
          av_history_span, act_candidate_objs, *drive_passage_ptr, obj_sampler);
    }

    ParallelFor(0, objs_out.size(), threadpool, [&](int i) {
      const auto& id = *(id_vec[i]);
      const auto& obj_out = *(out_vec[i]);
      const auto& resampled_motion_history =
          obj_sampler.GetResampledMotionHistoryById(id);
      const auto& obj_proto =
          prediction_context.object_history_manager().at(id).object_proto();
      const auto& actnet_trajs = obj_out.prob_trajs;
      ParallelRiskType risk_type = ParallelRiskType::kNone;
      if (parallel_high_risk_objs_map.find(id) !=
          parallel_high_risk_objs_map.end()) {
        risk_type = parallel_high_risk_objs_map[id];
      }
      VLOG(2) << "Post processing object " << id << " high risk "
              << static_cast<int>(risk_type);
      auto trajs = PostProcessingTrajectories(
          prediction_context, obj_proto, resampled_motion_history, actnet_trajs,
          cutin_sl_net_out_map, current_ts, risk_type,
          PredictionType::PT_ACTNET_J5);
      preds[i] = ObjectActNetPred{.pred_trajs = std::move(trajs),
                                  .startup_prob = obj_out.startup_prob};
    });
    for (int i = 0; i < id_vec.size(); ++i) {
      res[*(id_vec[i])] = std::move(preds[i]);
    }
  }
  timer.Mark("ActNetJ5 predictor::Postprocessing.");
  if (FLAGS_print_prediction_time_stats) {
    planner::PrintMultiTimerReportStat(timer);
  }
  return res;
}

ObjectsActNetPredMap MakeActNetLocalJ5Prediction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const actnetlocalj5::ActNetLocalJ5Inferencer& act_net_local_j5_inferencer,
    const cutin_sl_net_j5::CutinNetJ5Inferencer* cutin_sl_inferencer_ptr,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool) {
  SCOPED_QTRACE("MakeActNetLocalJ5Prediction");
  ScopedMultiTimer timer(
      "ActNetLocalJ5 predictor::MakeActNetLocalJ5Prediction");
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
  const int base_quota_num = actnetlocalj5::kMaxPredObjectsNum / 8;
  const std::map<PredictTypePrio, int> type_max_num_map = {
      {PredictTypePrio::HIGH, base_quota_num * 5},
      {PredictTypePrio::MED, base_quota_num * 2},
      {PredictTypePrio::LOW, base_quota_num * 1}};
  const auto act_candidate_objs = SelectPredictedObjectsByTypePriority(
      av_history.back().val.bounding_box(), actnetlocalj5::kMaxPredObjectsNum,
      objects_history, type_prio_map, type_max_num_map);
  timer.Mark("ActNetLocalJ5 predictor::Select ActNetLocalJ5 objects.");

  VLOG(3) << "Number of candidate ActNetLocalJ5 Objects is "
          << act_candidate_objs.size();
  const double current_ts = obj_sampler.current_time();

  auto objs_out = act_net_local_j5_inferencer.PredictForObjects(
      act_candidate_objs, obj_sampler, &map_sampler, thread_pool);
  timer.Mark("ActNetLocalJ5 predictor::Inference.");

  const ObjectHistorySpan av_history_span =
      prediction_context.av_context().GetAvObjectHistory().GetHistory();

  // make cutin prediction
  ObjectsCutinSLNetPredMap cutin_sl_net_out_map;
  if (const auto* drive_passage_ptr = prediction_context.av_drive_passage();
      drive_passage_ptr != nullptr) {
    std::vector<ObjectIDType> cutin_sl_net_objs_ids =
        SelectCutinSLNetPredictedObjects(av_history_span, kMaxCutinPreNum,
                                         act_candidate_objs, *drive_passage_ptr,
                                         obj_sampler);

    if (FLAGS_prediction_enable_auxiliary_cutin_sl_net_j5 &&
        cutin_sl_inferencer_ptr != nullptr && act_candidate_objs.size() > 0) {
      cutin_sl_net_out_map = MakeCutinSLNetPrediction(
          prediction_context, cutin_sl_net_objs_ids, *cutin_sl_inferencer_ptr,
          obj_sampler, &map_sampler);
    }
  }

  ObjectsActNetPredMap res;
  {
    SCOPED_QTRACE("ActNetLocalJ5 Predictor::PostProcessingTrajs");
    std::vector<const ObjectIDType*> id_vec;
    std::vector<const AgentCentricObjectOut*> out_vec;
    std::vector<ObjectActNetPred> preds;
    id_vec.reserve(objs_out.size());
    out_vec.reserve(objs_out.size());
    preds.resize(objs_out.size());
    for (const auto& [id, obj_out] : objs_out) {
      id_vec.push_back(&id);
      out_vec.push_back(&obj_out);
    }

    std::unordered_map<ObjectIDType, ParallelRiskType>
        parallel_high_risk_objs_map;
    if (const auto* av_drive_passage_ptr =
            prediction_context.av_drive_passage();
        av_drive_passage_ptr != nullptr) {
      parallel_high_risk_objs_map =
          SelectParallelHighRiskObjects(av_history_span, act_candidate_objs,
                                        *av_drive_passage_ptr, obj_sampler);
    }

    ParallelFor(0, objs_out.size(), thread_pool, [&](int i) {
      const auto& id = *(id_vec[i]);
      const auto& obj_out = *(out_vec[i]);
      const auto& resampled_motion_history =
          obj_sampler.GetResampledMotionHistoryById(id);
      const auto& obj_proto =
          prediction_context.object_history_manager().at(id).object_proto();
      const auto& actnet_trajs = obj_out.prob_trajs;
      ParallelRiskType risk_type = ParallelRiskType::kNone;
      if (parallel_high_risk_objs_map.find(id) !=
          parallel_high_risk_objs_map.end()) {
        risk_type = parallel_high_risk_objs_map[id];
      }
      VLOG(2) << "Post processing object " << id << " high risk "
              << static_cast<int>(risk_type);
      auto trajs = PostProcessingTrajectories(
          prediction_context, obj_proto, resampled_motion_history, actnet_trajs,
          cutin_sl_net_out_map, current_ts, risk_type,
          PredictionType::PT_ACTNET_LOCAL_J5);
      preds[i] = ObjectActNetPred{.pred_trajs = std::move(trajs),
                                  .startup_prob = obj_out.startup_prob};
    });
    for (int i = 0; i < id_vec.size(); ++i) {
      res[*(id_vec[i])] = std::move(preds[i]);
    }
  }

  timer.Mark("ActNetLocalJ5 predictor::Postprocessing.");
  if (FLAGS_print_prediction_time_stats) {
    planner::PrintMultiTimerReportStat(timer);
  }
  return res;
}

}  // namespace prediction
}  // namespace qcraft
