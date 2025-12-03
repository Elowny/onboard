#include "onboard/prediction/scheduler/scheduler_utils.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <utility>

#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/long_term/long_term_behavior_estimator.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/predictor/heuristic_pedestrian_predictor.h"
#include "onboard/prediction/predictor/kinematic_predictor.h"
#include "onboard/prediction/predictor/l2_lane_follow_predictor.h"
#include "onboard/prediction/predictor/lane_selection_net_j5_predictor.h"
#include "onboard/prediction/predictor/vehicle_lane_follow_predictor.h"
#include "onboard/prediction/predictor/void_predictor.h"
#include "onboard/prediction/scheduler/priority_analyzer.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/elements_history.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kMaxStaticTrajLen = 0.25;  // m.
constexpr double kMinStartupProb = 0.1;
const std::map<PredictionType, int> kPredictorPriorityMap = {
    {PT_STATIONARY, 0},
    {PT_CUTIN_SL_NET, 101},
    {PT_REVERSE_CYCV, 1},
    {PT_ACTNET, 1},
    {PT_ACTNET_J5, 1},
    {PT_ACTNET_LOCAL_J5, 1},
    {PT_PROPHNET, 2},
    {PT_PED_KINEMATIC, 3},
    {PT_BIKE_LANE_FOLLOW, 3},
    {PT_VEHICLE_LANE_FOLLOW, 3},
    {PT_L2_LANE_FOLLOW, 3},
    {PT_LANE_SELECTION_NET, 97},
    {PT_CYCV, 4},
    {PT_CTRA, 4},
    {PT_VOID, 1},
    {PT_ORACLE, 99},
};

ObjectProto GenerateObjectProtoFromObjectMotionState(
    const ObjectProto& ref, absl::Span<const ObjectMotionState> states) {
  const auto& state = states.back();
  ObjectProto object = ref;
  object.set_timestamp(state.timestamp);
  if (object.moving_state() == ObjectProto::MS_STATIC) {
    return object;
  }
  state.pos.ToProto(object.mutable_pos());
  state.vel.ToProto(object.mutable_vel());
  const auto rotation_angle = state.heading - object.yaw();
  object.set_yaw(state.heading);
  state.bbox.ToProto(object.mutable_bounding_box());
  std::vector<Vec2d> contour_pts;
  contour_pts.reserve(ref.contour_size());
  for (const auto& pt : ref.contour()) {
    contour_pts.push_back(Vec2d(pt));
  }
  const Polygon2d contour(std::move(contour_pts));
  // Object current position from raw perception result.
  const auto perception_pos = Vec2d(ref.pos());
  // Align perception contour to the last resampled state.
  const auto state_contour = contour.Transform(
      perception_pos, /*cos_angle=*/fast_math::Cos(rotation_angle),
      /*sin_angle=*/fast_math::Sin(rotation_angle),
      /*translation=*/state.pos - perception_pos);
  // Clear origin contour proto and add aligned contour.
  object.mutable_contour()->Clear();
  for (const auto& point : state_contour.points()) {
    point.ToProto(object.add_contour());
  }
  return object;
}
}  // namespace

// Check if current predictor has higher priority than input predictor.
//  HIGH PRIORITY ===> LOW PRIORITY   0 ===> 99
bool IsHighOrEqualPriority(PredictionType cur, PredictionType in) {
  const auto* cur_priority = FindOrNull(kPredictorPriorityMap, cur);
  const auto* in_priority = FindOrNull(kPredictorPriorityMap, in);
  QCHECK_NOTNULL(cur_priority);
  QCHECK_NOTNULL(in_priority);
  if (*cur_priority <= *in_priority) {
    return true;
  }
  return false;
}

// Try insert the predicted trajectories if predictor relation is satisfied.
void TryInsertTrajectories(
    std::map<std::string, std::vector<PredictedTrajectory>>* obj_traj_map,
    std::vector<PredictedTrajectory> trajs, const ObjectIDType& obj_id) {
  QCHECK_GE(trajs.size(), 1);
  const auto* prev_trajs = FindOrNull(*obj_traj_map, obj_id);
  if (prev_trajs != nullptr) {
    const auto prev_type = prev_trajs->front().type();
    const auto cur_type = trajs.front().type();
    if (IsHighOrEqualPriority(prev_type, cur_type)) {
      return;
    }
  }
  (*obj_traj_map)[obj_id] = std::move(trajs);
}

void RecordPredSpeeGapQevent(const ObjectIDType& obj_id, double obj_v,
                             absl::Span<const PredictedTrajectory> obj_trajs) {
  const double avg_first_pt_v =
      std::accumulate(obj_trajs.begin(), obj_trajs.end(), 0.0,
                      [&](double sum, const auto& traj) {
                        return sum + traj.points().front().v();
                      }) /
      obj_trajs.size();
  if (std::fabs(avg_first_pt_v - obj_v) > std::max(obj_v * 0.2, 1.0)) {
    QEVENT_EVERY_N("runlin", "cur_speed_gap_between_prediction_perception", 10,
                   [&](QEvent* qevent) {
                     qevent->AddField("object_id", obj_id)
                         .AddField("pred_v", avg_first_pt_v)
                         .AddField("obj_v", obj_v);
                   });
  }
}

std::vector<PredictedTrajectory> FilterOutStaticAndLowProbTrajectory(
    absl::Span<const PredictedTrajectory> trajs, double startup_prob) {
  std::vector<PredictedTrajectory> res;
  int new_idx = 0;
  double sum_prob = 0.0;
  for (const auto& traj : trajs) {
    const double prob = traj.probability();
    if (traj.points().back().s() < kMaxStaticTrajLen ||
        prob < kMinStartupProb) {
      continue;
    }
    res.push_back(traj);
    res.back().set_index(new_idx);
    sum_prob += prob;
    new_idx++;
  }
  if (new_idx != 0) {
    for (auto& traj : res) {
      traj.set_probability(startup_prob * traj.probability() / sum_prob);
    }
  }
  return res;
}

void PredictStationaryAndReversed(
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistorySampler& obj_sampler,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    bool ignore_off_road) {
  SCOPED_QTRACE("PredictStationaryAndReversed");
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto& obj = objs_to_predict[i];
    const auto& motion_history =
        obj_sampler.GetResampledMotionHistoryById(obj->id());
    if (obj->GetHistory().back().val.IsStationary()) {
      std::vector<PredictedTrajectory> trajs = {
          MakeStationaryPrediction(motion_history, kPredictionDuration)};
      TryInsertTrajectories(object_trajs, std::move(trajs), obj->id());
      continue;
    }
    if (obj->GetHistory().back().val.IsReversed()) {
      std::vector<PredictedTrajectory> trajs;
      if (object_scenarios.at(obj->id()).road_status() == ORS_OFF_ROAD &&
          ignore_off_road) {
        trajs = {MakeVoidPrediction(motion_history)};
      } else {
        trajs = {
            MakeReverseCYCVPrediction(motion_history, kPredictionDuration)};
      }
      TryInsertTrajectories(object_trajs, std::move(trajs), obj->id());
      continue;
    }
  }
}

void PredictIgnoredObjects(
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistorySampler& obj_sampler,
    const std::map<ObjectIDType, ObjectPredictionPriorityInfo>&
        object_priorities,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs) {
  SCOPED_QTRACE("PredictIgnoredObjects");
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto& obj = objs_to_predict[i];
    const auto& id = obj->id();
    if (object_trajs->count(id) != 0) {
      continue;
    }
    const auto& priority = object_priorities.at(obj->id()).priority;
    if (priority != OPP_P3) {
      continue;
    }
    const auto& motion_history = obj_sampler.GetResampledMotionHistoryById(id);
    std::vector<PredictedTrajectory> trajs = {
        MakeVoidPrediction(motion_history)};
    TryInsertTrajectories(object_trajs, std::move(trajs), obj->id());
  }
}

void PredictObjectsByHeuristic(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs,
    bool ignore_off_road) {
  SCOPED_QTRACE("PredictObjectsByHeuristic");
  using IdTrajsPair = std::pair<std::string, std::vector<PredictedTrajectory>>;
  std::vector<std::optional<IdTrajsPair>> all_trajs;
  all_trajs.resize(objs_to_predict.size());
  ParallelFor(0, objs_to_predict.size(), thread_pool, [&](int i) {
    const auto& obj = objs_to_predict[i];
    const auto& id = obj->id();
    if (object_trajs->count(id) != 0) {
      return;
    }
    const auto& motion_history = obj_sampler.GetResampledMotionHistoryById(id);
    const auto obj_type = GuessType(*obj);
    // 1. Do heuristic pedestrian prediction.
    if (obj_type == OT_PEDESTRIAN) {
      std::vector<PredictedTrajectory> trajs;
      if (object_scenarios.at(id).road_status() == ORS_OFF_ROAD &&
          ignore_off_road) {
        trajs = {MakeVoidPrediction(motion_history)};
      } else {
        trajs = {MakeHeuristicPedestrianPrediction(motion_history)};
      }
      all_trajs[i] = std::make_pair(id, std::move(trajs));
      return;
    }

    // 2. Do heuristic bike prediction (for non-predicted bikes).
    if (obj_type == OT_MOTORCYCLIST || obj_type == OT_CYCLIST ||
        obj_type == OT_TRICYCLIST) {
      auto traj = MakeAssitDrivingLaneFollowPrediction(motion_history,
                                                       prediction_context);
      std::vector<PredictedTrajectory> trajs = {traj};
      all_trajs[i] = std::make_pair(id, std::move(trajs));
      return;
    }

    // 3. Do vehicle lane follow prediction (for non-predicted vehicles).
    if (obj_type == OT_VEHICLE || obj_type == OT_LARGE_VEHICLE) {
      auto trajs = MakeVehicleLaneFollowPrediction(
          motion_history, prediction_context, object_scenarios.at(id),
          ignore_off_road);
      all_trajs[i] = std::make_pair(id, std::move(trajs));
      return;
    }

    // 4. Default obj: const yaw const velocity prediction.
    std::vector<PredictedTrajectory> trajs;
    if (object_scenarios.at(id).road_status() == ORS_OFF_ROAD &&
        ignore_off_road) {
      trajs = {MakeVoidPrediction(motion_history)};
    } else {
      trajs = {MakeCYCVPrediction(motion_history, kPredictionDuration)};
    }
    all_trajs[i] = std::make_pair(id, std::move(trajs));
  });
  for (auto& trajs_opt : all_trajs) {
    if (trajs_opt.has_value()) {
      (*object_trajs)[trajs_opt->first] = std::move(trajs_opt->second);
    }
  }
}

void PredictObjectsByLaneSelection(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs,
    const ModelPool& model_pool, bool ignore_off_road) {
  SCOPED_QTRACE("PredictObjectsByLaneSelection");
  std::vector<const ObjectHistory*> filtered_by_priority_objs;
  filtered_by_priority_objs.reserve(objs_to_predict.size());

  for (const auto* obj : objs_to_predict) {
    if (object_trajs->count(obj->id()) != 0 ||
        (object_scenarios.at(obj->id()).road_status() == ORS_OFF_ROAD &&
         ignore_off_road)) {
      continue;
    }
    filtered_by_priority_objs.push_back(obj);
  }

  auto lane_selection_net_out_map = MakeLaneSelectionNetJ5Prediction(
      prediction_context, filtered_by_priority_objs,
      *model_pool.GetLaneSelectionNetJ5Inferencer(), object_scenarios,
      obj_sampler, thread_pool);
  for (const auto& [id, lane_selection_out] : lane_selection_net_out_map) {
    if (lane_selection_out.pred_trajs.empty()) {
      continue;
    }
    TryInsertTrajectories(object_trajs, lane_selection_out.pred_trajs, id);
  }
}

void PredictUnderParkingScenario(
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs) {
  FUNC_QTRACE();
  ParallelFor(0, objs_to_predict.size(), thread_pool, [&](int i) {
    const auto& obj = objs_to_predict[i];
    const auto& id = obj->id();
    const auto& motion_history = obj_sampler.GetResampledMotionHistoryById(id);
    const auto obj_latest_state = obj->GetHistory().back();
    PredictedTrajectory pred_traj;
    if (obj_latest_state.val.IsStationary()) {
      pred_traj =
          MakeStationaryPrediction(motion_history, kParkingPredictionDuration);
    } else if (obj_latest_state.val.IsReversed()) {
      pred_traj =
          MakeReverseCYCVPrediction(motion_history, kParkingPredictionDuration);
    } else {
      pred_traj =
          MakeCYCVPrediction(motion_history, kParkingPredictionDuration);
    }

    TryInsertTrajectories(object_trajs, {std::move(pred_traj)}, id);
  });
}

void AssemblePredictionResult(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistorySampler& obj_sampler,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const std::map<ObjectIDType, ObjectPredictionPriorityInfo>&
        object_priorities,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs,
    std::map<ObjectIDType, ObjectPredictionResult>* results) {
  FUNC_QTRACE();
  const auto& long_term_mgr =
      prediction_context.object_long_term_history_manager();
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto hist = objs_to_predict[i]->GetHistory();
    const auto& id = hist.id();
    auto* trajs = FindOrNull(*object_trajs, id);
    if (trajs != nullptr) {
      for (const auto& traj : *trajs) {
        QCHECK(!traj.points().empty());
      }
      if (!hist.back().val.IsStationary() && !hist.back().val.IsReversed() &&
          !trajs->empty()) {
        RecordPredSpeeGapQevent(id, hist.back().val.v(), *trajs);
      }
      ObjectLongTermBehaviorProto long_term_behavior;
      const auto* long_term_hist = long_term_mgr.FindOrNull(id);
      if (long_term_hist != nullptr) {
        long_term_behavior = EstimateLongTermBehavior(
            prediction_context.av_context(),
            prediction_context.vehicle_geometry_params(),
            object_scenarios.at(id), *long_term_hist);
      }
      // Make sure this sampler resamples history based on tracker history and
      // cached history!
      const auto& resampled_hist =
          obj_sampler.GetResampledMotionHistoryById(id);
      auto obj_proto = GenerateObjectProtoFromObjectMotionState(
          hist.back().val.object_proto(),
          absl::MakeSpan(resampled_hist.states));
      (*results)[id] = ObjectPredictionResult{
          .id = id,
          .priority = object_priorities.at(id).priority,
          .priority_annotation = object_priorities.at(id).priority_annotation,
          .scenario = object_scenarios.at(id),
          .perception_object = std::move(obj_proto),
          .stop_time_info = objs_to_predict[i]->GetStopTimeInfo(),
          .long_term_behavior = long_term_behavior,
          .trajectories = {std::move((*object_trajs)[id])}};
    }
  }
}

}  // namespace prediction
}  // namespace qcraft
