#include "onboard/prediction/predictor/lane_selection_net_j5_predictor.h"

#include <algorithm>
#include <cmath>
#include <optional>  // for optional, operator==, nullopt
#include <ostream>   // for operator<<, basic_ostream
#include <string>    // for operator<<
#include <utility>
#include <vector>

#include "absl/status/statusor.h"  // for StatusOr
#include "absl/strings/str_cat.h"  // for StrCat
#include "glog/logging.h"  // for COMPACT_GOOGLE_LOG_INFO, LogMessage, VLOG

#include "onboard/async/parallel_for.h"
#include "onboard/container/strong_int.h"  // for StrongInt
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"                  // for LanePath
#include "onboard/maps/lane_point.h"                 // for LanePoint
#include "onboard/math/frenet_common.h"              // for FrenetCoordinate
#include "onboard/math/piecewise_linear_function.h"  // for PiecewiseLinearFunction
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/planner/planner_semantic_map_manager.h"  // for PlannerSemanticMapManager
#include "onboard/planner/router/drive_passage_builder.h"  // for BuildDrivePassageForPrediction
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/prediction/container/av_context.h"           // for AvContext
#include "onboard/prediction/container/object_history_span.h"  // for ObjectHistorySpan
#include "onboard/prediction/container/objects_history.h"  // for ObjectsHistory
#include "onboard/prediction/container/prediction_object.h"  // for PredictionObject
#include "onboard/prediction/net/horizon/lane_selection_net.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/prediction/util/drive_passage_util.h"
#include "onboard/prediction/util/lane_follow_util.h"
#include "onboard/prediction/util/lane_path_util.h"  // for FindPossibleLanePathsByCTRATPrediction
#include "onboard/prediction/util/safe_guard_util.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/perception.pb.h"  // for ObjectProto, ObjectType
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/elements_history.h"  // for Node

namespace qcraft {
namespace prediction {
namespace {
// Lane selection heuritic info for post process.
struct AgentHeuristicInfo {
  double cur_lat_speed;
  double agent_half_width;
  std::optional<FrenetCoordinate> sl_pos_opt;
  std::optional<double> angle_diff_dp_opt;
  double left_bound_l = kDefaultHalfLaneWidth;
  double right_bound_l = -kDefaultHalfLaneWidth;
  std::optional<double> agent_min_dist_to_bounds;
  std::optional<bool> is_in_lane_opt;
  std::optional<bool> is_on_boundary_opt;
  ObjectType agent_type;
};

constexpr double kHardLateralMovementThreshold = 0.6;             // m/s
constexpr double kHardAngleDiffDpThreshold = 8.0 * M_PI / 180.0;  // rad
constexpr double kMinimalDpProb = 0.2;

// Sampling step of drive passage.
constexpr double kDrivePassageStepS = 4.0;  // m

PredictedTrajectoryProto::LaneSelectionTrajType AnalyzeTrajType(
    const planner::DrivePassage& dp, const ObjectHistorySampler& obj_sampler,
    const ObjectIDType& agent_id) {
  const auto sl_or = dp.QueryFrenetCoordinateAt(
      obj_sampler.GetResampledMotionHistoryById(agent_id).states.back().pos);

  if (!sl_or.ok()) {
    return PredictedTrajectoryProto::LSTT_VOID;
  }
  const auto& sl = sl_or.value();
  const auto bounds_or = dp.QueryNearestBoundaryLateralOffset(sl.s);
  if (!bounds_or.ok()) {
    return PredictedTrajectoryProto::LSTT_VOID;
  }
  double min_l = -lane_selection_net::kDefaultLaneSelectionHalfLaneWidth;
  double max_l = lane_selection_net::kDefaultLaneSelectionHalfLaneWidth;

  min_l = std::max(min_l, bounds_or->first);
  max_l = std::min(max_l, bounds_or->second);

  if (sl.l > min_l && sl.l < max_l) {
    return PredictedTrajectoryProto::LSTT_LANE_KEEPING;
  } else {
    return PredictedTrajectoryProto::LSTT_LANE_CHANGING;
  }
}

double CalcLaneSelectionTargetL(
    const AgentHeuristicInfo& agent_heuristic_info) {
  if (!agent_heuristic_info.sl_pos_opt.has_value() ||
      !agent_heuristic_info.angle_diff_dp_opt.has_value())
    return 0.0;

  const auto& sl_pos = *agent_heuristic_info.sl_pos_opt;
  const auto& angle_diff_dp = *agent_heuristic_info.angle_diff_dp_opt;
  const double cur_lat_speed = agent_heuristic_info.cur_lat_speed;
  const double left_bound_l = agent_heuristic_info.left_bound_l;
  const double right_bound_l = agent_heuristic_info.right_bound_l;
  const double agent_half_width = agent_heuristic_info.agent_half_width;
  const auto& agent_type = agent_heuristic_info.agent_type;

  double target_l = sl_pos.l + cur_lat_speed * kLateralSpeedLookAheadTime;
  const double angle_diff_buffer = agent_type == OT_LARGE_VEHICLE
                                       ? kLargeVehicleAngleDiffBuffer
                                       : kNormalAngleDiffBuffer;
  // base conditions
  const bool agent_on_left = sl_pos.l > 0.0;
  const bool agent_on_right = sl_pos.l < 0.0;
  const bool moving_right =
      (cur_lat_speed < -kLatSpeedBuffer || angle_diff_dp > angle_diff_buffer);
  const bool moving_left =
      (cur_lat_speed > kLatSpeedBuffer || angle_diff_dp < -angle_diff_buffer);

  // If the predicted lane path is on the right direction of current lateral
  // speed or angle, clamp the agent box in the lane.
  const bool approching_center =
      (agent_on_left && moving_right) || (agent_on_right && moving_left);
  if (approching_center) {
    target_l = std::clamp(target_l, right_bound_l + agent_half_width,
                          left_bound_l - agent_half_width);
  }

  // If the predicted lane path is on the wrong direction of current lateral
  // speed or angle, maintain current lateral offset.
  if (agent_on_left && moving_left) {
    target_l = std::max(sl_pos.l, target_l);
  }
  if (agent_on_right && moving_right) {
    target_l = std::min(sl_pos.l, target_l);
  }

  return target_l;
}

bool SelectLanePathByModelAndHeuristicInfo(
    const AgentHeuristicInfo& agent_heuristic_info, double dp_score,
    bool is_score_max, bool only_trust_model) {
  const auto& sl_pos_opt = agent_heuristic_info.sl_pos_opt;
  const auto& is_in_lane_opt = agent_heuristic_info.is_in_lane_opt;
  const auto& is_on_boundary_opt = agent_heuristic_info.is_on_boundary_opt;
  const auto& angle_diff_dp_opt = agent_heuristic_info.angle_diff_dp_opt;
  const auto& agent_min_dist_to_bounds =
      agent_heuristic_info.agent_min_dist_to_bounds;
  const auto& agent_type = agent_heuristic_info.agent_type;

  // if heuristic info is not valid, trust the model
  const bool model_selected =
      (dp_score >= lane_selection_net::kTrajGenThreshould) || is_score_max;
  if (only_trust_model || !sl_pos_opt.has_value() ||
      !is_in_lane_opt.has_value() || !is_on_boundary_opt.has_value() ||
      !angle_diff_dp_opt.has_value() || !agent_min_dist_to_bounds.has_value()) {
    return model_selected;
  }

  const double angle_diff_buffer = agent_type == OT_LARGE_VEHICLE
                                       ? kLargeVehicleAngleDiffBuffer
                                       : kNormalAngleDiffBuffer;
  // selection conditions
  const double cur_lat_speed = agent_heuristic_info.cur_lat_speed;
  const bool approaching_lane_center =
      (sl_pos_opt->l > 0.0 && (cur_lat_speed < -kLatSpeedBuffer ||
                               *angle_diff_dp_opt > angle_diff_buffer)) ||
      (sl_pos_opt->l < 0.0 && (cur_lat_speed > kLatSpeedBuffer ||
                               *angle_diff_dp_opt < -angle_diff_buffer));

  const bool approaching_lane_center_hard =
      (sl_pos_opt->l > 0.0 &&
       (cur_lat_speed < -kHardLateralMovementThreshold ||
        *angle_diff_dp_opt > kHardAngleDiffDpThreshold)) ||
      (sl_pos_opt->l < 0.0 &&
       (cur_lat_speed > kHardLateralMovementThreshold ||
        *angle_diff_dp_opt < -kHardAngleDiffDpThreshold));

  // lane keep selection conditions
  constexpr double kLaneKeepAngleThreshold = 5.0 * M_PI / 180.0;  // rad;
  constexpr double kLaneKeepTrajGenThreshould = 0.5;              // prob
  const bool not_on_boundary_lane_keep =
      (std::abs(*angle_diff_dp_opt) < kLaneKeepAngleThreshold) &&
      (dp_score >= kLaneKeepTrajGenThreshould);

  // lane change selection conditions
  constexpr double kMinDpScoreThreshold = 0.2;
  constexpr double kLaneChangeTrajGenThreshould = 0.5;  // prob
  const bool close_lane_change_model_selected =
      (dp_score >= kLaneChangeTrajGenThreshould) || is_score_max ||
      approaching_lane_center_hard;
  const bool not_on_boundary_lane_change =
      approaching_lane_center && dp_score > kMinDpScoreThreshold &&
      ((agent_min_dist_to_bounds > kDefaultHalfLaneWidth &&
        agent_min_dist_to_bounds < 2 * kDefaultHalfLaneWidth &&
        (model_selected || approaching_lane_center_hard)) ||
       (agent_min_dist_to_bounds < kDefaultHalfLaneWidth &&
        close_lane_change_model_selected));

  // Selection logic
  /*
  If the agent is in the current lane path (lane keep prediction):
    1. Trust the model without extra conditions.
    2. The logic should gurantee trajectory continuity when agent is entering
  the lane. Hence, select the lane path if agent is on the boundary and
  approaching lane center.
    3. Minor interference is filterd by selecting the lane path
  if agent is not on the boundary, but its heading aligns with the lane and the
  probability is larger than 0.5,
  */
  if (*is_in_lane_opt) {
    if (model_selected) return true;
    if (*is_on_boundary_opt && approaching_lane_center) return true;
    if (!(*is_on_boundary_opt) && not_on_boundary_lane_keep) return true;
  }

  /*
  If the agent is out of the current lane path (lane change prediction):
    1. If model is on the boundary and approaching the lane center, select the
  lane path.
    2. If model is on the boundary and the model selected the lane path, trust
  it.
    3. If agent is not on the boundary, select the lane path according to the
  nearest distance from agent to both boundaries. If the agent is far away(the
  distance is larger than half lane width), we trust the model with standard
  threshold. However, when the agent is moving fast laterally, we also select
  the lane path. Otherwise, we loosen the probability threshold since the agent
  is close to the target lane.
  */
  if (!(*is_in_lane_opt)) {
    if (*is_on_boundary_opt && (approaching_lane_center || model_selected))
      return true;
    if (!(*is_on_boundary_opt) && not_on_boundary_lane_change) return true;
  }
  return false;
}

std::optional<PredictedTrajectory> DevelopLaneSelectionPrediction(
    const ObjectMotionHistory& obj_hist,
    const planner::DrivePassage& drive_passage, double path_prob,
    double target_l, int idx, double perception_acc, bool is_mapless) {
  const auto& cur_state = obj_hist.states.back();
  std::vector<ObjectMotionState> states;
  states.reserve(obj_hist.states.size());
  const double cur_v = cur_state.vel.norm();
  for (const auto& state : obj_hist.states) {
    if (state.timestamp >= cur_state.timestamp - kHistoryTime) {
      states.push_back(state);
    }
  }

  double accel = 0.0;
  if (FLAGS_only_use_perception_acc) {
    accel = perception_acc;
  } else {
    double fitted_accel =
        LineFitAccelerationByMotionHistory(states, kAccelerationFitTime);
    // At low speed, we try to intentionally under-estimate the brake to avoid
    // ego's overreaction.
    VLOG(2) << "Before underestimation " << fitted_accel;
    if (fitted_accel < 0) {
      fitted_accel *= kLowSpeedUnderestimateRatioPlf(cur_v);
    }
    VLOG(2) << "After underestimation " << fitted_accel;
    accel = fitted_accel;
  }

  const double pred_horizon = IsSlowCutinObj(drive_passage, cur_state)
                                  ? kSlowCutinPredictionDuration
                                  : kComfortableHorizon;
  auto traj_pts_or = GeneratePredictedTrajectoryPoints(
      cur_state, drive_passage, accel, pred_horizon, target_l);
  if (!traj_pts_or.ok() || traj_pts_or->empty()) {
    return std::nullopt;
  }
  auto traj_pts = std::move(traj_pts_or).value();
  const int confident_index = static_cast<int>(traj_pts.size()) - 1;

  const auto extension_info =
      ExtendLaneFollowTraj(drive_passage, is_mapless, pred_horizon, &traj_pts);

  return PredictedTrajectory(
      /*probability=*/path_prob,
      absl::StrCat("lane selection:", drive_passage.lane_path().DebugString(),
                   extension_info),
      PredictionType::PT_LANE_SELECTION_NET, idx, std::move(traj_pts),
      /*is_reversed=*/false, confident_index);
}

inline bool IsObjectBehindAv(const Vec2d& av_pos, const Vec2d& obj_pos,
                             double av_heading) {
  const Vec2d av_unit_vec = Vec2d::FastUnitFromAngle(av_heading);
  const Vec2d vec_av_to_obj = obj_pos - av_pos;
  return av_unit_vec.Dot(vec_av_to_obj) < 0.0;
}

}  // namespace

std::vector<planner::DrivePassage> BuildLaneSelectionObjectDrivePassages(
    const ObjectMotionHistory& obj_hist, const PredictionContext& context,
    bool build_intersection) {
  ScopedMultiTimer timer("Begin to BuildObjectDrivePassage");
  const auto& cur_state = obj_hist.states.back();
  const auto& psmm = context.semantic_map_manager();
  auto lane_paths = FindPossibleLanePathsByCTRATPrediction(cur_state, psmm);
  // Filter out lane paths that are not close enough.
  lane_paths = FilterLanePathByDistance(
      lane_paths, psmm, cur_state.pos, cur_state.heading,
      lane_selection_net::kMaxConsideredLanePathNum);

  timer.Mark("BuildObjectDrivePassage::Build lane paths");

  std::vector<planner::DrivePassage> dps;
  dps.reserve(lane_paths.size());
  for (auto lp : lane_paths) {
    if (!build_intersection) {
      lp = BuildLanePathWithoutVirtualLaneAhead(lp, psmm, cur_state.pos,
                                                cur_state.heading);
    }
    const auto closest_lane_point_or = planner::
        FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
            psmm.GetLevel(), psmm, cur_state.pos, lp, cur_state.heading);
    if (!closest_lane_point_or.ok()) continue;
    const auto pruned_lane_path = PruneLanePathByLength(
        psmm, lp, *closest_lane_point_or, kObjectDrivePassageFrontLength,
        kObjectDrivePassageBackLength);
    auto dp = planner::BuildDrivePassageForPrediction(
        psmm, pruned_lane_path, kDrivePassageStepS, /*avoid_loop=*/true,
        /*backward_extend_len=*/0.0, kMaxLateralBoundaryForPredictionDp);
    if (dp.ok()) {
      VLOG(2) << "Lane path id " << pruned_lane_path.lane_id(0);
      dps.push_back(std::move(*dp));
    }
  }
  VLOG(2) << "lane selection id: " << obj_hist.id
          << " num of candidate dps: " << dps.size();
  return dps;
}

AgentDrivePassagesMap SelectLaneSelectionJ5PredictedObjectsWithDps(
    const Box2d& ego_box,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistorySampler& obj_sampler,
    const PredictionContext& prediction_context,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    ThreadPool* thread_pool) {
  ScopedMultiTimer timer(
      "lane selection net "
      "predictor::SelectLaneSelectionJ5PredictedObjectsWithDps");
  timer.Mark("lane selection net predictor: funtion start");

  FUNC_QTRACE();
  const Vec2d& av_pos = ego_box.center();
  const double av_heading = ego_box.heading();

  // Find candidate objects on road with right types
  std::vector<const ObjectHistory*> considered_lane_selection_objs;
  considered_lane_selection_objs.reserve(objs_to_predict.size());
  for (const ObjectHistory* const obj : objs_to_predict) {
    const Vec2d obj_pos = Vec2d(obj->object_proto().pos());

    if (FLAGS_prediction_lane_selection_ignore_rear_objects &&
        IsObjectBehindAv(av_pos, obj_pos, av_heading)) {
      continue;
    }

    // focus on noa highway scenario, only consider vehicle
    constexpr double kMaxLaneSelectionConsiderAngle =
        30.0 * M_PI / 180.0;  // rad

    const double obj_heading = obj->object_proto().yaw();
    const auto& obj_pred_scenario = object_scenarios.at(obj->id());
    if ((obj->type() == OT_VEHICLE || obj->type() == OT_LARGE_VEHICLE) &&
        obj_pred_scenario.road_status() == ObjectRoadStatus::ORS_ON_ROAD &&
        std::abs(NormalizeAngle(av_heading - obj_heading)) <
            kMaxLaneSelectionConsiderAngle) {
      considered_lane_selection_objs.push_back(obj);
    }
  }
  // timer.Mark("lane selection net predictor: found objests");

  // Sort the objects by distance to av for filtering far objects.
  std::sort(considered_lane_selection_objs.begin(),
            considered_lane_selection_objs.end(),
            [&av_pos](const ObjectHistory* x, const ObjectHistory* y) {
              const auto x_pos = Vec2d(x->object_proto().pos());
              const auto y_pos = Vec2d(y->object_proto().pos());
              return x_pos.DistanceSquareTo(av_pos) <
                     y_pos.DistanceSquareTo(av_pos);
            });
  // timer.Mark("lane selection net predictor: sorted objects");

  // Get possible lane paths for each objects
  AgentDrivePassagesMap result;

  const int pred_obj_num =
      std::min(lane_selection_net::kLaneSelectionMaxPredObjNum,
               static_cast<int>(considered_lane_selection_objs.size()));
  using IdDpsPair =
      std::pair<ObjectIDType, std::vector<const planner::DrivePassage*>>;
  std::vector<IdDpsPair> all_dps;
  all_dps.resize(pred_obj_num);

  ParallelFor(0, pred_obj_num, thread_pool, [&](int i) {
    const auto& agent_hist = considered_lane_selection_objs[i];
    const auto& agent_id = agent_hist->id();
    const auto& agent_history =
        obj_sampler.GetResampledMotionHistoryById(agent_id);
    // timer.Mark("lane selection net predictor: build dp start");

    constexpr double kMaxOffset = 3.5;  // m.
    constexpr double kMaxHeadingDiff = M_PI / 3.0;
    constexpr int kMaxDpNum = 3;

    const auto& cur_state = agent_history.states.back();
    auto filtered_dps = FindNearbyDrivePassages(
        prediction_context.drive_passages(), cur_state.pos, cur_state.heading,
        kMaxOffset, kMaxHeadingDiff, kMaxDpNum);

    // timer.Mark("lane selection net predictor: build dp end");

    all_dps[i] = std::make_pair(agent_id, filtered_dps);
  });

  for (auto& id_dps_pair : all_dps) {
    if (id_dps_pair.second.size() > 0) {
      result[id_dps_pair.first] = std::move(id_dps_pair.second);
    }
  }

  timer.Mark("lane selection net predictor: dp generated");

  if (FLAGS_print_prediction_time_stats) {
    planner::PrintMultiTimerReportStat(timer);
  }
  return result;
}

AgentHeuristicInfo PrepareHeuristicInfo(
    const ObjectHistorySampler& obj_sampler, const ObjectIDType& agent_id,
    const planner::DrivePassage& drive_passage) {
  // Get agent history states
  const auto& agent_motion_hist =
      obj_sampler.GetResampledMotionHistoryById(agent_id);
  const auto& cur_state = agent_motion_hist.states.back();
  std::vector<ObjectMotionState> states;
  states.reserve(agent_motion_hist.states.size());
  for (const auto& state : agent_motion_hist.states) {
    if (state.timestamp >= cur_state.timestamp - kHistoryTime) {
      states.push_back(state);
    }
  }

  AgentHeuristicInfo result;
  // get agent type
  result.agent_type = agent_motion_hist.type;
  // get agent current speed
  const double cur_lat_speed = LineFitLateralSpeedByMotionHistory(
      states, drive_passage, kHistoryTime, /*clamp_by_lane_width=*/false);
  result.cur_lat_speed =
      std::clamp(cur_lat_speed, -kLateralSpeedClamp, kLateralSpeedClamp);

  // get agent width
  result.agent_half_width = cur_state.bbox.half_width();

  // prepare agent current sl pos for later usage
  const auto& pos = cur_state.pos;
  const auto sl_or = drive_passage.QueryFrenetCoordinateAt(pos);
  if (sl_or.ok()) {
    const auto& sl = sl_or.value();
    result.sl_pos_opt = sl;

    // get angle diff to lane path
    const auto path_angle_or = drive_passage.QueryTangentAngleAtS(sl_or->s);
    if (path_angle_or.ok()) {
      result.angle_diff_dp_opt =
          NormalizeAngle(path_angle_or.value() - cur_state.heading);
    }

    // get l offset of left and rigt boundaries
    const auto bounds_or =
        drive_passage.QueryNearestBoundaryLateralOffset(sl.s);
    if (bounds_or.ok()) {
      result.left_bound_l = std::min(result.left_bound_l, bounds_or->second);
      result.right_bound_l = std::max(result.right_bound_l, bounds_or->first);
    }

    // get agent dist to left and right boundaries
    const double agent_offset_to_l_bound = sl_or->l - result.left_bound_l;
    const double agent_offset_to_r_bound = sl_or->l - result.right_bound_l;
    result.agent_min_dist_to_bounds = std::min(
        std::abs(agent_offset_to_l_bound), std::abs(agent_offset_to_r_bound));

    // get agent is_in_lane info
    result.is_in_lane_opt =
        (sl_or->l < result.left_bound_l) && (sl_or->l > result.right_bound_l);

    // get agent is_on_boundary info.
    const double dist_to_bound_buffer =
        agent_motion_hist.type == OT_LARGE_VEHICLE
            ? kLargeVehicleDistToBoundBuffer
            : kNormalDistToBoundBuffer;

    const auto fbox_or = drive_passage.QueryFrenetBoxAt(cur_state.bbox);
    if (fbox_or.ok()) {
      result.is_on_boundary_opt =
          (sl_or->l < result.right_bound_l &&
           fbox_or->l_max >= result.right_bound_l - dist_to_bound_buffer) ||
          (sl_or->l > result.left_bound_l &&
           fbox_or->l_min <= result.left_bound_l + dist_to_bound_buffer);
    }
  }
  return result;
}

ObjectsLaneSelectionNetPredMap
GenerateLaneSelectionTrajsByModelAndHeuristicInfo(
    const AgentDrivePassagesMap& agent_dps_map,
    const LaneSelectionInferencerOutputMap& infer_result_map,
    const ObjectHistorySampler& obj_sampler,
    const PredictionContext& prediction_context, bool is_mapless,
    ThreadPool* thread_pool) {
  ObjectsLaneSelectionNetPredMap res;

  std::vector<ObjectIDType> output_ids;
  output_ids.reserve(infer_result_map.size());
  for (const auto& agent_with_dps_scores : infer_result_map) {
    output_ids.push_back(agent_with_dps_scores.first);
  }

  using IdOutputsPair = std::pair<ObjectIDType, ObjectLaneSelectionNetPred>;
  std::vector<IdOutputsPair> all_outputs;
  all_outputs.resize(output_ids.size());

  ParallelFor(0, output_ids.size(), thread_pool, [&](int i) {
    const auto& agent_id = output_ids[i];
    const auto& drive_passages = agent_dps_map.at(agent_id);
    const auto& infer_result = infer_result_map.at(agent_id);

    if (drive_passages.size() <= 0) {
      return;
    }

    // skip the agent if any result is invalid
    const int no_valid_num =
        std::count_if(infer_result.is_valid.begin(),
                      infer_result.is_valid.end(), [](bool x) { return !x; });
    if (no_valid_num > 0) {
      return;
    }

    // Prepare needed vars.
    ObjectLaneSelectionNetPred obj_output;
    const int max_score_idx = max_element(infer_result.dp_scores.begin(),
                                          infer_result.dp_scores.end()) -
                              infer_result.dp_scores.begin();
    const auto& obj_hist = obj_sampler.GetResampledMotionHistoryById(agent_id);
    const auto& obj_proto = prediction_context.object_history_manager()
                                .at(obj_hist.id)
                                .object_proto();

    const auto perception_accel_vec = Vec2d(obj_proto.accel());
    const auto perception_vel_vec = Vec2d(obj_proto.vel());
    double perception_acc = 0.0;
    if (perception_vel_vec.norm() > 0) {
      perception_acc = perception_accel_vec.dot(perception_vel_vec.Unit());
    }

    for (int i = 0; i < drive_passages.size(); ++i) {
      const auto& dp = *drive_passages[i];
      // Trajectory generation logic.
      double dp_score = infer_result.dp_scores[i];
      const auto agent_heuristic_info =
          PrepareHeuristicInfo(obj_sampler, agent_id, dp);
      const bool is_selected = SelectLanePathByModelAndHeuristicInfo(
          agent_heuristic_info, dp_score, i == max_score_idx,
          /*only_trust_model=*/false);
      if (is_selected) {
        const double target_l = CalcLaneSelectionTargetL(agent_heuristic_info);
        if (dp_score < kMinimalDpProb) dp_score = 0.3;
        auto pred_traj_opt = DevelopLaneSelectionPrediction(
            obj_hist, dp, dp_score, target_l, i, perception_acc, is_mapless);
        if (pred_traj_opt == std::nullopt) {
          continue;
        }
        const auto traj_type = AnalyzeTrajType(dp, obj_sampler, agent_id);
        pred_traj_opt->set_lane_selection_traj_type(traj_type);
        obj_output.pred_trajs.push_back(std::move(*pred_traj_opt));
      }
    }

    // Add safe guard brake trajectory.
    // Compute perception acc.
    // Try to generate brake traj.
    auto brake_traj_or = CreateSafeGuardBrakeTrajectory(
        obj_hist, obj_output.pred_trajs, perception_acc,
        prediction_context.av_context().GetAvCurrentSpeed(),
        PredictionType::PT_LANE_SELECTION_NET);
    if (brake_traj_or.has_value()) {
      obj_output.pred_trajs.push_back(brake_traj_or.value());
    }

    if (obj_output.pred_trajs.size() > 0) {
      all_outputs[i] = std::make_pair(agent_id, obj_output);
      NormalizeAndDescSortTrajProbs(
          absl::MakeSpan(all_outputs[i].second.pred_trajs));
    }
  });

  for (auto& id_output_pair : all_outputs) {
    if (id_output_pair.second.pred_trajs.size() > 0) {
      res[id_output_pair.first] = std::move(id_output_pair.second);
    }
  }

  return res;
}

ObjectsLaneSelectionNetPredMap MakeLaneSelectionNetJ5Prediction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const lane_selection_net::LaneSelectionNetJ5Inferencer&
        lane_selection_net_j5_inferencer,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool) {
  SCOPED_QTRACE("MakeLaneSelectionNetJ5Prediction");
  ScopedMultiTimer timer(
      "LaneSelectionNet predictor::MakeLaneSelectionNetJ5Prediction");
  const auto& av_context = prediction_context.av_context();
  const ObjectHistorySpan av_history =
      av_context.GetAvObjectHistory().GetHistory();
  const auto agent_dps_map = SelectLaneSelectionJ5PredictedObjectsWithDps(
      av_history.back().val.bounding_box(), objects_history, obj_sampler,
      prediction_context, object_scenarios, thread_pool);
  timer.Mark("LaneSelectionNet predictor::Select LaneSelectionNet objects.");
  VLOG(3) << "Number of candidate LaneSelectionNet Objects is "
          << agent_dps_map.size();

  const auto infer_result_map =
      lane_selection_net_j5_inferencer.PredictForObjects(obj_sampler,
                                                         agent_dps_map);

  timer.Mark("LaneSelectionNet predictor::Inference.");

  ObjectsLaneSelectionNetPredMap res;
  {
    SCOPED_QTRACE("LaneSelectionNet Predictor::PostProcessingTrajs");
    res = GenerateLaneSelectionTrajsByModelAndHeuristicInfo(
        agent_dps_map, infer_result_map, obj_sampler, prediction_context,
        IsOnlineMapMode(prediction_context.autonomy_state()), thread_pool);
  }

  return res;
}

}  // namespace prediction
}  // namespace qcraft
