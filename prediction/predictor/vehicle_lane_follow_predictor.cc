#include "onboard/prediction/predictor/vehicle_lane_follow_predictor.h"

#include <algorithm>  // for max, min, clamp, sort
#include <cmath>
#include <limits>
#include <memory>
#include <optional>  // for optional
#include <ostream>   // for basic_ostream::operator<<, operator<<, basic_ostream
#include <string>    // for string, operator<<
#include <utility>   // for move, pair
#include <vector>    // for vector, allocator

#include "absl/status/statusor.h"  // for StatusOr
#include "absl/strings/str_cat.h"  // for StrCat
#include "absl/types/span.h"       // for Span
#include "gflags/gflags.h"         // for DEFINE_bool
#include "glog/logging.h"  // for COMPACT_GOOGLE_LOG_INFO, LogMessage, VLOG

#include "onboard/global/buffered_logger.h"  // for BufferedLoggerWrapper
#include "onboard/global/timer.h"            // for ScopedMultiTimer
#include "onboard/global/trace.h"            // for SCOPED_QTRACE, ScopedTrace
#include "onboard/lite/check.h"              // for QCHECK_GT
#include "onboard/maps/lane_path.h"          // for LanePath
#include "onboard/maps/maps_common.h"        // for LaneInfo
#include "onboard/maps/proto/semantic_map.pb.h"  // for LaneProto
#include "onboard/math/fast_math.h"              // for Sin
#include "onboard/math/frenet_common.h"       // for FrenetCoordinate, FrenetBox
#include "onboard/math/geometry/box2d.h"      // for Box2d
#include "onboard/math/geometry/segment2d.h"  // for Segment2d
#include "onboard/math/piecewise_linear_function.h"  // for PiecewiseLinearFunction
#include "onboard/math/util.h"  // for Sigmoid, NormalizeAngle
#include "onboard/math/vec.h"   // for Vec2d, MatrixBase::norm
#include "onboard/planner/common/multi_timer_util.h"  // for PrintMultiTimerReportStat
#include "onboard/planner/planner_semantic_map_manager.h"  // for PlannerSemanticMapManager
#include "onboard/planner/router/drive_passage.h"  // for DrivePassage
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/prediction/container/av_context.h"       // for AvContext
#include "onboard/prediction/container/object_history.h"   // for ObjectHistory
#include "onboard/prediction/container/objects_history.h"  // for ObjectsHistory
#include "onboard/prediction/container/prediction_object.h"  // for PredictionObject
#include "onboard/prediction/prediction_defs.h"  // for ObjectMotionState, ObjectMotionHistory, kPredictionTimeStep
#include "onboard/prediction/prediction_flags.h"  // for FLAGS_only_use_perception_acc
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/predictor/l2_lane_follow_predictor.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/prediction/util/kinematic_model.h"  // for ObjectMotionStateToUniCycleState, BicycleModelState
#include "onboard/prediction/util/lane_follow_util.h"
#include "onboard/prediction/util/safe_guard_util.h"
#include "onboard/prediction/util/trajectory_developer.h"  // for CombinePathAndSpeedForPredictedTrajectoryPoints, DevelopC...
#include "onboard/proto/perception.pb.h"     // for ObjectProto
#include "onboard/proto/prediction.pb.h"     // for PredictionType
#include "onboard/utils/elements_history.h"  // for Node

namespace qcraft {
namespace prediction {
namespace {
DEFINE_bool(print_vehicle_lane_follow_time_stats, false,
            "Print vehicle lane follow time stats.");
/*
 *  Final trajectory output related consts.
 */
constexpr int kMaxPrediction = 2;
constexpr double kKeepProb = 0.5;
// Pure pursuit path generation related parameters.
constexpr double kBikeLanePenalty = 0.1;
constexpr double kEmergencyLanePenalty = 0.2;
constexpr int kMinConfidenceIndex = 10;

/*
 * Cost is computed as the sum of predicted lateral offset cost and current
 * lateral offset cost. The offset cost is calculated as a sigmoid of vehicle's
 * lateral offset to the center line. if the lateral offset equals to the half
 * lane width, the cost is 0.5.
 */

bool ObjAheadAv(const Box2d& av_box, const Box2d& obj_box) {
  const auto& pts = obj_box.GetCornersCounterClockwise();
  const auto& av_seg =
      Segment2d(av_box.RearCenterPoint(), av_box.FrontCenterPoint());
  double seg_length = av_seg.length();
  for (const auto& pt : pts) {
    const double proj = av_seg.ProjectOntoUnit(pt);
    if (proj > seg_length) {
      return true;
    }
  }
  return false;
}

struct DrivePassageRankInfo {
  const planner::DrivePassage* dp;
  double prob = 0.0;
};

double PredictedLateralOffsetProb(absl::Span<const ObjectMotionState> hist,
                                  const planner::DrivePassage& dp,
                                  double lateral_speed_clamp,
                                  double lateral_speed_pred_time) {
  const auto& pos = hist.back().pos;
  const auto sl_or = dp.QueryFrenetCoordinateAt(pos);
  if (!sl_or.ok()) {
    return 0.0;
  }
  const auto& sl = sl_or.value();
  const auto bounds_or = dp.QueryNearestBoundaryLateralOffset(sl.s);
  double min_l = -kDefaultHalfLaneWidth;
  double max_l = kDefaultHalfLaneWidth;
  if (bounds_or.ok()) {
    min_l = std::max(min_l, bounds_or->first);
    max_l = std::min(max_l, bounds_or->second);
  }

  double cur_lat_speed = LineFitLateralSpeedByMotionHistory(
      hist, dp, kHistoryTime, /*clamp_by_lane_width=*/true);
  cur_lat_speed =
      std::clamp(cur_lat_speed, -lateral_speed_clamp, lateral_speed_clamp);
  double target_l = sl.l + cur_lat_speed * lateral_speed_pred_time;
  if (sl.l > 0) {
    target_l = std::max(min_l, target_l);
  } else {
    target_l = std::min(max_l, target_l);
  }

  double dis_to_bound = 0.0;
  if (target_l < 0) {
    dis_to_bound = target_l - min_l;
  } else {
    dis_to_bound = max_l - target_l;
  }

  return Sigmoid(dis_to_bound);
}

double CurrentLateralOffsetProb(absl::Span<const ObjectMotionState> hist,
                                const planner::DrivePassage& dp) {
  const auto& cur_state = hist.back();
  const auto& pos = cur_state.pos;
  const auto sl_or = dp.QueryFrenetCoordinateAt(pos);
  if (!sl_or.ok()) {
    return 0.0;
  }
  double cur_lateral_offset = std::numeric_limits<double>::max();
  cur_lateral_offset = sl_or->l;
  const auto ref_angle_or = dp.QueryTangentAngleAtS(sl_or->s);
  double angle_diff = 0.0;
  if (ref_angle_or.ok()) {
    angle_diff = NormalizeAngle(cur_state.heading - ref_angle_or.value());
  }
  const double length = cur_state.bbox.length();
  cur_lateral_offset += (0.5 * length * fast_math::Sin(angle_diff));
  const auto bounds_or = dp.QueryNearestBoundaryLateralOffset(sl_or->s);
  double min_l = -kDefaultHalfLaneWidth;
  double max_l = kDefaultHalfLaneWidth;
  if (bounds_or.ok()) {
    min_l = std::max(min_l, bounds_or->first);
    max_l = std::min(max_l, bounds_or->second);
  }
  double dis_to_bound = 0.0;
  if (cur_lateral_offset < 0) {
    dis_to_bound = cur_lateral_offset - min_l;
  } else {
    dis_to_bound = max_l - cur_lateral_offset;
  }

  return Sigmoid(dis_to_bound);
}
double ComputeDrivePassageProb(absl::Span<const ObjectMotionState> hist,
                               const planner::PlannerSemanticMapManager& psmm,
                               const planner::DrivePassage& dp,
                               double lateral_speed_clamp,
                               double lateral_speed_pred_time) {
  const double future_prob = PredictedLateralOffsetProb(
      hist, dp, lateral_speed_clamp, lateral_speed_pred_time);
  const double cur_prob = CurrentLateralOffsetProb(hist, dp);
  double penalty = 1.0;
  const auto lane_infos = GetLanesInfoBreakIfNotFound(psmm, dp.lane_path());
  for (const auto* lane_info : lane_infos) {
    if (lane_info->Type() == mapping::LaneProto::BICYCLE_ONLY) {
      penalty = std::min(penalty, kBikeLanePenalty);
    }
    if (lane_info->Type() == mapping::LaneProto::EMERGENCY) {
      penalty = std::min(penalty, kEmergencyLanePenalty);
    }
  }
  return penalty * std::max(future_prob, cur_prob);
}
std::vector<DrivePassageRankInfo> RankObjectDrivePassages(
    const ObjectMotionHistory& obj_hist,
    const planner::PlannerSemanticMapManager& psmm,
    absl::Span<const std::unique_ptr<planner::DrivePassage>> dps) {
  ScopedMultiTimer timer("Begin to BuildObjectDrivePassage");
  const auto& cur_state = obj_hist.states.back();
  constexpr double kMaxOffset = 3.5;  // m.
  constexpr double kMaxHeadingDiff = M_PI / 3.0;
  constexpr int kMaxDpNum = 4;
  // sorted from closest to farest
  auto filtered_dps =
      FindNearbyDrivePassages(dps, cur_state.pos, cur_state.heading, kMaxOffset,
                              kMaxHeadingDiff, kMaxDpNum);
  filtered_dps =
      FilterDrivePassagesByCTRA(filtered_dps, cur_state, kMaxPrediction);
  std::vector<ObjectMotionState> states;
  states.reserve(obj_hist.states.size());
  for (const auto& state : obj_hist.states) {
    if (state.timestamp >= cur_state.timestamp - kHistoryTime) {
      states.push_back(state);
    }
  }

  std::vector<DrivePassageRankInfo> dp_ranks;
  dp_ranks.reserve(filtered_dps.size());
  for (const auto* dp : filtered_dps) {
    const double prob = ComputeDrivePassageProb(
        states, psmm, *dp, kLateralSpeedClamp, kLateralSpeedLookAheadTime);
    dp_ranks.push_back({.dp = dp, .prob = prob});
  }

  std::sort(dp_ranks.begin(), dp_ranks.end(),
            [](const auto& a, const auto& b) { return a.prob > b.prob; });

  return dp_ranks;
}

std::vector<PredictedTrajectory> DevelopVehicleLaneFollowPrediction(
    const ObjectMotionHistory& obj_hist, const PredictionContext& context,
    const ObjectPredictionScenario& scenario, double perception_acc,
    bool ignore_off_road) {
  ScopedMultiTimer timer(
      "VehicleLaneFollow::DevelopVehicleLaneFollowPrediction");

  const auto& cur_state = obj_hist.states.back();
  std::vector<ObjectMotionState> states;
  states.reserve(obj_hist.states.size());
  for (const auto& state : obj_hist.states) {
    if (state.timestamp >= cur_state.timestamp - kHistoryTime) {
      states.push_back(state);
    }
  }

  const bool is_mapless = IsOnlineMapMode(context.autonomy_state());
  auto dp_ranks = RankObjectDrivePassages(
      obj_hist, context.semantic_map_manager(), context.drive_passages());

  // Only keep dps with prob larger than 0.5 or keep the highest if all smaller
  // than 0.5;
  int erase_idx = 1;
  // Keep maximally kMaxPrediction predictions.
  for (; erase_idx < std::min<int>(dp_ranks.size(), kMaxPrediction);
       ++erase_idx) {
    if (dp_ranks[erase_idx].prob < kKeepProb) {
      break;
    }
  }
  if (erase_idx < dp_ranks.size()) {
    dp_ranks.resize(erase_idx);
  }
  timer.Mark("VehicleLaneFollow::Build & evaluate drive passages");
  std::vector<PredictedTrajectory> res;
  res.reserve(dp_ranks.size());
  double sum_prob = 0.0;
  const double cur_v = cur_state.vel.norm();
  for (int idx = 0, n = dp_ranks.size(); idx < n; ++idx) {
    const auto& dp_rank = dp_ranks[idx];
    VLOG(2) << "car_id =" << obj_hist.id;
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

    double pred_horizon = kComfortableHorizon;
    const auto is_slow_cutin = IsSlowCutinObj(*dp_rank.dp, cur_state);
    if (is_slow_cutin) {
      pred_horizon = kSlowCutinPredictionDuration;
    }

    const auto target_l_or = CalculateLaneFollowTargetL(
        states, *dp_rank.dp, obj_hist.type,
        scenario.intersection_status() == OIS_IN_INTERSECTION);
    const double target_l = target_l_or.ok() ? target_l_or.value() : 0.0;

    auto traj_pts_or = GeneratePredictedTrajectoryPoints(
        cur_state, *dp_rank.dp, accel, pred_horizon, target_l);
    if (!traj_pts_or.ok() || traj_pts_or->empty()) {
      continue;
    }
    auto traj_pts = std::move(traj_pts_or).value();
    const int confident_index = static_cast<int>(traj_pts.size()) - 1;
    const auto extension_info =
        ExtendLaneFollowTraj(*dp_rank.dp, is_mapless, pred_horizon, &traj_pts);

    res.push_back(PredictedTrajectory(
        /*probability=*/dp_rank.prob,
        absl::StrCat("Veh lane follow:", dp_rank.dp->lane_path().DebugString(),
                     extension_info),
        PredictionType::PT_VEHICLE_LANE_FOLLOW, idx, std::move(traj_pts),
        /*is_reversed=*/false, confident_index));
    sum_prob += dp_rank.prob;
  }
  timer.Mark("VehicleLaneFollow::Generate trajectories");
  if (FLAGS_print_vehicle_lane_follow_time_stats) {
    planner::PrintMultiTimerReportStat(timer);
  }
  if (res.empty()) {
    const auto& av = context.av_context().GetAvObjectHistory().back().val;
    const auto& av_box = av.bounding_box();
    const auto& obj_box = cur_state.bbox;
    const bool obj_ahead_av = ObjAheadAv(av_box, obj_box);
    if ((scenario.road_status() == ORS_OFF_ROAD || (!obj_ahead_av)) &&
        ignore_off_road) {
      auto void_traj_pts =
          DevelopVoidTrajectory(ObjectMotionStateToUniCycleState(cur_state));
      return {PredictedTrajectory(
          /*probability=*/1.0, "Veh lane follow: VOID", PredictionType::PT_VOID,
          0, std::move(void_traj_pts),
          /*is_reversed=*/false, /*last_confident_index=*/0)};
    }

    // HACK(xiangjun): Try double direction lane follow prediction.
    const double heading_diff =
        std::fabs(NormalizeAngle(cur_state.heading - av.heading()));
    // If an object is moving toward our direction, try use a bicycle lane
    // follow prediction.
    if (heading_diff > M_PI * 3.0 / 4.0) {
      return {MakeAssitDrivingLaneFollowPrediction(obj_hist, context)};
    }

    double accel = 0.0;
    if (FLAGS_only_use_perception_acc) {
      accel = perception_acc;
    } else {
      accel = std::clamp(LineFitAccelerationByMotionHistory(
                             obj_hist.states, kAccelerationFitTime),
                         kMaxVehicleBrake, kMaxVehicleAcc);
    }
    auto ctra_state = ObjectMotionStateToUniCycleState(obj_hist.states.back());
    ctra_state.acc = accel;
    auto ctra_traj_pts = DevelopForwardCTRATrajectory(
        ctra_state, kPredictionTimeStep, kMaintainAccTime, kMaintainAccTime,
        kSafeHorizon);
    return {PredictedTrajectory(
        /*probability=*/1.0, "Veh lane follow: CTRA (no lane path)",
        PredictionType::PT_VEHICLE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false, /*last_confident_index=*/kMinConfidenceIndex)};
  }

  QCHECK_GT(sum_prob, 0.0);
  for (auto& traj : res) {
    traj.set_probability(traj.probability() / sum_prob);
  }
  return res;
}
}  // namespace

std::vector<PredictedTrajectory> MakeVehicleLaneFollowPrediction(
    const ObjectMotionHistory& obj_hist, const PredictionContext& context,
    const ObjectPredictionScenario& scenario, bool ignore_off_road) {
  SCOPED_QTRACE("MakeVehicleLaneFollowPrediction");
  const auto& obj_proto =
      context.object_history_manager().at(obj_hist.id).object_proto();
  // Compute perception acc.
  const auto perception_accel_vec = Vec2d(obj_proto.accel());
  const auto perception_vel_vec = Vec2d(obj_proto.vel());
  double perception_acc = 0.0;
  if (perception_vel_vec.norm() > 0) {
    perception_acc = perception_accel_vec.dot(perception_vel_vec.Unit());
  }
  auto trajs = DevelopVehicleLaneFollowPrediction(
      obj_hist, context, scenario, perception_acc, ignore_off_road);
  auto brake_traj_or = CreateSafeGuardBrakeTrajectory(
      obj_hist, trajs, perception_acc, context.av_context().GetAvCurrentSpeed(),
      PredictionType::PT_VEHICLE_LANE_FOLLOW);
  if (brake_traj_or.has_value()) {
    trajs.push_back(brake_traj_or.value());
    NormalizeAndDescSortTrajProbs(absl::MakeSpan(trajs));
  }
  return trajs;
}
}  // namespace prediction
}  // namespace qcraft
