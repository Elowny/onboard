#include "onboard/planner/selector/traj_cost_features.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/lite/check.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

#define btoa(x) ((x) ? "true" : "false")
#define btof(x) ((x) ? 1.0 : 0.0)

namespace qcraft::planner {

namespace {

constexpr double kMinStandardProgressDiff = 50.0;     // m.
constexpr double kMinLaneSpeedLimit = 10.0;           // m/s.
constexpr double kHighWayLowSpeedLowerBound = 5.56;   // m/s.
constexpr double kHighWayLowSpeedUpperBound = 11.11;  // m/s.
constexpr double kRadicalPorgressFactor = 2.0;
constexpr double kConservativePorgressFactor = 0.5;
constexpr double kLaneSpeedDiffToLimitRatio = 0.4;  // s.
constexpr double kCutOffLaneSpeedDiffRatio = 0.1;
constexpr double kBeginLaneSpeedDiffRatioForHighway = 0.15;
constexpr double kForceLaneChangeDiffRatioForHighway = 0.2;
constexpr double kHighwayProgressFactor = 1.5;
constexpr double kBeginLaneSpeedDiffCostFactor = 0.35;
constexpr double kForceLaneChangeDiffCostFactor = 0.7;
constexpr double kSpeedIncrementUpperBoundRatio = 0.3;
constexpr double kMinFollowLeaderTime = 2.0;        // s.
constexpr double kMinFollowDistance = 10.0;         // m.
constexpr double kMinSpeedIncrementUpperBound = 4;  // m/s
constexpr double kSpeedIncrementLowerBoundRatio = 0.1;
constexpr double kMinSpeedIncrementLowerBound = 1;  // m/s
constexpr double kMaxOppositeLcTimeBound = 50.0;    // s.
constexpr double kMinOppositeLcTimeBound = 30.0;    // s.
constexpr double kRedLightSpeedBase = 6.0;          // m/s.
constexpr double kRedLightWaitingTimeBase = 20.0;   // s.
constexpr double kRedLightWaitingLeaderFactor = 0.2;
constexpr double kMaxRedLightDistance = 90.0;  // m.
constexpr double kStandardAverageTrajDiff = 1.6 * kDefaultHalfLaneWidth;
constexpr double kCrossBoundaryFactor = 0.6;
constexpr double kLcEffectBase = 2.0;                     // m/s^2
constexpr double kReachDestinationCutOffDist = 100.0;     // m.
constexpr double kRouteLengthCutOffDist = 2000.0;         // m.
constexpr double kBeginRouteLengthLcForHighWay = 1500.0;  // m.
constexpr double kBeginRouteTtcForHighWay = 30.0;         // s.
constexpr double kForceRouteLengthLcForHighWay = 100.0;   // m.
constexpr double kForceRouteTtcForHighWay = 20.0;         // s.
constexpr double kBeginRouteLcCostFactorForHighWay = 0.35;
constexpr double kForceRouteLcCostFactorForHighWay = 1.3;
constexpr double kBeginRouteTtcForHighWayConservative = 60.0;  // s.
constexpr double kForceRouteTtcForHighWayConservative = 50.0;  // s.
constexpr double kConsiderTtcSpeedLowerBound = 5.0;            // m/s.
constexpr double kNonOccludingLeaderFactor = 2.0;
constexpr double kDivergeAngleDiff = 0.05;
constexpr double kMaxDivergeDistance = 300.0;            // m.
constexpr double kConsiderPrevTrajTime = 4.0;            // s.
constexpr double kBackCheckCurveDistance = 20.0;         // m.
constexpr double kCalculateCurvatureStep = 4.0;          // m.
constexpr double kLengthAlongRouteBaseForLeft = 800.0;   // m.
constexpr double kLengthAlongRouteBaseForRight = 700.0;  // m.
constexpr double kMinLenForLaneChange = 400.0;           // m.
constexpr double kMinLenForLaneChangeHighWay = 1500.0;   // m.
constexpr double kMergeLaneLengthBaseHighWay = 4500.0;   // m.
constexpr double kMergeLaneLengthBase = 1500.0;          // m.
constexpr double kStalledObjectLengthBase = 100.0;       // m.
constexpr double kLengthAlongRouteObviousThreshold = 0.8;
constexpr double kLengthBeforeMergeLaneObviousThreshold = 0.9;
constexpr double kBehindStalledObjObviousThreshold = 0.5;
constexpr double kBeginLcLengthAlongRouteThreshold = 0.3;
constexpr double kBeginLcForPreviewThreshold = 0.9;
constexpr double kBeginLcForDiscourageRightMostThreshold = 0.6;
constexpr double kBeginLcForMergeLaneThreshold = 0.2;
constexpr double kDiscourageRightMostSpeedLowerBound = 11.11;  // m/s.
constexpr double kDiscourageRightMostSpeedUpperBound = 22.22;  // m/s.
constexpr double kCurveSpeedConstraint = 0.15;
constexpr double kConsiderMinLcNumToTarget = 2;
constexpr double kForbidBehaviorCost = 1000.0;
constexpr double kMaxDecelerationAcc = -1.0;  // m/s^2
constexpr double kMinDecelerationAcc = -3.0;  // m/s^2
constexpr double kDistanceEpsilon = 1.0;      // m.

}  // namespace

absl::StatusOr<CostVec> TrajProgressCost::ComputeCost(
    const EstPlannerOutput& planner_output,
    std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  CostVec cost_vec(size());
  QCHECK_NOTNULL(extra_info);
  if (!planner_enable_obstacle_lane_change_) {
    extra_info->emplace_back(absl::StrFormat("Disable obstacle lane change"));
    return cost_vec;
  }

  const bool in_high_way = common_feature()->in_high_way;
  const auto& scheduler_output = planner_output.scheduler_output;
  const auto& lane_feature_info = FindOrDieNoPrint(
      common_feature()->lane_feature_infos, scheduler_output.Hash());
  const auto lc_stage =
      planner_output.scheduler_output.lane_change_state.stage();
  const bool lc_ongoing = lc_stage == LaneChangeStage::LCS_EXECUTING ||
                          lc_stage == LaneChangeStage::LCS_RETURN ||
                          lc_stage == LaneChangeStage::LCS_PAUSE;
  // 1. Compute the trajectory length.
  if (planner_output.traj_points.size() < 1) {
    return absl::InternalError("TrajProgressCost: traj_pts.size() < 1");
  }
  ASSIGN_OR_RETURN(
      const auto point_proj,
      scheduler_output.drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
          Vec2dFromApolloTrajectoryPointProto(
              planner_output.traj_points.back())),
      _ << "Last traj point not on drive passage.");
  const double standard_progress =
      std::max(kMinStandardProgressDiff, kTrajectoryTimeHorizon * ego_v_);

  cost_vec.at(0) =
      std::max(0.0, standard_progress - point_proj.s) / standard_progress;

  // 2. Regularization is related to lane speed limit.
  const auto [init_leader_dist, init_leader_speed, max_leader_speed,
              lane_speed_limit_by_leader, lane_speed_limit, block_id] =
      FindOrDieNoPrint(lane_speed_map_, scheduler_output.Hash());
  // Consider different driving style.
  const auto driving_style_factor =
      lane_change_style_ == LaneChangeStyle::LC_STYLE_NORMAL
          ? 1.0
          : (lane_change_style_ == LaneChangeStyle::LC_STYLE_RADICAL
                 ? kRadicalPorgressFactor
                 : kConservativePorgressFactor);

  // Avoid unnecessary lc when leader v is far higher than ego.
  const auto speed_lower_bound =
      std::max(kMinSpeedIncrementLowerBound,
               kSpeedIncrementLowerBoundRatio * lane_speed_limit);
  const auto speed_upper_bound =
      std::max(kMinSpeedIncrementUpperBound,
               kSpeedIncrementUpperBoundRatio * lane_speed_limit);
  const double speed_increment =
      std::min(std::max(speed_lower_bound, init_leader_speed - ego_v_),
               speed_upper_bound);

  const double slow_leader_factor = (speed_upper_bound - speed_increment) /
                                    (speed_upper_bound - speed_lower_bound);
  double slow_av_factor = 1.0;
  if (in_high_way) {
    // Avoid overtake lc in low speed in highway.
    // If need lc in low speed, driver can choose paddle lane change.
    if (ego_v_ < kHighWayLowSpeedLowerBound) {
      slow_av_factor = 0.0;
    } else if (ego_v_ < kHighWayLowSpeedUpperBound) {
      slow_av_factor = LinearInterpolate(0.0, 1.0, kHighWayLowSpeedLowerBound,
                                         kHighWayLowSpeedUpperBound, ego_v_);
    } else {
      slow_av_factor = 1.0;
    }
  }

  double target_near_leader_factor = 1.0;
  const bool is_nearest_leader =
      init_leader_dist <= min_leader_dist_ + kDistanceEpsilon;
  if (in_high_way && !is_nearest_leader && !lc_ongoing) {
    // When there is fast but near leader in target lane.
    // Avoid unnecessary progress cost for lk.
    const double min_follow_distance =
        std::max(kMinFollowDistance, kMinFollowLeaderTime * ego_v_);
    target_near_leader_factor =
        std::clamp(min_leader_dist_ / min_follow_distance, 0.0, 1.0);
  }

  const double lane_speed_diff_ratio =
      driving_style_factor * slow_leader_factor * slow_av_factor *
      target_near_leader_factor *
      (max_lane_speed_ - lane_speed_limit_by_leader) /
      std::max(lane_speed_limit, kMinLaneSpeedLimit);
  traj_feature_output->progress_factor = 1.0 - lane_speed_diff_ratio;
  if (in_high_way) {
    if (lane_speed_diff_ratio < kCutOffLaneSpeedDiffRatio) {
      // Ignore 10% lower leader.
      cost_vec.at(1) = 0.0;
    } else if (lane_speed_diff_ratio < kBeginLaneSpeedDiffRatioForHighway) {
      // Smooth cost from 10% to 15%.
      cost_vec.at(1) = LinearInterpolate(
          0.0, kBeginLaneSpeedDiffCostFactor, kCutOffLaneSpeedDiffRatio,
          kBeginLaneSpeedDiffRatioForHighway, lane_speed_diff_ratio);
    } else if (lane_speed_diff_ratio < kForceLaneChangeDiffRatioForHighway) {
      // Lane change for 15%.
      cost_vec.at(1) = LinearInterpolate(
          kBeginLaneSpeedDiffCostFactor, kForceLaneChangeDiffCostFactor,
          kBeginLaneSpeedDiffRatioForHighway,
          kForceLaneChangeDiffRatioForHighway, lane_speed_diff_ratio);
    } else {
      // Force lane change for 20%.
      cost_vec.at(1) = LinearInterpolate(kForceLaneChangeDiffCostFactor, 1.0,
                                         kForceLaneChangeDiffRatioForHighway,
                                         1.0, lane_speed_diff_ratio);
    }
    cost_vec.at(1) = std::min(1.0, cost_vec.at(1)) * kHighwayProgressFactor;
  } else {
    // Lane change for 20% (for non-highway).
    cost_vec.at(1) = lane_speed_diff_ratio / kLaneSpeedDiffToLimitRatio;
    cost_vec.at(1) = std::min(1.0, cost_vec.at(1));
  }

  // 3. Consider when target lane has non-block front obj.
  cost_vec.at(2) = btof(in_high_way && lc_ongoing &&
                        !lane_feature_info.front_non_block_obj_ids.empty());

  // Generate lane change reason.
  if ((max_lane_speed_ - lane_speed_limit) / max_lane_speed_ >
      kCutOffLaneSpeedDiffRatio) {
    traj_feature_output->lane_change_for_road_speed_limit = true;
  }
  if (lane_feature_info.nearest_leader.has_value()) {
    const auto& nearest_leader = *lane_feature_info.nearest_leader;
    if (!nearest_leader.is_stationary) {
      traj_feature_output->lane_change_for_moving_obj = true;
    } else {
      if (nearest_leader.obj_type == ObjectType::OT_LARGE_VEHICLE ||
          nearest_leader.obj_type == ObjectType::OT_VEHICLE) {
        if (nearest_leader.is_stalled) {
          traj_feature_output->lane_change_for_stalled_vehicle = true;
        } else {
          traj_feature_output->lane_change_for_stationary_vehicle = true;
        }
      } else {
        traj_feature_output->lane_change_for_stationary_obj = true;
      }
    }
  }
  if (block_id.has_value() && *block_id == "stopline") {
    // Consider route cost for merge lane.
    traj_feature_output->lane_change_for_route_cost = true;
  }

  extra_info->emplace_back(absl::StrFormat("Progress: %.2f / %.2f",
                                           point_proj.s, standard_progress));
  extra_info->emplace_back(
      absl::StrFormat("Origin Lane limit: %.2f m/s", lane_speed_limit));
  extra_info->emplace_back(absl::StrFormat("Lane limit by leader: %.2f m/s",
                                           lane_speed_limit_by_leader));
  if (block_id.has_value()) {
    extra_info->back() += absl::StrFormat(" from %s", *block_id);
    extra_info->emplace_back(
        absl::StrFormat("Leader perception v: %.2f, max prediction v: %.2f m/s",
                        init_leader_speed, max_leader_speed));
    extra_info->emplace_back(
        absl::StrFormat("Leader init dist: %.2f", init_leader_dist));
  }
  extra_info->emplace_back(absl::StrFormat(
      "Lane progress factor: %.2f", traj_feature_output->progress_factor));
  if (lane_feature_info.nearest_leader.has_value()) {
    extra_info->emplace_back(absl::StrFormat(
        "Nearest leader: %s, s: %.2f, v: %.2f, type: %s",
        lane_feature_info.nearest_leader->obj_id,
        lane_feature_info.nearest_leader->obj_s,
        lane_feature_info.nearest_leader->obj_v,
        ObjectType_Name(lane_feature_info.nearest_leader->obj_type)));
  }
  extra_info->emplace_back(
      absl::StrFormat("Slow leader factor: %.2f", slow_leader_factor));
  extra_info->emplace_back(
      absl::StrFormat("Driving style factor: %.2f", driving_style_factor));
  extra_info->emplace_back(
      absl::StrFormat("Slow av factor: %.2f", slow_av_factor));
  extra_info->emplace_back(absl::StrFormat("Target near leader factor: %.2f",
                                           target_near_leader_factor));
  if (!lane_feature_info.front_non_block_obj_ids.empty()) {
    extra_info->emplace_back(absl::StrFormat(
        "Front non-block obj: %s",
        absl::StrJoin(lane_feature_info.front_non_block_obj_ids, ", ")));
  }
  return cost_vec;
}

absl::StatusOr<CostVec> TrajMaxJerkCost::ComputeCost(
    const EstPlannerOutput& planner_output,
    std::vector<std::string>* extra_info,
    TrajFeatureOutput* /*traj_feature_output*/) const {
  CostVec cost_vec(size());
  QCHECK_NOTNULL(extra_info);

  const auto& traj_pts = planner_output.traj_points;
  const int n_pts = traj_pts.size();
  int max_accel_idx = -1, max_decel_idx = -1, max_lat_idx = -1,
      max_deacc_idx = -1;

  double accel_jerk_cost = 0.0, decel_jerk_cost = 0.0, deacc_cost = 0.0;
  for (int i = 0; i < n_pts; ++i) {
    const auto lon_jerk_cost = traj_pts[i].j() * coeffs_[i];
    if (lon_jerk_cost > accel_jerk_cost) {
      accel_jerk_cost = lon_jerk_cost;
      max_accel_idx = i;
    }
    if (lon_jerk_cost < decel_jerk_cost) {
      decel_jerk_cost = lon_jerk_cost;
      max_decel_idx = i;
    }

    if (traj_pts[i].a() > kMaxDecelerationAcc) {
      continue;
    }
    const auto lon_acc_cost = coeffs_[i] *
                              (kMaxDecelerationAcc - traj_pts[i].a()) /
                              (kMaxDecelerationAcc - kMinDecelerationAcc);
    if (deacc_cost < lon_acc_cost) {
      deacc_cost = lon_acc_cost;
      max_deacc_idx = i;
    }
  }
  cost_vec.at(0) = std::max(accel_jerk_cost / accel_jerk_constraint_,
                            decel_jerk_cost / decel_jerk_constraint_);

  std::vector<double> psi(n_pts);
  psi[0] = CalcPsi(traj_pts[0], traj_pts[1]);
  for (int i = 1; i < n_pts - 1; ++i) {
    psi[i] = CalcPsi(traj_pts[i - 1], traj_pts[i + 1]);
  }
  psi[n_pts - 1] = CalcPsi(traj_pts[n_pts - 1], traj_pts[n_pts - 2]);

  double max_lat_jerk_cost = 0.0;
  for (int i = 0; i < n_pts; ++i) {
    const double lat_jerk_cost = CalcLatJerk(traj_pts[i], psi[i]) * coeffs_[i];
    if (lat_jerk_cost > max_lat_jerk_cost) {
      max_lat_jerk_cost = lat_jerk_cost;
      max_lat_idx = i;
    }
  }
  cost_vec.at(1) = max_lat_jerk_cost / lat_jerk_constraint_;

  cost_vec.at(2) = deacc_cost;

  if (max_accel_idx != -1) {
    extra_info->emplace_back(absl::StrFormat(
        "Max accel jerk cost %.2f at %d",
        accel_jerk_cost / accel_jerk_constraint_, max_accel_idx));
  }
  if (max_decel_idx != -1) {
    extra_info->emplace_back(absl::StrFormat(
        "Max decel jerk cost %.2f at %d",
        decel_jerk_cost / decel_jerk_constraint_, max_decel_idx));
  }
  if (max_lat_idx != -1) {
    extra_info->emplace_back(
        absl::StrFormat("Max lat jerk cost %.2f at %d",
                        max_lat_jerk_cost / lat_jerk_constraint_, max_lat_idx));
  }

  if (max_deacc_idx != -1) {
    extra_info->emplace_back(
        absl::StrFormat("Max deacc cost %.2f when deacc is %.2f", deacc_cost,
                        traj_pts[max_deacc_idx].a()));
  }

  return cost_vec;
}

absl::StatusOr<CostVec> TrajLaneChangeCost::ComputeCost(
    const EstPlannerOutput& planner_output,
    std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  CostVec cost_vec(size());
  QCHECK_NOTNULL(extra_info);

  const auto& drive_passage = planner_output.scheduler_output.drive_passage;
  const bool target_switched = prev_lp_from_current_->front().lane_id() !=
                               drive_passage.lane_path().front().lane_id();
  const auto lc_stage =
      planner_output.scheduler_output.lane_change_state.stage();
  const bool lc_ongoing = lc_stage == LaneChangeStage::LCS_EXECUTING ||
                          lc_stage == LaneChangeStage::LCS_RETURN ||
                          lc_stage == LaneChangeStage::LCS_PAUSE;
  traj_feature_output->is_perform_lane_change = lc_ongoing;
  traj_feature_output->lane_change_left =
      lc_ongoing ? planner_output.scheduler_output.lane_change_state.lc_left()
                 : false;
  ASSIGN_OR_RETURN(const auto ego_sl,
                   drive_passage.QueryFrenetCoordinateAt(ego_pos_),
                   _ << "Ego pos not in drive passage.");
  const double lat_offset = std::abs(ego_sl.l);

  cost_vec.at(0) = btof(target_switched);
  cost_vec.at(1) = lc_ongoing ? lat_offset / kDefaultLaneWidth : 0.0;

  double avg_dist = 0.0;
  int valid_traj_size = 0;
  if (prev_traj_ff_or_.ok()) {
    const auto& traj_pts = planner_output.traj_points;
    for (const auto& pt : traj_pts) {
      if (pt.relative_time() > kConsiderPrevTrajTime) {
        break;
      }
      avg_dist += std::abs(
          prev_traj_ff_or_->XYToSL(Vec2dFromApolloTrajectoryPointProto(pt)).l);
      valid_traj_size++;
    }
    if (valid_traj_size != 0) {
      // if valid traj size = 0, avg_dist = 0
      avg_dist /= valid_traj_size;
    }
  }
  cost_vec.at(2) = avg_dist / kStandardAverageTrajDiff;

  const double ego_lc_speed = std::max(ego_v_, kMinLCSpeed);
  constexpr double kLcPreviewTime = 5.0;               // s.
  constexpr double kCheckIntersectionInterval = 10.0;  // s.
  const double preview_t = (lat_offset / kDefaultLaneWidth) * kLcPreviewTime;
  const double preview_s = ego_sl.s + ego_lc_speed * preview_t;
  const bool ego_in_intersection =
      IsInTlControlledIntersection(*psmm_, drive_passage, ego_sl.s);
  bool preview_in_intersection = ego_in_intersection;
  for (double accum_s = ego_sl.s + kCheckIntersectionInterval;
       accum_s < preview_s; accum_s += kCheckIntersectionInterval) {
    preview_in_intersection =
        preview_in_intersection ||
        IsInTlControlledIntersection(*psmm_, drive_passage, accum_s);
  }

  // Avoid lc in curve road.
  std::vector<double> headings;
  std::vector<double> factors;
  for (double accum_s = ego_sl.s - kBackCheckCurveDistance; accum_s < preview_s;
       accum_s += kCalculateCurvatureStep) {
    const auto heading_or = drive_passage.QueryTangentAngleAtS(accum_s);
    if (!heading_or.ok()) {
      continue;
    }
    factors.push_back(2.0 - std::max(0.0, accum_s - ego_sl.s) /
                                (preview_s - ego_sl.s));
    headings.push_back(*heading_or);
  }

  double average_curvature = 0.0;
  if (headings.size() > 1) {
    for (int i = 0; i < headings.size() - 1; ++i) {
      average_curvature +=
          std::fabs(AngleDifference(headings.at(i), headings.at(i + 1))) *
          factors.at(i);
    }
    average_curvature =
        average_curvature / (kCalculateCurvatureStep * (headings.size() - 1));
  }
  const double speed_curvature = average_curvature * std::sqrt(ego_lc_speed);
  cost_vec.at(3) = std::min(1.0, btof(lc_ongoing && !preview_in_intersection) *
                                     speed_curvature / kCurveSpeedConstraint);

  // Avoid lc in intersection.
  const double forbidden_lc_in_intersection =
      planner_enable_lane_change_in_intersection_ ? 1.0 : kForbidBehaviorCost;
  cost_vec.at(4) = btof(lc_ongoing && preview_in_intersection) *
                   forbidden_lc_in_intersection;

  // Avoid lc in redlight with low speed.
  const double length_before_intersection =
      common_feature()->length_before_intersection;
  // Need more patience when the line is long.
  const double valid_redlight_distance =
      std::min(length_before_intersection, kMaxRedLightDistance);
  const double time_waiting =
      std::max(time_since_last_red_light_ -
                   valid_redlight_distance * kRedLightWaitingLeaderFactor,
               0.0);

  // Avoid lc in redlight with low speed
  cost_vec.at(5) =
      (lc_ongoing && !ego_in_intersection)
          ? std::max(0.0, 1.0 - time_waiting / kRedLightWaitingTimeBase) *
                std::max(0.0, 1.0 - ego_v_ / kRedLightSpeedBase)
          : 0.0;

  // Avoid opposite lc in short time.
  double opposite_lc_factor = 0.0;
  if (time_since_last_lane_change_ < kMinOppositeLcTimeBound) {
    opposite_lc_factor = 1.0;
  } else if (time_since_last_lane_change_ < kMaxOppositeLcTimeBound) {
    opposite_lc_factor = LinearInterpolate(1.0, 0.0, kMinOppositeLcTimeBound,
                                           kMaxOppositeLcTimeBound,
                                           time_since_last_lane_change_);
  }
  cost_vec.at(6) =
      btof(lc_ongoing &&
           last_lc_info_.lc_left() !=
               planner_output.scheduler_output.lane_change_state.lc_left()) *
      opposite_lc_factor;

  // Prevent too radical lane change.
  cost_vec.at(7) = lc_ongoing
                       ? std::min(1.0, Sqr(planner_output.follower_max_decel /
                                           kLcEffectBase))
                       : 0.0;

  // Discourage lane keep after turn signal.
  cost_vec.at(8) = btof(!lc_ongoing && already_turn_on_pre_turn_signal_);

  // Discourage lane change with opposite diverge.
  std::string diverge_info_str;
  std::optional<double> diverge_distance = std::nullopt;
  if (lc_ongoing) {
    for (int i = 1; i < drive_passage.lane_path().lane_ids_size(); ++i) {
      const auto lane_id = drive_passage.lane_path().lane_id(i);
      SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, *psmm_, lane_id);
      if (lane_info.incoming_lanes().empty()) continue;
      SMM_ASSIGN_LANE_OR_CONTINUE(prev_lane_info, *psmm_,
                                  lane_info.incoming_lanes().front());
      if (prev_lane_info.outgoing_lanes().size() < 2) continue;
      const auto prev_tangent = prev_lane_info.GetTangent(1.0).FastAngle();
      const auto curr_tangent = lane_info.GetTangent(0.0).FastAngle();
      const double angle_difference =
          AngleDifference(prev_tangent, curr_tangent);
      diverge_info_str =
          absl::StrFormat("Diverge lane: %d, Angle diff: %.2f",
                          lane_info.id.value(), angle_difference);
      if (std::fabs(angle_difference) < kDivergeAngleDiff) continue;
      if ((angle_difference > 0) ^
          (planner_output.scheduler_output.lane_change_state.lc_left())) {
        // Lane change direction is opposite with diverge direction.
        diverge_distance = drive_passage.lane_path().start_s(i);
        break;
      }
    }
  }
  if (!diverge_distance.has_value() ||
      *diverge_distance > kMaxDivergeDistance) {
    cost_vec.at(9) = 0.0;
  } else if (*diverge_distance > preview_s) {
    cost_vec.at(9) = LinearInterpolate(0.0, 1.0, kMaxDivergeDistance, preview_s,
                                       *diverge_distance);
  } else {
    cost_vec.at(9) = 1.0;
  }

  constexpr double kDblMaxDisplay = 1e6;
  extra_info->emplace_back(absl::StrFormat("Has switched target lane path: %s",
                                           btoa(target_switched)));
  extra_info->emplace_back(
      absl::StrFormat("Is performing lane change: %s", btoa(lc_ongoing)));
  extra_info->emplace_back(absl::StrFormat("Ego lat offset: %.2f", lat_offset));
  extra_info->emplace_back(
      absl::StrFormat("Average dist to prev traj: %.2f", avg_dist));
  extra_info->emplace_back(absl::StrFormat("Preview in intersection: %s",
                                           btoa(preview_in_intersection)));
  extra_info->emplace_back(
      absl::StrFormat("Average curv: %.2f, cost percent: %.2f", speed_curvature,
                      cost_vec.at(3)));
  extra_info->emplace_back(
      absl::StrFormat("Enable lc in intersection: %s",
                      btoa(planner_enable_lane_change_in_intersection_)));
  extra_info->emplace_back(absl::StrFormat(
      "Time since last lc: %.2f s, cost: %.2f",
      std::clamp(time_since_last_lane_change_, 0.0, kDblMaxDisplay),
      opposite_lc_factor));
  extra_info->emplace_back(absl::StrFormat(
      "Time since last redlight: %.2f s, waiting: %.2f",
      std::clamp(time_since_last_red_light_, 0.0, kDblMaxDisplay),
      std::clamp(time_waiting, 0.0, kDblMaxDisplay)));
  extra_info->emplace_back(absl::StrFormat("Follower max decel: %.2f",
                                           planner_output.follower_max_decel));
  extra_info->emplace_back(
      absl::StrFormat("Already turn on pre turn signal: %s",
                      btoa(already_turn_on_pre_turn_signal_)));
  if (!diverge_info_str.empty()) {
    extra_info->emplace_back(diverge_info_str);
  }
  if (diverge_distance.has_value()) {
    extra_info->emplace_back(absl::StrFormat(
        "Opposite diverge lc distance: %.2f", *diverge_distance));
  }
  return cost_vec;
}

absl::StatusOr<CostVec> TrajCrossSolidBoundaryCost::ComputeCost(
    const EstPlannerOutput& planner_output,
    std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  QCHECK_NOTNULL(extra_info);
  CostVec cost_vec(size());

  // 1. get variables from input
  constexpr int kCheckEveryNPt = 5;
  constexpr double kTrajectorySExtension = 10.0;  // m.

  const auto& traj_pts = planner_output.traj_points;
  const auto& drive_passage = planner_output.scheduler_output.drive_passage;
  const auto& change_stage =
      planner_output.scheduler_output.lane_change_state.stage();

  // Only check a first small part of trajectory on lane change pause
  // to avoid end of trajectory cross line
  const int check_first_n =
      change_stage == LaneChangeStage::LCS_PAUSE
          ? std::min<int>(CeilToInt(0.3 * traj_pts.size()) + 1, traj_pts.size())
          : traj_pts.size();
  const auto last_pt_in_rear_center =
      Vec2dFromApolloTrajectoryPointProto(traj_pts[check_first_n - 1]);
  const auto heading = traj_pts[check_first_n - 1].path_point().theta();
  const auto unit = Vec2d::UnitFromAngle(heading);
  const auto last_pt_in_front_center =
      last_pt_in_rear_center + unit * ego_front_to_ra_;

  // 2. get solid boundary from drive passage
  ASSIGN_OR_RETURN(const auto last_pt_in_front_center_frenet,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       last_pt_in_front_center),
                   _ << "Last considered traj point not on drive passage.");
  ASSIGN_OR_RETURN(const auto first_point_sl,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       Vec2dFromApolloTrajectoryPointProto(traj_pts.front())),
                   _ << "First traj point not on drive passage.");
  const auto solid_boundaries = FindSolidBoundaryIntervals(
      drive_passage, first_point_sl,
      last_pt_in_front_center_frenet.s + kTrajectorySExtension);

  // 3. generate ego car trajectory box
  std::vector<Box2d> ego_boxes;
  ego_boxes.reserve(
      CeilToInt(check_first_n / static_cast<float>(kCheckEveryNPt)));
  const double rear_to_real_center = ego_front_to_ra_ - 0.5 * ego_length_;
  double dist_to_start = 0.0;
  for (int i = 0; i < check_first_n; i += kCheckEveryNPt) {
    const auto& traj_pt = traj_pts[i];
    const double heading = traj_pt.path_point().theta();
    const auto traj_pt_vec = Vec2dFromApolloTrajectoryPointProto(traj_pt);
    double lat_tolerance_error = 0.0;
    if (i >= kCheckEveryNPt) {
      // Simply consider traj length as longitudinal length.
      dist_to_start += traj_pt_vec.DistanceTo(
          Vec2dFromApolloTrajectoryPointProto(traj_pts[i - kCheckEveryNPt]));
      lat_tolerance_error = GetLatBoundaryToleranceError(dist_to_start);
    }
    Box2d ego_box(traj_pt_vec, heading, ego_length_,
                  ego_width_ - lat_tolerance_error);
    ego_box.Shift(Vec2d::UnitFromAngle(heading) * rear_to_real_center);
    ego_boxes.emplace_back(std::move(ego_box));
  }
  // For low speed condition before a stop line.
  const Segment2d last_pt_to_ref_center_seg(
      last_pt_in_front_center,
      *drive_passage.QueryPointXYAtS(last_pt_in_front_center_frenet.s));
  ego_boxes.emplace_back(last_pt_to_ref_center_seg, ego_width_);

  const auto start_l_or =
      drive_passage.QueryFrenetLatOffsetAt(ego_boxes.front().center());
  const auto end_l_or =
      drive_passage.QueryFrenetLatOffsetAt(ego_boxes.back().center());
  const double ego_half_width = ego_width_ * 0.5;

  // 4. calculate cost for different solid boudary type
  const double forbidden_cross_solid_line_cost =
      planner_enable_cross_solid_boundary_ ? 1.0 : kForbidBehaviorCost;

  cost_vec.at(0) = CalculateCrossingBoundary(
                       drive_passage, solid_boundaries, ego_boxes, change_stage,
                       {StationBoundaryType::SOLID_WHITE,
                        StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE,
                        StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE},
                       start_l_or, end_l_or, ego_half_width) *
                   forbidden_cross_solid_line_cost;
  cost_vec.at(1) = CalculateCrossingBoundary(
                       drive_passage, solid_boundaries, ego_boxes, change_stage,
                       {StationBoundaryType::SOLID_YELLOW}, start_l_or,
                       end_l_or, ego_half_width) *
                   forbidden_cross_solid_line_cost;
  cost_vec.at(2) = CalculateCrossingBoundary(
                       drive_passage, solid_boundaries, ego_boxes, change_stage,
                       {StationBoundaryType::SOLID_DOUBLE_YELLOW}, start_l_or,
                       end_l_or, ego_half_width) *
                   forbidden_cross_solid_line_cost;
  cost_vec.at(3) = CalculateCrossingBoundary(
      drive_passage, solid_boundaries, ego_boxes, change_stage,
      {StationBoundaryType::CURB}, start_l_or, end_l_or, ego_half_width);

  if (cost_vec.at(0) > kCrossBoundaryFactor ||
      cost_vec.at(1) > kCrossBoundaryFactor ||
      cost_vec.at(2) > kCrossBoundaryFactor) {
    traj_feature_output->cross_solid_boundary = true;
  }

  extra_info->emplace_back(
      absl::StrFormat("Solid white: %.2f", cost_vec.at(0)));
  extra_info->emplace_back(
      absl::StrFormat("Solid yellow: %.2f", cost_vec.at(1)));
  extra_info->emplace_back(
      absl::StrFormat("Solid double yellow: %.2f", cost_vec.at(2)));
  extra_info->emplace_back(absl::StrFormat("Curb: %.2f", cost_vec.at(3)));
  extra_info->emplace_back(
      absl::StrFormat("Enable lc cross solid boundary: %s",
                      btoa(planner_enable_cross_solid_boundary_)));
  extra_info->emplace_back(
      absl::StrFormat("Cross solid boundary: %s",
                      btoa(traj_feature_output->cross_solid_boundary)));

  return cost_vec;
}

absl::StatusOr<CostVec> TrajRouteLookAheadCost::ComputeCost(
    const EstPlannerOutput& planner_output,
    std::vector<std::string>* extra_info,
    TrajFeatureOutput* traj_feature_output) const {
  QCHECK_NOTNULL(extra_info);
  CostVec cost_vec(size());

  const auto start_lane_id =
      planner_output.scheduler_output.drive_passage.lane_path()
          .front()
          .lane_id();
  const auto scheduler_hash = planner_output.scheduler_output.Hash();
  const double len_before_merge_lane =
      FindWithDefault(len_before_merge_lane_map_, start_lane_id, 0.0);
  const double length_along_route =
      FindWithDefault(len_along_route_map_, scheduler_hash, 0.0);
  const double raw_len_along_route =
      FindWithDefault(raw_len_along_route_map_, scheduler_hash, 0.0);
  const auto front_stalled_obj_info =
      FindOrDieNoPrint(front_stalled_obj_map_, scheduler_hash);
  const double driving_dist = FindOrDie(driving_dist_map_, start_lane_id);
  const double len_before_intersection =
      FindWithDefault(len_before_intersection_map_, start_lane_id, 0.0);
  const int lc_num_to_targets =
      FindOrDie(lc_num_to_targets_map_, start_lane_id);
  const int lc_num_within_driving_dist =
      FindOrDie(lc_num_within_driving_dist_map_, start_lane_id);
  const bool is_right_most_lane =
      FindOrDie(is_right_most_lane_map_, start_lane_id);
  const bool is_max_len_along_route =
      length_along_route >= max_len_along_route_ - kDistanceEpsilon;
  const bool is_min_len_along_route =
      length_along_route <= min_len_along_route_ + kDistanceEpsilon;
  const bool is_valid_merge =
      FindOrDie(is_valid_merge_lane_map_, start_lane_id);
  const bool is_min_lc_num = lc_num_to_targets == min_lc_num_;
  const bool is_max_lc_num = lc_num_to_targets == max_lc_num_;
  const bool in_high_way = common_feature()->in_high_way;
  traj_feature_output->in_highway = in_high_way;
  traj_feature_output->in_route_target_lane = is_min_lc_num;
  traj_feature_output->driving_dist = driving_dist;
  if (is_valid_merge) {
    const auto& drive_passage = planner_output.scheduler_output.drive_passage;
    auto merge_point_or = drive_passage.QueryPointXYAtS(len_before_merge_lane);
    if (merge_point_or.ok()) {
      traj_feature_output->merge_point = std::move(*merge_point_or);
    }
  }
  // 1. length along route cost
  cost_vec.at(0) = 0.0;
  const double length_consider_congestion =
      length_along_route *
      (is_non_occluding_leader_ ? kNonOccludingLeaderFactor : 1.0);
  if (in_high_way) {
    // Need to lc early in high-way.
    if (!is_max_len_along_route && !is_min_lc_num) {
      const double speed = std::max(kConsiderTtcSpeedLowerBound, ego_v_);
      const double force_route_ttc = use_conservative_ttc_
                                         ? kForceRouteTtcForHighWayConservative
                                         : kForceRouteTtcForHighWay;
      const double begin_route_ttc = use_conservative_ttc_
                                         ? kBeginRouteTtcForHighWayConservative
                                         : kBeginRouteTtcForHighWay;
      const double force_route_lc_length =
          std::clamp(force_route_ttc * speed, kForceRouteLengthLcForHighWay,
                     kBeginRouteLengthLcForHighWay);
      const double begin_route_lc_length =
          std::clamp(begin_route_ttc * speed, force_route_lc_length + 1.0,
                     kBeginRouteLengthLcForHighWay);
      if (length_consider_congestion > kRouteLengthCutOffDist) {
        cost_vec.at(0) = 0.0;
      } else if (length_consider_congestion > kBeginRouteLengthLcForHighWay) {
        cost_vec.at(0) = LinearInterpolate(
            0.0, kBeginRouteLcCostFactorForHighWay, kRouteLengthCutOffDist,
            kBeginRouteLengthLcForHighWay, length_consider_congestion);
      } else if (length_consider_congestion > begin_route_lc_length) {
        cost_vec.at(0) = kBeginRouteLcCostFactorForHighWay;
      } else if (length_consider_congestion > force_route_lc_length) {
        cost_vec.at(0) = LinearInterpolate(
            kBeginRouteLcCostFactorForHighWay,
            kForceRouteLcCostFactorForHighWay, begin_route_lc_length,
            force_route_lc_length, length_consider_congestion);
      } else {
        cost_vec.at(0) = kForceRouteLcCostFactorForHighWay;
      }

      // Check if we need to request lane change.
      if (!front_stalled_obj_info.has_value()) {
        // Ignore short length along route caused by front stalled object.
        // Need driver confirm when length between conservative and radical.
        if (length_consider_congestion <
                kBeginRouteTtcForHighWayConservative * speed &&
            length_consider_congestion > kBeginRouteTtcForHighWay * speed) {
          traj_feature_output->reach_need_confirmation_distance = true;
        }
      }
    }
  } else {
    const double length_along_route_base = is_right_turn_
                                               ? kLengthAlongRouteBaseForRight
                                               : kLengthAlongRouteBaseForLeft;
    if (!is_max_len_along_route && !is_min_lc_num) {
      cost_vec.at(0) = Sqr(
          1.0 - std::min(1.0, length_along_route / length_along_route_base));
    }
    cost_vec.at(0) = std::min(1.0, cost_vec.at(0));
  }

  // 2. avoid too short driving dist.
  cost_vec.at(1) =
      is_min_lc_num || is_max_len_along_route
          ? 0.0
          : Sqr(1.0 -
                std::min(1.0, driving_dist / kReachDestinationCutOffDist));

  // 3. preview beyond the local map horizon for lane changes
  // that are far away.
  const double length_for_one_lc =
      in_high_way ? kMinLenForLaneChangeHighWay : kMinLenForLaneChange;
  cost_vec.at(2) =
      btof(!is_min_lc_num &&
           lc_num_within_driving_dist * length_for_one_lc > driving_dist &&
           lc_num_within_driving_dist >= kConsiderMinLcNumToTarget);

  // 4. discourage right most lane cost
  const bool discourage_right_most = !planner_is_bus_model_ &&
                                     is_right_most_lane &&
                                     enable_discourage_right_most_cost_;
  double discourage_right_most_factor = 0.0;
  if (ego_v_ > kDiscourageRightMostSpeedUpperBound) {
    discourage_right_most_factor = 1.0;
  } else if (ego_v_ > kDiscourageRightMostSpeedLowerBound) {
    discourage_right_most_factor =
        LinearInterpolate(0.0, 1.0, kDiscourageRightMostSpeedLowerBound,
                          kDiscourageRightMostSpeedUpperBound, ego_v_);
  }
  cost_vec.at(3) = btof(discourage_right_most) * discourage_right_most_factor;

  // 5. generate prohibited stalled obj cost
  // when there is only one single lane ,we can borrow lane in other direction
  if (front_stalled_obj_info.has_value()) {
    constexpr double kFollowDistance = 5.0;  // m.
    const double distance_factor = std::clamp(
        1.0 - (front_stalled_obj_info->stalled_obj_s - kFollowDistance) /
                  kStalledObjectLengthBase,
        0.0, 1.0);
    cost_vec.at(4) =
        (is_max_len_along_route && !is_min_len_along_route)
            ? 0.0
            : front_stalled_obj_info->punish_factor * distance_factor;
    traj_feature_output->is_blocked_by_stalled_obj = true;
  }
  // 6. for merge lane cost
  const double merge_lane_length_base =
      in_high_way ? kMergeLaneLengthBaseHighWay : kMergeLaneLengthBase;
  cost_vec.at(5) = is_valid_merge
                       ? Sqr(1.0 - std::min(1.0, len_before_merge_lane /
                                                     merge_lane_length_base))
                       : 0.0;

  // 7. encourage right most lane cost for bus mode.
  cost_vec.at(6) = btof(enable_encourage_right_most_cost_ &&
                        planner_is_bus_model_ && !is_right_most_lane);

  if (cost_vec.at(0) > kLengthAlongRouteObviousThreshold ||
      cost_vec.at(4) > kBehindStalledObjObviousThreshold ||
      cost_vec.at(5) > kLengthBeforeMergeLaneObviousThreshold) {
    traj_feature_output->has_obvious_route_cost = true;
  }

  // Generate lane change reason.
  if (cost_vec.at(3) > kBeginLcForDiscourageRightMostThreshold) {
    traj_feature_output->lane_change_for_right_most_lane = true;
  }
  if (cost_vec.at(0) > kBeginLcLengthAlongRouteThreshold) {
    // Ignore short length along route caused by front stalled object.
    if (!front_stalled_obj_info.has_value()) {
      traj_feature_output->lane_change_for_route_cost = true;
    }
  }
  if (cost_vec.at(2) > kBeginLcForPreviewThreshold ||
      cost_vec.at(5) > kBeginLcForMergeLaneThreshold) {
    traj_feature_output->lane_change_for_route_cost = true;
  }

  // generate debug information
  extra_info->emplace_back(
      absl::StrFormat("Length along route: %.2f", length_along_route));
  extra_info->emplace_back(
      absl::StrFormat("Raw Length along route: %.2f", raw_len_along_route));
  extra_info->emplace_back(absl::StrFormat("Length before merge lane: %.2f.",
                                           len_before_merge_lane));
  extra_info->emplace_back(
      absl::StrFormat("Is valid merging: %s.", btoa(is_valid_merge)));
  extra_info->emplace_back(
      absl::StrFormat("Length along route before intersection: %.2f.",
                      len_before_intersection));
  extra_info->emplace_back(absl::StrFormat(
      "Is non occluding leader: %s, length: %.2f.",
      btoa(is_non_occluding_leader_), length_consider_congestion));
  extra_info->emplace_back(absl::StrFormat(
      "Reach need confirmation distance: %s",
      btoa(traj_feature_output->reach_need_confirmation_distance)));
  extra_info->emplace_back(absl::StrFormat("Use conservative route ttc: %s",
                                           btoa(use_conservative_ttc_)));
  extra_info->emplace_back(
      absl::StrFormat("has obvious route cost: %s",
                      btoa(traj_feature_output->has_obvious_route_cost)));
  extra_info->emplace_back(
      absl::StrFormat("Remaining Driving distance: %.2f.", driving_dist));
  extra_info->emplace_back(
      absl::StrFormat("Lane change num to target: %d", lc_num_to_targets));
  extra_info->emplace_back(absl::StrFormat(
      "Lane change num within driving dist: %d", lc_num_within_driving_dist));
  extra_info->emplace_back(absl::StrFormat(
      "Highway: %s, Right most: %s, Discourage: %s", btoa(in_high_way),
      btoa(is_right_most_lane), btoa(enable_discourage_right_most_cost_)));
  extra_info->emplace_back(absl::StrFormat("Discourage right most factor: %.2f",
                                           discourage_right_most_factor));
  extra_info->emplace_back(absl::StrFormat(
      "Encourage right most: %s, Bus: %s",
      btoa(enable_encourage_right_most_cost_), btoa(planner_is_bus_model_)));
  extra_info->emplace_back(absl::StrFormat("Turn left: %s, Turn right: %s",
                                           btoa(is_left_turn_),
                                           btoa(is_right_turn_)));
  extra_info->emplace_back(
      absl::StrFormat("Is min lc num: %s, Is max lc num %s",
                      btoa(is_min_lc_num), btoa(is_max_lc_num)));
  if (front_stalled_obj_info.has_value()) {
    extra_info->emplace_back(absl::StrFormat(
        "Stalled id: %s, Factor: %.2f", front_stalled_obj_info->stalled_obj_id,
        front_stalled_obj_info->punish_factor));
  } else {
    extra_info->emplace_back("No front stalled object found.");
  }
  return cost_vec;
}

absl::StatusOr<CostVec> TrajBoundaryExpansionCost::ComputeCost(
    const EstPlannerOutput& planner_output,
    std::vector<std::string>* extra_info,
    TrajFeatureOutput* /*traj_feature_output*/) const {
  CostVec cost_vec(size());
  QCHECK_NOTNULL(extra_info);

  ASSIGN_OR_RETURN(
      const double ego_l,
      planner_output.scheduler_output.drive_passage.QueryFrenetLatOffsetAt(
          ego_pos_),
      _ << "Plan start point is not on drive passage!");

  const auto& path_boundary = planner_output.scheduler_output.sl_boundary;
  const auto& left_l_vec = path_boundary.target_left_l_vector();
  const auto& right_l_vec = path_boundary.target_right_l_vector();
  const double left_offset =
      std::accumulate(left_l_vec.begin(), left_l_vec.end(), 0.0) /
      path_boundary.size();
  const double right_offset =
      std::accumulate(right_l_vec.begin(), right_l_vec.end(), 0.0) /
      path_boundary.size();

  const double dist = left_offset > -right_offset
                          ? std::abs(left_offset - ego_l)
                          : std::abs(ego_l - right_offset);

  constexpr double kDefaultOneAndHalfLaneWidth = 1.5 * kDefaultLaneWidth;
  cost_vec.at(0) = dist / kDefaultOneAndHalfLaneWidth;

  extra_info->emplace_back(
      absl::StrFormat("Average width left: %.2f", left_offset));
  extra_info->emplace_back(
      absl::StrFormat("Average width right: %.2f", -right_offset));

  return cost_vec;
}

}  // namespace qcraft::planner
