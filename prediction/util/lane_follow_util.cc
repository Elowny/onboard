#include "onboard/prediction/util/lane_follow_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace prediction {
namespace {
/*
 *  Pure pursuit path generation related parameters.
 */
const PiecewiseLinearFunction<double, double> kProbExtensionLengthPlf(
    std::vector<double>{0.5, 1.0, 2.0, 4.0},
    std::vector<double>{1.0, 2.0, 3.0, 5.0});
}  // namespace

bool IsSlowCutinObj(const planner::DrivePassage& dp,
                    const ObjectMotionState& cur_state) {
  const auto sl_or = dp.QueryFrenetCoordinateAt(cur_state.pos);
  if (!sl_or.ok()) return false;

  const auto path_angle_or = dp.QueryTangentAngleAtS(sl_or->s);
  if (!path_angle_or.ok()) return false;

  // Is objet near av and moving slow.
  const double cur_speed = cur_state.vel.norm();
  const bool is_obj_near = std::abs(sl_or->l) < 2 * kDefaultHalfLaneWidth;
  const bool is_moving_slow = cur_speed < kSlowVelTreshold;

  // Is object facing lane center
  const auto angle_diff_dp =
      NormalizeAngle(path_angle_or.value() - cur_state.heading);
  const bool agent_on_left = sl_or->l > 0.0;
  const bool agent_on_right = sl_or->l < 0.0;
  const bool facing_right = angle_diff_dp > kSlowAngleDiffThreshold;
  const bool facing_left = angle_diff_dp < -kSlowAngleDiffThreshold;
  const bool facing_center =
      (agent_on_left && facing_right) || (agent_on_right && facing_left);

  if (is_obj_near && is_moving_slow && facing_center) return true;
  return false;
}

bool IsObjectApproachingDrivePassage(const planner::DrivePassage& dp,
                                     const ObjectMotionState& cur_state,
                                     const ObjectType& obj_type,
                                     double lane_min_l, double lane_max_l,
                                     double cur_lat_speed) {
  const double angle_diff_buffer = obj_type == OT_LARGE_VEHICLE
                                       ? kLargeVehicleAngleDiffBuffer
                                       : kNormalAngleDiffBuffer;

  const auto sl_or = dp.QueryFrenetCoordinateAt(cur_state.pos);
  if (!sl_or.ok()) {
    return false;
  }
  const auto path_angle = dp.QueryTangentAngleAtS(sl_or->s);
  if (!path_angle.ok()) {
    return false;
  }
  const double angle_diff =
      NormalizeAngle(path_angle.value() - cur_state.heading);
  return (sl_or->l < lane_min_l && (cur_lat_speed > kLatSpeedBuffer ||
                                    angle_diff < -angle_diff_buffer)) ||
         (sl_or->l > lane_max_l &&
          (cur_lat_speed < -kLatSpeedBuffer || angle_diff > angle_diff_buffer));
}

absl::StatusOr<double> CalcTargetLForObjectCrossingBoundary(
    const ObjectMotionState& cur_state, double cur_lat_speed, double lane_min_l,
    double lane_max_l, const planner::DrivePassage& dp,
    const ObjectType& obj_type, double target_l, bool is_hd_map,
    bool is_slow_cutin) {
  constexpr double kSlowObjectDistToBoundBuffer = 0.1;
  constexpr double kHdMapDistToBoundBuffer = -0.1;

  double dist_to_bound_buffer = 0.0;
  if (is_slow_cutin) {
    dist_to_bound_buffer = kSlowObjectDistToBoundBuffer;
  } else {
    dist_to_bound_buffer =
        is_hd_map ? kHdMapDistToBoundBuffer : kNormalDistToBoundBuffer;
  }

  Box2d obj_box = cur_state.bbox;
  const auto& pos = cur_state.pos;
  const auto sl_or = dp.QueryFrenetCoordinateAt(pos);

  const auto fbox_or = dp.QueryFrenetBoxAt(obj_box);
  if (!fbox_or.ok() || !sl_or.ok()) {
    return absl::FailedPreconditionError(
        "Can not find fbox or sl pos of the object");
  }
  // Check if either side of object is on boundary or not.
  const bool obj_on_boundary =
      (sl_or->l < lane_min_l &&
       fbox_or->l_max >= lane_min_l - dist_to_bound_buffer) ||
      (sl_or->l > lane_max_l &&
       fbox_or->l_min <= lane_max_l + dist_to_bound_buffer);
  if (!obj_on_boundary) {
    return absl::FailedPreconditionError("Object not on boundary");
  }
  // If object fbox is on the lane boundary and heading diff is large,
  // estimate a target l based on heading diff.
  const auto prob_point = pos + (obj_box.half_length() +
                                 kProbExtensionLengthPlf(obj_box.length())) *
                                    Vec2d::UnitFromAngle(cur_state.heading);
  const auto prob_sl_or = dp.QueryFrenetCoordinateAt(prob_point);
  if (prob_sl_or.ok()) {
    if (sl_or->l < lane_min_l) {
      target_l = std::clamp(target_l, prob_sl_or->l, 0.0);
    } else if (sl_or->l > lane_max_l) {
      target_l = std::clamp(target_l, 0.0, prob_sl_or->l);
    }
  }
  // Further, Object on the lane boundary with its speed and heading toward
  // the lane center is considered as cutin obj, hence set target_l to 0.
  if (IsObjectApproachingDrivePassage(dp, cur_state, obj_type, lane_min_l,
                                      lane_max_l, cur_lat_speed)) {
    target_l = 0.0;
  }

  return target_l;
}

absl::StatusOr<double> CalculateLaneFollowTargetL(
    absl::Span<const ObjectMotionState> hist, const planner::DrivePassage& dp,
    const ObjectType& obj_type, bool is_in_intersection) {
  if (is_in_intersection) {
    return absl::FailedPreconditionError(
        "Not support target lateral offset calculation in intersection.");
  }

  const auto& cur_state = hist.back();
  const auto& pos = cur_state.pos;

  ASSIGN_OR_RETURN(const auto sl, dp.QueryFrenetCoordinateAt(pos));

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
      std::clamp(cur_lat_speed, -kLateralSpeedClamp, kLateralSpeedClamp);

  constexpr double kNearCenterBuffer = 0.75;
  double target_l = sl.l + cur_lat_speed * kLateralSpeedLookAheadTime;
  // If agent far from center, do not over-estimate the cut-in intention. If
  // agent is near the lane center with certain threshold, we allow the
  // predictor to estimate a cut-in target l.
  if (sl.l > kNearCenterBuffer) {
    target_l = std::max(min_l, target_l);
  } else if (sl.l < -kNearCenterBuffer) {
    target_l = std::min(max_l, target_l);
  }

  // If object is on the lane boundary, correct it according to heading and
  // lateral speed contitions.
  const auto target_l_or = CalcTargetLForObjectCrossingBoundary(
      cur_state, cur_lat_speed, min_l, max_l, dp, obj_type, target_l,
      /*is_hd_map=*/false, /*is_slow_cutin=*/IsSlowCutinObj(dp, cur_state));
  target_l = target_l_or.ok() ? target_l_or.value() : target_l;

  const double lk_min_l = std::min(min_l + cur_state.bbox.half_width(), sl.l);
  const double lk_max_l = std::max(max_l - cur_state.bbox.half_width(), sl.l);

  // If the vehicle is currently within the lane boundary and the future target
  // l does not cross the lane boundary, we limit it within the lane to avoid
  // interfering with nearby vehicle.
  const bool target_within_boundary = (target_l > min_l && target_l < max_l);
  // Target l and sl.l are not on the same side of lane center with a certain
  // threshold.
  const bool target_cross_center =
      (target_l < 0.0 && sl.l > kNearCenterBuffer) ||
      (target_l > 0.0 && sl.l < -kNearCenterBuffer);
  const bool lk_intention = target_within_boundary || target_cross_center;

  const double half_lane_width = 0.5 * (max_l - min_l);
  if (sl.l > min_l && sl.l < max_l) {
    if (lk_intention) {
      target_l = std::clamp(target_l, lk_min_l, lk_max_l);
    }
    // If vehicle is not lane keeping, we limit the lane change magnitude.
    target_l =
        std::clamp(target_l, min_l - half_lane_width, max_l + half_lane_width);
  }

  return target_l;
}

std::string ExtendLaneFollowTraj(
    const planner::DrivePassage& dp, const bool is_mapless, double pred_horizon,
    std::vector<PredictedTrajectoryPoint>* traj_pts) {
  const int cur_size = traj_pts->size();
  const bool need_extension =
      (cur_size < static_cast<int>(pred_horizon / kPredictionTimeStep)) &&
      is_mapless;
  if (need_extension) {
    auto tangent_or = dp.QueryTangentAt(traj_pts->back().pos());
    // If tangent projection is not correct and trajectory size larger than 1,
    // we try previous point.
    if (!tangent_or.ok() && traj_pts->size() > 1) {
      traj_pts->pop_back();
      tangent_or = dp.QueryTangentAt(traj_pts->back().pos());
    }
    Vec2d tangent;
    if (tangent_or.ok()) {
      tangent = tangent_or.value();
    } else {
      tangent = Vec2d::FastUnitFromAngle(traj_pts->back().theta());
    }
    ExtendTrajectoryAlongTangent(pred_horizon, tangent, traj_pts);
  }
  std::string extension_info = "";
  if (need_extension) {
    extension_info = " extended from " + std::to_string(cur_size);
  } else {
    extension_info = " no extension";
  }
  return extension_info;
}
}  // namespace prediction
}  // namespace qcraft
