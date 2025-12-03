#include "onboard/planner/speed/decider/pre_brake_decider.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <ostream>
#include <set>
#include <string>
#include <utility>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/decider/interaction_util.h"
#include "onboard/planner/speed/decider/pre_brake_util.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kMildAccel = 0.2;  // m/s^2.

constexpr double kPedestrainStandoffFollowDist = 2.0;    // m.
constexpr double kPedestrainEmergencyBrakeAccel = -1.2;  // m/s^2.
constexpr double kPedestrainEmergencyBrakeTime = 2.0;    // s.
constexpr double kPedestrainEmergencyBrakeVel =
    -kPedestrainEmergencyBrakeAccel * kPedestrainEmergencyBrakeTime;  // m/s.
constexpr double kPedestrainEmergencyBrakeDist =
    0.5 * kPedestrainEmergencyBrakeVel * kPedestrainEmergencyBrakeTime +
    kPedestrainStandoffFollowDist;  // m.

constexpr double kCyclistStandoffFollowDist = 2.0;    // m.
constexpr double kCyclistEmergencyBrakeAccel = -1.0;  // m/s^2.
constexpr double kCyclistEmergencyBrakeTime = 1.0;    // s.
constexpr double kCyclistEmergencyBrakeVel =
    -kCyclistEmergencyBrakeAccel * kCyclistEmergencyBrakeTime;  // m/s.
constexpr double kCyclistEmergencyBrakeDist =
    0.5 * kCyclistEmergencyBrakeVel * kCyclistEmergencyBrakeTime +
    kCyclistStandoffFollowDist;  // m.

constexpr double kVehicleStandoffFollowDist = 3.0;    // m.
constexpr double kVehicleEmergencyBrakeAccel = -1.0;  // m/s^2.
constexpr double kVehicleEmergencyBrakeTime = 0.5;    // s.
constexpr double kVehicleEmergencyBrakeVel =
    -kVehicleEmergencyBrakeAccel * kVehicleEmergencyBrakeTime;  // m/s.
constexpr double kVehicleEmergencyBrakeDist =
    0.5 * kVehicleEmergencyBrakeVel * kVehicleEmergencyBrakeTime +
    kVehicleStandoffFollowDist;  // m.
// NOTE: Since std::abs is consexpr function only since c++23, we use the minus
// sign here as a temporary solution.

constexpr double kEps = 1e-3;
// First: origin st boundary, second: modified st boundaries.
using StBoundaryWithDecisionPtrPair =
    std::pair<StBoundaryWithDecision*, std::vector<StBoundaryWithDecision*>>;

inline void MakeIgnoreDecision(
    const std::string& decision_info,
    const StBoundaryProto::IgnoreReason& ignore_reason,
    StBoundaryWithDecision* st_boundary_wd) {
  st_boundary_wd->set_decision_type(StBoundaryProto::IGNORE);
  st_boundary_wd->set_decision_reason(StBoundaryProto::PRE_BRAKE_DECIDER);
  st_boundary_wd->set_ignore_reason(ignore_reason);
  st_boundary_wd->set_decision_info(decision_info);
}

inline bool IsLeadPedestrain(const StBoundaryWithDecision& st_boundary_wd) {
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  return st_boundary.object_type() == StBoundaryProto::PEDESTRIAN &&
         st_boundary_wd.decision_type() == StBoundaryProto::LEAD;
}

std::optional<VtSpeedLimit> MakePedestrainPreBrakeDecisionForStBoundary(
    const SpeedFinderParamsProto::PreBrakeDeciderParamsProto& params,
    const SpacetimeTrajectoryManager& st_traj_mgr, double current_v,
    double max_v, double time_step, int step_num,
    const StBoundaryWithDecision& st_boundary_wd) {
  if (st_boundary_wd.decision_type() != StBoundaryProto::FOLLOW) {
    return std::nullopt;
  }

  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  if (st_boundary.object_type() != StBoundaryProto::PEDESTRIAN) {
    return std::nullopt;
  }

  if (params.ignore_late_cut_in_ped()) {
    QCHECK(st_boundary.traj_id().has_value());
    const auto& traj_id = st_boundary.traj_id();
    const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));
    const auto& overlap_infos = st_boundary.overlap_infos();
    QCHECK(!overlap_infos.empty());
    const auto& first_overlap_info = overlap_infos.front();
    const auto* first_overlap_obj_point =
        traj->states()[first_overlap_info.obj_idx].traj_point;

    const double ped_s = first_overlap_obj_point->s();
    const double ped_v = traj->planner_object().pose().v();
    const double ped_t = ped_s / (ped_v + kEps);
    constexpr double kPedPreBrakeMaxTime = 6.0;  // s.
    // HACK(bo): Ignore all pedestrains that overlap with AV's path after 6s.
    if (ped_t > kPedPreBrakeMaxTime) {
      const auto info = absl::StrCat(
          "Ignore all pedestrains that overlap with AV's path after 6s ",
          *traj_id);
      return GenerateConstAccSpeedLimit(
          /*start_t=*/0.0, kPedestrainEmergencyBrakeTime, current_v, max_v,
          max_v, 0.0, time_step, step_num, info);
    }
  }

  double object_dist = st_boundary.bottom_left_point().s();
  constexpr double kMaxConsiderTime = 1.0;  // s.
  for (const auto& lower_point : st_boundary.lower_points()) {
    if (lower_point.t() >
        st_boundary.bottom_left_point().t() + kMaxConsiderTime) {
      break;
    }
    object_dist = std::min(object_dist, lower_point.s());
  }

  if (object_dist < kPedestrainEmergencyBrakeDist + kEps ||
      st_boundary.bottom_left_point().t() <
          kPedestrainEmergencyBrakeTime + kEps) {
    return std::nullopt;
  }

  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id();
  // Due to safety concerns, we need to ensure AV could fully stop in front of
  // pedestrains.
  const double ped_rel_v = 0.0;

  const double pre_brake_dist = object_dist - kPedestrainEmergencyBrakeDist;
  const double pre_brake_acc = std::min(
      kMildAccel,
      0.5 * (Sqr(kPedestrainEmergencyBrakeVel + ped_rel_v) - Sqr(current_v)) /
          pre_brake_dist);
  const double pre_brake_time =
      std::abs(pre_brake_acc) > kEps
          ? (sqrt(Sqr(current_v) + 2.0 * pre_brake_acc * pre_brake_dist) -
             current_v) /
                pre_brake_acc
          : pre_brake_acc / (current_v + kEps);
  // NOTE: Speed limit also allows to accelerate mildly (when
  // pre_brake_acc > 0.0). We want to accelerate mildly if current velocity is
  // lower than speed limit.
  constexpr double kMildAccTimeLimit = 5.0;  // s.
  if (pre_brake_acc > 0.0 && pre_brake_time > kMildAccTimeLimit) {
    return std::nullopt;
  }
  const auto info = absl::StrCat("Pre brake for pedestrian ", *traj_id);
  return GenerateConstAccSpeedLimit(
      /*start_t=*/0.0, pre_brake_time + kPedestrainEmergencyBrakeTime,
      current_v,
      /*min_v=*/0.0, max_v, pre_brake_acc, time_step, step_num, info);
}

struct YieldingResult {
  double acceleration = 0.0;
  double wait_time = 0.0;
};

YieldingResult CalcYieldingAccAndWaitTime(double start_v, double end_v,
                                          double yielding_distance,
                                          double yielding_time) {
  YieldingResult yielding_result;
  if (yielding_distance <= 0.0) {
    yielding_result.acceleration = -std::numeric_limits<double>::max();
    yielding_result.wait_time = yielding_time;
    return yielding_result;
  }

  const double t_stop = 2.0 * yielding_distance / (start_v + kEps);
  if (t_stop > yielding_time) {
    yielding_result.acceleration = std::min(
        2.0 * (yielding_distance / Sqr(yielding_time + kEps) -
               start_v / (yielding_time + kEps)),
        0.5 * (Sqr(end_v) - Sqr(start_v)) / (yielding_distance + kEps));
    yielding_result.wait_time = 0.0;
  } else {
    yielding_result.acceleration = -start_v / t_stop;
    yielding_result.wait_time = yielding_time - t_stop;
  }
  return yielding_result;
}

bool NeedPreBrake(const YieldingResult& av_yielding_result,
                  const YieldingResult& obj_yielding_result,
                  StOverlapMetaProto::OverlapPriority av_priority,
                  StOverlapMetaProto::OverlapSource overlap_source,
                  double obj_acceleration) {
  constexpr double kAccLimitMergeCompensation = -0.3;
  constexpr double kAccLimitUpperLimit = -1.0;
  constexpr double kAccHighPriorityFactor = 1.0;
  constexpr double kAccEqualPriorityFactor = 1.3;
  constexpr double kAccLowPriorityFactor = 1.8;
  const PiecewiseLinearFunction<double> obj_acc_factor_plf(
      {-1.0, -0.5, -0.3, 0.3, 0.5, 1.0}, {0.5, 0.8, 1.0, 1.0, 1.35, 2.0});

  double wait_time_limit = 0.0;
  double acc_limit = 0.0;

  switch (av_priority) {
    case StOverlapMetaProto::HIGH: {
      wait_time_limit = 0.0;
      acc_limit = kAccHighPriorityFactor * obj_yielding_result.acceleration;
      break;
    }
    case StOverlapMetaProto::UNKNOWN_PRIORITY:
    case StOverlapMetaProto::EQUAL: {
      wait_time_limit = obj_yielding_result.wait_time;
      acc_limit = kAccEqualPriorityFactor * obj_yielding_result.acceleration;
      break;
    }
    case StOverlapMetaProto::LOW: {
      wait_time_limit = std::numeric_limits<double>::max();
      acc_limit = kAccLowPriorityFactor * obj_yielding_result.acceleration;
      break;
    }
  }
  wait_time_limit += kEps;
  acc_limit = std::min(acc_limit, kAccLimitUpperLimit) *
              obj_acc_factor_plf(obj_acceleration);
  // Av need to be more conservative when merging.
  if (overlap_source == StOverlapMetaProto::LANE_MERGE) {
    acc_limit += kAccLimitMergeCompensation;
  }
  return av_yielding_result.wait_time < wait_time_limit &&
         av_yielding_result.acceleration > acc_limit;
}

inline bool SatisfyPrerequisite(const StBoundary& st_boundary) {
  if (!st_boundary.overlap_meta().has_value()) return false;
  const bool is_priority_lane_cross =
      st_boundary.overlap_meta()->priority() != StOverlapMetaProto::LOW &&
      st_boundary.overlap_meta()->source() == StOverlapMetaProto::LANE_CROSS;
  const double pre_brake_s = is_priority_lane_cross
                                 ? st_boundary.min_s()
                                 : st_boundary.bottom_left_point().s();
  if (st_boundary.object_type() == StBoundaryProto::VEHICLE) {
    return (pre_brake_s > kVehicleEmergencyBrakeDist &&
            st_boundary.bottom_left_point().t() > kVehicleEmergencyBrakeTime);
  } else if (st_boundary.object_type() == StBoundaryProto::CYCLIST) {
    return (pre_brake_s > kCyclistEmergencyBrakeDist &&
            st_boundary.bottom_left_point().t() > kCyclistEmergencyBrakeTime);
  }
  return false;
}

inline bool IgnoreWithoutPreBrake(
    const StBoundary& st_boundary, double current_v,
    StBoundaryProto::DecisionReason decision_reason) {
  if (decision_reason == StBoundaryProto::INTERACTIVE_DECIDER) {
    return false;
  }
  const PiecewiseLinearFunction<double> vehicle_vel_time_limit_plf(
      {0.0, 5.0, 10.0, 20.0}, {3.0, 4.0, 6.0, 10.0});
  const PiecewiseLinearFunction<double> cyclist_vel_time_limit_plf(
      {0.0, 5.0, 10.0, 20.0}, {2.5, 3.0, 4.0, 6.0});

  if (st_boundary.object_type() == StBoundaryProto::VEHICLE) {
    return st_boundary.bottom_left_point().t() >
           vehicle_vel_time_limit_plf(current_v);
  } else if (st_boundary.object_type() == StBoundaryProto::CYCLIST) {
    return st_boundary.bottom_left_point().t() >
           cyclist_vel_time_limit_plf(current_v);
  }
  return false;
}

bool IsUncertainVehicle(const SpacetimeTrajectoryManager& st_traj_mgr,
                        const VehicleGeometryParamsProto& vehicle_geo_params,
                        const DiscretizedPath& path,
                        const StBoundaryWithDecision& st_boundary_wd,
                        const SpeedVector& preliminary_speed) {
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE &&
      st_boundary.object_type() != StBoundaryProto::CYCLIST) {
    return false;
  }
  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));
  const auto& overlap_meta = *st_boundary.overlap_meta();

  const bool is_parallel_merging = IsParallelMerging(overlap_meta);
  const bool is_parallel_front_merging =
      is_parallel_merging &&
      HasYieldingIntentionToFrontAv(path.front(), traj->planner_object(),
                                    vehicle_geo_params, overlap_meta,
                                    preliminary_speed, st_boundary.min_t());
  if (is_parallel_front_merging) return false;

  const auto& overlap_infos = st_boundary.overlap_infos();
  QCHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();

  const auto av_ttc_info =
      GetAvOverlapTtcInfo(overlap_infos, overlap_meta, path, preliminary_speed);
  const auto object_ttc_info =
      GetObjectOverlapTtcInfo(first_overlap_info, *traj);
  const bool is_object_reach_first_aggressive =
      object_ttc_info.ttc_lower_limit < av_ttc_info.ttc_upper_limit;
  const bool is_object_reach_first =
      object_ttc_info.ttc_upper_limit < av_ttc_info.ttc_lower_limit;

  return ((IsObjectInAvFov(path.front(), traj->planner_object()) ||
           IsAggressiveLeading(preliminary_speed, st_boundary.min_t()) ||
           overlap_meta.priority() != StOverlapMetaProto::HIGH) &&
          is_object_reach_first_aggressive) ||
         is_object_reach_first;
}

std::optional<VtSpeedLimit> GenerateUncertainVehicleSpeedLimit(
    const StBoundary& st_boundary, const SpeedVector& preliminary_speed,
    double current_v, double max_v, double time_step, int step_num,
    double av_end_v, double av_yielding_time, double emergency_brake_dist,
    double emergency_brake_vel, double emergency_brake_time,
    bool is_priority_lane_cross,
    StBoundaryProto::DecisionReason decision_reason, const std::string& info) {
  const double pre_brake_dist =
      is_priority_lane_cross
          ? st_boundary.min_s() - emergency_brake_dist
          : st_boundary.bottom_left_point().s() - emergency_brake_dist;
  const double pre_brake_acc =
      is_priority_lane_cross
          ? std::min(kMildAccel, 2.0 * (pre_brake_dist / Sqr(av_yielding_time) -
                                        current_v / av_yielding_time))
          : std::min(
                kMildAccel,
                0.5 * (Sqr(emergency_brake_vel + av_end_v) - Sqr(current_v)) /
                    pre_brake_dist);
  const double pre_brake_time =
      std::abs(pre_brake_acc) > kEps
          ? (sqrt(Sqr(current_v) + 2.0 * pre_brake_acc * pre_brake_dist) -
             current_v) /
                pre_brake_acc
          : pre_brake_dist / (current_v + kEps);
  // NOTE: Speed limit also allows to accelerate mildly (when
  // pre_brake_acc > 0.0). We want to accelerate mildly if current velocity is
  // lower than speed limit.
  constexpr double kMildAccTimeLimit = 5.0;  // s.
  double min_v = 0.0;
  if ((pre_brake_acc > 0.0 && pre_brake_time > kMildAccTimeLimit) ||
      pre_brake_time > preliminary_speed.back().t() + kEps) {
    min_v = max_v;
  }
  if (IgnoreWithoutPreBrake(st_boundary, current_v, decision_reason)) {
    min_v = max_v;
  }
  return GenerateConstAccSpeedLimit(
      /*start_t=*/0.0, pre_brake_time + emergency_brake_time, current_v, min_v,
      max_v, pre_brake_acc, time_step, step_num, info);
}

std::optional<VtSpeedLimit> MakeUncertainVehiclePreBrakeDecisionForStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geo_params, double current_v,
    double max_v, double time_step, int step_num,
    StBoundaryProto::DecisionReason decision_reason,
    const StBoundaryWithDecision& st_boundary_wd,
    const SpeedVector& preliminary_speed) {
  FUNC_QTRACE();

  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();

  if (!SatisfyPrerequisite(st_boundary)) {
    return std::nullopt;
  }
  if (!IsUncertainVehicle(st_traj_mgr, vehicle_geo_params, path, st_boundary_wd,
                          preliminary_speed)) {
    return std::nullopt;
  }
  double standoff_follow_dist = 0.0;
  double emergency_brake_dist = 0.0;
  double emergency_brake_vel = 0.0;
  double emergency_brake_time = 0.0;
  if (st_boundary.object_type() == StBoundaryProto::VEHICLE) {
    standoff_follow_dist = kVehicleStandoffFollowDist;
    emergency_brake_dist = kVehicleEmergencyBrakeDist;
    emergency_brake_vel = kVehicleEmergencyBrakeVel;
    emergency_brake_time = kVehicleEmergencyBrakeTime;
  } else {
    standoff_follow_dist = kCyclistStandoffFollowDist;
    emergency_brake_dist = kCyclistEmergencyBrakeDist;
    emergency_brake_vel = kCyclistEmergencyBrakeVel;
    emergency_brake_time = kCyclistEmergencyBrakeTime;
  }

  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));

  if (decision_reason != StBoundaryProto::INTERACTIVE_DECIDER) {
    const auto& obj_contour = traj->contour();
    const auto& current_path_point = path.front();
    const auto current_path_dir =
        Vec2d::FastUnitFromAngle(current_path_point.theta());
    const auto& obj_most_front_point =
        obj_contour.points()[obj_contour.ExtremePoint(current_path_dir)];
    // To imitate human drivers, objects that not pass the driver of AV will not
    // take into consideration.
    constexpr double kDriverAwarenessFactor = 0.25;
    const double driver_awareness_area =
        vehicle_geo_params.front_edge_to_center() -
        vehicle_geo_params.length() * kDriverAwarenessFactor;
    const bool behind_av_driver =
        (obj_most_front_point - ToVec2d(current_path_point))
            .Dot(current_path_dir) < driver_awareness_area;
    if (behind_av_driver &&
        st_boundary.overlap_meta()->priority() != StOverlapMetaProto::LOW) {
      return std::nullopt;
    }
  }

  const bool is_priority_lane_cross =
      st_boundary.overlap_meta()->priority() != StOverlapMetaProto::LOW &&
      st_boundary.overlap_meta()->source() == StOverlapMetaProto::LANE_CROSS;
  const double av_yielding_time = is_priority_lane_cross
                                      ? st_boundary.max_t()
                                      : st_boundary.bottom_left_point().t();
  const double av_yielding_distance =
      is_priority_lane_cross
          ? st_boundary.min_s() - standoff_follow_dist
          : st_boundary.bottom_left_point().s() - standoff_follow_dist;

  const auto& overlap_infos = st_boundary.overlap_infos();
  QCHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto& last_overlap_info = overlap_infos.back();
  const auto* first_overlap_obj_point =
      traj->states()[first_overlap_info.obj_idx].traj_point;
  const auto& first_overlap_av_point = path[first_overlap_info.av_start_idx];
  const auto& last_overlap_av_point = path[last_overlap_info.av_start_idx];
  const auto av_speed_point =
      first_overlap_av_point.s() > preliminary_speed.back().s()
          ? preliminary_speed.back()
          : preliminary_speed.EvaluateByS(first_overlap_av_point.s());
  const auto theta_diff_cos = fast_math::Cos(first_overlap_av_point.theta() -
                                             first_overlap_obj_point->theta());
  const double obj_end_v =
      is_priority_lane_cross
          ? max_v
          : std::max(0.0, av_speed_point->v() * theta_diff_cos);
  const double av_end_v =
      is_priority_lane_cross
          ? max_v
          : std::max(0.0, first_overlap_obj_point->v() * theta_diff_cos);
  const double obj_start_v = traj->pose().v();

  const double object_yielding_time = CalcObjectYieldingTime(
      overlap_infos, *st_boundary.overlap_meta(), path, preliminary_speed);
  const double obj_yielding_distance =
      first_overlap_obj_point->s() - standoff_follow_dist;
  const auto obj_yielding_result = CalcYieldingAccAndWaitTime(
      obj_start_v, obj_end_v, obj_yielding_distance, object_yielding_time);
  const auto av_yielding_result = CalcYieldingAccAndWaitTime(
      current_v, av_end_v, av_yielding_distance, av_yielding_time);

  if (NeedPreBrake(av_yielding_result, obj_yielding_result,
                   st_boundary.overlap_meta()->priority(),
                   st_boundary.overlap_meta()->source(),
                   traj->planner_object().pose().a())) {
    const auto info =
        absl::StrCat("Pre brake for uncertain vehicle ", *traj_id);
    return GenerateUncertainVehicleSpeedLimit(
        st_boundary, preliminary_speed, current_v, max_v, time_step, step_num,
        av_end_v, av_yielding_time, emergency_brake_dist, emergency_brake_vel,
        emergency_brake_time, is_priority_lane_cross, decision_reason, info);
  } else {
    // For uncertain objects, prediction info maybe inaccurate.
    // Here we use perception-related items to re-calculate pre-brake speed
    // limit.
    const double obj_v_per = traj->planner_object().pose().v();
    const double av_yielding_time_per =
        is_priority_lane_cross
            ? last_overlap_av_point.s() / (current_v + kEps)
            : first_overlap_obj_point->s() / std::max(obj_v_per, kEps);
    const double av_end_v_per = is_priority_lane_cross
                                    ? max_v
                                    : std::max(0.0, obj_v_per * theta_diff_cos);

    const auto obj_yielding_result_per = CalcYieldingAccAndWaitTime(
        obj_v_per, obj_end_v, obj_yielding_distance, object_yielding_time);
    const auto av_yielding_result_per = CalcYieldingAccAndWaitTime(
        current_v, av_end_v_per, av_yielding_distance, av_yielding_time_per);
    if (NeedPreBrake(av_yielding_result_per, obj_yielding_result_per,
                     st_boundary.overlap_meta()->priority(),
                     st_boundary.overlap_meta()->source(),
                     traj->planner_object().pose().a())) {
      const auto info = absl::StrCat(
          "Pre brake with perception info for uncertain vehicle ", *traj_id);
      return GenerateUncertainVehicleSpeedLimit(
          st_boundary, preliminary_speed, current_v, max_v, time_step, step_num,
          av_end_v_per, av_yielding_time_per, emergency_brake_dist,
          emergency_brake_vel, emergency_brake_time, is_priority_lane_cross,
          decision_reason, info);
    }
  }
  return std::nullopt;
}

std::map<std::string, StBoundaryWithDecisionPtrPair> FindModifiedStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  const std::string m_key = "|m";
  std::map<std::string, StBoundaryWithDecisionPtrPair> modified_st_boundary_map;
  for (auto& st_boundary_wd : *st_boundaries_wd) {
    // Consider all lead objects with "|m" tag for pre-brake.
    if (st_boundary_wd.decision_type() != StBoundaryProto::LEAD) {
      continue;
    }

    // Decision made by pre decider is considered as a certain decision.
    if (st_boundary_wd.decision_reason() == StBoundaryProto::PRE_DECIDER) {
      continue;
    }

    if (st_boundary_wd.id().find(m_key) == std::string::npos) {
      continue;
    }

    if (st_boundary_wd.raw_st_boundary()->is_protective()) {
      continue;
    }

    const auto& traj_id = st_boundary_wd.raw_st_boundary()->traj_id();
    if (traj_id.has_value()) {
      if (!ContainsKey(modified_st_boundary_map, *traj_id)) {
        modified_st_boundary_map[*traj_id] = std::make_pair(
            nullptr, std::vector<StBoundaryWithDecision*>{&st_boundary_wd});
      } else {
        modified_st_boundary_map[*traj_id].second.push_back(&st_boundary_wd);
      }
    }
  }

  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if (st_boundary_wd.id().find(m_key) != std::string::npos) {
      continue;
    }

    if (st_boundary_wd.raw_st_boundary()->is_protective()) {
      continue;
    }

    const auto& traj_id = st_boundary_wd.raw_st_boundary()->traj_id();
    if (traj_id.has_value() &&
        ContainsKey(modified_st_boundary_map, *traj_id)) {
      auto& origin_st_boundary_wd = modified_st_boundary_map[*traj_id].first;
      if (origin_st_boundary_wd == nullptr ||
          st_boundary_wd.raw_st_boundary()->min_t() <
              origin_st_boundary_wd->raw_st_boundary()->min_t()) {
        origin_st_boundary_wd = &st_boundary_wd;
      }
    } else if ((st_boundary_wd.decision_reason() ==
                    StBoundaryProto::SAMPLING_DP &&
                st_boundary_wd.decision_type() == StBoundaryProto::LEAD) ||
               st_boundary_wd.decision_reason() ==
                   StBoundaryProto::INTERACTIVE_DECIDER) {
      // For those ignored st-boundary without corresponding modified
      // st-boundary (e.g. object's modified prediction trajectory completely
      // stops before invading AV path), we also want to add pre-brake.
      modified_st_boundary_map[*traj_id] = std::make_pair(
          &st_boundary_wd, std::vector<StBoundaryWithDecision*>{});
    }
  }

  return modified_st_boundary_map;
}

bool IgnoreConflictStBoundary(
    const StBoundary& st_boundary, double time_step,
    const std::optional<VtSpeedLimit>& speed_limit_opt) {
  if (speed_limit_opt.has_value()) {
    const auto& speed_limit = speed_limit_opt.value();
    double s = 0.0;
    for (int i = 1; i < speed_limit.size(); ++i) {
      const double t = static_cast<double>(i) * time_step;
      const auto s_range = st_boundary.GetBoundarySRange(t);
      s += 0.5 * time_step *
           (speed_limit[i].speed_limit + speed_limit[i - 1].speed_limit);
      if (s_range.has_value()) {
        const auto upper_s_limit = s_range->first;
        if (upper_s_limit > s) {
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

std::optional<VtSpeedLimit> MakePedestrainPreBrakeDecision(
    const SpeedFinderParamsProto::PreBrakeDeciderParamsProto& params,
    const SpacetimeTrajectoryManager& st_traj_mgr, double current_v,
    double max_v, double time_step, int step_num,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  FUNC_QTRACE();
  std::optional<VtSpeedLimit> speed_limit = std::nullopt;
  for (auto& st_boundary_wd : *st_boundaries_wd) {
    std::optional<VtSpeedLimit> speed_limit_opt =
        MakePedestrainPreBrakeDecisionForStBoundary(params, st_traj_mgr,
                                                    current_v, max_v, time_step,
                                                    step_num, st_boundary_wd);
    if (speed_limit_opt.has_value()) {
      if (speed_limit.has_value()) {
        MergeVtSpeedLimit(speed_limit_opt.value(), &speed_limit.value());
      } else {
        speed_limit = std::move(speed_limit_opt);
      }
      MakeIgnoreDecision(
          absl::StrCat(st_boundary_wd.id(),
                       " is ignored by pedestrian pre-brake decider"),
          StBoundaryProto::PEDESTRIAN_PRE_BRAKE, &st_boundary_wd);
    } else if (IsLeadPedestrain(st_boundary_wd)) {
      MakeIgnoreDecision(
          absl::StrCat(
              st_boundary_wd.id(),
              "(LEAD pedestrain) is ignored by pedestrian pre-brake decider"),
          StBoundaryProto::PEDESTRIAN_PRE_BRAKE, &st_boundary_wd);
    }
  }
  return speed_limit;
}

std::optional<VtSpeedLimit> MakeUncertainVehiclePreBrakeDecision(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geo_params, double current_v,
    double max_v, double time_step, int step_num,
    const SpeedVector& preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  FUNC_QTRACE();

  std::optional<VtSpeedLimit> speed_limit = std::nullopt;
  std::set<std::string> uncertain_vehicle_set;

  const auto modified_st_boundary_map =
      FindModifiedStBoundary(st_boundaries_wd);

  for (const auto& [_, st_boundary_wd_pair] : modified_st_boundary_map) {
    const auto& [origin_st_boundary_wd, modified_st_boundaries_wd] =
        st_boundary_wd_pair;
    QCHECK_NOTNULL(origin_st_boundary_wd);
    const auto decision_reason =
        modified_st_boundaries_wd.empty()
            ? origin_st_boundary_wd->decision_reason()
            : modified_st_boundaries_wd.front()->decision_reason();
    std::optional<VtSpeedLimit> speed_limit_opt =
        MakeUncertainVehiclePreBrakeDecisionForStBoundary(
            st_traj_mgr, path, vehicle_geo_params, current_v, max_v, time_step,
            step_num, decision_reason, *origin_st_boundary_wd,
            preliminary_speed);
    if (speed_limit_opt.has_value()) {
      if (speed_limit.has_value()) {
        MergeVtSpeedLimit(speed_limit_opt.value(), &speed_limit.value());
      } else {
        speed_limit = std::move(speed_limit_opt);
      }
      if (!modified_st_boundaries_wd.empty()) {
        const auto& object_id =
            modified_st_boundaries_wd.front()->raw_st_boundary()->object_id();
        if (object_id.has_value()) {
          uncertain_vehicle_set.insert(*object_id);
        }
      }
    }
  }
  for (const auto& [_, st_boundary_wd_pair] : modified_st_boundary_map) {
    const auto& [origin_st_boundary_wd, modified_st_boundaries_wd] =
        st_boundary_wd_pair;
    if (modified_st_boundaries_wd.empty()) continue;
    const auto& origin_st_boundary = *origin_st_boundary_wd->raw_st_boundary();
    const auto& object_id =
        modified_st_boundaries_wd.front()->raw_st_boundary()->object_id();
    if (SatisfyPrerequisite(origin_st_boundary) && object_id.has_value() &&
        ContainsKey(uncertain_vehicle_set, *object_id)) {
      for (auto* modified_st_boundary_wd : modified_st_boundaries_wd) {
        MakeIgnoreDecision(
            absl::StrCat(modified_st_boundary_wd->id(),
                         " is ignored by uncertain vehicle pre-brake decider"),
            StBoundaryProto::VEHICLE_PRE_BRAKE, modified_st_boundary_wd);
      }
    } else {
      for (auto* modified_st_boundary_wd : modified_st_boundaries_wd) {
        if (IgnoreConflictStBoundary(
                *modified_st_boundary_wd->raw_st_boundary(), time_step,
                speed_limit)) {
          MakeIgnoreDecision(
              absl::StrCat(modified_st_boundary_wd->id(),
                           " is ignored due to conflict with speed "
                           "limit by pre-brake decider"),
              StBoundaryProto::VEHICLE_PRE_BRAKE, modified_st_boundary_wd);
        }
      }
    }
  }

  return speed_limit;
}

}  // namespace planner
}  // namespace qcraft
