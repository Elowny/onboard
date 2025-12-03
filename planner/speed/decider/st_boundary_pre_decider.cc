#include "onboard/planner/speed/decider/st_boundary_pre_decider.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/ml/act_net_speed/act_net_speed_decider.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/decider/pre_brake_util.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kMildDecel = -0.6;  // m/s^2.

constexpr double kEps = 1e-3;

void MakeHitCurrentAvPositionIgnoreDecision(
    StBoundaryWithDecision* st_boundary_with_decision) {
  // Do not make decision if already have one.
  if (st_boundary_with_decision->decision_type() != StBoundaryProto::UNKNOWN) {
    return;
  }
  // If a prediction trajectory
  const StBoundary* st_boundary = st_boundary_with_decision->st_boundary();
  if (st_boundary->bottom_left_point().s() <= kEps &&
      st_boundary->bottom_left_point().t() >= 0.0) {
    st_boundary_with_decision->set_decision_type(StBoundaryProto::IGNORE);
    st_boundary_with_decision->set_decision_reason(
        StBoundaryProto::IGNORE_TRACKER_DECIDER);
    st_boundary_with_decision->set_decision_info(
        "ignore hit current Av position");
    return;
  }
}

// Currently only for freespace planner
void MakeIgnoreDecisionForDistantObjects(
    StBoundaryWithDecision* st_boundary_with_decision) {
  // Do not make decision if already have one.
  if (st_boundary_with_decision->decision_type() != StBoundaryProto::UNKNOWN) {
    return;
  }
  // If a prediction trajectory && too far (ttc > 5s) from current pose
  if (st_boundary_with_decision->st_boundary()->bottom_left_point().t() > 5.0) {
    st_boundary_with_decision->set_decision_type(StBoundaryProto::IGNORE);
    st_boundary_with_decision->set_decision_reason(
        StBoundaryProto::IGNORE_TRACKER_DECIDER);
    st_boundary_with_decision->set_decision_info("ignore too distant objects");
  }
}

bool MakeConstraintDecisionForStBoundary(
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    StBoundaryWithDecision* st_boundary_with_decision) {
  // Do not make decision if already have one.
  if (st_boundary_with_decision->decision_type() != StBoundaryProto::UNKNOWN) {
    return false;
  }
  const auto& st_boundary = *st_boundary_with_decision->raw_st_boundary();
  if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
    return false;
  }
  QCHECK(st_boundary.traj_id().has_value());
  if (leading_trajs.find(*st_boundary.traj_id()) != leading_trajs.end()) {
    return true;
  }
  return false;
}

std::optional<VtSpeedLimit> MakeParallelCutInPreBrakeDecisionForStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const DrivePassage& drive_passage, double current_v, double max_v,
    double time_step, int step_num,
    const StBoundaryWithDecision& st_boundary_wd) {
  if (st_boundary_wd.decision_type() != StBoundaryProto::UNKNOWN &&
      st_boundary_wd.decision_type() != StBoundaryProto::FOLLOW) {
    return std::nullopt;
  }

  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE) {
    return std::nullopt;
  }

  constexpr double kTimeUpperLimit = 5.0;  // s.
  if (st_boundary.bottom_left_point().t() > kTimeUpperLimit) {
    return std::nullopt;
  }

  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));
  const auto& overlap_infos = st_boundary.overlap_infos();
  QCHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      traj->states()[first_overlap_info.obj_idx].traj_point;

  // Do not need pre brake if object already has overlap with AV.
  if (first_overlap_obj_point->t() < kEps) {
    return std::nullopt;
  }

  const double current_rel_v = traj->planner_object().pose().v() - current_v;
  constexpr double kPreBrakeVelocityThre = 2.0;  // m/s.
  if (std::abs(current_rel_v) > kPreBrakeVelocityThre) {
    return std::nullopt;
  }

  const auto& obj_contour = traj->contour();

  const auto& av_point = path.front();
  const Vec2d av_heading_dir = Vec2d::FastUnitFromAngle(av_point.theta());
  const Vec2d av_left_dir = av_heading_dir.Perp();
  const Vec2d av_pos = Vec2d(av_point.x(), av_point.y());
  const Vec2d& obj_pos = traj->pose().pos();

  constexpr double kParallelHeadingThreUpper = d2r(10.0);
  constexpr double kParallelHeadingThreLower = -d2r(5.0);
  const double sgn = std::copysign(1.0, (obj_pos - av_pos).Dot(av_left_dir));
  double ref_theta = av_point.theta();
  const auto obj_frenet_coord = drive_passage.QueryFrenetCoordinateAt(obj_pos);
  if (obj_frenet_coord.ok()) {
    const auto obj_frenet_theta =
        drive_passage.QueryTangentAngleAtS(obj_frenet_coord->s);
    if (obj_frenet_theta.ok()) {
      ref_theta = *obj_frenet_theta;
    }
  }
  const double theta_diff =
      NormalizeAngle(ref_theta - traj->pose().theta()) * sgn;

  if (theta_diff > kParallelHeadingThreUpper ||
      theta_diff < kParallelHeadingThreLower) {
    return std::nullopt;
  }

  const PiecewiseLinearFunction<double> vel_lat_buffer_plf(
      {0.0, 3.0, 10.0, 20.0}, {0.6, 0.75, 1.0, 1.3});
  const PiecewiseLinearFunction<double> rel_vel_lon_buffer_front_plf(
      {-1.0, 2.0}, {3.0, 0.0});
  const PiecewiseLinearFunction<double> rel_vel_lon_buffer_back_plf(
      {-1.0, 2.0}, {1.0, -1.0});
  const PiecewiseLinearFunction<double> theta_diff_lat_buffer_factor_plf(
      {0.0, d2r(1.0), d2r(2.0), d2r(10.0)}, {1.0, 1.1, 1.3, 1.7});

  const double lon_buffer_back = rel_vel_lon_buffer_back_plf(current_rel_v);
  const double lon_buffer_front = rel_vel_lon_buffer_front_plf(current_rel_v);
  const double lat_buffer = vel_lat_buffer_plf(current_v) *
                            theta_diff_lat_buffer_factor_plf(theta_diff);

  Vec2d front_most, back_most, left_most, right_most;
  obj_contour.ExtremePoints(av_heading_dir, &back_most, &front_most);
  obj_contour.ExtremePoints(av_left_dir, &right_most, &left_most);

  const double front_dis = (front_most - av_pos).Dot(av_heading_dir) -
                           vehicle_geo_params.front_edge_to_center();
  const double back_dis = (back_most - av_pos).Dot(av_heading_dir) -
                          vehicle_geo_params.front_edge_to_center();
  const double left_dis = (left_most - av_pos).Dot(av_left_dir);
  const double right_dis = (right_most - av_pos).Dot(av_left_dir);

  // If the object surpass AV with kLonBufferBackVehicleFraction times its
  // vehicle length (means the driver of the object surpass AV), we take it into
  // consideration.
  constexpr double kLonBufferBackVehicleFraction = 0.25;
  if (front_dis <
          std::min(kLonBufferBackVehicleFraction * (front_dis - back_dis),
                   lon_buffer_back) ||
      back_dis > lon_buffer_front) {
    return std::nullopt;
  }

  if (std::max(-left_dis, right_dis) >
      0.5 * vehicle_geo_params.width() + lat_buffer) {
    return std::nullopt;
  }

  // We do not want to fully stop by pre-brake.
  constexpr double kMinVel = 0.1;  // m/s.
  const double min_v = std::max(
      traj->planner_object().pose().v() - kPreBrakeVelocityThre, kMinVel);
  const double rel_dist = lon_buffer_front - back_dis;
  double decel_time, const_vel_time;

  const double decel_max_time = (min_v - current_v) / kMildDecel;
  const double decel_max_rel_dist =
      -0.5 * kMildDecel * Sqr(decel_max_time) + current_rel_v * decel_max_time;
  if (decel_max_rel_dist < rel_dist) {
    decel_time = decel_max_time;
    const_vel_time =
        (rel_dist - decel_max_rel_dist) /
        std::max(traj->planner_object().pose().v() - min_v, kMinVel);
  } else {
    decel_time = (current_rel_v -
                  sqrt(Sqr(current_rel_v) - 2.0 * kMildDecel * rel_dist)) /
                 kMildDecel;
    const_vel_time = 0.0;
  }

  // Decelerate to -2.0m/s relative velocity with mild brake until AV is fully
  // behind object.
  const auto info = absl::StrCat("Pre brake for parallel vehicle ", *traj_id);
  return GenerateConstAccSpeedLimit(
      /*start_t=*/0.0, decel_time + const_vel_time, current_v, min_v, max_v,
      kMildDecel, time_step, step_num, info);
}

bool MakeLaneChangeLeadDecisionForStBoundary(
    const absl::flat_hash_set<std::string>& follower_set,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const StBoundaryWithDecision& st_boundary_with_decision, double current_v) {
  const auto& st_boundary = *st_boundary_with_decision.raw_st_boundary();
  if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
    return false;
  }
  QCHECK(st_boundary.object_id().has_value());
  if (follower_set.find(*st_boundary.object_id()) != follower_set.end()) {
    return true;
  }
  constexpr double kQuickStopMaxVel = 3.0;  // m/s.
  if (current_v < kQuickStopMaxVel) return false;
  if (!st_boundary.overlap_meta().has_value() ||
      !st_boundary.overlap_meta()->has_front_most_projection_distance() ||
      st_boundary.overlap_meta()->source() != StOverlapMetaProto::AV_CUTIN) {
    return false;
  }
  constexpr double kSafetyDistance = 4.0;  // m.
  const double decel_dist =
      -st_boundary.overlap_meta()->front_most_projection_distance() -
      vehicle_geo_params.back_edge_to_center() - kSafetyDistance;
  if (decel_dist <= 0.0) return false;
  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));
  const double obj_vel = std::abs(traj->planner_object().pose().v());
  const double rel_vel = obj_vel - current_v;
  if (rel_vel <= 0.0) return true;
  constexpr double kMildDecelInv = 1.0 / 1.2;  // m/ss.
  constexpr double kReactionTime = 0.5;        // s.
  return kReactionTime * rel_vel + 0.5 * Sqr(rel_vel) * kMildDecelInv <
         decel_dist;
}

bool MakeLargeVehicleBlindSpotDecisionForStBoundary(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const StBoundaryWithDecision& st_boundary_wd) {
  if (st_boundary_wd.decision_type() != StBoundaryProto::UNKNOWN) {
    return false;
  }

  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  QCHECK(st_boundary.traj_id().has_value());
  const auto& traj_id = st_boundary.traj_id();
  const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));
  const auto& planner_object = traj->planner_object();
  const auto& raw_obj_contour = planner_object.raw_contour();
  const Vec2d& obj_pos = planner_object.pose().pos();
  const auto obj_frenet_box =
      drive_passage.QueryFrenetBoxAtContour(raw_obj_contour);
  const auto obj_frenet_coord = drive_passage.QueryFrenetCoordinateAt(obj_pos);
  if (!obj_frenet_box.ok() || !obj_frenet_coord.ok()) return false;

  const bool is_on_left = obj_frenet_coord->l > 0.0;
  const double center_s = 0.5 * (obj_frenet_box->s_max + obj_frenet_box->s_min);
  const auto ref_theta = drive_passage.QueryTangentAngleAtS(center_s);
  QCHECK(ref_theta.ok());
  const double theta_diff =
      NormalizeAngle(planner_object.pose().theta() - *ref_theta);
  constexpr double kLargeVehicleCutInHeadingThres = d2r(0.75);  // 0.75 deg.
  const bool has_cutin_intention =
      is_on_left ? theta_diff < -kLargeVehicleCutInHeadingThres
                 : theta_diff > kLargeVehicleCutInHeadingThres;
  if (!has_cutin_intention) {
    return false;
  }

  const auto boundary = drive_passage.QueryEnclosingLaneBoundariesAtS(center_s);
  if (!boundary.right.has_value() || !boundary.left.has_value()) {
    return false;
  }

  constexpr double kHalfLaneLineWidth = 0.1;  // m.
  const bool has_invaded_ego_lane =
      is_on_left ? obj_frenet_box->l_min <
                       boundary.left->lat_offset + kHalfLaneLineWidth
                 : obj_frenet_box->l_max >
                       boundary.right->lat_offset - kHalfLaneLineWidth;
  if (!has_invaded_ego_lane) {
    return false;
  }

  constexpr double kLargeVehicleLatBuffer = 0.8;  // m.
  const double min_passable_width =
      vehicle_geo_params.width() + 2.0 * kLargeVehicleLatBuffer;
  const double passable_width =
      is_on_left ? obj_frenet_box->l_min - boundary.right->lat_offset
                 : boundary.left->lat_offset - obj_frenet_box->l_max;

  return passable_width < min_passable_width;
}

void MakePreDecisionForStBoundary(
    const SpeedFinderParamsProto::StBoundaryPreDeciderParamsProto& params,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const absl::flat_hash_set<std::string>& follower_set,
    const TrafficGapResult& lane_change_gap,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const DrivePassage& drive_passage, double current_v, double max_v,
    double time_step, int step_num, StBoundaryWithDecision* st_boundary_wd,
    std::optional<VtSpeedLimit>* speed_limit) {
  const auto make_decision = [](const std::string& decision_info,
                                StBoundaryProto::DecisionType decision_type,
                                StBoundaryWithDecision* st_boundary_wd) {
    st_boundary_wd->set_decision_type(decision_type);
    st_boundary_wd->set_decision_reason(StBoundaryProto::PRE_DECIDER);
    st_boundary_wd->set_decision_info(decision_info);
  };

  if (!st_boundary_wd->raw_st_boundary()->is_protective()) {
    // 1. Make pre decision from upstream input.
    if (MakeConstraintDecisionForStBoundary(leading_trajs, st_boundary_wd)) {
      make_decision(
          absl::StrCat(
              st_boundary_wd->id(),
              " is assigned a follow decision due to upstream constraint"),
          StBoundaryProto::FOLLOW, st_boundary_wd);
      return;
    }

    // 2. Make pre decision for parallel cut-in and add pre-brake.
    if (params.enable_parallel_cut_in_pre_brake()) {
      std::optional<VtSpeedLimit> speed_limit_opt =
          MakeParallelCutInPreBrakeDecisionForStBoundary(
              st_traj_mgr, path, vehicle_geo_params, drive_passage, current_v,
              max_v, time_step, step_num, *st_boundary_wd);
      if (speed_limit_opt.has_value()) {
        if (speed_limit->has_value()) {
          MergeVtSpeedLimit(speed_limit_opt.value(), &speed_limit->value());
        } else {
          *speed_limit = std::move(speed_limit_opt);
        }
        make_decision(
            absl::StrCat(
                st_boundary_wd->id(),
                " is assigned a follow decision due to parallel cut-in"),
            StBoundaryProto::FOLLOW, st_boundary_wd);
        return;
      }
    }

    // 3. Make lane change lead decision.
    if (MakeLaneChangeLeadDecisionForStBoundary(follower_set, st_traj_mgr,
                                                vehicle_geo_params,
                                                *st_boundary_wd, current_v)) {
      if (params.enable_lane_change_lead_decision()) {
        make_decision(
            absl::StrCat(st_boundary_wd->id(),
                         " is assigned a lead decision due to lane change"),
            StBoundaryProto::LEAD, st_boundary_wd);
      } else {
        // We don't want to make aggressive lane change lead decision for bus.
        make_decision(
            absl::StrCat(st_boundary_wd->id(),
                         " is assigned a ignore decision due to lane change"),
            StBoundaryProto::IGNORE, st_boundary_wd);
      }
      return;
    }
  } else {
    // 4. Make lane change gap decision.
    if (st_boundary_wd->raw_st_boundary()->protection_type() ==
        StBoundaryProto::LANE_CHANGE_GAP) {
      if (lane_change_gap.leader_id.has_value() &&
          st_boundary_wd->object_id() == *lane_change_gap.leader_id) {
        make_decision(
            absl::StrCat(
                st_boundary_wd->id(),
                " is assigned a follow decision due to lane change gap"),
            StBoundaryProto::FOLLOW, st_boundary_wd);
        return;
      } else if (lane_change_gap.follower_id.has_value() &&
                 st_boundary_wd->object_id() == *lane_change_gap.follower_id) {
        make_decision(
            absl::StrCat(st_boundary_wd->id(),
                         " is assigned a lead decision due to lane change gap"),
            StBoundaryProto::LEAD, st_boundary_wd);
        return;
      }
    }

    // 5. Make large vehicle blind spot decision.
    if (st_boundary_wd->raw_st_boundary()->protection_type() ==
            StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT &&
        MakeLargeVehicleBlindSpotDecisionForStBoundary(
            st_traj_mgr, drive_passage, vehicle_geo_params, *st_boundary_wd)) {
      make_decision(
          absl::StrCat(
              st_boundary_wd->id(),
              " is assigned a follow decision due to large vehicle blind spot"),
          StBoundaryProto::FOLLOW, st_boundary_wd);
      return;
    }
  }
}

}  // namespace

void MakeFreespacePreDecisionForStBoundaries(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  QCHECK_NOTNULL(st_boundaries_with_decision);
  for (auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    if (st_boundary_with_decision.st_boundary()->source_type() ==
        StBoundarySourceTypeProto::ST_OBJECT) {
      MakeHitCurrentAvPositionIgnoreDecision(&st_boundary_with_decision);
      MakeIgnoreDecisionForDistantObjects(&st_boundary_with_decision);
    }
  }
}

void MakePreDecisionForStBoundaries(
    const PreDeciderInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::optional<VtSpeedLimit>* speed_limit) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(input.params);
  QCHECK_NOTNULL(input.leading_trajs);
  QCHECK_NOTNULL(input.follower_set);
  QCHECK_NOTNULL(input.lane_change_gap);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.path);
  QCHECK_NOTNULL(input.vehicle_geo_params);
  QCHECK_NOTNULL(input.drive_passage);
  QCHECK_GT(input.max_v, 0.0);
  QCHECK_GT(input.time_step, 0.0);
  QCHECK_GT(input.trajectory_steps, 0);

  QCHECK_NOTNULL(st_boundaries_with_decision);
  QCHECK_NOTNULL(speed_limit);
  // <id of the corresponding original st-boundary, protective st-boundary
  // pointer>
  absl::flat_hash_map<std::string, StBoundaryWithDecision*>
      protective_st_boundary_wd_map;
  for (auto& st_boundary_wd : *st_boundaries_with_decision) {
    if (!st_boundary_wd.raw_st_boundary()->is_protective() ||
        st_boundary_wd.raw_st_boundary()->protection_type() ==
            StBoundaryProto::LANE_CHANGE_GAP ||
        st_boundary_wd.raw_st_boundary()->protection_type() ==
            StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT) {
      continue;
    }
    const auto& protected_st_boundary_id =
        st_boundary_wd.raw_st_boundary()->protected_st_boundary_id();
    if (!protected_st_boundary_id.has_value()) continue;
    protective_st_boundary_wd_map.emplace(*protected_st_boundary_id,
                                          &st_boundary_wd);
  }

  for (auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    if (st_boundary_with_decision.raw_st_boundary()->is_protective() &&
        st_boundary_with_decision.raw_st_boundary()->protection_type() !=
            StBoundaryProto::LANE_CHANGE_GAP &&
        st_boundary_with_decision.raw_st_boundary()->protection_type() !=
            StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT) {
      continue;
    }
    MakePreDecisionForStBoundary(
        *input.params, *input.leading_trajs, *input.follower_set,
        *input.lane_change_gap, *input.st_traj_mgr, *input.path,
        *input.vehicle_geo_params, *input.drive_passage, input.current_v,
        input.max_v, input.time_step, input.trajectory_steps,
        &st_boundary_with_decision, speed_limit);
    if (const auto protective_st_boundary_wd_ptr = FindOrNull(
            protective_st_boundary_wd_map, st_boundary_with_decision.id());
        nullptr != protective_st_boundary_wd_ptr) {
      auto& protective_st_boundary_wd = **protective_st_boundary_wd_ptr;
      // The decision of protective st-boundary follows the decision of the
      // corresponding original st-boundary.
      if (protective_st_boundary_wd.decision_type() !=
          st_boundary_with_decision.decision_type()) {
        protective_st_boundary_wd.set_decision_type(
            st_boundary_with_decision.decision_type());
        protective_st_boundary_wd.set_decision_reason(
            StBoundaryProto::FOLLOW_PROTECTED);
        protective_st_boundary_wd.set_ignore_reason(
            st_boundary_with_decision.ignore_reason());
        protective_st_boundary_wd.set_decision_info(
            st_boundary_with_decision.decision_info());
      }
    }
  }

  // Add decision if unknown.
  if (input.run_act_net_speed_decision) {
    const auto act_net_speed_decision_state =
        ActNetMakeInteractiveSpeedDecision(
            ActNetInteractionDecisionInput{
                .av_path = input.path,
                .st_traj_mgr = input.st_traj_mgr,
                .av_context = input.planner_av_context,
                .real_objects = input.real_objects,
                .virtual_objects = input.virtual_objects,
                .planner_model_pool = input.planner_model_pool,
                .plan_time = input.plan_time,
                .vehicle_geom = input.vehicle_geo_params,
            },
            st_boundaries_with_decision);
    QCHECK_OK(act_net_speed_decision_state);
  }
}

}  // namespace planner
}  // namespace qcraft
