#include "onboard/planner/speed/speed_optimizer_config_dispatcher.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_approx_overlap.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner {
namespace {

using SpeedOptimizerParamsProto =
    SpeedFinderParamsProto::SpeedOptimizerParamsProto;

inline bool IsObjectExceedAvFrontEdge(
    const SpacetimeObjectTrajectory& traj, const PathPoint& av_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  QCHECK_GT(traj.states().size(), 0);
  const auto& first_state = traj.states()[0];
  const Vec2d av_tan = Vec2d::FastUnitFromAngle(av_point.theta());
  const Vec2d obj_center = first_state.contour.CircleCenter();
  return av_tan.Dot(obj_center - ToVec2d(av_point)) +
             first_state.contour.CircleRadius() >
         vehicle_geometry_params.front_edge_to_center();
}

inline bool IsObjectAlmostParallelWithAv(const SpacetimeObjectTrajectory& traj,
                                         const PathPoint& av_point) {
  QCHECK_GT(traj.states().size(), 0);
  const auto& first_state = traj.states()[0];
  const double heading_diff =
      std::abs(AngleDifference(av_point.theta(), first_state.box.heading()));
  constexpr double kMaxHeadingDiffThres = d2r(45.0);  // 45 degrees.
  return heading_diff < kMaxHeadingDiffThres;
}

// Determine whether the path collides with the linear predicted area of
// the object.
bool HasOverlapWithLinearPredictedObject(
    const SpacetimeObjectTrajectory& st_traj, const PathApprox& path_approx,
    double av_radius, int path_last_index, double path_step) {
  const auto& origin_contour = st_traj.planner_object().contour();
  constexpr double kForwardTime = 2.5;  // s.
  const auto transform_contour =
      origin_contour.Shift(kForwardTime * st_traj.planner_object().velocity());
  const Polygon2d sweeped_area =
      Polygon2d::MergeTwoPolygons(origin_contour, transform_contour);
  constexpr double kLatBuffer = 0.5;  // m.
  const Polygon2d sweeped_area_with_buffer =
      sweeped_area.ExpandByDistance(kLatBuffer);

  constexpr double kSearchRadiusBuffer = 0.2;  // m.
  const double search_radius =
      av_radius + sweeped_area_with_buffer.CircleRadius() + kSearchRadiusBuffer;
  return HasPathApproxOverlapWithPolygon(path_approx, path_step,
                                         /*first_index=*/0, path_last_index,
                                         sweeped_area_with_buffer,
                                         search_radius);
}

std::optional<std::string> IsInLowSpeedCreepMode(
    absl::Span<const StBoundaryWithDecision> st_boundaries_wd,
    const SpacetimeTrajectoryManager& traj_mgr,
    const DrivePassage* drive_passage, const PathPoint& current_path_point,
    const SegmentMatcherKdtree& path_kd_tree) {
  if (drive_passage != nullptr) {
    const auto av_frenet =
        drive_passage->QueryFrenetCoordinateAt(ToVec2d(current_path_point));
    if (!av_frenet.ok()) return std::nullopt;
    const auto av_s_offset =
        drive_passage->QueryNearestBoundaryLateralOffset(av_frenet->s);
    if (!av_s_offset.ok()) return std::nullopt;
    // Av lateral pos is outside the target lane.
    if (av_frenet->l < av_s_offset->first ||
        av_frenet->l > av_s_offset->second) {
      return std::nullopt;
    }
    // Check forward intersection.
    constexpr double kLookForwardDist = 100.0;  // m.
    constexpr double kLookForwardStep = 2.0;    // m.
    const double end_s = std::min(kLookForwardDist, drive_passage->end_s());
    for (double s = av_frenet->s; s < end_s; s += kLookForwardStep) {
      if (drive_passage->FindNearestStationAtS(s).is_in_intersection()) {
        return std::nullopt;
      }
    }
  }

  absl::flat_hash_set<std::string> processed_object_id;
  std::optional<std::string> nearest_on_path_object_id;
  double nearest_on_path_object_s = 0.0;
  for (auto& st_boundary_wd : st_boundaries_wd) {
    const auto& object_id = st_boundary_wd.object_id();
    if (!object_id.has_value() || processed_object_id.contains(*object_id)) {
      continue;
    }
    const auto& traj_id = st_boundary_wd.traj_id();
    if (!traj_id.has_value()) continue;
    const SpacetimeObjectTrajectory* traj =
        QCHECK_NOTNULL(traj_mgr.FindTrajectoryById(*traj_id));
    if (!traj->long_term_behavior().is_slow_front_vehicle) continue;
    const StBoundary& raw_st_boundary = *st_boundary_wd.raw_st_boundary();
    if (raw_st_boundary.object_type() != StBoundaryProto::VEHICLE) {
      continue;
    }
    if (st_boundary_wd.decision_type() != StBoundaryProto::FOLLOW) continue;
    if (raw_st_boundary.overlap_meta()->pattern() != StOverlapMetaProto::STAY) {
      continue;
    }
    if (raw_st_boundary.min_t() > 0.0) continue;
    const auto& obj_pos = traj->planner_object().pose().pos();
    double obj_pos_s, obj_pos_l;
    if (path_kd_tree.GetProjection(obj_pos.x(), obj_pos.y(), /*is_clamp=*/false,
                                   &obj_pos_s, &obj_pos_l)) {
      constexpr double kLateralThres = 0.8;  // m.
      if (std::abs(obj_pos_l) > kLateralThres) continue;
      if (!nearest_on_path_object_id.has_value() ||
          obj_pos_s < nearest_on_path_object_s) {
        nearest_on_path_object_id = *object_id;
        nearest_on_path_object_s = obj_pos_s;
      }
    }
    processed_object_id.insert(*object_id);
  }
  return nearest_on_path_object_id;
}

std::optional<std::string> IsInSafetyMode(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr, const PathApprox& path_approx,
    const SegmentMatcherKdtree& /*path_kd_tree*/, double av_radius,
    double path_step_length, int path_last_index, double current_v,
    const PathPoint& current_path_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  constexpr double kSafetyCheckTime = 2.0;  // s.
  constexpr double kSafetyBuffer = 1.5;     // m.
  constexpr double kProbThres = 0.2;
  constexpr double kTimeStep = 0.1;  // s.

  absl::flat_hash_map<std::string, double> object_id_to_prob;
  absl::flat_hash_map<std::string, bool> obj_collision_ret;
  for (const auto& stb_wd : st_boundaries_with_decision) {
    const StBoundary* raw_st_boundary = stb_wd.raw_st_boundary();
    if (raw_st_boundary->source_type() !=
        StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (stb_wd.decision_type() == StBoundaryProto::UNKNOWN ||
        stb_wd.decision_type() == StBoundaryProto::IGNORE) {
      continue;
    }

    const auto& overlap_meta = raw_st_boundary->overlap_meta();
    const bool is_consider_back_obj =
        overlap_meta.has_value() &&
        (overlap_meta->source() == StOverlapMetaProto::LANE_MERGE ||
         overlap_meta->source() == StOverlapMetaProto::AV_CUTIN);

    const auto& traj_id = stb_wd.traj_id();
    QCHECK(traj_id.has_value());
    const auto* st_traj = QCHECK_NOTNULL(traj_mgr.FindTrajectoryById(*traj_id));
    if (!is_consider_back_obj &&
        !IsObjectExceedAvFrontEdge(*st_traj, current_path_point,
                                   vehicle_geometry_params)) {
      continue;
    }

    const auto& object_id = raw_st_boundary->object_id();
    QCHECK(object_id.has_value());
    constexpr double kEps = 1e-3;
    if (stb_wd.decision_type() == StBoundaryProto::FOLLOW &&
        st_traj->is_hard_braking() &&
        IsObjectAlmostParallelWithAv(*st_traj, current_path_point) &&
        raw_st_boundary->min_t() < kEps) {
      return *object_id;
    }

    if (!obj_collision_ret.contains(*object_id)) {
      obj_collision_ret.emplace(
          *object_id, HasOverlapWithLinearPredictedObject(
                          *st_traj, path_approx, av_radius, path_last_index,
                          path_step_length));
    }

    if (!FindOrDie(obj_collision_ret, *object_id)) {
      continue;
    }

    for (double t = 0.0; t < kSafetyCheckTime; t += kTimeStep) {
      if (!InRange(t, raw_st_boundary->min_t(), raw_st_boundary->max_t())) {
        continue;
      }
      const auto s_range = raw_st_boundary->GetBoundarySRange(t);
      QCHECK(s_range.has_value());
      if (current_v * t > s_range->second - kSafetyBuffer) {
        object_id_to_prob[*object_id] += raw_st_boundary->probability();
        if (FindOrDie(object_id_to_prob, *object_id) > kProbThres) {
          return *object_id;
        }
        break;
      }
    }
  }
  return std::nullopt;
}

void DispatchSpeedOptimizerConfigBySafetyMode(
    SpeedOptimizerParamsProto* speed_optimizer_param) {
  QCHECK_NOTNULL(speed_optimizer_param);

  speed_optimizer_param->set_enable_prediction_impact_factor_decay(false);
  speed_optimizer_param->set_enable_comfort_brake_speed(false);
  speed_optimizer_param->set_enable_const_speed_ref_v(true);
  constexpr double kSafetyModePredImpactFactor = 1.0;
  speed_optimizer_param->set_prediction_impact_factor(
      kSafetyModePredImpactFactor);
  speed_optimizer_param->set_enable_lead_decision(false);
  constexpr double kSFollowWeakWeight = 2.5;
  speed_optimizer_param->set_s_follow_weak_weight(kSFollowWeakWeight);
  constexpr double kSFollowStrongWeight = 10.0;
  speed_optimizer_param->set_s_follow_strong_weight(kSFollowStrongWeight);
}

}  // namespace

std::optional<SpeedOptimizerParamsProto> DispatchSpeedOptimizerConfig(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr,
    const DrivePassage* drive_passage, const PathApprox& path_approx,
    const SegmentMatcherKdtree& path_kd_tree, double av_radius,
    double path_step_length, int path_last_index, double current_v,
    const PathPoint& current_path_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedOptimizerParamsProto& raw_speed_optimizer_params,
    const SpeedFinderParamsProto::SpeedOptimizerConfigDispatcherParams&
    /*config_dispatcher_params*/,
    SpeedFinderDebugProto* speed_finder_debug) {
  QCHECK_NOTNULL(speed_finder_debug);
  FUNC_QTRACE();
  std::optional<SpeedOptimizerParamsProto> speed_optimizer_params;

  if (!speed_optimizer_params.has_value()) {
    speed_optimizer_params = raw_speed_optimizer_params;
  }
  const auto safety_mode_obj_id =
      IsInSafetyMode(st_boundaries_with_decision, traj_mgr, path_approx,
                     path_kd_tree, av_radius, path_step_length, path_last_index,
                     current_v, current_path_point, vehicle_geometry_params);

  if (safety_mode_obj_id.has_value()) {
    QEVENT_EVERY_N_SECONDS("pingshi", "safety_mode_triggered", 2.0,
                           [&](QEvent* qevent) {
                             qevent->AddField("object_id", *safety_mode_obj_id);
                           });
    DispatchSpeedOptimizerConfigBySafetyMode(&(*speed_optimizer_params));
  }

  const auto slow_front_obj_id =
      IsInLowSpeedCreepMode(st_boundaries_with_decision, traj_mgr,
                            drive_passage, current_path_point, path_kd_tree);
  if (slow_front_obj_id.has_value()) {
    QEVENT_EVERY_N_SECONDS("pingshi", "low_speed_creep_mode_triggered", 2.0,
                           [&](QEvent* qevent) {
                             qevent->AddField("object_id", *slow_front_obj_id);
                           });
    speed_finder_debug->set_slow_front_vehicle(*slow_front_obj_id);
  }

  return speed_optimizer_params;
}

}  // namespace qcraft::planner
