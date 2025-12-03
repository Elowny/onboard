#include "onboard/planner/common/lane_change_safety.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <float.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <ostream>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/lane_change_safety_params.h"
#include "onboard/planner/common/lane_change_safety_util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

inline bool HasEnteredTargetLane(double center_l, double half_width) {
  return std::abs(center_l) < kEnterTargetLateralThreshold + half_width;
}

inline bool IsBlockingObjectAbreast(const FrenetBox& ego_box,
                                    const FrenetBox& obj_box,
                                    double obj_front_ext) {
  return HasEnteredTargetLane(obj_box.center_l(), 0.5 * obj_box.width()) &&
         ego_box.s_max > obj_box.s_min &&
         obj_box.s_max + std::max(kMinLonBufferToFront, obj_front_ext) >
             ego_box.s_min;
}

inline Box2d LerpBox2d(const Box2d& box1, const Box2d& box2, double t) {
  // Assume same size.
  return Box2d(box1.half_length(), box1.half_width(),
               Lerp(box1.center(), box2.center(), t),
               LerpAngle(box1.heading(), box2.heading(), t));
}

absl::StatusOr<Box2d> FindPredictedObjectBox(
    absl::Span<const SpacetimeObjectState> obj_states,
    double start_time_offset) {
  const auto iter =
      std::lower_bound(obj_states.begin(), obj_states.end(), start_time_offset,
                       [](const SpacetimeObjectState& state, double t) {
                         return state.traj_point->t() < t;
                       });
  if (iter == obj_states.end()) {
    return absl::NotFoundError(
        "The given time offset is not covered by the predicted trajectory.");
  }
  if (iter == obj_states.begin()) return iter->box;

  const auto prev_iter = std::prev(iter);
  return LerpBox2d(prev_iter->box, iter->box,
                   (start_time_offset - prev_iter->traj_point->t()) /
                       (iter->traj_point->t() - prev_iter->traj_point->t()));
}

bool IsLeavingTargetLanePath(
    const FrenetFrame& target_frenet_frame, bool ego_lc_left,
    const FrenetBox& ego_cur_box, const FrenetBox& obj_cur_box,
    absl::Span<const SpacetimeObjectState> obj_states) {
  constexpr double kPredPreviewTime = 5.0;  // s.
  const auto obj_preview_box_or =
      FindPredictedObjectBox(obj_states, kPredPreviewTime);
  const auto obj_preview_pos = obj_preview_box_or.ok()
                                   ? obj_preview_box_or->center()
                                   : obj_states.back().traj_point->pos();
  const double obj_preview_l = target_frenet_frame.XYToSL(obj_preview_pos).l;
  const double obj_width = obj_cur_box.width();
  if (HasEnteredTargetLane(obj_preview_l, 0.5 * obj_width)) return false;

  const double obj_cur_l = obj_cur_box.center_l();
  const bool obj_lc_left = obj_preview_l > obj_cur_l;
  if ((ego_lc_left == obj_lc_left && ego_cur_box.s_max < obj_cur_box.s_min) ||
      (ego_lc_left != obj_lc_left &&
       ego_cur_box.s_min < obj_cur_box.s_max + kMinLonBufferToFront)) {
    return false;
  }

  constexpr double kLeaveLaneLatRatio = 0.15;
  return (obj_lc_left && obj_cur_box.l_min > -kLeaveLaneLatRatio * obj_width) ||
         (!obj_lc_left && obj_cur_box.l_max < kLeaveLaneLatRatio * obj_width);
}

bool PathHasOverlap(absl::Span<const Box2d> ego_boxes, double obj_v,
                    absl::Span<const SpacetimeObjectState> obj_states) {
  constexpr double kFrontExtensionTime = 2.0;    // s.
  constexpr double kLateralExtension = 2 * 0.5;  // m.
  // First check the current state.
  auto obj_cur_ext_box =
      obj_states[0].box.ExtendedAtFront(obj_v * kFrontExtensionTime);
  obj_cur_ext_box.LateralExtend(kLateralExtension);
  if (ego_boxes[0].HasOverlap(obj_cur_ext_box)) return true;

  // Check the whole path.
  for (const auto& obj_state : obj_states) {
    Box2d obj_ext_box = obj_state.box;
    obj_ext_box.LateralExtend(kLateralExtension);
    for (const auto& ego_box : ego_boxes) {
      if (ego_box.HasOverlap(obj_ext_box)) {
        return true;
      }
    }
  }
  return false;
}

double ComputeEnterTargetTime(
    const FrenetFrame& target_frenet_frame,
    absl::Span<const SpacetimeObjectState> obj_states) {
  const double obj_half_width = obj_states[0].box.width() * 0.5;
  for (const auto& state : obj_states) {
    const auto obj_sl = target_frenet_frame.XYToSL(state.traj_point->pos());
    if (HasEnteredTargetLane(obj_sl.l, obj_half_width)) {
      return state.traj_point->t();
    }
  }
  return DBL_MAX;
}

absl::Status CheckDeceleration(double lon_dist, absl::string_view name_lead,
                               absl::string_view name_follow, double v_lead,
                               double v_follow, double response_time,
                               double lead_time, double max_allowed_decel,
                               double* max_decel = nullptr) {
  const double buffered_lon_dist =
      lon_dist -
      ComputeSafeResponseInterval(v_lead, v_follow, response_time, lead_time);
  if (buffered_lon_dist < 0.0) {
    return absl::CancelledError(
        absl::StrFormat("No space left for %s to decelerate behind %s.",
                        name_follow, name_lead));
  }

  const double v_diff = std::max(0.0, v_follow - v_lead);
  const double decel = Sqr(v_diff) / (2.0 * buffered_lon_dist);
  if (decel > max_allowed_decel) {
    return absl::CancelledError(
        absl::StrFormat("Too large deceleration (-%.2fm/s^2) for %s behind %s.",
                        decel, name_follow, name_lead));
  }
  if (max_decel != nullptr && decel > *max_decel) *max_decel = decel;

  return absl::OkStatus();
}

AlignedObjectInfo CreateAlignedObjectInfo(
    const std::string& id, const std::vector<Vec2d>& contour_points,
    double heading, double v, double a) {
  AlignedObjectInfo proto;
  proto.set_id(id.data());
  for (const auto& pt : contour_points) {
    pt.ToProto(proto.add_contour_points());
  }
  proto.set_heading(heading);
  proto.set_v(v);
  proto.set_a(a);
  return proto;
}

double CalculateMaxAllowedDec(ObjectType obj_type, double obj_v) {
  if (obj_type != OT_LARGE_VEHICLE) {
    return kMaxAllowedDecelForObject;
  }
  // For large vehicle.
  constexpr double kLowSpeedThreshold = 4.167;  // m/s, i.e. 15kph.
  if (obj_v < kLowSpeedThreshold) {
    return kMaxAllowedDecelForSlowLargeObject;
  }
  return kMaxAllowedDecelForFastLargeObject;
}
}  // namespace

absl::Status CheckLaneChangeSafety(
    const std::vector<ApolloTrajectoryPointProto>& ego_traj_pts,
    const FrenetFrame& target_frenet_frame, double speed_limit,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geom, LaneChangeStyle lc_style,
    absl::Duration path_look_ahead_duration,
    absl::flat_hash_set<std::string>* follower_set, double* follower_max_decel,
    std::string* unsafe_object_id, LaneChangeSafetyDebugProto* debug_proto) {
  const int ego_traj_size = ego_traj_pts.size();
  std::vector<Box2d> ego_boxes;
  ego_boxes.reserve(ego_traj_size);
  int enter_target_idx = -1;
  const double ego_half_width = vehicle_geom.width() * 0.5;
  for (int i = 0; i < ego_traj_size; ++i) {
    const auto& traj_pt = ego_traj_pts[i];
    const Vec2d traj_pos = Vec2dFromApolloTrajectoryPointProto(traj_pt);
    const auto ego_sl = target_frenet_frame.XYToSL(traj_pos);
    ego_boxes.push_back(
        ComputeAvBox(traj_pos, traj_pt.path_point().theta(), vehicle_geom));
    if (enter_target_idx == -1 &&
        HasEnteredTargetLane(ego_sl.l, ego_half_width)) {
      enter_target_idx = i;
    }
  }

  constexpr int kMustEnterTargetStep = 0.8 * kTrajectorySteps;
  if (enter_target_idx == -1 || enter_target_idx > kMustEnterTargetStep) {
    // If the trajectory does not enter the target lane in time, consider as
    // unsafe to enter now.
    return absl::CancelledError("Trajectory not entering the target lane.");
  }
  const auto& ego_cur_box = ego_boxes.front();
  ASSIGN_OR_RETURN(
      const auto ego_cur_frenet_box,
      target_frenet_frame.QueryFrenetBoxAt(ego_cur_box),
      _ << "Cannot project the current ego box onto drive passage.");
  const double preview_time = FLAGS_planner_main_loop_interval *
                              FLAGS_planner_async_low_freq_cycle_iterations;
  const double ego_v =
      ego_traj_pts.front().v() + ego_traj_pts.front().a() * preview_time;

  *debug_proto->mutable_ego_aligned_info() = CreateAlignedObjectInfo(
      /*id=*/"ego", ego_cur_box.GetCornersCounterClockwise(),
      ego_cur_box.heading(), ego_v, ego_traj_pts.front().a());
  // Check for stationary objects.
  constexpr double kStationaryBuffer = 0.3;  // m.
  for (const auto* traj_ptr : st_traj_mgr.stationary_object_trajs()) {
    // Use contour rather than bounding box for stationary objects since their
    // heading might be largely deviated.
    const auto& obj_contour = traj_ptr->contour();
    ASSIGN_OR_CONTINUE(
        const auto obj_frenet_box,
        target_frenet_frame.QueryFrenetBoxAtContour(obj_contour));
    const auto& object = traj_ptr->planner_object();
    *debug_proto->add_object_aligned_infos() = CreateAlignedObjectInfo(
        object.id(), obj_contour.points(), object.object_proto().yaw(),
        /*v=*/0.0, /*a=*/0.0);
    // Ignore stationary objects behind ego or not on the target lane.
    if (obj_frenet_box.s_max < ego_cur_frenet_box.s_min ||
        std::abs(obj_frenet_box.center_l()) >
            0.5 * obj_frenet_box.width() + ego_half_width + kStationaryBuffer) {
      continue;
    }

    for (int i = 0; i < ego_traj_size; ++i) {
      if (obj_contour.HasOverlap(ego_boxes[i])) {
        if (IsBlockingObjectAbreast(ego_cur_frenet_box, obj_frenet_box,
                                    /*obj_front_ext=*/0.0)) {
          // Not safe if colliding object lies abreast of the ego vehicle.
          *unsafe_object_id = object.id();
          return absl::CancelledError(
              absl::StrFormat("Object %s currently abreast.", object.id()));
        }
        if (enter_target_idx == 0) {
          // If already entered target lane, only check for very dangrerous
          // situation where some vehicle is too close.
          continue;
        }

        const double lon_dist = obj_frenet_box.s_min - ego_cur_frenet_box.s_max;
        if (auto check_status =
                CheckDeceleration(lon_dist, object.id(), "ego",
                                  /*v_lead=*/0.0, ego_v, /*response_time=*/0.0,
                                  kEgoFollowTimeBuffer, kMaxAllowedDecelForEgo);
            !check_status.ok()) {
          *unsafe_object_id = object.id();
          return check_status;
        }
        break;
      }
    }
  }
  // Check for moving objects.
  const double path_start_time_offset =
      absl::ToDoubleSeconds(path_look_ahead_duration);
  const double ego_enter_target_time = enter_target_idx * kTrajectoryTimeStep;
  const bool lc_left = ego_cur_frenet_box.center_l() < 0.0;
  const double conserv = ComputeLcConservFactor(
      ego_cur_frenet_box, ego_half_width, lc_left, lc_style);
  debug_proto->set_conservative_factor(conserv);
  const double follow_obj_resp_time = conserv * kFollowerStandardResponseTime;
  for (const auto* traj_ptr : st_traj_mgr.moving_object_trajs()) {
    QCHECK(!traj_ptr->states().empty())
        << "No prediction state for trajectory " << traj_ptr->traj_id();
    ASSIGN_OR_CONTINUE(
        const auto obj_cur_box,
        FindPredictedObjectBox(traj_ptr->states(), path_start_time_offset));
    ASSIGN_OR_CONTINUE(
        const auto obj_cur_frenet_box,
        target_frenet_frame.QueryFrenetBoxWithHeading(obj_cur_box));
    double obj_max_decel = 0.0;
    double lon_dist = 0.0;
    const auto& object = traj_ptr->planner_object();
    const double obj_v = ego_cur_frenet_box.s_max < obj_cur_frenet_box.s_min
                             ? object.pose().v()
                             : EstimateObjectSpeed(object, preview_time);
    auto* object_aligned_info = debug_proto->add_object_aligned_infos();
    *object_aligned_info = CreateAlignedObjectInfo(
        object.id(), obj_cur_box.GetCornersCounterClockwise(),
        obj_cur_box.heading(), obj_v, object.pose().a());
    // Ignore objects that are currently leaving the target lane path behind or
    // abreast the ego vehicle.
    if (IsLeavingTargetLanePath(target_frenet_frame, lc_left,
                                ego_cur_frenet_box, obj_cur_frenet_box,
                                traj_ptr->states())) {
      object_aligned_info->set_safety_decison("Safe: leaving target path.");
      continue;
    }

    const double ego_lead_time =
        ego_cur_frenet_box.s_max < obj_cur_frenet_box.s_min
            ? 0.0  // obj at front, no extension
            : ComputeEgoLeadTime(speed_limit, ego_v, obj_v);
    // Check if the object is currently abreast of the ego vehicle.
    const double obj_front_extension =
        conserv * obj_v * ego_lead_time *
        obj_cur_box.tangent().Dot(target_frenet_frame.InterpolateTangentByS(
            obj_cur_frenet_box.center_s()));
    if (IsBlockingObjectAbreast(ego_cur_frenet_box, obj_cur_frenet_box,
                                obj_front_extension)) {
      *unsafe_object_id = object.id();
      object_aligned_info->set_safety_decison("Unsafe: object is abreast.");
      return absl::CancelledError(
          absl::StrFormat("Object %s currently abreast.", object.id()));
    }

    if (enter_target_idx == 0) {
      // If already entered target lane, only check for very dangrerous
      // situation where some vehicle is too close.
      object_aligned_info->set_safety_decison(
          "Safe: ego already in target lane.");
      continue;
    }

    if (!PathHasOverlap(ego_boxes, obj_v, traj_ptr->states())) {
      object_aligned_info->set_safety_decison("Safe: Path not has overlap.");
      continue;
    }

    // Check for time to enter target lane.
    const double obj_enter_target_time = std::max(
        0.0, ComputeEnterTargetTime(target_frenet_frame, traj_ptr->states()) -
                 path_start_time_offset);
    if (obj_enter_target_time > ego_enter_target_time) {
      object_aligned_info->set_safety_decison(
          "Safe: object enter target time is bigger than ego.");
      continue;
    }

    // Check for follower's deceleration.
    if (obj_cur_frenet_box.s_max < ego_cur_frenet_box.s_min) {  // Ego at front.
      // Record all considered objects that should follow the ego vehicle.
      follower_set->insert(object.id());

      if (ego_v > obj_v) {
        object_aligned_info->set_safety_decison(
            "Safe: rear object v is lower than ego.");
        continue;
      }
      const double lat_overlap =
          std::min(ego_cur_frenet_box.l_max, obj_cur_frenet_box.l_max) -
          std::max(ego_cur_frenet_box.l_min, obj_cur_frenet_box.l_min);
      if (lat_overlap > 0.5 * obj_cur_frenet_box.width()) {
        object_aligned_info->set_safety_decison(
            "Safe: rear object lat overlap is bigger.");
        continue;
      }

      lon_dist = ego_cur_frenet_box.s_min - obj_cur_frenet_box.s_max;
      const double max_allowed_decel =
          CalculateMaxAllowedDec(object.type(), obj_v);
      auto check_status = CheckDeceleration(
          lon_dist, "ego", object.id(), ego_v, obj_v, follow_obj_resp_time,
          conserv * ego_lead_time, max_allowed_decel, &obj_max_decel);
      if (follower_max_decel != nullptr &&
          obj_max_decel > *follower_max_decel) {
        *follower_max_decel = obj_max_decel;
      }
      if (!check_status.ok()) {
        *unsafe_object_id = object.id();
        object_aligned_info->set_safety_decison(
            absl::StrFormat("Unsafe: %s.", check_status.message()));
        return check_status;
      }
    } else {  // Obj at front.
      lon_dist = obj_cur_frenet_box.s_min - ego_cur_frenet_box.s_max;

      const double ego_follow_time = ComputeEgoFollowTime(obj_v, ego_v);
      if (auto check_status = CheckDeceleration(
              lon_dist, object.id(), "ego", obj_v, ego_v, kEgoResponseTime,
              conserv * ego_follow_time, kMaxAllowedDecelForEgo);
          !check_status.ok()) {
        object_aligned_info->set_safety_decison(
            absl::StrFormat("Unsafe: %s.", check_status.message()));
        *unsafe_object_id = object.id();
        return check_status;
      }
    }
    object_aligned_info->set_safety_decison(
        absl::StrFormat("Safe: max deceleration -%.2fm/s^2, dist %.2f is safe.",
                        obj_max_decel, lon_dist));
  }

  return absl::OkStatus();
}

}  // namespace qcraft::planner
