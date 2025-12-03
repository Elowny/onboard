#include "onboard/planner/hmi_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/optimization/ddp/object_cost_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {

namespace {
// Check if a nudge interval effective: if interval is close to start point,
// duration is long enough and l offset is large enough.
bool IsIntervalEffiective(double plan_start_point_theta_diff_to_lane,
                          int start_nudge_index, int direction,
                          int end_nudge_index, double l_nudge_max) {
  constexpr int kNudgeEffectiveIndexRange = 10;
  constexpr int kNudgeEffectiveStartIndex = 5;
  constexpr double kMaxNudgeBuffer = 0.2;
  constexpr double kThetaThreshold = 0.08;
  const bool in_loop_effective =
      start_nudge_index < kNudgeEffectiveStartIndex ||
      (Sign(plan_start_point_theta_diff_to_lane) == direction &&
       std::abs(plan_start_point_theta_diff_to_lane) > kThetaThreshold);
  return in_loop_effective &&
         (end_nudge_index - start_nudge_index) > kNudgeEffectiveIndexRange &&
         std::abs(l_nudge_max) > kMaxNudgeBuffer;
}

inline bool IsInLane(double l, double left_offset, double right_offset) {
  return l >= right_offset && l <= left_offset;
}

inline bool ComputeOffsetAtS(const DrivePassage& drive_passage, double s,
                             double* left_offset, double* right_offset) {
  QCHECK_NOTNULL(left_offset);
  QCHECK_NOTNULL(right_offset);

  const auto offset_at_s = drive_passage.QueryNearestBoundaryLateralOffset(s);
  if (!offset_at_s.ok()) return false;
  const bool is_virtual = drive_passage.FindNearestStationAtS(s).is_virtual();

  constexpr double kDefaultOffset = 1.8;  // m.
  *left_offset = is_virtual ? kDefaultOffset : offset_at_s->second;
  *right_offset = is_virtual ? -kDefaultOffset : offset_at_s->first;
  return true;
}

inline double ComputeLeftRatio(const FrenetBox& frenet_box,
                               double left_offset) {
  return (left_offset - frenet_box.l_min) / frenet_box.width();
}

inline double ComputeRightRatio(const FrenetBox& frenet_box,
                                double right_offset) {
  return (frenet_box.l_max - right_offset) / frenet_box.width();
}

std::optional<std::pair<double, double>> ComputeFrontObjectSAndV(
    const DrivePassage& drive_passage, const PlannerObject& obj,
    double ego_front_edge_s) {
  const auto frenet_box = drive_passage.QueryFrenetBoxAtContour(obj.contour());
  if (!frenet_box.ok() || frenet_box->s_min <= ego_front_edge_s) {
    return std::nullopt;
  }

  double left_offset = 0.0;
  double right_offset = 0.0;
  if (!ComputeOffsetAtS(drive_passage, frenet_box->center_s(), &left_offset,
                        &right_offset)) {
    return std::nullopt;
  }

  const auto lane_center_tan =
      drive_passage.QueryTangentAtS(frenet_box->center_s());
  if (!lane_center_tan.ok()) return std::nullopt;
  const double obj_vel_dir = obj.velocity().Dot(lane_center_tan->Perp());
  const double obj_vel = obj.velocity().Dot(*lane_center_tan);
  const bool is_obj_cutin = frenet_box->center_l() * obj_vel_dir < 0.0;

  const bool center_in_lane =
      IsInLane(frenet_box->center_l(), left_offset, right_offset);
  const bool left_in_lane =
      IsInLane(frenet_box->l_max, left_offset, right_offset);
  const bool right_in_lane =
      IsInLane(frenet_box->l_min, left_offset, right_offset);
  const double in_left_boundary_ratio =
      ComputeLeftRatio(*frenet_box, left_offset);
  const double in_right_boundary_ratio =
      ComputeRightRatio(*frenet_box, right_offset);
  const auto s_and_v =
      std::make_pair(frenet_box->s_min - ego_front_edge_s, obj_vel);

  constexpr double kCutinLaneRatio = 0.2;
  constexpr double kCutoutLaneRatio = 0.65;
  QCHECK_GT(frenet_box->width(), 0.0);
  if (is_obj_cutin) {
    // Object cutin.
    if (center_in_lane ||
        (right_in_lane && in_left_boundary_ratio > kCutinLaneRatio) ||
        (left_in_lane && in_right_boundary_ratio > kCutinLaneRatio)) {
      return s_and_v;
    }

  } else if (center_in_lane) {
    // Object cutout and center in lane.
    if (obj_vel_dir > 0.0 ? in_left_boundary_ratio > kCutoutLaneRatio
                          : in_right_boundary_ratio > kCutoutLaneRatio) {
      return s_and_v;
    }
  }

  // Object ride on lane boundary.
  constexpr double kThetaDiffThres = d2r(5.0);  // 5 deg.
  constexpr double kRideOnLaneRatio = 0.1;
  if (std::abs(NormalizeAngle(obj.velocity().FastAngle() -
                              lane_center_tan->FastAngle())) <
      kThetaDiffThres) {
    if (right_in_lane && in_left_boundary_ratio > kRideOnLaneRatio) {
      return s_and_v;
    }
    if (left_in_lane && in_right_boundary_ratio > kRideOnLaneRatio) {
      return s_and_v;
    }
  }

  return std::nullopt;
}

}  // namespace

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
absl::StatusOr<std::optional<NudgeOjbectInfo>> ExtractNudgeObjectId(
    int trajectory_steps, double trajectory_time_step, LaneChangeStage lc_stage,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const std::vector<TrajectoryPoint>& result_points,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);
  const int free_index = static_cast<int>(
      (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);

  std::optional<NudgeOjbectInfo> nudge_object_info_optional;
  // Only extrack nudge object id when lane keeping.
  if (lc_stage != LaneChangeStage::LCS_NONE) return nudge_object_info_optional;
  // If trajectory in front of av have nudge behavior.
  const auto& plan_start_point = result_points.front();
  const auto& plan_start_frenet_point =
      drive_passage.QueryFrenetCoordinateAt(plan_start_point.pos());
  if (!plan_start_frenet_point.ok()) {
    return absl::OutOfRangeError(
        "Plan start point don't in drive passage range.");
  }
  const auto start_point_lane_theta =
      drive_passage.QueryTangentAngleAtS(plan_start_frenet_point->s);
  if (!start_point_lane_theta.ok()) {
    return absl::OutOfRangeError(
        "Plan start point s don't in drive passage range.");
  }
  const double plan_start_point_theta_diff_to_lane =
      NormalizeAngle(plan_start_point.theta() - *start_point_lane_theta);

  // Firstly, Obtain effective nudge intervals.
  constexpr double kLOffsetThreshold = 0.2;
  constexpr int kNudgeEffectiveStartIndex = 5;

  std::optional<int> direction;
  std::optional<double> l_nudge_max;
  std::optional<int> start_nudge_index;
  std::optional<int> end_nudge_index;

  // Loop for nudge interval obtained.
  for (int k = 0; k < free_index; ++k) {
    const auto& traj_point = result_points[k];
    const auto& frenet_pt =
        drive_passage.QueryFrenetCoordinateAt(traj_point.pos());
    if (!frenet_pt.ok()) {
      return absl::OutOfRangeError(
          absl::StrFormat("%d traj point out of drive passage range.", k));
    }
    const double center_l_at_s =
        path_sl_boundary.QueryReferenceCenterL(frenet_pt->s);
    const double l_offset = frenet_pt->l - center_l_at_s;
    if (!start_nudge_index.has_value() &&
        std::abs(l_offset) > kLOffsetThreshold) {
      start_nudge_index = k;
    }
    if (start_nudge_index.has_value()) {
      if (!l_nudge_max.has_value()) {
        l_nudge_max = l_offset;
        direction = Sign(l_offset);
      } else {
        if (Sign(l_offset) == *direction) {
          if (*direction > 0) {
            l_nudge_max = std::max(*l_nudge_max, l_offset);
          } else {
            l_nudge_max = std::min(*l_nudge_max, l_offset);
          }
        }
      }
    }
    // Get end index of the nudge interval, and if current nudge interval is
    // noneffective, clear state and find next interval.
    const bool l_offset_noneffective =
        (*direction > 0 ? l_offset <= kLOffsetThreshold
                        : l_offset >= -kLOffsetThreshold) ||
        k == free_index - 1;
    if (l_nudge_max.has_value() && !end_nudge_index.has_value() &&
        l_offset_noneffective) {
      end_nudge_index = k - 1;
      if (!IsIntervalEffiective(plan_start_point_theta_diff_to_lane,
                                *start_nudge_index, *direction,
                                *end_nudge_index, *l_nudge_max) &&
          *end_nudge_index < kNudgeEffectiveStartIndex) {
        direction.reset();
        l_nudge_max.reset();
        start_nudge_index.reset();
        end_nudge_index.reset();
      } else {
        break;
      }
    } else {
      continue;
    }
  }

  // Secondly, Extract nudge object id: In nudge the interval, find object whose
  // traj closest to av traj and on the side matching with nudge direction.
  double l_min = std::numeric_limits<double>::infinity();
  const SpacetimeObjectTrajectory* min_dist_object_ptr = nullptr;
  std::vector<FrenetBox> av_sl_boxes;
  if (l_nudge_max.has_value() &&
      IsIntervalEffiective(plan_start_point_theta_diff_to_lane,
                           *start_nudge_index, *direction, *end_nudge_index,
                           *l_nudge_max)) {
    // Get av sl boxes first.
    av_sl_boxes.reserve(free_index);
    for (int k = 0; k < free_index; ++k) {
      const auto& traj_point = result_points[k];
      const auto box = ComputeAvBox(traj_point.pos(), traj_point.theta(),
                                    vehicle_geometry_params);
      auto frenet_box = drive_passage.QueryFrenetBoxAt(box);
      if (!frenet_box.ok()) {
        return absl::OutOfRangeError(absl::StrFormat(
            "%d traj point box out of drive passage range.", k));
      }
      av_sl_boxes.push_back(*frenet_box);
    }

    const auto get_nudge_object_id_for_step =
        [&l_min, &min_dist_object_ptr](
            int direction, const FrenetBox& object_frenet_box,
            const FrenetBox& av_frenet_box,
            const SpacetimeObjectTrajectory* object_ptr) {
          constexpr double kSRangeExtendTime = 0.7;  // s.
          const double s_range_extend =
              object_ptr->pose().v() * kSRangeExtendTime;
          int object_side = 0;
          if (av_frenet_box.l_min > object_frenet_box.l_max) {
            object_side = 1;
          } else if (object_frenet_box.l_min > av_frenet_box.l_max) {
            object_side = -1;
          }

          if (object_side == direction) {
            const bool has_s_overlap =
                !(av_frenet_box.s_max <
                  (object_frenet_box.s_min - s_range_extend)) &&
                !(av_frenet_box.s_min >
                  (object_frenet_box.s_max + s_range_extend));
            if (has_s_overlap) {
              double l_diff = 0.0;
              if (direction > 0) {
                l_diff = av_frenet_box.l_min - object_frenet_box.l_max;
              } else {
                l_diff = object_frenet_box.l_min - av_frenet_box.l_max;
              }
              if (l_diff < l_min) {
                l_min = l_diff;
                min_dist_object_ptr = object_ptr;
              }
            }
          }
        };

    const auto& spacetime_trajs = st_planner_object_traj.trajectories;
    const int num_trajs = spacetime_trajs.size();
    // Loop to find object closest to av traj.
    for (int i = 0; i < num_trajs; ++i) {
      const auto& traj = spacetime_trajs[i];
      if (leading_trajs.find(std::string(traj.traj_id())) !=
          leading_trajs.end()) {
        continue;
      }
      const auto states = optimizer::SampleObjectStates(
          trajectory_steps, trajectory_time_step, traj.states());
      std::optional<FrenetBox> stationary_object_frenet_box;
      if (traj.is_stationary()) {
        const auto frenet_box_or =
            drive_passage.QueryFrenetBoxAtContour(traj.contour());
        if (!frenet_box_or.ok()) {
          break;
        }
        stationary_object_frenet_box = *frenet_box_or;
      }
      for (int k = *start_nudge_index;
           k < *end_nudge_index && k < states.size() && k < free_index; ++k) {
        if (stationary_object_frenet_box.has_value()) {
          get_nudge_object_id_for_step(
              *direction, *stationary_object_frenet_box, av_sl_boxes[k], &traj);
        } else {
          const auto& state = states[k];
          const auto& contour = state.contour;
          const auto frenet_box_or =
              drive_passage.QueryFrenetBoxAtContour(contour);
          if (!frenet_box_or.ok()) {
            break;
          }
          get_nudge_object_id_for_step(*direction, *frenet_box_or,
                                       av_sl_boxes[k], &traj);
        }
      }
    }
  }

  // TODO(runbing): Output nudge id and direction to hmi.
  constexpr double kNudgeBuffer = 1.2;
  if (min_dist_object_ptr != nullptr &&
      (l_min < kNudgeBuffer ||
       min_dist_object_ptr->object_type() == ObjectType::OT_LARGE_VEHICLE)) {
    if (min_dist_object_ptr->object_type() == ObjectType::OT_LARGE_VEHICLE) {
      QEVENT_EVERY_N_SECONDS(
          "runbing", "traj_opt_nudge_large_vehicle", 0.2, [&](QEvent* qevent) {
            qevent->AddField("l_min", l_min)
                .AddField("min_object_id", min_dist_object_ptr->object_id());
          });
    } else {
      QEVENT_EVERY_N_SECONDS(
          "runbing", "traj_opt_nudge_object", 0.2, [&](QEvent* qevent) {
            qevent->AddField("l_min", l_min)
                .AddField("min_object_id", min_dist_object_ptr->object_id());
          });
    }
    LOG(INFO) << min_dist_object_ptr->object_id() << " " << l_min;
    NudgeOjbectInfo nudge_object_info;
    nudge_object_info.id = min_dist_object_ptr->object_id();
    nudge_object_info.direction = *direction;

    const auto object_frenet_box =
        drive_passage.QueryFrenetBoxAt(min_dist_object_ptr->bounding_box());
    if (!object_frenet_box.ok()) {
      return absl::OutOfRangeError("Object is not on drive passage.");
    }
    nudge_object_info.arc_dist_to_object =
        std::max(0.0, object_frenet_box->s_min - av_sl_boxes.front().s_max);

    nudge_object_info.type =
        min_dist_object_ptr->object_type() == ObjectType::OT_LARGE_VEHICLE
            ? NudgeOjbectInfo::Type::kLargeVehicle
            : NudgeOjbectInfo::Type::kNormal;

    nudge_object_info_optional = std::move(nudge_object_info);
  }
  return nudge_object_info_optional;
}

std::optional<AlertedFrontVehicleInfo> GetAlertedFrontVehicle(
    const DrivePassage& drive_passage, const PlannerObjectManager& obj_mgr,
    const ApolloTrajectoryPointProto& start_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedFinderParamsProto& speed_finder_params) {
  constexpr double kTimeHeadway = 2.0;
  const double preview_distance =
      start_point.v() * kTimeHeadway +
      speed_finder_params.follow_standstill_distance();
  std::optional<std::string> obj_id;
  double min_s = std::numeric_limits<double>::max();
  double obj_v = 0.0;

  const auto plan_start_point_sl_or = drive_passage.QueryFrenetCoordinateAt(
      Vec2d(start_point.path_point().x(), start_point.path_point().y()));
  if (!plan_start_point_sl_or.ok()) return std::nullopt;
  const double ego_front_edge_s =
      plan_start_point_sl_or->s +
      vehicle_geometry_params.front_edge_to_center();

  for (const PlannerObject& obj : obj_mgr.planner_objects()) {
    if (obj.type() != ObjectType::OT_VEHICLE &&
        obj.type() != ObjectType::OT_LARGE_VEHICLE)
      continue;
    const auto obj_sv =
        ComputeFrontObjectSAndV(drive_passage, obj, ego_front_edge_s);
    if (obj_sv.has_value()) {
      if (obj_sv->first > preview_distance) continue;
      if (obj_sv->first < min_s) {
        min_s = obj_sv->first;
        obj_v = obj_sv->second;
        obj_id = obj.id();
      }
    }
  }
  return obj_id.has_value()
             ? std::optional(AlertedFrontVehicleInfo{
                   .obj_id = *obj_id, .obj_v = obj_v, .min_s = min_s})
             : std::nullopt;
}

bool GetCollisionWarningRequest(
    const bool prev_collision_warning_request,
    const std::optional<AlertedFrontVehicleInfo>& target_front_vehicle,
    double ego_v, double follow_time_headway) {
  constexpr double kSpeedThres = 0.2;  // m/s.
  if (!target_front_vehicle.has_value() || ego_v <= kSpeedThres) {
    return false;
  }

  constexpr double kEnterStandstill = 1.5;  // m.
  constexpr double kExitStandstill = 4.5;   // m.
  constexpr double kEnterTimeHeadwayGain = 0.5;
  constexpr double kExitTimeHeadwayGain = 0.8;
  const PiecewiseLinearFunction<double, double> risk_time_headway_rel_v_plf = {
      {-13.0, 2.0}, {0.7, 0.25}};
  const PiecewiseLinearFunction<double, double> prediction_time_rel_v_plf = {
      {-15.0, -5.0}, {-5.0, 3.0}};

  const double rel_v = target_front_vehicle->obj_v - ego_v;
  const double risk_time_headway = risk_time_headway_rel_v_plf(rel_v);
  const double predcition_time = prediction_time_rel_v_plf(rel_v);

  const double standstill =
      prev_collision_warning_request ? kExitStandstill : kEnterStandstill;
  if (target_front_vehicle->min_s <= ego_v * risk_time_headway + standstill) {
    return true;
  }

  const double time_headway_gain = prev_collision_warning_request
                                       ? kExitTimeHeadwayGain
                                       : kEnterTimeHeadwayGain;
  if (ego_v * predcition_time - target_front_vehicle->obj_v * predcition_time >=
      ego_v * follow_time_headway * time_headway_gain) {
    return true;
  }

  return false;
}

bool WhetherSelectCurrentBranchForHighlightVehicle(
    const DrivePassage& drive_passage, const Box2d& av_box, double heading) {
  const auto av_frenet_box = drive_passage.QueryFrenetBoxAt(av_box);
  if (!av_frenet_box.ok()) return false;
  double left_offset = 0.0;
  double right_offset = 0.0;
  if (!ComputeOffsetAtS(drive_passage, av_frenet_box->center_s(), &left_offset,
                        &right_offset)) {
    return false;
  }

  const auto lane_center_tan =
      drive_passage.QueryTangentAtS(av_frenet_box->center_s());
  if (!lane_center_tan.ok()) return false;

  const double av_dir =
      Vec2d::FastUnitFromAngle(heading).Dot(lane_center_tan->Perp());
  const bool is_av_cutin = av_frenet_box->center_l() * av_dir < 0.0;

  QCHECK_GT(av_frenet_box->width(), 0.0);
  const bool center_in_lane =
      IsInLane(av_frenet_box->center_l(), left_offset, right_offset);
  const bool l_max_in_lane =
      IsInLane(av_frenet_box->l_max, left_offset, right_offset);
  const bool l_min_in_lane =
      IsInLane(av_frenet_box->l_min, left_offset, right_offset);
  const double in_left_boundary_ratio =
      ComputeLeftRatio(*av_frenet_box, left_offset);
  const double in_right_boundary_ratio =
      ComputeRightRatio(*av_frenet_box, right_offset);

  constexpr double kAvCutinRatio = 0.2;
  constexpr double kAvCutoutRatio = 0.8;

  if (is_av_cutin) {
    if (center_in_lane ||
        (l_min_in_lane && in_left_boundary_ratio > kAvCutinRatio) ||
        (l_max_in_lane && in_right_boundary_ratio > kAvCutinRatio)) {
      return true;
    }
  } else if (center_in_lane) {
    if (av_dir > 0.0 ? in_left_boundary_ratio > kAvCutoutRatio
                     : in_right_boundary_ratio > kAvCutoutRatio) {
      return true;
    }
  }
  return false;
}

}  // namespace planner
}  // namespace qcraft
