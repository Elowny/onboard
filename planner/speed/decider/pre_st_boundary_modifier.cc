#include "onboard/planner/speed/decider/pre_st_boundary_modifier.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/decider/st_boundary_modifier_util.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {
namespace {

std::optional<StBoundaryModificationResult> ModifyOncomingStBoundary(
    const SpeedFinderParamsProto::StBoundaryPreModifierParamsProto& params,
    const StGraph& st_graph, const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeObjectTrajectory& st_traj, double current_v,
    const DiscretizedPath& path) {
  // An st-boundary is considered to be ONCOMING if:
  // 1. Its first overlap time is less than 0.5s;
  // 2. Its first-overlap heading diff is beyond certain threshold;
  // 3. Its first-overlap s_lower is larger than last-overlap s_lower.
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  constexpr double kMaxTimeLimit = 0.5;  // s.

  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return std::nullopt;
  }

  if (st_boundary.bottom_left_point().t() > kMaxTimeLimit) {
    return std::nullopt;
  }

  // Do not modify prediction for objects making unprotected left-turn.
  if (st_boundary.overlap_meta()->source() == StOverlapMetaProto::LANE_CROSS) {
    return std::nullopt;
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  QCHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto first_overlap_obj_heading =
      st_traj.states()[first_overlap_info.obj_idx].box.heading();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kOnComingThreshold = d2r(150.0);
  constexpr double kReverseDrivingThreshold = d2r(30.0);
  const double abs_rel_heading = std::abs(NormalizeAngle(
      first_overlap_obj_heading - first_overlap_av_middle_heading));
  const bool is_oncoming_object = abs_rel_heading >= kOnComingThreshold;
  const bool is_reverse_driving_object =
      abs_rel_heading <= kReverseDrivingThreshold &&
      st_traj.trajectory().is_reversed();
  if (!is_oncoming_object && !is_reverse_driving_object) {
    return std::nullopt;
  }

  VLOG(2) << "St-boundary " << st_boundary_wd.id()
          << " is considered to be ONCOMING.";

  // Only modify the oncoming prediction if it would cause uncomfortable brake.
  const auto& bottom_right_point =
      st_boundary_wd.st_boundary()->bottom_right_point();
  const double const_speed_s = current_v * bottom_right_point.t();
  if (bottom_right_point.s() > const_speed_s) {
    // No brake is needed.
    return std::nullopt;
  }
  constexpr double kAvMinVel = 3.0;  // m/s^2.
  const double max_decel_s =
      std::max(0.5 * Sqr(current_v) / params.oncoming_uncomfortable_decel(),
               kAvMinVel * bottom_right_point.t());
  if (max_decel_s < bottom_right_point.s() && is_oncoming_object) {
    const double estimated_av_decel = 2.0 *
                                      (const_speed_s - bottom_right_point.s()) /
                                      Sqr(bottom_right_point.t());
    if (estimated_av_decel < params.oncoming_uncomfortable_decel()) {
      return std::nullopt;
    }
  }

  // Modify oncoming spacetime trajectory.
  VLOG(2) << "Modify ONCOMING st-boundary " << st_boundary.id();
  constexpr double kOncomingReactionTime = 0.5;         // s.
  constexpr double kOncomingObjectDecel = -1.8;         // m/s^2.
  constexpr double kOncomingObjectMildDecel = -0.8;     // m/s^2.
  constexpr double kReverseDrivingReactionTime = 0.0;   // s.
  constexpr double kReverseDrivingObjectDecel = -20.0;  // m/s^2.

  // If AV is doing lane change or lane borrow to an opposite lane, we consider
  // oncoming object will decelerate more mildly to make AV more conservative.
  const double reaction_time = is_reverse_driving_object
                                   ? kReverseDrivingReactionTime
                                   : kOncomingReactionTime;
  double obj_decel;
  if (is_reverse_driving_object) {
    obj_decel = kReverseDrivingObjectDecel;
  } else {
    obj_decel =
        st_boundary.overlap_meta()->source() == StOverlapMetaProto::AV_CUTIN
            ? std::min(st_traj.planner_object().pose().a(),
                       kOncomingObjectMildDecel)
            : std::min(st_traj.planner_object().pose().a(),
                       kOncomingObjectDecel);
  }

  // Make new spacetime trajectory.
  // FIXME(renjie): The current modification type for OBJECT_CUT_IN is
  // LON_LAT_MODIFIABLE but we modifies oncoming st-boundaries longitudinally
  // just to be in consistent with the old logic. The modification method should
  // be subject to the modification type given in overlap meta.
  auto new_st_traj = CreateSpacetimeTrajectoryByDecelAfterDelay(
      st_traj, reaction_time, obj_decel);

  // Generate new st_boundaries.
  auto new_st_boundaries = st_graph.MapMovingSpacetimeObject(
      new_st_traj, /*generate_lane_change_gap=*/false,
      /*calc_moving_close_traj=*/false);
  double yield_time_buffer = 0.0;

  constexpr double kObjectMinNudgeSpeed = 1.0;         // m/s.
  constexpr double kObjectNudgeYieldTimeBuffer = 1.0;  // s.
  if (const double obj_vel = st_traj.planner_object().pose().v();
      obj_vel > kObjectMinNudgeSpeed) {
    const double max_kappa =
        std::min(params.oncoming_nudge_max_lat_acc() / Sqr(obj_vel),
                 params.oncoming_nudge_max_kappa());
    const double max_lambda =
        params.oncoming_nudge_max_lat_acc() / Cube(obj_vel);
    const std::vector<double> lambdas{-max_lambda, max_lambda};

    double min_desired_acc = ComputeOncomingObjectDesiredAccel(
        new_st_boundaries, current_v, yield_time_buffer);
    for (const double lambda : lambdas) {
      auto nudge_st_traj =
          CreateNudgeSpacetimeTrajectory(st_traj, lambda, max_kappa);
      auto nudge_st_boundaries = st_graph.MapMovingSpacetimeObject(
          nudge_st_traj, /*generate_lane_change_gap=*/false,
          /*calc_moving_close_traj=*/false);
      if (const double desired_acc = ComputeOncomingObjectDesiredAccel(
              nudge_st_boundaries, current_v, kObjectNudgeYieldTimeBuffer);
          desired_acc > min_desired_acc) {
        yield_time_buffer = kObjectNudgeYieldTimeBuffer;
        new_st_traj = std::move(nudge_st_traj);
        new_st_boundaries = std::move(nudge_st_boundaries);
        min_desired_acc = desired_acc;
      }
    }
  }

  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  st_boundaries_wd.reserve(new_st_boundaries.size());
  const auto modifier_type = StBoundaryModifierProto::ONCOMING;
  for (auto& st_boundary : new_st_boundaries) {
    st_boundary->set_id(absl::StrCat(st_boundary->id(), "|m"));
    if (st_boundary_wd.decision_type() == StBoundaryProto::UNKNOWN) {
      st_boundaries_wd.emplace_back(std::move(st_boundary),
                                    st_boundary_wd.decision_type(),
                                    st_boundary_wd.decision_reason());
      st_boundaries_wd.back().SetTimeBuffer(/*pass_time=*/0.0,
                                            yield_time_buffer);
    } else {
      st_boundaries_wd.emplace_back(
          std::move(st_boundary), st_boundary_wd.decision_type(),
          st_boundary_wd.decision_reason(),
          absl::StrCat(
              st_boundary_wd.decision_info(), " and keep it after modified by ",
              StBoundaryModifierProto::ModifierType_Name(modifier_type)),
          st_boundary_wd.follow_standstill_distance(),
          st_boundary_wd.lead_standstill_distance(),
          /*pass_time=*/0.0, yield_time_buffer);
    }
  }

  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = modifier_type});
}

std::optional<StBoundaryModificationResult> ModifyStBoundaryForOncoming(
    const PreStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd) {
  std::optional<StBoundaryModificationResult> res = std::nullopt;
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  // Only pre-modify st-boundaries having overlap meta.
  if (!st_boundary.overlap_meta().has_value()) return res;

  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) return res;

  QCHECK(st_boundary.object_type() == StBoundaryProto::VEHICLE ||
         st_boundary.object_type() == StBoundaryProto::CYCLIST ||
         st_boundary.object_type() == StBoundaryProto::PEDESTRIAN)
      << StBoundaryProto::ObjectType_Name(st_boundary.object_type());

  const auto& traj_id = st_boundary.traj_id();
  QCHECK(traj_id.has_value());

  const auto* traj =
      QCHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));

  // Modify oncoming predictions that would cause uncomfortable brake.
  res = ModifyOncomingStBoundary(*input.params, *input.st_graph, st_boundary_wd,
                                 *traj, input.current_v, *input.path);
  if (res.has_value()) {
    return res;
  }

  return res;
}

}  // namespace

void PreModifyStBoundaries(
    const PreStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(input.params);
  QCHECK_NOTNULL(input.st_graph);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.path);

  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      pre_processed_st_objects;

  ModifyAndUpdateStBoundaries(
      input,
      std::function<std::optional<StBoundaryModificationResult>(
          const PreStboundaryModifierInput&, const StBoundaryWithDecision&)>(
          ModifyStBoundaryForOncoming),
      &pre_processed_st_objects, st_boundaries_wd);

  // Merge newly processed trajectories with the original ones.
  for (auto& [traj_id, traj] : pre_processed_st_objects) {
    processed_st_objects->insert_or_assign(traj_id, std::move(traj));
  }
}

}  // namespace planner
}  // namespace qcraft
