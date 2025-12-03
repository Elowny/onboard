#ifndef ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_MODIFIER_UTIL_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_MODIFIER_UTIL_H_

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>  // IWYU pragma: keep  // for std::move
#include <vector>

#include "absl/strings/str_cat.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

struct AccelPoint {
  AccelPoint(double time, double acc) : t(time), a(acc) {}
  double t = 0.0;
  double a = 0.0;
};

inline PathPoint GetPathPointFromPredictedTrajectoryPoint(
    const prediction::PredictedTrajectoryPoint& pred_traj_point) {
  PathPoint path_point;
  path_point.set_x(pred_traj_point.pos().x());
  path_point.set_y(pred_traj_point.pos().y());
  path_point.set_theta(pred_traj_point.theta());
  path_point.set_s(pred_traj_point.s());
  path_point.set_kappa(pred_traj_point.kappa());
  return path_point;
}

inline void SetPredictedTrajectoryPointSpatialInfoFromPathPoint(
    const PathPoint& path_point,
    prediction::PredictedTrajectoryPoint* pred_traj_point) {
  pred_traj_point->set_s(path_point.s());
  pred_traj_point->set_pos(Vec2d(path_point.x(), path_point.y()));
  pred_traj_point->set_theta(path_point.theta());
  pred_traj_point->set_kappa(path_point.kappa());
}

struct StBoundaryModificationResult {
  // Will be empty if the new trajectory has no overlap with path.
  std::vector<StBoundaryWithDecision> newly_generated_st_boundaries_wd;
  SpacetimeObjectTrajectory processed_st_traj;
  StBoundaryModifierProto::ModifierType modifier_type;
};

SpacetimeObjectTrajectory CreateSpacetimeTrajectoryByDecelAfterDelay(
    const SpacetimeObjectTrajectory& st_traj, double delay, double decel);

SpacetimeObjectTrajectory CreateNudgeSpacetimeTrajectory(
    const SpacetimeObjectTrajectory& st_traj, double lambda, double max_kappa);

/**
 * @brief Calculate the minimum deceleration of ego vehicle to avoid collision
 * with oncoming st boundaries.
 * @param st_boundaries oncoming object st boundaries.
 * @param v_ego current velocity of ego vehicle.
 * @param yield_time_buffer time buffer for yielding.
 * @return return the desired deceleration. The result would be 0.0 if
 * deceleration is unnecessary.
 */
double ComputeOncomingObjectDesiredAccel(
    const std::vector<StBoundaryRef>& st_boundaries, double v_ego,
    double yield_time_buffer);

/**
 * @brief Calculate the minimum deceleration an object has to take if it wishes
 * to stop within a given distance and time delay. The object is assumed to keep
 * its initial velocity and acceleration within the time delay.
 * @param s stop distance.
 * @param v0 initial velocity.
 * @param a0 initial acceleration.
 * @param t_delay time delay.
 * @return return std::nullopt if a legal deceleration can't be calculated.
 */
inline std::optional<double> ComputeSafeStopDecelWithDelayedAction(
    double s, double v0, double a0, double t_delay) {
  if (v0 < 0.0) {
    return std::nullopt;
  }
  if (a0 < 0.0) {
    t_delay = std::min(std::fabs(v0 / a0), t_delay);
  }
  const double s_delay = (v0 + 0.5 * a0 * t_delay) * t_delay;
  if (s_delay >= s) {
    return std::nullopt;
  }
  const double v_delay = std::max(0.0, v0 + a0 * t_delay);
  const double ds = s - s_delay;
  return -0.5 * v_delay * v_delay / ds;
}

template <typename T>
void ModifyAndUpdateStBoundaries(
    const T& modifier_input,
    const std::function<std::optional<StBoundaryModificationResult>(
        const T&, const StBoundaryWithDecision&)>& modifier_func,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  FUNC_QTRACE();

  std::vector<StBoundaryWithDecision> all_newly_generated_st_boundaries_wd;

  const auto ignore_modified_st_boundary =
      [](const std::string& decision_info,
         StBoundaryWithDecision* st_boundary_wd,
         std::optional<StBoundaryModifierProto::ModifierType>
             modifier_type_opt = std::nullopt) {
        st_boundary_wd->set_id(absl::StrCat(st_boundary_wd->id(), "|raw"));
        st_boundary_wd->set_decision_type(StBoundaryProto::IGNORE);
        const auto modifier_type =
            modifier_type_opt.value_or(StBoundaryModifierProto::UNKNOWN);
        if (modifier_type == StBoundaryModifierProto::LEADING) {
          st_boundary_wd->set_decision_reason(StBoundaryProto::SAMPLING_DP);
        } else if (modifier_type == StBoundaryModifierProto::LON_INTERACTIVE) {
          st_boundary_wd->set_decision_reason(
              StBoundaryProto::INTERACTIVE_DECIDER);
        } else if (modifier_type == StBoundaryModifierProto::PRE_DECISION) {
          st_boundary_wd->set_decision_reason(StBoundaryProto::PRE_DECIDER);
        } else {
          st_boundary_wd->set_decision_reason(
              StBoundaryProto::ST_BOUNDARY_MODIFIER);
        }
        st_boundary_wd->set_ignore_reason(
            StBoundaryProto::ST_BOUNDARY_MODIFIED);
        st_boundary_wd->set_decision_info(decision_info);
      };

  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if (const auto& traj_id = st_boundary_wd.raw_st_boundary()->traj_id();
        traj_id.has_value() &&
        processed_st_objects->find(*traj_id) != processed_st_objects->end()) {
      // Ignore the st-boundary if its associated prediction has been
      // modified.
      ignore_modified_st_boundary("ignore associated st-boundary modified",
                                  &st_boundary_wd);
    } else {
      auto res = modifier_func(modifier_input, st_boundary_wd);
      if (res.has_value()) {
        all_newly_generated_st_boundaries_wd.reserve(
            all_newly_generated_st_boundaries_wd.size() +
            res->newly_generated_st_boundaries_wd.size());
        std::move(res->newly_generated_st_boundaries_wd.begin(),
                  res->newly_generated_st_boundaries_wd.end(),
                  std::back_inserter(all_newly_generated_st_boundaries_wd));
        QCHECK(st_boundary_wd.traj_id().has_value());
        if (!st_boundary_wd.raw_st_boundary()->is_protective()) {
          processed_st_objects->insert_or_assign(
              *st_boundary_wd.traj_id(), std::move(res->processed_st_traj));
        }
        ignore_modified_st_boundary(
            absl::StrCat(
                "ignore modified by ",
                StBoundaryModifierProto::ModifierType_Name(res->modifier_type)),
            &st_boundary_wd, res->modifier_type);
      }
    }
  }

  // Append newly generated st-boundaries with decision to original ones.
  std::move(all_newly_generated_st_boundaries_wd.begin(),
            all_newly_generated_st_boundaries_wd.end(),
            std::back_inserter(*st_boundaries_wd));
}

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_MODIFIER_UTIL_H_
