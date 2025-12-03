#include "onboard/planner/plan/async_planner_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include <memory> for allocator_traits<>::value_type

#include <algorithm>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "google/protobuf/repeated_ptr_field.h"  // for RepeatedPtrField, RepeatedPtrIterator

#include "onboard/global/buffered_logger.h"  // for BufferedLoggerWrapper
#include "onboard/lite/check.h"              // for QCHECK
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/common/proto/planner_status.pb.h"  // for PlannerStatusProto
#include "onboard/planner/decision/proto/constraint.pb.h"  // for ConstraintProto
#include "onboard/planner/est_planner_output.h"  // for EstPlannerDebug, EstPlannerOutput
#include "onboard/planner/initializer/proto/initializer.pb.h"  // for InitializerDebugProto
#include "onboard/planner/ml/captain_net/proto/captain_net_debug.pb.h"  // for CapnetTrajectoryDebugProto
#include "onboard/planner/object/proto/planner_object.pb.h"  // for FilteredTrajectories, SpacetimePlannerObject...
#include "onboard/planner/optimization/proto/optimizer.pb.h"  // for TrajectoryOptimizerDebugProto
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"  // for TrajectoryValidationResultProto
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"  // for SpeedFinderDebugProto
#include "onboard/planner/util/path_util.h"

namespace qcraft::planner {

void UpdateAsyncCounter(int low_freq_cycle, int* counter) {
  *counter = (*counter + 1) % (low_freq_cycle + 1);
}

absl::StatusOr<std::vector<PathPoint>> AlignPathWithPlanStartPoint(
    const std::vector<PathPoint>& path,
    const ApolloTrajectoryPointProto& plan_start_point) {
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  if (path.empty()) {
    return absl::InvalidArgumentError(
        "The provided path is empty when align path with plan start point.");
  }
  const DiscretizedPath discretized_path(path);
  const double ego_path_s = discretized_path.XYToSL(ego_pos).s;

  // TODO(boqian): return error if start point is far away from path.

  if (ego_path_s >= discretized_path.length()) {
    return absl::NotFoundError("Plan start point exceeds the latest path!");
  }

  auto aligned_path = path;
  const auto next_it = std::upper_bound(
      aligned_path.begin(), aligned_path.end(), ego_path_s,
      [](double s, const PathPoint& path_pt) { return s < path_pt.s(); });

  if (next_it == aligned_path.begin()) {
    constexpr double kStartPointBehindPathThres = 0.5;  // m.
    if (ego_path_s > -kStartPointBehindPathThres) {
      return aligned_path;
    }
    return absl::NotFoundError("Plan start point is behind path.");
  }

  PathPoint lerp_point;
  LerpPathPoint(
      *(next_it - 1), *next_it,
      (ego_path_s - (next_it - 1)->s()) / (next_it->s() - (next_it - 1)->s()),
      &lerp_point);

  constexpr double kEpsilonDist = 0.01;  // m.
  aligned_path.erase(
      aligned_path.begin(),
      next_it->s() > ego_path_s + kEpsilonDist ? next_it - 1 : next_it);

  aligned_path[0] = lerp_point;
  aligned_path[0].set_s(0.0);

  const double s_offset =
      aligned_path[1].s() -
      Vec2d(aligned_path[1].x(), aligned_path[1].y())
          .DistanceTo(Vec2d(aligned_path[0].x(), aligned_path[0].y()));
  for (int i = 1; i < aligned_path.size(); ++i) {
    aligned_path[i].set_s(aligned_path[i].s() - s_offset);
  }

  return aligned_path;
}

void StitchStPathTrajectoryWithPastTrajectory(
    const TrajectoryProto& previous_trajectory,
    const std::optional<int>& start_index_on_prev_traj,
    const std::vector<PathPoint>& st_path_points,
    std::vector<PathPoint>* st_path_points_including_past) {
  // If reset, do not stitch previous trajectory.
  if (!start_index_on_prev_traj.has_value()) {
    st_path_points_including_past->clear();
    st_path_points_including_past->assign(st_path_points.begin(),
                                          st_path_points.end());
    return;
  }
  st_path_points_including_past->clear();
  st_path_points_including_past->reserve(
      previous_trajectory.past_points_size() + st_path_points.size());
  if (!previous_trajectory.past_points().empty()) {
    for (const auto& traj_point : previous_trajectory.past_points()) {
      st_path_points_including_past->push_back(traj_point.path_point());
    }
  }

  if (*start_index_on_prev_traj != 0) {
    QCHECK(!previous_trajectory.trajectory_point().empty());
    for (int i = 0; i < *start_index_on_prev_traj; ++i) {
      st_path_points_including_past->push_back(
          previous_trajectory.trajectory_point(i).path_point());
    }
  }

  const double current_path_point_offset =
      st_path_points_including_past->empty()
          ? 0.0
          : st_path_points_including_past->front().s();

  if (!st_path_points_including_past->empty()) {
    for (int i = 0; i < st_path_points_including_past->size(); ++i) {
      auto& path_point_cur = (*st_path_points_including_past)[i];
      path_point_cur.set_s(path_point_cur.s() - current_path_point_offset);
    }
  }
  const double start_index = st_path_points_including_past->size();
  st_path_points_including_past->insert(st_path_points_including_past->end(),
                                        st_path_points.begin(),
                                        st_path_points.end());
  if (!previous_trajectory.trajectory_point().empty()) {
    const double offset =
        previous_trajectory.trajectory_point(*start_index_on_prev_traj)
            .path_point()
            .s() -
        current_path_point_offset;
    for (int i = start_index; i < st_path_points_including_past->size(); ++i) {
      auto& path_point_cur = (*st_path_points_including_past)[i];
      path_point_cur.set_s(path_point_cur.s() + offset);
    }
  }
  const double offset = st_path_points_including_past->front().s();
  for (int i = 0; i < st_path_points_including_past->size(); ++i) {
    auto& path_point_cur = (*st_path_points_including_past)[i];
    path_point_cur.set_s(path_point_cur.s() - offset);
  }
}

// No valid output but still contains some debug information.
bool IsNonEmptyPlannerResult(const PlannerStatus& status) {
  switch (status.status_code()) {
    case PlannerStatusProto::OK:
    case PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE:
    case PlannerStatusProto::INITIALIZER_FAILED:
    case PlannerStatusProto::OPTIMIZER_FAILED:
    case PlannerStatusProto::PATH_EXTENSION_FAILED:
    case PlannerStatusProto::SPEED_OPTIMIZER_FAILED:
    case PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED:
    case PlannerStatusProto::SELECTOR_FAILED:
    case PlannerStatusProto::MODEL_PRECONDITION_CHECK_FAILED:
    case PlannerStatusProto::MODEL_INFERENCE_FAILED:
    case PlannerStatusProto::MODEL_OUTPUT_POSTPROCESS_FAILED:
    case PlannerStatusProto::BRANCH_RESULT_IGNORED:
    case PlannerStatusProto::LC_SAFETY_CHECK_FAILED:
      return true;
    case PlannerStatusProto::UNINITIALIZED:
    case PlannerStatusProto::ROUTE_MSG_UNAVAILABLE:
    case PlannerStatusProto::GEAR_NOT_READY:
    case PlannerStatusProto::INPUT_INCORRECT:
    case PlannerStatusProto::ROUTING_FAILED:
    case PlannerStatusProto::OBJECT_MANAGER_FAILED:
    case PlannerStatusProto::REFERENCE_PATH_UNAVAILABLE:
    case PlannerStatusProto::SCHEDULER_UNAVAILABLE:
    case PlannerStatusProto::FALLBACK_PREVIOUS_INFO_UNAVAILABLE:
    case PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED:
    case PlannerStatusProto::RESET_PREV_TARGET_LANE_PATH_FAILED:
    case PlannerStatusProto::UPDATE_TARGET_LANE_PATH_FAILED:
    case PlannerStatusProto::LOCAL_LANE_MAP_BUILDER_FAILED:
    case PlannerStatusProto::PLANNER_ABNORMAL_EXIT:
    case PlannerStatusProto::TRAFFIC_LIGHT_INFO_COLLECTOR_FAILED:
    case PlannerStatusProto::PLANNER_STATE_INCOMPLETE:
    case PlannerStatusProto::EXPERT_TRAJ_UNAVAILABLE:
    case PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED:
    case PlannerStatusProto::EXPERT_SPEED_IN_CONTRADICTORY_WITH_SPEED_FINDER:
    case PlannerStatusProto::LOW_FREQ_SCHEDULE_FUTURE_FAILED:
    case PlannerStatusProto::LOW_FREQ_RESULT_NOT_YET_AVAILABLE:
    case PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE:
    case PlannerStatusProto::OPTIMIZER_DIFF_INTENTION_EXPERT:
    case PlannerStatusProto::LCC_ODC_CHECK_FAIL:
    case PlannerStatusProto::BUILD_DRIVING_MAP_FAILED:
    case PlannerStatusProto::ACC_CORRIDOR_FAILED:
    case PlannerStatusProto::ACC_SPEED_FAILED:
    case PlannerStatusProto::ACC_TRAJECTORY_VALIDATION_FAILED:
    case PlannerStatusProto::PROJECT_TO_ONLINE_MAP_FAILED:
    case PlannerStatusProto::ACC_ALL_PLAN_FAIL:
    case PlannerStatusProto::PLANNER_INTERNAL_FAILED:
    case PlannerStatusProto::DRIVERLESS_PLAN_MAIN_LOOP_FAILED:
    case PlannerStatusProto::NOA_MAIN_LOOP_FAILED:
    case PlannerStatusProto::ALCC_MAIN_LOOP_FAILED:
    case PlannerStatusProto::ACC_MAIN_LOOP_FAILED:
    case PlannerStatusProto::MAPLESS_NOA_MAIN_LOOP_FAILED:
    case PlannerStatusProto::APA_MAIN_LOOP_FAILED:
      return false;
  }
}

void FillHighFreqPlannerDebug(AsyncPlannerOutput* async_output,
                              EstPlannerDebugProto* est_planner_debug) {
  const auto& est_output = async_output->latest_low_freq_output->est_output;
  const auto& est_debug = async_output->latest_low_freq_output->est_debug;

  async_output->est_status.ToProto(est_planner_debug->mutable_planner_status());
  *est_planner_debug->mutable_filtered() =
      std::move(async_output->filtered_prediction_trajectories);

  *est_planner_debug->mutable_st_planner_object_trajectories() =
      est_debug.st_planner_object_trajectories;

  ToSchedulerOutputProto(est_output.scheduler_output,
                         est_planner_debug->mutable_scheduler());

  *est_planner_debug->mutable_constraint() =
      std::move(async_output->decision_constraints);

  *est_planner_debug->mutable_initializer() = est_debug.initializer_debug_proto;

  *est_planner_debug->mutable_trajectory_optimizer() =
      est_debug.optimizer_debug_proto;

  *est_planner_debug->mutable_lane_change_safety_debug_proto() =
      est_debug.lane_change_safety_debug_proto;

  *est_planner_debug->mutable_st_path_points() = {
      est_output.st_path_points.begin(), est_output.st_path_points.end()};

  *est_planner_debug->mutable_capnet_traj() = est_debug.capnet_traj_debug;

  *est_planner_debug->mutable_speed_finder() =
      std::move(async_output->speed_finder_debug);

  *est_planner_debug->mutable_traj_validation() =
      std::move(async_output->traj_validation_result);
}

void UpdateAsyncCounterAfterLowFreqRetrieved(
    AsyncPlannerState* async_planner_state) {
  if (async_planner_state->secondary_counter.has_value()) {
    async_planner_state->counter = kAsyncCounterInitVal;
    async_planner_state->secondary_counter = std::nullopt;
  } else if (async_planner_state->task_transition) {
    async_planner_state->task_transition = false;
    async_planner_state->secondary_counter = kAsyncCounterInitVal;
  } else {
    async_planner_state->counter = kAsyncCounterInitVal;
  }
}

}  // namespace qcraft::planner
