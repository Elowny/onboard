#include "onboard/planner/object/spacetime_planner_object_trajectories_builder.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories_filter.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories_finder.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

SpacetimePlannerObjectTrajectories GetSpacetimePlannerObjectTrajectories(
    absl::Span<const SpacetimeObjectTrajectory> candidate_trajs,
    const std::vector<
        std::unique_ptr<SpacetimePlannerObjectTrajectoriesFinder>>& finders,
    const std::vector<
        std::unique_ptr<SpacetimePlannerObjectTrajectoriesFilter>>& filters,
    double start_offset) {
  constexpr double kEmergencyAvoidanceObjHorizon = 1.5;

  SpacetimePlannerObjectTrajectories res;
  res.trajectories.reserve(candidate_trajs.size());
  res.trajectory_infos.reserve(candidate_trajs.size());
  res.st_start_offset = start_offset;
  // Pick trajectories for spacetime planner.
  for (const auto& traj : candidate_trajs) {
    for (const auto& finder : finders) {
      const auto selected_reason = finder->Find(traj);
      if (selected_reason != SpacetimePlannerObjectTrajectoryReason::NONE) {
        if (selected_reason !=
            SpacetimePlannerObjectTrajectoryReason::DRIVE_IN) {
          bool is_filtered = false;
          for (const auto& filter : filters) {
            if (filter->Filter(traj)) {
              is_filtered = true;
              break;
            }
          }
          if (is_filtered) {
            break;
          }
        }
        double st_planner_traj_horizon = kSpacetimePlannerTrajectoryHorizon;
        if (selected_reason ==
            SpacetimePlannerObjectTrajectoryReason::EMERGENCY_AVOIDANCE) {
          st_planner_traj_horizon = kEmergencyAvoidanceObjHorizon;
        }
        ASSIGN_OR_CONTINUE(
            auto truncated_traj,
            traj.CreateTruncatedCopy(start_offset, st_planner_traj_horizon));

        res.trajectories.push_back(std::move(truncated_traj));
        res.trajectory_infos.push_back(
            {.traj_index = traj.traj_index(),
             .object_id = traj.planner_object().is_sim_agent()
                              ? traj.planner_object().base_id()
                              : traj.planner_object().id(),
             .reason = selected_reason});
        res.trajectory_ids.insert(std::string(traj.traj_id()));
        break;
      }
    }
  }

  return res;
}

}  // namespace

SpacetimePlannerObjectTrajectories BuildSpacetimePlannerObjectTrajectories(
    const SpacetimePlannerObjectTrajectoriesBuilderInput& input,
    absl::Span<const SpacetimeObjectTrajectory> trajectories) {
  FUNC_QTRACE();

  QCHECK_NOTNULL(input.passage);
  QCHECK_NOTNULL(input.sl_boundary);
  QCHECK_NOTNULL(input.veh_geom);
  QCHECK_NOTNULL(input.plan_start_point);
  QCHECK_NOTNULL(input.prev_st_trajs);
  QCHECK_NOTNULL(input.spacetime_planner_object_trajectories_params);

  const auto& config = input.spacetime_planner_object_trajectories_params
                           ->spacetime_planner_object_trajectories_config();

  // Consider all objects in spacetime planner.
  std::vector<std::unique_ptr<SpacetimePlannerObjectTrajectoriesFinder>>
      finders;
  if (config.enable_all_finder()) {
    finders.push_back(
        std::make_unique<AllSpacetimePlannerObjectTrajectoriesFinder>());
  }
  if (config.enable_stationary_finder()) {
    // Use customized finders for spacetime objects.
    finders.push_back(
        std::make_unique<StationarySpacetimePlannerObjectTrajectoriesFinder>(
            input.psmm, input.passage->lane_path()));
  }
  const auto& curr_path_point = input.plan_start_point->path_point();
  const auto av_box =
      ComputeAvBox(Vec2d(curr_path_point.x(), curr_path_point.y()),
                   curr_path_point.theta(), *input.veh_geom);
  if (config.enable_drive_in_filter()) {
    finders.push_back(
        std::make_unique<DrivingInSpacetimePlannerObjectTrajectoriesFinder>(
            av_box, input.plan_start_point->v(), input.lane_change_state,
            input.psmm, input.passage));
  }

  if (config.enable_front_side_moving_finder()) {  // NOLINT
    finders.push_back(std::make_unique<
                      FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder>(
        av_box, input.passage, input.sl_boundary, input.plan_start_point->v(),
        input.prev_st_trajs, input.time_aligned_prev_traj, input.veh_geom));
  }
  if (config.enable_dangerous_side_moving_finder()) {  // NOLINT
    finders.push_back(
        std::make_unique<
            DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder>(
            av_box, input.passage, input.plan_start_point->v()));
  }
  // Activate with caution, all front trajectories will be considered by
  // spacetime planner.
  if (config.enable_front_moving_finder()) {
    finders.push_back(
        std::make_unique<FrontMovingSpacetimePlannerObjectTrajectoriesFinder>(
            input.passage, input.plan_start_point, input.veh_geom->length()));
  }

  std::vector<std::unique_ptr<SpacetimePlannerObjectTrajectoriesFilter>>
      filters;
  if (config.enable_cutin_vehicle_filter()) {  // NOLINT
    filters.push_back(
        std::make_unique<CutInVehicleSpacetimePlannerObjectTrajectoriesFilter>(
            input.passage, input.sl_boundary, av_box,
            input.plan_start_point->v()));
  }
  if (config.enable_crossing_filter()) {
    filters.push_back(
        std::make_unique<CrossingSpacetimePlannerObjectTrajectoriesFilter>(
            input.passage, &input.spacetime_planner_object_trajectories_params
                                ->crossing_filter_params()));
  }
  if (config.enable_reverse_vehicle_filter()) {  // NOLINT
    filters.push_back(std::make_unique<
                      ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter>(
        input.passage, input.sl_boundary));
  }
  if (config.enable_beyond_stop_line_filter()) {  // NOLINT
    filters.push_back(std::make_unique<
                      BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter>(
        input.passage, input.stop_lines));
  }
  if (config.enable_occluded_filter()) {
    filters.push_back(
        std::make_unique<
            OccludedVehicleSpacetimePlannerObjectTrajectoriesFilter>());
  }

  if (config.enable_behavior_conflict_filter()) {
    filters.push_back(std::make_unique<
                      BehaviorConflictSpacetimePlannerObjectTrajectoriesFilter>(
        *input.passage, *input.sl_boundary, input.planner_objects, av_box,
        input.plan_start_point->v()));
  }

  return GetSpacetimePlannerObjectTrajectories(trajectories, finders, filters,
                                               input.st_planner_start_offset);
}

}  // namespace planner
}  // namespace qcraft
