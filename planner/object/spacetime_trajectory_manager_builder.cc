#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"

#include <ostream>
#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/assist/alcc_target_selector.h"
#include "onboard/planner/object/drive_passage_filter.h"
#include "onboard/planner/object/low_likelihood_filter.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/planner/planner_flags.h"

namespace qcraft {
namespace planner {

SpacetimeTrajectoryManager BuildSpacetimeTrajectoryManager(
    const SpacetimeTrajectoryManagerBuilderInput& input,
    ThreadPool* thread_pool) {
  QCHECK_NOTNULL(input.passage);
  QCHECK_NOTNULL(input.sl_boundary);
  QCHECK_NOTNULL(input.obj_mgr);

  // Build spacetime object manager.
  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold,
      FLAGS_planner_only_use_most_likely_trajectory);
  const DrivePassageFilter drive_passage_filter(
      input.passage, input.sl_boundary, input.on_vision_map);

  std::vector<const TrajectoryFilter*> filters;
  filters.push_back(&low_likelihood_filter);
  filters.push_back(&drive_passage_filter);
  return SpacetimeTrajectoryManager(filters, input.obj_mgr->planner_objects(),
                                    thread_pool);
}

SpacetimeTrajectoryManager BuildSpacetimeTrajectoryManagerWithStTrajCutinFilter(
    const PathSlBoundary& map_path_sl, const DrivePassage& drive_passage,
    const PlannerObjectManager& obj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom, bool on_vision_map,
    ThreadPool* thread_pool) {
  // 1.0 Build st traj mgr.
  auto st_traj_mgr = BuildSpacetimeTrajectoryManager(
      SpacetimeTrajectoryManagerBuilderInput{.passage = &drive_passage,
                                             .sl_boundary = &map_path_sl,
                                             .obj_mgr = &obj_mgr,
                                             .on_vision_map = on_vision_map},
      thread_pool);
  // 2.0 Filter cutin spacetime trajectories.
  const auto status = SelectAlccTarget(
      map_path_sl, drive_passage, plan_start_point, vehicle_geom, &st_traj_mgr);
  if (!status.ok()) {
    QLOG(INFO) << "Fail to select alcc target: " << status.message();
  }
  return st_traj_mgr;
}

}  // namespace planner
}  // namespace qcraft
