#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_BUILDER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_BUILDER_H_

#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct SpacetimePlannerObjectTrajectoriesBuilderInput {
  const PlannerSemanticMapManager* psmm;
  const DrivePassage* passage;
  const PathSlBoundary* sl_boundary;
  const LaneChangeStateProto* lane_change_state;
  const VehicleGeometryParamsProto* veh_geom;
  const ApolloTrajectoryPointProto* plan_start_point;
  double st_planner_start_offset;
  const SpacetimePlannerObjectTrajectoriesProto* prev_st_trajs;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj;
  absl::Span<const ConstraintProto::StopLineProto> stop_lines;
  const SpacetimePlannerObjectTrajectoriesParamsProto*
      spacetime_planner_object_trajectories_params;
  absl::Span<const PlannerObject> planner_objects;
};

SpacetimePlannerObjectTrajectories BuildSpacetimePlannerObjectTrajectories(
    const SpacetimePlannerObjectTrajectoriesBuilderInput& input,
    absl::Span<const SpacetimeObjectTrajectory> trajectories);

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_BUILDER_H_
