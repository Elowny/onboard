#ifndef ONBOARD_PLANNER_SPEED_CONSTRAINT_GENERATOR_H_
#define ONBOARD_PLANNER_SPEED_CONSTRAINT_GENERATOR_H_

#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/path_semantic_analyzer.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_graph.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct SpeedConstraintGeneratorOutput {
  std::vector<ConstraintProto::PathSpeedRegionProto> path_speed_regions;
  std::vector<ConstraintProto::PathStopLineProto> path_stop_lines;
};

SpeedConstraintGeneratorOutput GenerateStationaryCloseObjectConstraints(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& traj_mgr,
    const DiscretizedPath& path, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary, double av_speed);

SpeedConstraintGeneratorOutput GenerateDenseTrafficFlowConstraint(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const DiscretizedPath& path, double plan_start_v,
    const VehicleGeometryParamsProto& vehicle_geometry_params);

}  // namespace planner

}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_CONSTRAINT_GENERATOR_H_
