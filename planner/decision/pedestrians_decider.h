#ifndef ONBOARD_PLANNER_DECISION_PEDESTRIANS_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_PEDESTRIANS_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<std::vector<ConstraintProto::SpeedRegionProto>>
BuildPedestriansConstraints(
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const PathSlBoundary& sl_boundary,
    const SpacetimeTrajectoryManager& st_traj_mgr);

}  // namespace planner
}  // namespace qcraft

#endif
