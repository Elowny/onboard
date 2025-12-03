#ifndef ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_
#define ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"

#include "onboard/math/frenet_common.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// Returns leading objects that we should not pass. Currently all leading
// objects are associated with our lane path.
std::vector<ConstraintProto::LeadingObjectProto> FindLeadingObjects(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const PathSlBoundary& sl_boundary, LaneChangeStage lc_stage,
    const SceneOutputProto& scene_reasoning,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ApolloTrajectoryPointProto& plan_start_point,
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetBox& ego_frenet_box, bool borrow_lane_boundary);

std::vector<ConstraintProto::LeadingObjectProto>
DeriveLeadingObjectsFromCaptainNetTrajectory(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DrivePassage& passage,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<ApolloTrajectoryPointProto>& traj_points);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_
