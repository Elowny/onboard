#ifndef ONBOARD_PREDICTION_POST_PROCESS_TRAJECTORY_MODIFIER_H_
#define ONBOARD_PREDICTION_POST_PROCESS_TRAJECTORY_MODIFIER_H_

#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {

void CutoffTrajByLaneBoundary(
    const planner::PlannerSemanticMapManager& smm,
    const ObjectProto& object_proto,
    const ConflictResolutionConfigProto::ObjectConflictResolverConfig&
        object_config,
    mapping::LaneBoundaryProto::Type lane_boundary_type,
    PredictedTrajectory* traj);

void CutoffTrajByCurb(
    const planner::PlannerSemanticMapManager& smm,
    const ObjectProto& object_proto,
    const ConflictResolutionConfigProto::ObjectConflictResolverConfig&
        object_config,
    PredictedTrajectory* traj);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_POST_PROCESS_TRAJECTORY_MODIFIER_H_
