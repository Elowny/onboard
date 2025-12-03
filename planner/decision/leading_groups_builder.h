#ifndef ONBOARD_PLANNER_DECISION_LEADING_GROUPS_BUILDER_H_
#define ONBOARD_PLANNER_DECISION_LEADING_GROUPS_BUILDER_H_

#include <map>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"

#include "onboard/math/frenet_common.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

// Use trajectory id as key.
using LeadingGroup = std::map<std::string, ConstraintProto::LeadingObjectProto>;

std::vector<LeadingGroup> FindMultipleLeadingGroups(
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    bool lc_left, const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects, double ego_heading,
    const FrenetBox& ego_frenet_box,
    const VehicleGeometryParamsProto& vehicle_geom, const double cur_ego_v);

std::vector<LeadingGroup> DeriveMultipleLeadingGroupsFromCaptainNetTrajectory(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DrivePassage& passage,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<ApolloTrajectoryPointProto>& traj_points);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_DECISION_LEADING_GROUPS_BUILDER_H_
