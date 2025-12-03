#ifndef ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_HYBRID_A_STAR_H
#define ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_HYBRID_A_STAR_H

#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/freespace/directional_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<std::vector<DirectionalPath>> FindPathWithKDTree(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    double max_kappa, FreespaceTaskProto::TaskType task_type,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<SpecialBoundary>& special_boundaries,
    const PathPoint& start, const PathPoint& end,
    PathFinderDebugProto* debug_info);

absl::StatusOr<std::vector<DirectionalPath>> FindPath(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    FreespaceTaskProto::TaskType task_type, const FreespaceMap& freespace_map,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const PathPoint& start, const PathPoint& end,
    PathFinderDebugProto* debug_info);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_HYBRID_A_STAR_H
