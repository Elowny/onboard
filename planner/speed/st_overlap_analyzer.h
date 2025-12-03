#ifndef ONBOARD_PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_
#define ONBOARD_PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_

#include <vector>

#include "absl/types/span.h"

#include "onboard/math/frenet_frame.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/vehicle_shape.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/path_semantic_analyzer.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

bool IsAnalyzableStBoundary(const StBoundaryRef& st_boundary);

void AnalyzeStOverlaps(
    const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage* drive_passage,
    const KdTreeFrenetFrame* built_target_frenet_frame,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedFinderParamsProto::StOverlapAnalyzerParamsProto&
        st_overlap_analyzer_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v,
    std::vector<StBoundaryRef>* st_boundaries);

StOverlapMetaProto::OverlapPattern AnalyzeOverlapPattern(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_
