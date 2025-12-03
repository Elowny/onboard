#ifndef ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_CONFIG_DISPATCHER_H_
#define ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_CONFIG_DISPATCHER_H_

#include <optional>

#include "absl/types/span.h"

#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/common/path_approx.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

std::optional<SpeedFinderParamsProto::SpeedOptimizerParamsProto>
DispatchSpeedOptimizerConfig(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr,
    const DrivePassage* drive_passage, const PathApprox& path_approx,
    const SegmentMatcherKdtree& path_kd_tree, double av_radius,
    double path_step_length, int path_last_index, double current_v,
    const PathPoint& current_path_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedFinderParamsProto::SpeedOptimizerParamsProto&
        raw_speed_optimizer_params,
    const SpeedFinderParamsProto::SpeedOptimizerConfigDispatcherParams&
        config_dispatcher_params,
    SpeedFinderDebugProto* speed_finder_debug);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_CONFIG_DISPATCHER_H_
