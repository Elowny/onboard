#ifndef ONBOARD_PLANNER_SPEED_SPEED_FINDER_UTIL_H_
#define ONBOARD_PLANNER_SPEED_SPEED_FINDER_UTIL_H_

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/types/span.h"

#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_approx.h"
#include "onboard/planner/common/vehicle_shape.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_bound.h"
#include "onboard/planner/speed/speed_limit_provider.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/vt_speed_limit.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

void SetStBoundaryDebugInfo(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    SpeedFinderDebugProto* speed_finder_proto);

int GetSpeedFinderTrajectorySteps(double init_v, int max_traj_steps);

void PostProcessSpeedByFullStop(
    const SpeedFinderParamsProto& speed_finder_params, SpeedVector* speed_data);

std::vector<VehicleShapeBasePtr> BuildAvShapes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const DiscretizedPath& path_points);

std::unique_ptr<SegmentMatcherKdtree> BuildPathKdTree(
    const DiscretizedPath& path_points);

std::optional<PathApprox> BuildPathApproxForMirrors(
    const PathApprox& path_approx,
    const VehicleGeometryParamsProto& vehicle_geom);

std::vector<PartialSpacetimeObjectTrajectory> GetConsideredStObjects(
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& obj_mgr,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>
        processed_st_objects);

void CutoffSpeedByTimeHorizon(SpeedVector* speed_data);

// If the st-traj has been modified, the original one will also be inserted in
// the map.
std::unordered_map<std::string, const SpacetimeObjectTrajectory*>
GetAllOverlappedStObjectTrajs(
    const std::unordered_map<std::string, double>& considered_trajs,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    const SpacetimeTrajectoryManager& traj_mgr);

SpeedVector GenerateReferenceSpeed(
    const std::vector<SpeedBoundWithInfo>& min_speed_limit, double init_v,
    double ref_speed_bias, double ref_speed_static_limit_bias, double max_accel,
    double max_decel, double total_time, double delta_t);
// Speed limit is set to speed vector value + buffer
VtSpeedLimit GetVtSpeedLimitFromSpeedVector(
    const SpeedVector& preliminary_speed, int traj_steps, double time_step,
    double buffer);
SpeedVector GeneratePredictedAvSpeed(const SpeedVector& preliminary_speed,
                                     double av_speed, double av_acc,
                                     double plan_total_time);

SpeedBoundMapType EstimateSpeedBound(
    const SpeedLimitProvider& speed_limit_provider,
    const SpeedVector& preliminary_speed, double init_v,
    double allowed_max_speed, int knot_num, double delta_t,
    std::string_view base_name);

std::vector<SpeedBoundWithInfo> GenerateMinSpeedLimit(
    const std::vector<SpeedBoundWithInfo>& lim_1,
    const std::vector<SpeedBoundWithInfo>& lim_2, const std::string& info);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_SPEED_FINDER_UTIL_H_
