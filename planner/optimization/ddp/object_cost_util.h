#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_OBJECT_COST_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_OBJECT_COST_UTIL_H_

#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/optimization/ddp/path_time_corridor.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_defs.h"
#include "onboard/planner/optimization/problem/av_model_helper.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

namespace optimizer {

void AddObjectCosts(
    int trajectory_steps, double trajectory_time_step,
    std::string_view base_name, const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    const PathTimeCorridor& path_time_corridor,
    const FrenetFrame& init_traj_frenet_frame,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<AvModelHelper<Mfob>>& av_model_helpers,
    std::vector<LeadingInfo>* leading_min_s,
    std::vector<double>* left_l_boundary_for_nudge,
    std::vector<double>* right_l_boundary_for_nudge,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs, ThreadPool* thread_pool);

void CalcPartitionHalfContourInfo(const Vec2d& x, const Vec2d& obj_x,
                                  const Polygon2d& contour, double buffer,
                                  std::vector<Segment2d>* lines, Vec2d* ref_x,
                                  Vec2d* ref_tangent, double* offset);

// Re-sample spacetime object state to match with ddp optimizer time step.
std::vector<SpacetimeObjectState> SampleObjectStates(
    int trajectory_steps, double trajectory_time_step,
    absl::Span<const SpacetimeObjectState> states);

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_OBJECT_COST_UTIL_H_
