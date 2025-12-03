#ifndef ONBOARD_PLANNER_PLAN_SPEED_ACC_SPEED_DECISION_H_
#define ONBOARD_PLANNER_PLAN_SPEED_ACC_SPEED_DECISION_H_

#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_approx.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_limit_provider.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

namespace internal {
StOverlapMetaProto AnalyzeStOverlapForACC(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params);

void AnalyzeStOverlapsForACC(
    const DiscretizedPath& path, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    std::vector<StBoundaryRef>* st_boundaries);

std::vector<StBoundaryRef> FilterStBoundariesByCutinTime(
    absl::Span<StBoundaryRef> boundaries,
    const SpacetimeTrajectoryManager& st_traj_mgr);

std::vector<StBoundaryRef> FilterOncomingStBoundaries(
    absl::Span<StBoundaryRef> boundaries,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DiscretizedPath& av_path);

inline bool IsStBoundaryFromStationaryVehicle(const StBoundary& st_bound) {
  LOG(INFO) << static_cast<int>(st_bound.object_type());
  LOG(INFO) << st_bound.is_stationary();
  LOG(INFO) << static_cast<int>(st_bound.source_type());

  if (st_bound.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
    return false;
  }
  if (!st_bound.is_stationary()) return false;
  if (st_bound.object_type() != StBoundaryProto::VEHICLE) return false;
  return true;
}

inline bool IsStBoundaryFromObjects(const StBoundary& st_bound,
                                    const std::set<std::string>& object_ids) {
  const auto& traj_id_opt = st_bound.traj_id();
  if (!traj_id_opt.has_value()) return false;
  const auto obj_id =
      SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(*traj_id_opt);
  return object_ids.find(obj_id) != object_ids.end();
}

void ModifyStationaryObjectStBoundariesForCrowdedScene(
    const std::set<std::string>& crowded_side_objs,
    absl::Span<StBoundaryWithDecision> st_bounds_wd);

void PostProcessingReverseStBoundaries(absl::Span<StBoundaryRef> boundaries);

}  // namespace internal

struct AccSpeedDecisionInput {
  std::string base_name;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const DiscretizedPath* path = nullptr;
  const SpeedLimit* speed_limit = nullptr;
  const SegmentMatcherKdtree* path_kd_tree = nullptr;
  const PathApprox* path_approx = nullptr;
  int trajectory_steps = 0;       // trajectory steps.
  double plan_start_v = 0.0;      // m/s.
  double plan_start_a = 0.0;      // m/s^2.
  double user_speed_limit = 0.0;  // m/s.
  const std::set<std::string>* crowded_side_objs = nullptr;
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
};
struct AccSpeedDecisionOutput {
  SpeedLimitProvider speed_limit_provider;
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  InteractiveSpeedDebugProto interactive_speed_debug;
  SpeedVector sampling_dp_preliminary_speed;
};
absl::StatusOr<AccSpeedDecisionOutput> MakeAccSpeedDecision(
    const AccSpeedDecisionInput& input);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_SPEED_ACC_SPEED_DECISION_H_
