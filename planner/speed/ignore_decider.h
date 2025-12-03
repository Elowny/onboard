#ifndef ONBOARD_PLANNER_SPEED_IGNORE_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_IGNORE_DECIDER_H_

#include <optional>
#include <vector>

#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/vehicle_shape.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/path_semantic_analyzer.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/vt_speed_limit.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct IgnoreDeciderInput {
  const SpeedFinderParamsProto::IgnoreDeciderParamsProto* params = nullptr;
  const DiscretizedPath* path = nullptr;
  // Could be empty but not null.
  const std::vector<PathPointSemantic>* path_semantics = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const std::vector<VehicleShapeBasePtr>* av_shapes = nullptr;
  const SegmentMatcherKdtree* path_kd_tree = nullptr;
  double current_v = 0.0;
  double current_a = 0.0;
  double max_v = 0.0;
  double time_step = 0.0;
  int trajectory_steps = 0;
};

void MakeIgnoreAndPreBrakeDecisionForStBoundaries(
    const IgnoreDeciderInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::optional<VtSpeedLimit>* speed_limit);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_IGNORE_DECIDER_H_
