#ifndef ONBOARD_PLANNER_ML_ACT_NET_SPEED_ACT_NET_SPEED_DECIDER_H_
#define ONBOARD_PLANNER_ML_ACT_NET_SPEED_ACT_NET_SPEED_DECIDER_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct ActNetInteractionDecisionInput {
  const DiscretizedPath* av_path = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const prediction::AvContext* av_context = nullptr;
  const ObjectsProto* real_objects = nullptr;
  const ObjectsProto* virtual_objects = nullptr;
  const ModelPool* planner_model_pool = nullptr;
  absl::Time plan_time;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
};

absl::Status ActNetMakeInteractiveSpeedDecision(
    const ActNetInteractionDecisionInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision);

// Compare speed decision results and act net speed model output and record
// QEvents of decision differences.
void EvaluateActNetSpeedDecision(
    absl::Span<const StBoundaryWithDecision> st_bounds_wd);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ML_ACT_NET_SPEED_ACT_NET_SPEED_DECIDER_H_
