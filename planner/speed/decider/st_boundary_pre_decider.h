#ifndef ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_gap_result.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/vt_speed_limit.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct PreDeciderInput {
  const SpeedFinderParamsProto::StBoundaryPreDeciderParamsProto* params =
      nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const absl::flat_hash_set<std::string>* follower_set = nullptr;
  const TrafficGapResult* lane_change_gap = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const DiscretizedPath* path = nullptr;
  const VehicleGeometryParamsProto* vehicle_geo_params = nullptr;
  const DrivePassage* drive_passage = nullptr;
  double current_v = 0.0;
  double current_s = 0.0;
  double max_v = 0.0;
  double time_step = 0.0;
  int trajectory_steps = 0;
  const ModelPool* planner_model_pool = nullptr;
  const prediction::AvContext* planner_av_context = nullptr;
  const ObjectsProto* real_objects = nullptr;
  const ObjectsProto* virtual_objects = nullptr;
  absl::Time plan_time;
  bool run_act_net_speed_decision = false;
};

void MakeFreespacePreDecisionForStBoundaries(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision);

void MakePreDecisionForStBoundaries(
    const PreDeciderInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::optional<VtSpeedLimit>* speed_limit);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_
