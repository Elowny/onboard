#ifndef ONBOARD_PLANNER_SPEED_SPEED_FINDER_INPUT_H_
#define ONBOARD_PLANNER_SPEED_SPEED_FINDER_INPUT_H_

#include <map>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

struct SpeedFinderInput {
  std::string base_name;
  const DrivingMapTopo* driving_map_topo = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const absl::flat_hash_set<std::string>* follower_set = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  // Path that has been resampled with kPathSampleInterval.
  const DiscretizedPath* path = nullptr;
  // Raw path points without resampling.
  const std::vector<PathPoint>* st_path_points = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  double plan_start_v = 0.0;
  double plan_start_a = 0.0;
  double plan_start_j = 0.0;
  absl::Time plan_time;
  const prediction::AvContext* planner_av_context = nullptr;
  const ObjectsProto* real_objects = nullptr;
  const ObjectsProto* virtual_objects = nullptr;
  const ModelPool* planner_model_pool = nullptr;
  bool run_act_net_speed_decision = false;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_FINDER_INPUT_H_
