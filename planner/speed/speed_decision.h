#ifndef ONBOARD_PLANNER_SPEED_SPEED_DECISION_H_
#define ONBOARD_PLANNER_SPEED_SPEED_DECISION_H_

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/vehicle_shape.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_limit_provider.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_graph.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {
struct SpeedDecisionInput {
  std::string base_name;
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const VehicleDriveParamsProto* vehicle_drive_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const KdTreeFrenetFrame* built_target_frenet_frame = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const DrivingMapTopo* driving_map_topo = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  // Path that has been resampled with kPathSampleInterval.
  const DiscretizedPath* path = nullptr;
  // Raw path points without resampling.
  const std::vector<PathPoint>* st_path_points = nullptr;
  const std::vector<VehicleShapeBasePtr>* av_shapes = nullptr;
  const SegmentMatcherKdtree* path_kd_tree = nullptr;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const absl::flat_hash_set<std::string>* follower_set = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const absl::flat_hash_set<std::string>* congested_cutin_object_ids = nullptr;
  const prediction::AvContext* planner_av_context = nullptr;
  const ObjectsProto* real_objects = nullptr;
  const ObjectsProto* virtual_objects = nullptr;
  const ModelPool* planner_model_pool = nullptr;
  bool run_act_net_speed_decision = false;
  absl::Time plan_time;
  double plan_start_v = 0.0;
  double plan_start_a = 0.0;
  double planner_speed_cap = 0.0;
  int trajectory_steps = 0;
};
struct SpeedDecisionOutput {
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  SpeedLimitProvider speed_limit_provider;
  ConstraintManager constraint_mgr;
  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      processed_st_objects;
  SpeedVector preliminary_speed;
  std::unordered_map<std::string, double> overlap_trajs_info;
  InteractiveSpeedDebugProto interactive_speed_debug;
};

/**
 * @brief Generate st boundaries and make decision.
 * @return St boundaries with decision, speed limits and debug info.
 */
absl::StatusOr<SpeedDecisionOutput> MapStBoundariesAndMakeSpeedDecision(
    const SpeedDecisionInput& input, StGraph* st_graph,
    ThreadPool* thread_pool);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_DECISION_H_
