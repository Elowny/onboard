#ifndef ONBOARD_PLANNER_FREESPACE_FREESPACE_STOP_FINDER_H_
#define ONBOARD_PLANNER_FREESPACE_FREESPACE_STOP_FINDER_H_

#include <limits>
#include <string>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/planner_cluster_object.h"
#include "onboard/planner/object/planner_cluster_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct FreespaceStopFinderOutput {
  double stop_s = 0.0;
  double stationary_object_stop_s = std::numeric_limits<double>::infinity();
  std::string nearest_stationary_object_id = "";
  SpeedFinderDebugProto stop_finder_debug;
  vis::vantage::ChartDataProto st_graph_chart;
};

absl::StatusOr<FreespaceStopFinderOutput> FindFreespaceStop(
    const DiscretizedPath& path, bool forward, double plan_start_v,
    absl::Time plan_time, const PlannerSemanticMapManager* psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlannerClusterObjectManager& cluster_obj_mgr,
    const ConstraintManager& constraint_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const absl::flat_hash_set<PlannerClusterObject::Id>&
        stalled_cluster_objects,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params, ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_FREESPACE_STOP_FINDER_H_
