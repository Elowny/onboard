#ifndef ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_
#define ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
struct SpacetimeTrajectoryManagerBuilderInput {
  const DrivePassage* passage;
  const PathSlBoundary* sl_boundary;
  const PlannerObjectManager* obj_mgr;
  bool on_vision_map = false;
};

SpacetimeTrajectoryManager BuildSpacetimeTrajectoryManager(
    const SpacetimeTrajectoryManagerBuilderInput& input,
    ThreadPool* thread_pool);

SpacetimeTrajectoryManager BuildSpacetimeTrajectoryManagerWithStTrajCutinFilter(
    const PathSlBoundary& map_path_sl, const DrivePassage& drive_passage,
    const PlannerObjectManager& obj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom, bool on_vision_map,
    ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_
