#ifndef ONBOARD_PLANNER_ASSIST_ALCC_TARGET_SELECTOR_
#define ONBOARD_PLANNER_ASSIST_ALCC_TARGET_SELECTOR_

#include "absl/status/status.h"

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {

namespace planner {

absl::Status SelectAlccTarget(
    const PathSlBoundary& map_path_sl, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    SpacetimeTrajectoryManager* st_traj_mgr);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ASSIST_ALCC_TARGET_SELECTOR_
