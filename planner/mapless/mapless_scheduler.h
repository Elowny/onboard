#ifndef ONBOARD_PLANNER_MAPLESS_MAPLESS_SCHEDULER_H_
#define ONBOARD_PLANNER_MAPLESS_MAPLESS_SCHEDULER_H_

#include <optional>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/math/frenet_common.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct MaplessSchedulerOutput {
  DrivePassage drive_passage;
  PathSlBoundary sl_boundary;
  LaneChangeStateProto lane_change_state;
  FrenetBox av_frenet_box_on_drive_passage;
  bool borrow_lane = false;
};

struct MaplessSchedulerInput {
  const PlannerSemanticMapManager* psmm;
  const VehicleGeometryParamsProto* vehicle_geom;
  const ApolloTrajectoryPointProto* plan_start_point;
  const SpacetimeTrajectoryManager* st_traj_mgr;
  const std::vector<mapping::LanePath>* target_lp_vec;
  const mapping::LanePath* prev_target_lane_path = nullptr;
  const LaneChangeStateProto* prev_lc_state = nullptr;
  std::optional<double> cruising_speed_mps = std::nullopt;
  AutonomyStateProto::State autonomy_state = AutonomyStateProto::NOT_READY;
};

absl::StatusOr<std::vector<MaplessSchedulerOutput>> RunMaplessScheduler(
    const MaplessSchedulerInput& input, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_MAPLESS_MAPLESS_SCHEDULER_H_
