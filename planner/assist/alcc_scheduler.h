#ifndef ONBOARD_PLANNER_ASSIST_ALCC_SCHEDULER_H_
#define ONBOARD_PLANNER_ASSIST_ALCC_SCHEDULER_H_

#include <array>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"

#include "common/proto/qalc.pb.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::planner {

struct AlccSchedulerOutput {
  DrivePassage drive_passage;
  // l-s boundaries on drive passage center.
  PathSlBoundary sl_boundary;
  FrenetBox av_frenet_box_on_drive_passage;
  QALCState alc_state;
  LaneChangeDirection lc_direction;
};

struct AlccSchedulerInput {
  const PlannerSemanticMapManager* psmm;
  const VehicleGeometryParamsProto* vehicle_geom;
  const ApolloTrajectoryPointProto* plan_start_point;
  const SpacetimeTrajectoryManager* st_traj_mgr;
  const std::array<mapping::LanePath, 3>* candidate_lanes;
  DriverAction::LaneChangeCommand lc_cmd;
  QALCState prev_alc_state;
  std::optional<double> lcc_cruising_speed_mps;
  const HistoryBufferAbslTime<PiecewiseLinearFunction<double>>*
      online_map_drift_buffer = nullptr;
};

absl::StatusOr<std::vector<AlccSchedulerOutput>> RunAlccScheduler(
    const AlccSchedulerInput& input, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ASSIST_ALCC_SCHEDULER_H_
