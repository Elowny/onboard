#include "onboard/planner/assist/alcc_selector.h"

#include <optional>
#include <ostream>
#include <string_view>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/lite/logging.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/assist/assist_util.h"
#include "onboard/planner/assist/proto/plc_result.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

bool HasSolidLineNearBy(const DrivePassage& drive_passage,
                        absl::Span<const ApolloTrajectoryPointProto> traj_pts) {
  constexpr double kQueryT = 1.0;  // s.
  const auto query_traj_point =
      QueryApolloTrajectoryPointByT(traj_pts.begin(), traj_pts.end(), kQueryT);

  ASSIGN_OR_RETURN(const auto query_point_sl,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       Vec2dFromApolloTrajectoryPointProto(query_traj_point)),
                   false);

  const auto boundaries =
      drive_passage.QueryEnclosingLaneBoundariesAtS(query_point_sl.s);

  if (query_point_sl.l > 0.0 && boundaries.left.has_value()) {
    return boundaries.left->IsSolid(query_point_sl.l) &&
           query_point_sl.l > boundaries.left->lat_offset;
  } else if (query_point_sl.l < 0.0 && boundaries.right.has_value()) {
    return boundaries.right->IsSolid(query_point_sl.l) &&
           query_point_sl.l < boundaries.right->lat_offset;
  }

  return false;
}

}  // namespace

absl::StatusOr<int> RunAlccSelector(
    const PlannerSemanticMapManager& psmm,
    const VehicleGeometryParamsProto& vehicle_geom,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& results, int preferred_idx,
    PlcInternalResult* plc_result) {
  std::optional<int> candidate_result_index;
  bool preferred_traj_cross_solid_line = false;

  for (int i = results.size() - 1; i >= 0; --i) {
    if (!est_status[i].ok()) {
      QLOG(WARNING) << "Alcc failed task " << i << ": "
                    << est_status[i].message();
      continue;
    }

    if (!candidate_result_index.has_value()) {
      candidate_result_index = i;
    }

    const auto& est_result = results[i];
    const auto& scheduler_output = est_result.scheduler_output;

    ASSIGN_OR_CONTINUE(
        const auto has_cross_solid_line,
        HasTrajectoryCrossedSolidBoundary(
            scheduler_output.drive_passage, scheduler_output.sl_boundary,
            est_result.traj_points, vehicle_geom,
            scheduler_output.lane_change_state.stage() ==
                LaneChangeStage::LCS_PAUSE,
            psmm.IsOnVisionMap()));
    const auto has_solid_line_nearby = HasSolidLineNearBy(
        scheduler_output.drive_passage, est_result.traj_points);
    if (has_cross_solid_line || has_solid_line_nearby) {
      if (i == preferred_idx) {
        preferred_traj_cross_solid_line = true;
      }
      continue;
    }

    if (preferred_traj_cross_solid_line && plc_result != nullptr) {
      plc_result->status = PlcInternalStatus::SOLID_BOUNDARY;
      plc_result->left_solid_boundary =
          results[preferred_idx].scheduler_output.lane_change_state.lc_left();
    }
    return i;
  }

  if (candidate_result_index.has_value()) {
    return *candidate_result_index;
  }

  return absl::NotFoundError("No valid trajectory for selector");
}

}  // namespace qcraft::planner
