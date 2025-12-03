#ifndef ONBOARD_PLANNER_HMI_UTIL_H_
#define ONBOARD_PLANNER_HMI_UTIL_H_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<std::optional<NudgeOjbectInfo>> ExtractNudgeObjectId(
    int trajectory_steps, double trajectory_time_step, LaneChangeStage lc_stage,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const std::vector<TrajectoryPoint>& result_points,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params);

struct AlertedFrontVehicleInfo {
  std::string obj_id;
  double obj_v = 0.0;
  double min_s = 0.0;
};

std::optional<AlertedFrontVehicleInfo> GetAlertedFrontVehicle(
    const DrivePassage& drive_passage, const PlannerObjectManager& obj_mgr,
    const ApolloTrajectoryPointProto& start_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedFinderParamsProto& speed_finder_params);

bool GetCollisionWarningRequest(
    bool prev_collision_warning_request,
    const std::optional<AlertedFrontVehicleInfo>& target_front_vehicle,
    double ego_v, double follow_time_headway);

bool WhetherSelectCurrentBranchForHighlightVehicle(
    const DrivePassage& drive_passage, const Box2d& av_box, double heading);

template <typename T>
std::optional<int> SelectHighlightFrontVehicleBranchIndex(
    const std::vector<T>& multi_tasks, const Box2d& ego_box, double heading) {
  std::optional<int> highlight_vehicle_branch;
  for (int i = 0; i < multi_tasks.size(); ++i) {
    if (WhetherSelectCurrentBranchForHighlightVehicle(
            multi_tasks[i].drive_passage, ego_box, heading)) {
      highlight_vehicle_branch = i;
      break;
    }
  }
  return highlight_vehicle_branch;
}

inline void FillSameAlertedFrontVehicle(
    const std::optional<int>& highlight_vehicle_branch,
    std::vector<EstPlannerOutput>* results) {
  for (int i = 0; i < results->size(); ++i) {
    if (!highlight_vehicle_branch.has_value()) {
      (*results)[i].alerted_front_vehicle.reset();
    } else if (i != *highlight_vehicle_branch) {
      (*results)[i].alerted_front_vehicle =
          (*results)[*highlight_vehicle_branch].alerted_front_vehicle;
    }
  }
}

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_HMI_UTIL_H_
