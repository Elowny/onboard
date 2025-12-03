#ifndef ONBOARD_PLANNER_ASSIST_ASSIST_UITL_H_
#define ONBOARD_PLANNER_ASSIST_ASSIST_UITL_H_

#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "common/proto/qalc.pb.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/assist/proto/plc_result.pb.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/remote_assist.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

// TODO(zixuan): Split assist_util

absl::Status UpdateExternalCmdStatusFromRemoteAssist(
    const RemoteAssistToCarProto& proto, ExternalCommandStatus* status);

absl::Status UpdateExternalCmdQueueFromDriverAction(
    const DriverAction& driver_action, ExternalCommandQueue* queue);

// First: target lane path from start.
// Second: target lane path with behind.
absl::StatusOr<std::pair<mapping::LanePath, mapping::LanePath>>
AlignLanePathToThisFrame(const PlannerSemanticMapManager& psmm,
                         const DrivingMapTopo& driving_map_this_frame,
                         const mapping::LanePath& prev_lane_path,
                         const PoseProto& ego_pose, double required_min_length,
                         double projection_range, double keep_behind_length);

DriverAction::LaneChangeCommand ProcessLaneChangeCommands(
    const ExternalCommandQueue& ext_cmd_queue);

absl::StatusOr<bool> CrossedBoundary(const DrivePassage& dp,
                                     const Vec2d& ego_pos);

absl::StatusOr<bool> HasTrajectoryCrossedSolidBoundary(
    const DrivePassage& drive_passage, const PathSlBoundary& sl_boundary,
    const std::vector<ApolloTrajectoryPointProto>& traj_pts,
    const VehicleGeometryParamsProto& vehicle_geom, bool lc_pause,
    bool is_vision_map);

absl::StatusOr<QALCState> UpdateAlcState(
    QALCState state, const DrivePassage& drive_passage,
    const std::vector<ApolloTrajectoryPointProto>& traj_points,
    DriverAction::LaneChangeCommand lc_cmd,
    std::optional<Vec2dProto> lane_change_target_point, double preview_duration,
    double preview_length);

std::optional<Vec2dProto> UpdateLaneChangeTargetPoint(
    QALCState state, std::optional<Vec2dProto> lane_change_target_point,
    const DrivePassage& drive_passage);

void ReportPlcEventSignal(QALCState old_state, QALCState new_state,
                          DriverAction::LaneChangeCommand lc_cmd,
                          PlcInternalStatus plc_status);

inline bool IsLaneChangeState(QALCState state) {
  switch (state) {
    case ALC_ONGOING:
    case ALC_COMPLETED:
    case ALC_CROSSING_LANE:
    case ALC_RETURNING:
    case ALC_RETURN_COMPLETED:
      return true;
    case ALC_OFF:
    case ALC_STANDBY:
    case ALC_STANDBY_ENABLE:
    case ALC_PREPARE:
      return false;
  }
}

// Return true if vehicle is controlled by autonomous driving system laterally.
inline bool IsLateralAutonomousDrivingMode(AutonomyStateProto::State state) {
  switch (state) {
    case AutonomyStateProto::AUTO_DRIVE:
    case AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE:
    case AutonomyStateProto::AUTO_STEER_ONLY:
      return true;
    case AutonomyStateProto::NOT_READY:
    case AutonomyStateProto::READY_TO_AUTO_DRIVE:
    case AutonomyStateProto::EMERGENCY_TO_TAKEOVER:
    case AutonomyStateProto::READY_TO_REMOTE_DRIVE:
    case AutonomyStateProto::REMOTE_DRIVE:
    case AutonomyStateProto::EMERGENCY_TO_STOP:
    case AutonomyStateProto::AUTO_SPEED_ONLY:
    case AutonomyStateProto::SHUTDOWN:
    case AutonomyStateProto::INIT:
    case AutonomyStateProto::SHORT_BRAKE:
    case AutonomyStateProto::READY_OVERRIDE:
      return false;
  }
}

LaneChangeStateProto CalculateLaneChangeState(const FrenetBox& ego_frenet_box,
                                              QALCState state,
                                              LaneChangeDirection lc_direction);

inline LaneChangeStage AlcStateToLaneChangeStage(QALCState alc_state) {
  switch (alc_state) {
    case ALC_OFF:
    case ALC_STANDBY:
    case ALC_STANDBY_ENABLE:
    case ALC_COMPLETED:
    case ALC_RETURN_COMPLETED:
    case ALC_PREPARE:
      return LaneChangeStage::LCS_NONE;
    case ALC_ONGOING:
    case ALC_CROSSING_LANE:
      return LaneChangeStage::LCS_EXECUTING;
    case ALC_RETURNING:
      return LaneChangeStage::LCS_RETURN;
  }
}

inline double ConvertToFollowHeadwayTime(
    QRunEvent::LccFollowingDistanceLevel level) {
  switch (level) {
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NONE:
      return 1.7;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NEAR:
      return 1.2;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR:
      return 1.4;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM:
      return 1.7;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR:
      return 2.0;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR:
      return 2.4;
  }
}

inline QRunEvent::LccFollowingDistanceLevel GetNextFollowingDistanceLevel(
    QRunEvent::LccFollowingDistanceLevel level) {
  switch (level) {
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NONE:
      return QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NEAR:
      return QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR:
      return QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM:
      return QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR:
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR:
      return QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR;
  }
}

inline QRunEvent::LccFollowingDistanceLevel GetPreviousFollowingDistanceLevel(
    QRunEvent::LccFollowingDistanceLevel level) {
  switch (level) {
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NONE:
      return QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NEAR:
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR:
      return QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NEAR;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM:
      return QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR:
      return QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM;
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR:
      return QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR;
  }
}

// Large vehicle will have +1 level follow headway time.
inline void UpdateFollowHeadwayTimeAccordHmi(
    SpeedFinderParamsProto* speed_finder_params,
    QRunEvent::LccFollowingDistanceLevel level) {
  speed_finder_params->set_follow_time_headway(
      ConvertToFollowHeadwayTime(level));
  speed_finder_params->set_large_vehicle_follow_time_headway(
      ConvertToFollowHeadwayTime(GetNextFollowingDistanceLevel(level)));
}

absl::StatusOr<mapping::OnlineSemanticMapProto> BuildOnlineMapFromPrevLanePath(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map,
    const std::vector<Vec2d>& prev_lane_path_points, const Vec2d& ego_xy);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ASSIST_ASSIST_UITL_H_
