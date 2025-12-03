#include "onboard/planner/assist/assist_util.h"

#include <stdint.h>
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <stddef.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <optional>
#include <ostream>
#include <queue>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/hmi/events/run_event.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/selector/selector_util.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/online_semantic_map_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/remote_assist_common.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

DEFINE_int32(teleop_expire_seconds, 3, "Instruction expiration");

namespace qcraft::planner {

namespace {

Vec2d QueryPosFromTrajectoryByT(
    const std::vector<ApolloTrajectoryPointProto>& traj_points, double t) {
  return Vec2dFromApolloTrajectoryPointProto(
      QueryApolloTrajectoryPointByT(traj_points.begin(), traj_points.end(), t));
}

Vec2d QueryPosFromTrajectoryByS(
    const std::vector<ApolloTrajectoryPointProto>& traj_points, double s) {
  return Vec2dFromApolloTrajectoryPointProto(
      QueryApolloTrajectoryPointByS(traj_points.begin(), traj_points.end(), s));
}

absl::Status ValidateLiteHeader(const LiteHeader& header,
                                const absl::Duration& duration) {
  // backward compatibility
  if (header.timestamp() == 0) return absl::OkStatus();

  const auto delay = Clock::Now() - absl::FromUnixMicros(header.timestamp());
  if (delay > duration) {
    return absl::FailedPreconditionError(absl::StrFormat(
        "Channel[%s]: header timestamp is %.3f, now is %.3f, the "
        "delay is %.3f seconds.",
        header.channel(), header.timestamp() * 1e-6,
        ToUnixDoubleSeconds(Clock::Now()), absl::ToDoubleSeconds(delay)));
  }
  return absl::OkStatus();
}

absl::Status ValidateRemoteAssistMessage(
    const RemoteAssistToCarProto& ra_to_car) {
  return ValidateLiteHeader(ra_to_car.header(),
                            absl::Seconds(FLAGS_teleop_expire_seconds));
}

absl::Status ValidateDriverActionMessage(const DriverAction& driver_action) {
  return ValidateLiteHeader(driver_action.header(),
                            absl::Seconds(FLAGS_teleop_expire_seconds));
}

absl::StatusOr<bool> LaneChangeCompleted(
    const DrivePassage& dp,
    const std::vector<ApolloTrajectoryPointProto>& traj_points,
    std::optional<Vec2dProto> lane_change_target_point,
    double preview_duration) {
  constexpr double kAngleThreshold = d2r(10.0);
  bool lc_complete = false;
  if (lane_change_target_point.has_value() && !traj_points.empty()) {
    const auto xy = Vec2dFromProto(*lane_change_target_point);
    ASSIGN_OR_RETURN(const auto target_sl,
                     dp.QueryUnboundedFrenetCoordinateAt(xy));
    ASSIGN_OR_RETURN(
        const auto ego_sl,
        dp.QueryFrenetCoordinateAt(
            Vec2dFromApolloTrajectoryPointProto(traj_points.front())));
    if (target_sl.s > ego_sl.s) {
      return false;
    }
  }

  for (const auto& traj_point : traj_points) {
    if (traj_point.relative_time() > preview_duration) break;

    const double traj_heading = traj_point.path_point().theta();
    const auto xy = Vec2dFromApolloTrajectoryPointProto(traj_point);

    ASSIGN_OR_RETURN(const auto sl, dp.QueryFrenetCoordinateAt(xy));
    ASSIGN_OR_RETURN(const auto angle, dp.QueryTangentAngleAtS(sl.s));

    lc_complete =
        std::abs(sl.l) < kMaxLaneKeepLateralOffset &&
        std::abs(NormalizeAngle(angle - traj_heading)) < kAngleThreshold;

    if (!lc_complete) return lc_complete;
  }

  return lc_complete;
}

absl::StatusOr<mapping::LanePath> FindMostSimilarLanePath(
    const DrivingMapTopo& driving_map, const mapping::LanePath& lane_path,
    const PlannerSemanticMapManager& psmm) {
  const auto* start_lane = driving_map.GetLaneById(lane_path.front().lane_id());

  if (start_lane == nullptr) {
    return absl::NotFoundError(absl::StrCat(
        "start lane ", lane_path.front().lane_id(), " not found."));
  }

  std::vector<const DrivingMapTopo::Lane*> new_lanes = {start_lane};

  for (int origin_idx = 1;; ++origin_idx) {
    if (new_lanes.back()->outgoing_lane_ids.empty()) {
      break;
    }

    const mapping::ElementId next_id = origin_idx < lane_path.size()
                                           ? lane_path.lane_id(origin_idx)
                                           : mapping::kInvalidElementId;

    const auto it =
        std::find(new_lanes.back()->outgoing_lane_ids.begin(),
                  new_lanes.back()->outgoing_lane_ids.end(), next_id);

    // TODO(weijun): ForwardExtendLanePathWithMinimumHeadingDiff
    const auto new_next_id = it == new_lanes.back()->outgoing_lane_ids.end()
                                 ? new_lanes.back()->outgoing_lane_ids.front()
                                 : *it;

    const auto* new_next_lane = driving_map.GetLaneById(new_next_id);
    if (new_next_lane == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("lane ", new_next_id, " not found."));
    }

    new_lanes.push_back(new_next_lane);
  }

  std::vector<mapping::ElementId> lane_ids;
  lane_ids.reserve(new_lanes.size());
  for (const auto* lane : new_lanes) {
    lane_ids.push_back(lane->id);
  }

  return BuildLanePathFromData(
      mapping::LanePathData(new_lanes.front()->start_fraction,
                            new_lanes.back()->end_fraction,
                            std::move(lane_ids)),
      psmm);
}

double GetLatToleranceError(double dist, bool is_vision_map) {
  constexpr double kMaxLatToleranceErrorForVisionMap = 0.5;  // m.
  constexpr double kDistErrorRateForVisionMap = 0.01;
  constexpr double kMaxLatToleranceError = 0.2;  // m.
  constexpr double kDistErrorRate = 0.005;
  if (is_vision_map) {
    return std::min(kMaxLatToleranceErrorForVisionMap,
                    std::fabs(dist) * kDistErrorRateForVisionMap);
  }
  return std::min(kMaxLatToleranceError, std::fabs(dist) * kDistErrorRate);
}

mapping::ElementId FindMaxElementIdFromOnlineMap(
    const mapping::OnlineSemanticMapProto& online_map) {
  int64_t max_id = 1;
  for (const auto& lane : online_map.lanes()) {
    max_id = std::max(max_id, lane.id());
  }
  for (const auto& bound : online_map.boundaries()) {
    max_id = std::max(max_id, bound.id());
  }
  return mapping::ElementId(max_id);
}

// Return traj size if no such point found.
int FindLcCompleteTrajPointIndex(
    const DrivePassage& drive_passage,
    const std::vector<ApolloTrajectoryPointProto>& traj_pts, int start_idx) {
  const int n = traj_pts.size();
  for (int i = start_idx; i < n; ++i) {
    ASSIGN_OR_RETURN(const auto sl,
                     drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                         Vec2dFromApolloTrajectoryPointProto(traj_pts[i])),
                     n);
    if (std::fabs(sl.l) <= kMaxLaneKeepLateralOffset) return i;
  }

  return n;
}

}  // namespace

LaneChangeStateProto CalculateLaneChangeState(
    const FrenetBox& ego_frenet_box, QALCState state,
    LaneChangeDirection lc_direction) {
  LaneChangeStateProto proto;
  switch (state) {
    case ALC_OFF:
    case ALC_STANDBY:
    case ALC_STANDBY_ENABLE:
    case ALC_COMPLETED:
    case ALC_RETURN_COMPLETED:
    case ALC_PREPARE: {
      proto.set_stage(LaneChangeStage::LCS_NONE);
      return proto;
    }
    case ALC_ONGOING:
    case ALC_CROSSING_LANE: {
      proto.set_stage(LaneChangeStage::LCS_EXECUTING);
      proto.set_lc_left(lc_direction == LaneChangeDirection::LCD_LEFT);
      proto.set_entered_target_lane(proto.lc_left()
                                        ? ego_frenet_box.l_max > 0.0
                                        : ego_frenet_box.l_min < 0.0);
      return proto;
    }
    case ALC_RETURNING: {
      proto.set_stage(LaneChangeStage::LCS_RETURN);
      proto.set_lc_left(lc_direction == LaneChangeDirection::LCD_LEFT);
      proto.set_entered_target_lane(proto.lc_left()
                                        ? ego_frenet_box.l_max > 0.0
                                        : ego_frenet_box.l_min < 0.0);
      return proto;
    }
  }
}

std::optional<Vec2dProto> UpdateLaneChangeTargetPoint(
    QALCState state, std::optional<Vec2dProto> lane_change_target_point,
    const DrivePassage& drive_passage) {
  switch (state) {
    case QALCState::ALC_OFF:
    case QALCState::ALC_STANDBY:
    case QALCState::ALC_STANDBY_ENABLE:
    case QALCState::ALC_PREPARE:
    case QALCState::ALC_COMPLETED:
    case QALCState::ALC_RETURN_COMPLETED:
      return std::nullopt;
    case QALCState::ALC_RETURNING:
    case QALCState::ALC_CROSSING_LANE:
      return lane_change_target_point;
    case QALCState::ALC_ONGOING: {
      if (lane_change_target_point.has_value()) {
        return lane_change_target_point;
      } else {
        ASSIGN_OR_RETURN(const auto point,
                         drive_passage.QueryPointXYAtS(kAlccPlcPreviewDistance),
                         std::nullopt);
        Vec2dProto target_point;
        Vec2dToProto(point, &target_point);
        return target_point;
      }
    }
  }
}

absl::Status UpdateExternalCmdStatusFromRemoteAssist(
    const RemoteAssistToCarProto& proto, ExternalCommandStatus* status) {
  RETURN_IF_ERROR(ValidateRemoteAssistMessage(proto));

  switch (proto.request_case()) {
    case RemoteAssistToCarProto::kLeftBlinkerOverride:
      status->override_left_blinker_on =
          proto.left_blinker_override().has_on() &&
          proto.left_blinker_override().on();
      break;

    case RemoteAssistToCarProto::kRightBlinkerOverride:
      status->override_right_blinker_on =
          proto.right_blinker_override().has_on() &&
          proto.right_blinker_override().on();
      break;

    case RemoteAssistToCarProto::kEmergencyBlinkerOverride:
      status->override_emergency_blinker_on =
          proto.emergency_blinker_override().has_on() &&
          proto.emergency_blinker_override().on();
      break;

    case RemoteAssistToCarProto::kDoorOverride: {
      status->override_door_open = proto.door_override().open();
      break;
    }
    case RemoteAssistToCarProto::kLaneChangeStyle: {
      status->lane_change_style = proto.lane_change_style();
      break;
    }
    case RemoteAssistToCarProto::kEnableFeatureOverride: {
      const auto& enable_feature_override = proto.enable_feature_override();
      if (enable_feature_override.has_enable_traffic_light_stopping()) {
        status->enable_traffic_light_stopping =
            enable_feature_override.enable_traffic_light_stopping();
      }
      if (enable_feature_override.has_enable_lc_objects()) {
        status->enable_lc_objects = enable_feature_override.enable_lc_objects();
      }
      if (enable_feature_override.has_enable_pull_over()) {
        status->enable_pull_over = enable_feature_override.enable_pull_over();
      }
      break;
    }
    case RemoteAssistToCarProto::kStopVehicle:
      if (proto.stop_vehicle().has_brake()) {
        status->brake_to_stop =
            proto.stop_vehicle().brake() > 0.0
                ? std::optional<double>(proto.stop_vehicle().brake())
                : std::nullopt;
      }
      break;
    case RemoteAssistToCarProto::kEnableStopPolylineStoppingOverride:
      status->enable_stop_polyline_stopping =
          proto.enable_stop_polyline_stopping_override();
      break;
    case RemoteAssistToCarProto::kDrivingActionRequest:
    case RemoteAssistToCarProto::kHeartbeat:
    case RemoteAssistToCarProto::kDrivableAgentUpdateRequest:
    case RemoteAssistToCarProto::kUseManualControlCmd:
    case RemoteAssistToCarProto::kPlayAudioRequest:
    case RemoteAssistToCarProto::kAebRequest:
    case RemoteAssistToCarProto::REQUEST_NOT_SET:
      break;
  }

  return absl::OkStatus();
}

absl::Status UpdateExternalCmdQueueFromDriverAction(
    const DriverAction& driver_action, ExternalCommandQueue* queue) {
  RETURN_IF_ERROR(ValidateDriverActionMessage(driver_action));

  queue->pending_driver_actions.push_back(driver_action);
  return absl::OkStatus();
}

// First: target lane path from start.
// Second: target lane path with behind.
absl::StatusOr<std::pair<mapping::LanePath, mapping::LanePath>>
AlignLanePathToThisFrame(const PlannerSemanticMapManager& psmm,
                         const DrivingMapTopo& driving_map_this_frame,
                         const mapping::LanePath& prev_lane_path,
                         const PoseProto& ego_pose,
                         double /*required_min_length*/,
                         double projection_range, double keep_behind_length) {
  SCOPED_QTRACE("UpdateTargetLanePath");

  ASSIGN_OR_RETURN(
      const auto lane_path_in_dm,
      FindMostSimilarLanePath(driving_map_this_frame, prev_lane_path, psmm));

  // Project ego pose to prev_lane_path.
  const auto project_lane_path =
      lane_path_in_dm.BeforeArclength(projection_range);
  ASSIGN_OR_RETURN(
      const auto ff,
      BuildBruteForceFrenetFrame(SampleLanePathPoints(psmm, project_lane_path),
                                 /*down_sample_raw_points=*/true));
  const FrenetCoordinate project_sl =
      ff.XYToSL(Vec2d(ego_pose.pos_smooth().x(), ego_pose.pos_smooth().y()));

  /*
  // Extend prev_lane_path to make sure the length of lane path in front of ego
  // pose is at least required_min_length.
  const double forward_prev_lane_path_length =
      prev_lane_path.length() - project_sl.s;
  const auto extend_lane_path = ForwardExtendLanePathWithMinimumHeadingDiff(
      psmm, prev_lane_path,
      required_min_length - forward_prev_lane_path_length);
*/
  // Calculate target lane path from start.
  auto target_lane_path_from_start =
      lane_path_in_dm.AfterArclength(project_sl.s);

  // Calculate target lane path with behind.
  auto target_lane_path_with_behind =
      project_sl.s > keep_behind_length
          ? lane_path_in_dm.AfterArclength(project_sl.s - keep_behind_length)
          : lane_path_in_dm;

  return std::pair(std::move(target_lane_path_from_start),
                   std::move(target_lane_path_with_behind));
}

DriverAction::LaneChangeCommand ProcessLaneChangeCommands(
    const ExternalCommandQueue& ext_cmd_queue) {
  if (!ext_cmd_queue.pending_lane_change_requests.empty()) {
    switch (ext_cmd_queue.pending_lane_change_requests.back().direction()) {
      case LaneChangeRequestProto::LEFT:
        return DriverAction::LC_CMD_LEFT;
      case LaneChangeRequestProto::RIGHT:
        return DriverAction::LC_CMD_RIGHT;
      case LaneChangeRequestProto::CANCEL:
        return DriverAction::LC_CMD_CANCEL;
      case LaneChangeRequestProto::STRAIGHT:
        return DriverAction::LC_CMD_STRAIGHT;
    }
  }

  // NOTE(weijun): we may lose some key commands.
  for (const auto& action : ext_cmd_queue.pending_driver_actions) {
    if (action.has_lane_change_command() &&
        action.lane_change_command() == DriverAction::LC_CMD_CANCEL) {
      return DriverAction::LC_CMD_CANCEL;
    }
  }
  for (auto it = ext_cmd_queue.pending_driver_actions.rbegin();
       it != ext_cmd_queue.pending_driver_actions.rend(); ++it) {
    if (it->has_lane_change_command() &&
        (it->lane_change_command() == DriverAction::LC_CMD_LEFT ||
         it->lane_change_command() == DriverAction::LC_CMD_RIGHT)) {
      return it->lane_change_command();
    }
  }

  return DriverAction::LC_CMD_NONE;
}

absl::StatusOr<bool> CrossedBoundary(const DrivePassage& dp,
                                     const Vec2d& ego_pos) {
  ASSIGN_OR_RETURN(const auto sl, dp.QueryFrenetCoordinateAt(ego_pos));

  constexpr double kMaxHalfWidth = 2.5;  // m.
  if (std::abs(sl.l) > kMaxHalfWidth) return false;

  constexpr double kMinHalfWidth = 1.0;  // m.
  if (std::abs(sl.l) < kMinHalfWidth) return true;

  ASSIGN_OR_RETURN(const auto l_pair,
                   dp.QueryNearestBoundaryLateralOffset(sl.s));

  return sl.l > l_pair.first && sl.l < l_pair.second;
}

absl::StatusOr<bool> HasTrajectoryCrossedSolidBoundary(
    const DrivePassage& drive_passage, const PathSlBoundary& sl_boundary,
    const std::vector<ApolloTrajectoryPointProto>& traj_pts,
    const VehicleGeometryParamsProto& vehicle_geom, bool lc_pause,
    bool is_vision_map) {
  constexpr int kCheckEveryNPt = 5;
  constexpr double kTrajectorySExtension = 10.0;  // m.

  const int min_check_n =
      std::min<int>(CeilToInt(0.3 * traj_pts.size()) + 1, traj_pts.size());
  const int check_first_n =
      lc_pause ? min_check_n
               : (is_vision_map ? FindLcCompleteTrajPointIndex(
                                      drive_passage, traj_pts, min_check_n)
                                : traj_pts.size());

  const auto last_pt_in_rear_center =
      Vec2dFromApolloTrajectoryPointProto(traj_pts[check_first_n - 1]);
  const auto heading = traj_pts[check_first_n - 1].path_point().theta();
  const auto ego_width = vehicle_geom.width();
  const auto ego_length = vehicle_geom.length();
  const auto last_pt_in_front_center =
      last_pt_in_rear_center +
      Vec2d::FastUnitFromAngle(heading) * vehicle_geom.front_edge_to_center();
  ASSIGN_OR_RETURN(const auto last_pt_in_front_center_frenet,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       last_pt_in_front_center),
                   _ << "Last considered traj point not on drive passage.");
  ASSIGN_OR_RETURN(const auto first_point_sl,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       Vec2dFromApolloTrajectoryPointProto(traj_pts.front())),
                   _ << "First traj point not on drive passage.");

  const auto solid_boundaries = FindSolidBoundaryIntervals(
      drive_passage, first_point_sl,
      last_pt_in_front_center_frenet.s + kTrajectorySExtension);

  // Generate ego car trajectory box.
  std::vector<Box2d> ego_boxes;
  ego_boxes.reserve(
      CeilToInt(check_first_n / static_cast<float>(kCheckEveryNPt)));
  double dist_to_start = 0.0;
  for (int i = 0; i < check_first_n; i += kCheckEveryNPt) {
    const auto& traj_pt = traj_pts[i];
    const auto traj_pt_vec = Vec2dFromApolloTrajectoryPointProto(traj_pt);
    double lat_tolerance_error = 0.0;
    if (i >= kCheckEveryNPt) {
      // Simply consider traj length as longitudinal length.
      dist_to_start += traj_pt_vec.DistanceTo(
          Vec2dFromApolloTrajectoryPointProto(traj_pts[i - kCheckEveryNPt]));
      lat_tolerance_error = GetLatToleranceError(dist_to_start, is_vision_map);
    }
    ego_boxes.emplace_back(traj_pt_vec, traj_pt.path_point().theta(),
                           ego_length, ego_width - lat_tolerance_error);
  }

  // For low speed condition before a stop line.
  const Segment2d last_pt_to_ref_center_seg(
      last_pt_in_front_center,
      sl_boundary.QueryReferenceCenterXY(last_pt_in_front_center_frenet.s));
  ego_boxes.emplace_back(last_pt_to_ref_center_seg, ego_width);

  for (const auto& boundary : solid_boundaries) {
    const auto& boundary_pts = boundary.points;
    for (const auto& ego_box : ego_boxes) {
      for (int j = 1; j < boundary_pts.size(); ++j) {
        const Segment2d boundary_seg(boundary_pts[j - 1], boundary_pts[j]);
        if (ego_box.HasOverlap(boundary_seg)) {
          return true;
        }
      }
    }
  }
  return false;
}

absl::StatusOr<QALCState> UpdateAlcState(
    QALCState state, const DrivePassage& drive_passage,
    const std::vector<ApolloTrajectoryPointProto>& traj_points,
    DriverAction::LaneChangeCommand lc_cmd,
    std::optional<Vec2dProto> lane_change_target_point, double preview_duration,
    double preview_length) {
  switch (state) {
    case QALCState::ALC_OFF:
    case QALCState::ALC_STANDBY:
    case QALCState::ALC_STANDBY_ENABLE:
    case QALCState::ALC_PREPARE:
      return state;

    case QALCState::ALC_COMPLETED:
      return QALCState::ALC_STANDBY_ENABLE;

    case QALCState::ALC_RETURN_COMPLETED: {
      const bool is_lc = lc_cmd == DriverAction::LC_CMD_LEFT ||
                         lc_cmd == DriverAction::LC_CMD_RIGHT;
      return is_lc ? QALCState::ALC_PREPARE : QALCState::ALC_STANDBY_ENABLE;
    }

    case QALCState::ALC_CROSSING_LANE: {
      ASSIGN_OR_RETURN(const bool comp,
                       LaneChangeCompleted(drive_passage, traj_points,
                                           std::move(lane_change_target_point),
                                           preview_duration));
      return comp ? QALCState::ALC_COMPLETED : QALCState::ALC_CROSSING_LANE;
    }

    case QALCState::ALC_RETURNING: {
      ASSIGN_OR_RETURN(
          const bool comp,
          LaneChangeCompleted(drive_passage, traj_points,
                              /*lane_change_target_point=*/std::nullopt,
                              preview_duration));
      return comp ? QALCState::ALC_RETURN_COMPLETED : QALCState::ALC_RETURNING;
    }
    case QALCState::ALC_ONGOING: {
      const auto preview_t_pos =
          QueryPosFromTrajectoryByT(traj_points, preview_duration);
      ASSIGN_OR_RETURN(const bool preview_t_cross,
                       CrossedBoundary(drive_passage, preview_t_pos));

      const auto preview_s_pos =
          QueryPosFromTrajectoryByS(traj_points, preview_length);
      ASSIGN_OR_RETURN(const bool preview_s_cross,
                       CrossedBoundary(drive_passage, preview_s_pos));

      return (preview_t_cross || preview_s_cross) ? QALCState::ALC_CROSSING_LANE
                                                  : QALCState::ALC_ONGOING;
    }
  }
}

void ReportPlcEventSignal(QALCState old_state, QALCState new_state,
                          DriverAction::LaneChangeCommand lc_cmd,
                          PlcInternalStatus plc_status) {
  switch (new_state) {
    case ALC_OFF:
    case ALC_STANDBY:
    case ALC_STANDBY_ENABLE:
    case ALC_CROSSING_LANE:
      return;
    case ALC_PREPARE:
      if (old_state != ALC_PREPARE) {
        const bool has_solid_bound =
            plc_status == PlcInternalStatus::SOLID_BOUNDARY;
        if (lc_cmd == DriverAction::LC_CMD_LEFT) {
          QLOG(INFO) << "Start waiting to lane change to left.";
          const auto event_type =
              has_solid_bound ? QRunEvent::PLC_START_WAITING_SOLID_LINE_LEFT
                              : QRunEvent::PLC_START_WAITING_LEFT;
          QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                     event_type);
        } else {
          QLOG(INFO) << "Start waiting to lane change to right.";
          const auto event_type =
              has_solid_bound ? QRunEvent::PLC_START_WAITING_SOLID_LINE_RIGHT
                              : QRunEvent::PLC_START_WAITING_RIGHT;
          QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                     event_type);
        }
      }
      return;
    case ALC_ONGOING:
      if (old_state != ALC_ONGOING) {
        if (lc_cmd == DriverAction::LC_CMD_LEFT) {
          QLOG(INFO) << "Start lane changing to left.";
          QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                     QRunEvent::PLC_START_LANE_CHANGE_LEFT);
        } else {
          QLOG(INFO) << "Start lane changing to right.";
          QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                     QRunEvent::PLC_START_LANE_CHANGE_RIGHT);
        }
      }
      return;
    case ALC_RETURNING:
      if (old_state != ALC_RETURNING) {
        if (lc_cmd == DriverAction::LC_CMD_LEFT) {
          QLOG(INFO) << "Start returning to left.";
          QRUNEVENT_WITH_ENUM_NOTICE(
              QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
              QRunEvent::PLC_START_LANE_CHANGE_RETURN_LEFT);
        } else {
          QLOG(INFO) << "Start returning to right.";
          QRUNEVENT_WITH_ENUM_NOTICE(
              QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
              QRunEvent::PLC_START_LANE_CHANGE_RETURN_RIGHT);
        }
      }
      return;
    case ALC_COMPLETED:
      QLOG(INFO) << "Completed lane change.";
      QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                 QRunEvent::PLC_COMPLETE_LANE_CHANGE);
      return;
    case ALC_RETURN_COMPLETED:
      QLOG(INFO) << "Completed lane change return.";
      QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                 QRunEvent::PLC_COMPLETE_LANE_CHANGE_RETURN);
      return;
  }
}

absl::StatusOr<mapping::OnlineSemanticMapProto> BuildOnlineMapFromPrevLanePath(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map,
    const std::vector<Vec2d>& prev_lane_path_points, const Vec2d& ego_xy) {
  FUNC_QTRACE();

  ASSIGN_OR_RETURN(
      const auto prev_frenet_frame,
      BuildKdTreeFrenetFrame(prev_lane_path_points,
                             /*down_sample_raw_points=*/true),
      _.SetPrepend() << "Failed to build frenet frame using prev lane path: ");

  ASSIGN_OR_RETURN(const auto candidate_lanes,
                   FindCloseOnlineLaneIds(psmm, online_map, prev_frenet_frame,
                                          /*valid_lane_length=*/10.0,
                                          /*max_check_length=*/60.0,
                                          /*max_lat_offset_thres=*/2.0,
                                          /*avg_lat_offset_thres=*/1.5),
                   _.SetPrepend() << "No candidate lane available: ");

  const auto& s_vec = prev_frenet_frame.s_knots();
  const double ego_s = prev_frenet_frame.XYToSL(ego_xy).s;

  auto max_id = FindMaxElementIdFromOnlineMap(online_map);

  absl::flat_hash_set<mapping::ElementId> candidate_lane_set;
  candidate_lane_set.insert(candidate_lanes.begin(), candidate_lanes.end());

  struct ModifiedLaneInfo {
    mapping::ElementId id = mapping::kInvalidElementId;
    mapping::ElementId outgoing_id = mapping::kInvalidElementId;
    std::vector<Vec2d> points;
  };

  std::vector<ModifiedLaneInfo> modified_lanes;
  for (const auto id : candidate_lanes) {
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, id);

    const auto& points = lane_info.points_smooth;
    const double start_s = prev_frenet_frame.XYToSL(points.front()).s;
    const double end_s = prev_frenet_frame.XYToSL(points.back()).s;

    if (end_s - ego_s < 10.0) {
      continue;
    }

    bool has_valid_incoming = false;
    for (const auto incoming_id : lane_info.incoming_lanes()) {
      if (candidate_lane_set.contains(incoming_id)) {
        has_valid_incoming = true;
        break;
      }
    }

    if (has_valid_incoming) continue;

    const auto it = std::lower_bound(s_vec.begin(), s_vec.end(), start_s - 1.0);
    const int idx = std::distance(s_vec.begin(), it);
    if (idx == 0) continue;

    const int raw_idx = prev_frenet_frame.raw_indices()[idx - 1];

    const auto& prev_end_pt = prev_lane_path_points[raw_idx];
    const auto& next_pt = points.front();
    const double connect_dist = prev_end_pt.DistanceTo(next_pt);

    constexpr double kSampleDist = 1.0;  // m.
    const int new_lane_size =
        raw_idx + CeilToInt(connect_dist / kSampleDist) + 2;
    std::vector<Vec2d> new_points;
    new_points.reserve(new_lane_size);
    new_points.insert(new_points.end(), prev_lane_path_points.begin(),
                      prev_lane_path_points.begin() + raw_idx + 1);

    for (double sample_s = kSampleDist; sample_s + kSampleDist <= connect_dist;
         sample_s += kSampleDist) {
      new_points.push_back(Lerp(prev_end_pt, next_pt, sample_s / connect_dist));
    }
    new_points.push_back(next_pt);

    if (HasSmoothPointsCrossedBoundary(psmm, new_points, /*ignore_range=*/{},
                                       /*solid_only=*/true)) {
      continue;
    }

    modified_lanes.emplace_back(ModifiedLaneInfo{
        .id = ++max_id, .outgoing_id = id, .points = std::move(new_points)});
  }

  if (modified_lanes.empty()) {
    return absl::InternalError("No online map lane needs modification.");
  }

  auto new_online_map = online_map;
  for (const auto& new_lane : modified_lanes) {
    for (auto& lane : *new_online_map.mutable_lanes()) {
      if (lane.id() != new_lane.outgoing_id.value()) continue;
      lane.add_incoming_lane_ids(new_lane.id.value());
    }

    auto* lane_proto = new_online_map.mutable_lanes()->Add();
    lane_proto->set_id(new_lane.id.value());
    lane_proto->add_outgoing_lane_ids(new_lane.outgoing_id.value());

    auto* points_proto = lane_proto->mutable_smooth_points();
    points_proto->Reserve(new_lane.points.size());
    for (const auto& pt : new_lane.points) {
      auto* pt_proto = points_proto->Add();
      pt_proto->set_x(pt.x());
      pt_proto->set_y(pt.y());
    }
  }

  return new_online_map;
}

}  // namespace qcraft::planner
