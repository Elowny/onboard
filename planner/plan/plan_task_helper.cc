#include "onboard/planner/plan/plan_task_helper.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <utility>
#include <vector>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/global_pose.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/plan/proto/plan_task.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/post_process/post_process.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

DEFINE_bool(
    planner_force_init_uturn_task, false,
    "Force to init one uturn task to test uturn freespace planner offboard.");

DEFINE_string(planner_uturn_reference_lane_path, "",
              "Uturn reference lane path");

namespace qcraft::planner {

namespace {

ReachDestinationCondition CreateReachOffRoadRouteEndCondition() {
  ReachDestinationCondition condition;
  condition.set_speed_error(0.1);
  condition.mutable_pose_within_radius()->set_radius(1.0);
  condition.mutable_pose_within_radius()->set_heading_error(0.1);
  return condition;
}

ReachDestinationCondition CreateReachOnRoadPoseCondition() {
  ReachDestinationCondition condition;
  condition.set_speed_error(2.0);
  condition.mutable_pose_within_radius()->set_radius(3.0);
  condition.mutable_pose_within_radius()->set_heading_error(M_PI_4);
  return condition;
}

ReachDestinationCondition CreateReachOnRoadRouteEndCondition() {
  ReachDestinationCondition condition;
  condition.set_speed_error(0.05);
  condition.mutable_pose_within_radius()->set_radius(5.0);
  condition.mutable_pose_within_radius()->set_heading_error(
      std::numeric_limits<double>::infinity());
  return condition;
}

PlanTaskDestinationInfo CreateOffRoadStopTaskInfo(
    const OffRoadDestinationProto& offroad_dest) {
  PlanTaskDestination dest;
  if (offroad_dest.has_parking_spot()) {
    std::vector<mapping::ElementId> parking_spots;
    parking_spots.reserve(
        offroad_dest.parking_spot().specified_parking_spot_ids_size());
    for (const auto id :
         offroad_dest.parking_spot().specified_parking_spot_ids()) {
      parking_spots.push_back(mapping::ElementId(id));
    }
    dest.parking_spots = std::move(parking_spots);
  } else {
    QLOG(FATAL) << "Should not reached here.";
  }

  return PlanTaskDestinationInfo{
      .dest = std::move(dest),
      .end_speed = 0.0,
      .condition = CreateReachOffRoadRouteEndCondition()};
}

PlanTaskDestinationInfo CreateOffRoadStopTaskInfoFromEndStrategy(
    const RouteEndStrategyProto& end_proto) {
  QCHECK(end_proto.has_parking_spot());
  PlanTaskDestination dest;
  std::vector<mapping::ElementId> parking_spots;
  parking_spots.reserve(
      end_proto.parking_spot().specified_parking_spot_ids_size());
  for (const auto id : end_proto.parking_spot().specified_parking_spot_ids()) {
    parking_spots.push_back(mapping::ElementId(id));
  }
  dest.parking_spots = std::move(parking_spots);
  return PlanTaskDestinationInfo{
      .dest = std::move(dest),
      .end_speed = 0.0,
      .condition = CreateReachOffRoadRouteEndCondition()};
}

PlanTaskDestinationInfo CreateOffRoadDepartTaskInfo(
    const RouteDepartStategyProto& depart_proto) {
  PlanTaskDestination dest;

  QCHECK(!depart_proto.off_road().specified_onroad_points().empty());

  const auto& destination_proto =
      *depart_proto.off_road().specified_onroad_points().begin();

  if (destination_proto.has_lane_point()) {
    dest.lane_points = {mapping::LanePoint(destination_proto.lane_point())};

  } else {
    QLOG(FATAL) << "Data type not supported for now.";
  }

  return PlanTaskDestinationInfo{.dest = std::move(dest),
                                 .end_speed = 0.0,
                                 .condition = CreateReachOnRoadPoseCondition()};
}

PlanTaskDestinationInfo CreateOnRoadCruiseTaskInfo(
    const mapping::LanePoint& lane_point) {
  PlanTaskDestination dest;
  dest.lane_points = {lane_point};

  return PlanTaskDestinationInfo{
      .dest = std::move(dest),
      .end_speed = 0.0,
      .condition = CreateReachOnRoadRouteEndCondition()};
}

PlanTaskDestinationInfo CreateOnRoadCruiseBeforeUturnTaskInfo(
    const mapping::LanePoint& lane_point) {
  PlanTaskDestination dest;
  dest.lane_points = {lane_point};

  return PlanTaskDestinationInfo{
      .dest = std::move(dest),
      .end_speed = 1.0,
      .condition = CreateReachOnRoadRouteEndCondition()};
}

PlanTaskDestinationInfo CreateUturnTaskInfo(
    const PlannerSemanticMapManager& psmm, const mapping::LanePoint& goal,
    const mapping::LanePath& uturn_lane_path) {
  PlanTaskDestination dest;
  dest.lane_points = {goal};

  const Vec2d goal_pos = ComputeLanePointPos(psmm, goal);
  const Vec2d uturn_end_pos = ComputeLanePointPos(psmm, uturn_lane_path.back());

  ReachDestinationCondition condition;
  condition.set_speed_error(std::numeric_limits<double>::infinity());
  constexpr double kRadiusOffset = 5.0;  // m.
  constexpr double kMinRadius = 1.0;     // m.
  condition.mutable_pose_within_radius()->set_radius(
      std::max(goal_pos.DistanceTo(uturn_end_pos) - kRadiusOffset, kMinRadius));
  condition.mutable_pose_within_radius()->set_heading_error(M_PI_4);

  mapping::LanePathProto lp_proto;
  uturn_lane_path.ToProto(&lp_proto);
  return PlanTaskDestinationInfo{.dest = std::move(dest),
                                 .end_speed = 0.0,
                                 .condition = std::move(condition),
                                 .uturn_ref_lane_path = std::move(lp_proto)};
}

absl::StatusOr<std::pair<Vec2d, double>> ConvertDestinationToSmoothPose(
    const CoordinateConverter& cc, const PlanTaskDestination& task_dest,
    const PlannerSemanticMapManager& psmm) {
  if (task_dest.parking_spots.has_value()) {
    // const auto &parking_spot_info =
    //     psmm.semantic_map_manager()->FindParkingSpotByIdOrDie(
    //         task_dest.parking_spots->front());
    return std::make_pair(Vec2d(), 0.0);

  } else if (task_dest.lane_points.has_value()) {
    const auto& lane_point = task_dest.lane_points->front();
    const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(lane_point.lane_id());
    if (lane_info_ptr == nullptr) {
      return absl::NotFoundError("Current destination info is not loaded yet.");
    }
    return std::make_pair(ComputeLanePointPos(psmm, lane_point),
                          ComputeLanePointLerpTheta(psmm, lane_point));
  } else if (task_dest.global_pose.has_value()) {
    return std::make_pair(
        cc.GlobalToSmooth(Vec2d(task_dest.global_pose->pos.x(),
                                task_dest.global_pose->pos.y())),
        cc.GlobalYawToSmooth(task_dest.global_pose->heading));
  } else {
    QLOG(FATAL) << "Should not reach here.";
  }
}

PlanTaskDestinationInfo CreateUturnTaskFromLanePath(
    const mapping::LanePath& lane_path) {
  PlanTaskDestination dest;
  dest.lane_points = {lane_path.back()};
  mapping::LanePathProto lp_proto;
  lane_path.ToProto(&lp_proto);
  return PlanTaskDestinationInfo{.dest = std::move(dest),
                                 .end_speed = 1.0,
                                 .condition = CreateReachOnRoadPoseCondition(),
                                 .uturn_ref_lane_path = std::move(lp_proto)};
}

absl::StatusOr<std::vector<mapping::LanePath::LaneSegment>>
FindUturnSubLanePathWithinPreview(const PlannerSemanticMapManager& psmm,
                                  const mapping::LanePath& target_lane_path,
                                  double preview_distance) {
  std::vector<mapping::LanePath::LaneSegment> sub_lane_segments;
  for (const auto& lane_seg : target_lane_path) {
    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm, lane_seg.lane_id);
    if (lane_info.direction == mapping::LaneProto::UTURN) {
      if (sub_lane_segments.empty()) {
        if (lane_seg.start_s < preview_distance) {
          sub_lane_segments.push_back(lane_seg);
        } else {
          return absl::NotFoundError("beyond preview distance");
        }
      } else if (lane_seg.lane_index ==
                 sub_lane_segments.back().lane_index + 1) {
        sub_lane_segments.push_back(lane_seg);
      } else {
        break;
      }
    }
  }

  if (sub_lane_segments.empty()) {
    return absl::NotFoundError("");
  }

  return sub_lane_segments;
}

GlobalPose ConvertLanePointToGlobalPose(const PlannerSemanticMapManager& psmm,
                                        const mapping::LanePoint& lane_point,
                                        const CoordinateConverter& cc) {
  const Vec2d global_pos =
      cc.SmoothToGlobal(ComputeLanePointPos(psmm, lane_point));
  return GlobalPose{.pos = Vec3d(global_pos.x(), global_pos.y(), 0.0),
                    .heading = cc.SmoothYawToGlobalNoNormalize(
                        ComputeLanePointLerpTheta(psmm, lane_point))};
}

}  // namespace

std::deque<PlanTask> CreatePlanTasksQueueFromRoutingResult(
    const RouteManagerOutput& route_output,
    const PlannerSemanticMapManager& psmm) {
  std::deque<PlanTask> tasks;

  if (FLAGS_planner_force_init_uturn_task) {
    QCHECK(!FLAGS_planner_uturn_reference_lane_path.empty());
    mapping::LanePathProto lane_path_proto;
    file_util::StringToProto(FLAGS_planner_uturn_reference_lane_path,
                             &lane_path_proto);
    const auto lane_path_or =
        BuildLanePathFromData(mapping::LanePathData(lane_path_proto), psmm);
    QCHECK_OK(lane_path_or.status());
    tasks.emplace_back(UTURN_PLAN, CreateUturnTaskFromLanePath(*lane_path_or));
    return tasks;
  }

  const auto& stop_info = route_output.destination_stop;
  if (stop_info.has_stop_point() && stop_info.stop_point().has_off_road()) {
    // APA
    tasks.emplace_back(OFF_ROAD_PLAN, CreateOffRoadStopTaskInfo(
                                          stop_info.stop_point().off_road()));
    return tasks;
  }

  if (stop_info.has_depart_strategy()) {
    tasks.emplace_back(OFF_ROAD_PLAN, CreateOffRoadDepartTaskInfo(
                                          stop_info.depart_strategy()));
  }

  tasks.emplace_back(
      ON_ROAD_CRUISE_PLAN,
      CreateOnRoadCruiseTaskInfo(
          route_output.route_sections_from_current.destination()));

  if (stop_info.has_stop_strategy() &&
      stop_info.stop_strategy().has_parking_spot()) {
    tasks.emplace_back(OFF_ROAD_PLAN, CreateOffRoadStopTaskInfoFromEndStrategy(
                                          stop_info.stop_strategy()));
  }

  return tasks;
}

bool PlanTaskCompeleted(const PlanTask& task,
                        const AutonomyStateProto& autonomy_state,
                        const CoordinateConverter& cc,
                        const Vec2d& front_bumper_pos, double ego_heading,
                        double ego_v, const PlannerSemanticMapManager& psmm) {
  // Finish uturn task if not in auto mode.
  if (!IS_AUTO_DRIVE(autonomy_state.autonomy_state()) &&
      task.type() == UTURN_PLAN) {
    return true;
  }
  const PlanTaskDestinationInfo& dest_info = task.destination_info();

  const ReachDestinationCondition& condition =
      task.destination_info().condition;

  if (condition.has_pose_within_radius()) {
    const auto pose_or =
        ConvertDestinationToSmoothPose(cc, dest_info.dest, psmm);
    if (!pose_or.ok()) {
      return false;
    }

    if (std::abs(ego_v - dest_info.end_speed) > condition.speed_error()) {
      return false;
    }
    if (std::abs(NormalizeAngle(ego_heading - pose_or->second)) >
        condition.pose_within_radius().heading_error()) {
      return false;
    }
    if (pose_or->first.DistanceSquareTo(front_bumper_pos) >
        Sqr(condition.pose_within_radius().radius())) {
      return false;
    }
    return true;
  } else {
    QLOG(FATAL) << "Should not reached here.";
  }

  return false;
}

absl::StatusOr<std::vector<PlanTask>> SplitCruiseByUturnTask(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& prev_target_lane_path,
    const mapping::LanePoint& route_destination) {
  constexpr double kUturnPreviewDistance = 20.0;  // m.

  ASSIGN_OR_RETURN(const auto uturn_lane_segs,
                   FindUturnSubLanePathWithinPreview(
                       psmm, prev_target_lane_path, kUturnPreviewDistance));

  std::vector<mapping::ElementId> sub_lane_ids;
  sub_lane_ids.reserve(uturn_lane_segs.size());
  for (const auto& lane_seg : uturn_lane_segs) {
    sub_lane_ids.push_back(lane_seg.lane_id);
  }
  const mapping::LanePath uturn_lane_path(
      psmm.semantic_map_manager(), std::move(sub_lane_ids),
      uturn_lane_segs.front().start_fraction,
      uturn_lane_segs.back().end_fraction);

  std::vector<PlanTask> tasks;

  // Create cruise task before uturn.
  if (uturn_lane_segs.front().start_s > kMinCruiseLength) {
    tasks.emplace_back(
        ON_ROAD_CRUISE_PLAN,
        CreateOnRoadCruiseBeforeUturnTaskInfo(uturn_lane_path.front()));
  }

  // Create uturn task.

  // Find uturn task goal point.
  constexpr double kUturnGoalExtendDistance = 20.0;
  mapping::LanePoint uturn_goal = uturn_lane_path.back();
  for (int i = uturn_lane_segs.back().lane_index + 1;
       i < prev_target_lane_path.size(); ++i) {
    const auto& seg = prev_target_lane_path.lane_segment(i);

    if (seg.end_s - uturn_lane_segs.back().end_s > kUturnGoalExtendDistance) {
      const double delta_s =
          seg.end_s - uturn_lane_segs.back().end_s - kUturnGoalExtendDistance;
      SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm, seg.lane_id);
      const double lane_len = lane_info.length();
      uturn_goal = mapping::LanePoint(seg.lane_id,
                                      seg.end_fraction - delta_s / lane_len);
      break;
    }

    if (i + 1 == prev_target_lane_path.size()) {
      uturn_goal = prev_target_lane_path.back();
      break;
    }
  }

  tasks.emplace_back(UTURN_PLAN,
                     CreateUturnTaskInfo(psmm, uturn_goal, uturn_lane_path));

  // Create cruise task after uturn.
  if (uturn_goal != prev_target_lane_path.back()) {
    tasks.emplace_back(ON_ROAD_CRUISE_PLAN,
                       CreateOnRoadCruiseTaskInfo(route_destination));
  }

  return tasks;
}

absl::StatusOr<std::vector<PlanTask>> CreateUturnTask(
    const PlannerSemanticMapManager& psmm, const PoseProto& ego_pose,
    const mapping::LanePath& prev_target_lane_path,
    const mapping::LanePoint& route_destination,
    const std::optional<TrajectoryEndInfoProto>& prev_traj_end_info,
    const TrajectoryProto& prev_traj) {
  constexpr double kMaxSpeedForUturnTask = 0.02;  // m/s

  if (ego_pose.vel_body().x() > kMaxSpeedForUturnTask) {
    return absl::UnavailableError("speed too high.");
  }

  if (!prev_traj_end_info.has_value() ||
      (prev_traj_end_info->type() != StBoundarySourceTypeProto::ST_OBJECT &&
       prev_traj_end_info->type() !=
           StBoundarySourceTypeProto::IMPASSABLE_BOUNDARY &&
       prev_traj_end_info->type() !=
           StBoundarySourceTypeProto::PATH_BOUNDARY)) {
    return absl::UnavailableError(
        "previous trajectory is not blocked by stationary object or boundary.");
  }

  constexpr double kFullStopSpeed = 0.2;
  for (const auto& pt : prev_traj.trajectory_point()) {
    if (pt.v() > kFullStopSpeed) {
      return absl::UnavailableError(
          "previous trajectory is still driving forward.");
    }
  }

  constexpr double kUturnPreviewDistance = 20.0;  // m.

  ASSIGN_OR_RETURN(const auto uturn_lane_segs,
                   FindUturnSubLanePathWithinPreview(
                       psmm, prev_target_lane_path, kUturnPreviewDistance));

  if (uturn_lane_segs.front().start_s > 1e-3) {
    return absl::UnavailableError("not reached uturn.");
  }

  std::vector<mapping::ElementId> sub_lane_ids;
  sub_lane_ids.reserve(uturn_lane_segs.size());
  for (const auto& lane_seg : uturn_lane_segs) {
    sub_lane_ids.push_back(lane_seg.lane_id);
  }
  const mapping::LanePath uturn_lane_path(
      psmm.semantic_map_manager(), std::move(sub_lane_ids),
      uturn_lane_segs.front().start_fraction,
      uturn_lane_segs.back().end_fraction);

  std::vector<PlanTask> tasks;

  // Create uturn task.
  // Find uturn task goal point.
  constexpr double kUturnGoalExtendDistance = 20.0;
  mapping::LanePoint uturn_goal = uturn_lane_path.back();
  for (int i = uturn_lane_segs.back().lane_index + 1;
       i < prev_target_lane_path.size(); ++i) {
    const auto& seg = prev_target_lane_path.lane_segment(i);

    if (seg.end_s - uturn_lane_segs.back().end_s > kUturnGoalExtendDistance) {
      const double delta_s =
          seg.end_s - uturn_lane_segs.back().end_s - kUturnGoalExtendDistance;
      SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm, seg.lane_id);
      const double lane_len = lane_info.length();
      uturn_goal = mapping::LanePoint(seg.lane_id,
                                      seg.end_fraction - delta_s / lane_len);
      break;
    }

    if (i + 1 == prev_target_lane_path.size()) {
      uturn_goal = prev_target_lane_path.back();
      break;
    }
  }

  tasks.emplace_back(UTURN_PLAN,
                     CreateUturnTaskInfo(psmm, uturn_goal, uturn_lane_path));

  // Create cruise task after uturn.
  if (uturn_goal != prev_target_lane_path.back()) {
    tasks.emplace_back(ON_ROAD_CRUISE_PLAN,
                       CreateOnRoadCruiseTaskInfo(route_destination));
  }
  return tasks;
}

absl::StatusOr<PlanTask> CreateBlockedRoadTask(
    const PlannerSemanticMapManager& psmm, const PoseProto& pose,
    const VehicleGeometryParamsProto& veh_geo_params) {
  const auto front_bumper_pos =
      Vec2dFromPoseProto(pose) + veh_geo_params.front_edge_to_center() *
                                     Vec2d::FastUnitFromAngle(pose.yaw());
  const auto* lane_info = psmm.GetNearestLaneInfoWithHeadingAtLevel(
      psmm.GetLevel(), front_bumper_pos, pose.yaw(), /*radius=*/3.0, M_PI_4);

  if (lane_info == nullptr) {
    return absl::InternalError("Unable to project pose to lanes.");
  }

  const double backward_dist = 5.0 + veh_geo_params.length();
  ASSIGN_OR_RETURN(
      const auto lane_path,
      BuildLanePathFromData(
          mapping::LanePathData(
              /*start_fraction=*/0.0, /*end_fraction=*/1.0, {lane_info->id}),
          psmm));
  const auto extended_lane_path =
      BackwardExtendLanePath(psmm, lane_path, backward_dist);
  const auto points = SampleLanePathPoints(psmm, extended_lane_path);
  ASSIGN_OR_RETURN(const auto ff, BuildBruteForceFrenetFrame(
                                      points, /*down_sample_raw_points=*/true));

  const FrenetCoordinate sl = ff.XYToSL(front_bumper_pos);

  PlanTaskDestination dest;
  dest.lane_points = {
      extended_lane_path.ArclengthToLanePoint(sl.s - backward_dist)};
  ReachDestinationCondition condition;
  condition.set_speed_error(0.0);
  condition.mutable_pose_within_radius()->set_radius(veh_geo_params.length() +
                                                     0.5);
  condition.mutable_pose_within_radius()->set_heading_error(M_PI_4);

  return PlanTask(BLOCKED_PLAN,
                  PlanTaskDestinationInfo{.dest = std::move(dest),
                                          .end_speed = 0.0,
                                          .condition = std::move(condition)});
}

bool ReachedRouteEnd(const Vec2d& ego_pos, double ego_v,
                     const PlanTask& last_task, const CoordinateConverter& cc,
                     const PlannerSemanticMapManager& psmm) {
  // TODO(weijun): reach end condition should be configurable.
  const auto pose_or = ConvertDestinationToSmoothPose(
      cc, last_task.destination_info().dest, psmm);
  if (!pose_or.ok()) return false;

  constexpr double kReachEndSqrRadius = 10.0 * 10.0;
  constexpr double kSpeedThreshold = 0.8;  // m/s.
  return ego_pos.DistanceSquareTo(pose_or->first) < kReachEndSqrRadius &&
         ego_v < kSpeedThreshold;
}

bool IsHdMapBasedTask(PlanTaskType type) {
  switch (type) {
    case ON_ROAD_CRUISE_PLAN:
    case OFF_ROAD_PLAN:
    case UTURN_PLAN:
    case BLOCKED_PLAN:
      return true;

    case ALCC_PLAN:
    case ACC_PLAN:
    case MAPLESS_NOA:
    case APA_PLAN:
      return false;
  }
}

PlannerStatus ClassifyTaskErrorToPlannerStatus(PlanTaskType type,
                                               std::string_view error_msg,
                                               bool is_driverless_mode) {
  switch (type) {
    case OFF_ROAD_PLAN:
    case UTURN_PLAN:
    case BLOCKED_PLAN:
      return PlannerStatus(PlannerStatusProto::DRIVERLESS_PLAN_MAIN_LOOP_FAILED,
                           error_msg);

    case ON_ROAD_CRUISE_PLAN:
      return PlannerStatus(
          is_driverless_mode
              ? PlannerStatusProto::DRIVERLESS_PLAN_MAIN_LOOP_FAILED
              : PlannerStatusProto::NOA_MAIN_LOOP_FAILED,
          error_msg);

    case ALCC_PLAN:
      return PlannerStatus(PlannerStatusProto::ALCC_MAIN_LOOP_FAILED,
                           error_msg);
    case ACC_PLAN:
      return PlannerStatus(PlannerStatusProto::ACC_MAIN_LOOP_FAILED, error_msg);
    case MAPLESS_NOA:
      return PlannerStatus(PlannerStatusProto::MAPLESS_NOA_MAIN_LOOP_FAILED,
                           error_msg);
    case APA_PLAN:
      return PlannerStatus(PlannerStatusProto::APA_MAIN_LOOP_FAILED, error_msg);
  }
}

PreprocessPlannerObjectsOutput PreprocessPlannerObjects(
    const PlannerSemanticMapManager& psmm,
    const TrafficLightStatesProto& tl_states,
    const prediction::ConflictResolverParams& resolver_params,
    const ObjectsPredictionProto* prediction, const ObjectsProto* objects_proto,
    absl::Time plan_time, bool planner_consider_objects,
    ThreadPool* thread_pool) {
  PreprocessPlannerObjectsOutput output;
  if (!planner_consider_objects) {
    output.planner_objects = ObjectVector<PlannerObject>();
    return output;
  }

  if (prediction->post_process_in_planner()) {
    SCOPED_QTRACE("PredictionPostProcess");
    std::vector<prediction::ObjectPrediction> objects_prediction;

    const absl::Cleanup clean_objects_prediction_async = [&objects_prediction] {
      DestroyContainerAsyncMarkSource(std::move(objects_prediction),
                                      (QCRAFT_LOC).ToString());
    };

    objects_prediction.resize(prediction->objects_size());
    ParallelFor(0, prediction->objects_size(), thread_pool, [&](int i) {
      objects_prediction[i] =
          prediction::ObjectPrediction(prediction->objects(i));
    });
    const auto obj_preds_pp =
        prediction::FromObjectPredictions(absl::MakeSpan(objects_prediction));
    const auto red_tls = prediction::FindRedTrafficLights(psmm, tl_states);
    ConflictResolverDebugProto conflict_resolver_debug;
    const auto prediction_pp_state = prediction::RunPredictionPostProcess(
        psmm, red_tls, resolver_params, obj_preds_pp, &conflict_resolver_debug,
        thread_pool);
    output.prediction_post_process = std::move(conflict_resolver_debug);

    VLOG(1) << prediction_pp_state.message();
    output.planner_objects = BuildPlannerObjectsFromObjectPrediction(
        objects_proto, &objects_prediction, ToUnixDoubleSeconds(plan_time),
        thread_pool);
    return output;
  }
  output.planner_objects = BuildPlannerObjects(
      objects_proto, prediction, ToUnixDoubleSeconds(plan_time), thread_pool);
  return output;
}

absl::StatusOr<GlobalPose> CalculateGoalPoseByDestinationInfo(
    const PlannerSemanticMapManager& psmm, const CoordinateConverter& cc,
    const PlanTaskDestinationInfo& destination_info) {
  if (destination_info.dest.lane_points.has_value()) {
    return ConvertLanePointToGlobalPose(
        psmm, destination_info.dest.lane_points->front(), cc);
  }

  if (destination_info.dest.global_pose.has_value()) {
    return *destination_info.dest.global_pose;
  }

  return absl::InternalError("The destination info not be specified.");
}
}  // namespace qcraft::planner
