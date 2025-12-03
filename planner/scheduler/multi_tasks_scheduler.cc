#include "onboard/planner/scheduler/multi_tasks_scheduler.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <limits>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

constexpr double kEpsilon = 0.1;             // m.
constexpr double kForceMergeMaxSpeed = 2.0;  // m/s.

absl::StatusOr<LanePathInfo> FindNeighbor(
    const RouteSectionsInfo& route_sections_info,
    const std::vector<LanePathInfo>& lp_infos, bool lc_left,
    mapping::ElementId start_id) {
  const int cur_idx =
      FindOrDie(route_sections_info.front().id_idx_map, start_id);
  const int neighbor_idx = cur_idx + (lc_left ? 1 : -1);
  const auto& lane_ids = route_sections_info.front().lane_ids;
  if (neighbor_idx < 0 || neighbor_idx >= lane_ids.size()) {
    return absl::NotFoundError("Already leftmost/rightmost.");
  }
  const auto neighbor_id = lane_ids.at(neighbor_idx);

  for (const auto& lp_info : lp_infos) {
    if (lp_info.start_lane_id() == neighbor_id) return lp_info;
  }
  return absl::NotFoundError("Neighbor lane not viable.");
}

absl::StatusOr<SchedulerOutput> MakeLcPauseSchedulerOutput(
    const PlannerSemanticMapManager& psmm, SchedulerOutput scheduler_output,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SmoothedReferenceLineResultMap& smooth_result_map) {
  scheduler_output.lane_change_state.set_stage(LaneChangeStage::LCS_PAUSE);

  ASSIGN_OR_RETURN(
      scheduler_output.sl_boundary,
      BuildPathBoundaryFromPose(
          psmm, scheduler_output.drive_passage, plan_start_point, vehicle_geom,
          st_traj_mgr, scheduler_output.lane_change_state, smooth_result_map,
          /*borrow_lane_boundary=*/false, scheduler_output.should_smooth,
          /*unsafe_object_ids=*/{}),
      _ << "Fail to build path boundary.");

  return std::move(scheduler_output);
}

std::pair<int, int> CalculateLeftAndRightLcNum(
    const mapping::ElementId& lane_id, const PlannerSemanticMapManager& psmm,
    const RouteNaviInfo& route_navi_info) {
  int left_lc_num = std::numeric_limits<int>::max();
  int right_lc_num = std::numeric_limits<int>::max();
  SMM_ASSIGN_LANE_OR_RETURN(lane_info, psmm, lane_id,
                            std::make_pair(left_lc_num, right_lc_num));

  if (!lane_info.lane_neighbors_on_left.empty()) {
    const auto* left_lane_navi_info_ptr =
        FindOrNull(route_navi_info.route_lane_info_map,
                   lane_info.lane_neighbors_on_left.front().other_id);
    if (left_lane_navi_info_ptr != nullptr) {
      left_lc_num = left_lane_navi_info_ptr->min_lc_num_to_target;
    }
  }
  if (!lane_info.lane_neighbors_on_right.empty()) {
    const auto* right_lane_navi_info_ptr =
        FindOrNull(route_navi_info.route_lane_info_map,
                   lane_info.lane_neighbors_on_right.front().other_id);
    if (right_lane_navi_info_ptr != nullptr) {
      right_lc_num = right_lane_navi_info_ptr->min_lc_num_to_target;
    }
  }
  return std::make_pair(left_lc_num, right_lc_num);
}

bool CheckNeedSwitchRoute(const PlannerSemanticMapManager& psmm,
                          const LaneChangeStateProto& lane_change_state,
                          const RouteNaviInfo& route_navi_info,
                          const DrivePassage& drive_passage) {
  const auto lane_id = drive_passage.lane_path().front().lane_id();
  const auto* lane_navi_info_ptr =
      FindOrNull(route_navi_info.route_lane_info_map, lane_id);
  if (lane_navi_info_ptr == nullptr) return false;
  if (lane_navi_info_ptr->min_lc_num_to_target == 0 ||
      lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING ||
      lane_change_state.stage() == LaneChangeStage::LCS_RETURN) {
    return false;
  }
  constexpr double kMinLengthToSwitchAlterRoute = 2.0;  // m.

  if (lane_navi_info_ptr->max_reach_length > kMinLengthToSwitchAlterRoute) {
    return false;
  }

  const auto boundaries = drive_passage.QueryEnclosingLaneBoundariesAtS(0.0);
  const auto [left_lc_num, right_lc_num] =
      CalculateLeftAndRightLcNum(lane_id, psmm, route_navi_info);
  if (left_lc_num < right_lc_num) {
    return boundaries.left.has_value() && boundaries.left->IsSolid(0.0);
  } else if (right_lc_num < left_lc_num) {
    return boundaries.right.has_value() && boundaries.right->IsSolid(0.0);
  }
  return false;
}

std::pair<TurnSignal, TurnSignalReason> DecideRoutePrepareLcTurnSignal(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const RouteNaviInfo& route_navi_info) {
  TurnSignal planner_turn_signal = TURN_SIGNAL_NONE;
  TurnSignalReason turn_signal_reason = TURN_SIGNAL_OFF;
  constexpr double kTurnOnSignalPreviewDist = 300.0;  // m.
  const auto lane_id = drive_passage.lane_path().front().lane_id();
  const auto* lane_navi_info_ptr =
      FindOrNull(route_navi_info.route_lane_info_map, lane_id);
  if (lane_navi_info_ptr == nullptr ||
      lane_navi_info_ptr->min_lc_num_to_target == 0 ||
      lane_navi_info_ptr->max_reach_length > kTurnOnSignalPreviewDist) {
    return std::make_pair(planner_turn_signal, turn_signal_reason);
  }
  const auto [left_lc_num, right_lc_num] =
      CalculateLeftAndRightLcNum(lane_id, psmm, route_navi_info);
  const auto curr_lc_num = lane_navi_info_ptr->min_lc_num_to_target;
  planner_turn_signal =
      curr_lc_num > left_lc_num
          ? TURN_SIGNAL_LEFT
          : (curr_lc_num > right_lc_num ? TURN_SIGNAL_RIGHT : TURN_SIGNAL_NONE);
  turn_signal_reason = planner_turn_signal == TURN_SIGNAL_NONE
                           ? TURN_SIGNAL_OFF
                           : PREPARE_LANE_CHANGE_TURN_SIGNAL;
  return std::make_pair(planner_turn_signal, turn_signal_reason);
}

absl::StatusOr<bool> CheckNeedHelpToRouteChange(
    const DrivePassage& drive_passage, const RouteNaviInfo& route_navi_info,
    bool planner_is_l4_mode) {
  const auto start_lane_id = drive_passage.lane_path().front().lane_id();
  const auto* lane_navi_info_ptr =
      FindOrNull(route_navi_info.route_lane_info_map, start_lane_id);
  if (lane_navi_info_ptr == nullptr) {
    return absl::NotFoundError(absl::StrFormat(
        "Can not find lane %d in route_navi_info", start_lane_id));
  }
  if (lane_navi_info_ptr->min_lc_num_to_target == 0) {
    return false;
  }

  constexpr double kMinLcLengthBuffer = 10.0;  // m.
  if (planner_is_l4_mode) {
    return lane_navi_info_ptr->max_reach_length <= kMinLcLengthBuffer;
  } else {
    constexpr double kExtraLengthBuffer = 20.0;  // m.
    return lane_navi_info_ptr->max_reach_length <=
           (kMinLcLengthBuffer +
            (lane_navi_info_ptr->min_lc_num_to_target - 1) *
                kExtraLengthBuffer);
  }
}

bool IsLaneBlockedByObs(const DrivePassage& passage, const FrenetBox& box,
                        const double lat_buffer) {
  const auto [right_boundary, left_boundary] =
      passage.QueryEnclosingLaneBoundariesAtS(box.center_s());
  const double boundary_right_l =
      right_boundary.has_value()
          ? std::max(right_boundary->lat_offset, -kMaxHalfLaneWidth)
          : -kMaxHalfLaneWidth;
  const double boundary_left_l =
      left_boundary.has_value()
          ? std::min(left_boundary->lat_offset, kMaxHalfLaneWidth)
          : kMaxHalfLaneWidth;
  const double l_max = std::clamp(box.l_max, boundary_right_l, boundary_left_l);
  const double l_min = std::clamp(box.l_min, boundary_right_l, boundary_left_l);

  if (std::min(boundary_left_l - l_min, l_max - boundary_right_l) <
      lat_buffer) {
    return false;
  }
  return true;
}

std::pair<double, double> CalculateTrafficCongestion(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DrivePassage& passage,
    const FrenetBox& ego_frenet_box, const Vec2d& ego_pos,
    bool* blocked_abreast) {
  constexpr double kObsLatInLaneBuffer = 1.0;     // m.
  constexpr double kLongiOccupancyBuffer = 5.0;   // m.
  constexpr double kMinConsiderDistance = 100.0;  // m.
  constexpr double kFollowTime = 2.0;             // m.

  std::vector<FrenetBox> objs_on_target;
  absl::flat_hash_set<std::string> has_checked_set;
  // Find all obs in current lane.
  for (const auto& traj : st_traj_mgr.trajectories()) {
    // Remove duplicated object id.
    if (has_checked_set.contains(traj.object_id())) continue;
    has_checked_set.emplace(traj.object_id());

    ASSIGN_OR_CONTINUE(const auto aabbox,
                       passage.QueryFrenetBoxAtContour(traj.contour()));

    if (!IsLaneBlockedByObs(passage, aabbox, kObsLatInLaneBuffer)) {
      continue;
    }
    constexpr double kCheckAbreastLonBuffer = 1.0;  // m.
    constexpr double kCheckAbreastLatBuffer = 2.5;  // m.
    if (aabbox.s_max > ego_frenet_box.s_min - kCheckAbreastLonBuffer &&
        aabbox.s_min < ego_frenet_box.s_max + kCheckAbreastLonBuffer &&
        ((ego_frenet_box.l_max > aabbox.l_min &&
          ego_frenet_box.l_min - aabbox.l_max < kCheckAbreastLatBuffer) ||
         (aabbox.l_max > ego_frenet_box.l_min &&
          aabbox.l_min - ego_frenet_box.l_max < kCheckAbreastLatBuffer))) {
      *blocked_abreast = true;
    }
    objs_on_target.push_back(aabbox);
  }

  // Calculate standard congestion factor
  double standard_congestion_factor = 0.0, traffic_congestion_factor = 0.0;
  const auto speed_limit_or = passage.QuerySpeedLimitAt(ego_pos);
  if (speed_limit_or.ok()) {
    standard_congestion_factor =
        ego_frenet_box.length() * 2 /
        std::max(1.0, *speed_limit_or * kFollowTime + kLongiOccupancyBuffer);
  }

  if (objs_on_target.size() <= 1) {
    traffic_congestion_factor = 0.0;
    return std::make_pair(standard_congestion_factor,
                          traffic_congestion_factor);
  }

  // Sort all obstacle on target.
  std::stable_sort(
      objs_on_target.begin(), objs_on_target.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.s_min > rhs.s_min; });
  const double consider_max_length =
      std::max(objs_on_target.front().s_max - objs_on_target.back().s_min,
               kMinConsiderDistance / objs_on_target.size());
  double occupancy_length = objs_on_target.front().length();
  for (int i = 1; i < objs_on_target.size(); ++i) {
    occupancy_length +=
        std::min(kLongiOccupancyBuffer,
                 objs_on_target.at(i - 1).s_min - objs_on_target.at(i).s_max) +
        objs_on_target.at(i).length();
  }

  traffic_congestion_factor =
      std::min(1.0, occupancy_length / std::max(1.0, consider_max_length));

  return std::make_pair(standard_congestion_factor, traffic_congestion_factor);
}

mapping::LanePath UpdatePrevLanePathBeforeLc(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& route_sections_info,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const RouteNaviInfo& route_navi_info) {
  // Assume the first lane segment is always loaded.
  for (int i = 0; i + 1 < prev_lane_path_before_lc_from_start.size(); ++i) {
    const auto& next_lane_id =
        prev_lane_path_before_lc_from_start.lane_id(i + 1);
    if (psmm.FindLaneInfoOrNull(next_lane_id) == nullptr) {
      QLOG(INFO) << "Trimmed prev_lane_path_before_lc before " << next_lane_id
                 << ".";
      return prev_lane_path_before_lc_from_start.BeforeLaneIndexPoint(
          i, /*lane_fraction=*/1.0);
    }
  }

  auto updated_lane_path_or = ForwardExtendLanePathOnRouteSections(
      psmm, *route_sections_info.route_sections(),
      prev_lane_path_before_lc_from_start,
      route_sections_info.planning_horizon(), route_navi_info);
  if (updated_lane_path_or.ok()) {
    return std::move(updated_lane_path_or).value();
  }

  QLOG(WARNING) << "Updating lane path before lc failed: "
                << updated_lane_path_or.status().message();
  return prev_lane_path_before_lc_from_start;
}

}  // namespace

absl::StatusOr<SchedulerOutput> MakeSchedulerOutput(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& route_sections_info,
    const std::vector<LanePathInfo>& lp_infos, DrivePassage drive_passage,
    const LanePathInfo& lp_info, const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const LaneChangeStateProto& prev_lc_state,
    const RouteNaviInfo& route_navi_info, bool borrow, bool should_smooth,
    bool planner_is_l4_mode, AutonomyStateProto::State autonomy_state) {
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const Box2d ego_box = ComputeAvBox(
      ego_pos, plan_start_point.path_point().theta(), vehicle_geom);
  ASSIGN_OR_RETURN(
      auto ego_frenet_box, drive_passage.QueryFrenetBoxAt(ego_box),
      _ << "Ego box " << ego_box.DebugString() << " is out of drive passage!");

  const double ref_center_l = CalcAvhRefCenterL(
      psmm, drive_passage, ego_frenet_box, smooth_result_map, should_smooth);
  ASSIGN_OR_RETURN(
      auto lc_state,
      MakeLaneChangeState(drive_passage, ego_pos, ego_frenet_box,
                          prev_target_lane_path_from_start,
                          prev_lane_path_before_lc_from_start, prev_lc_state,
                          ref_center_l, autonomy_state),
      _ << "Making lane change state failed.");
  bool request_help_lane_change_by_route = false;
  const auto request_help_lane_change_by_route_or = CheckNeedHelpToRouteChange(
      drive_passage, route_navi_info, planner_is_l4_mode);
  if (request_help_lane_change_by_route_or.ok()) {
    request_help_lane_change_by_route = *request_help_lane_change_by_route_or;
  } else {
    QLOG(WARNING) << request_help_lane_change_by_route_or.status().message();
  }
  const bool switch_alternate_route =
      CheckNeedSwitchRoute(psmm, lc_state, route_navi_info, drive_passage);

  bool blocked_abreast = false;
  const auto [standard_congestion_factor, traffic_congestion_factor] =
      CalculateTrafficCongestion(st_traj_mgr, drive_passage, ego_frenet_box,
                                 ego_pos, &blocked_abreast);

  const auto [planner_turn_signal, turn_signal_reason] =
      DecideRoutePrepareLcTurnSignal(psmm, drive_passage, route_navi_info);

  if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
    ASSIGN_OR_RETURN(auto path_boundary,
                     BuildPathBoundaryFromPose(
                         psmm, drive_passage, plan_start_point, vehicle_geom,
                         st_traj_mgr, lc_state, smooth_result_map, borrow,
                         should_smooth, /*unsafe_object_ids=*/{}),
                     _ << "Fail to build path boundary.");
    return SchedulerOutput{
        .drive_passage = std::move(drive_passage),
        .sl_boundary = std::move(path_boundary),
        .lane_change_state = std::move(lc_state),
        .length_along_route = lp_info.length_along_route(),
        .max_reach_length = lp_info.max_reach_length(),
        .standard_congestion_factor = standard_congestion_factor,
        .traffic_congestion_factor = traffic_congestion_factor,
        .should_smooth = should_smooth,
        .borrow_lane = borrow,
        .is_solid_lane_change = lp_info.is_solid_lane_change(),
        .av_frenet_box_on_drive_passage = ego_frenet_box,
        .request_help_lane_change_by_route = request_help_lane_change_by_route,
        .switch_alternate_route = switch_alternate_route,
        .planner_turn_signal = planner_turn_signal,
        .turn_signal_reason = turn_signal_reason,
    };
  }

  const auto neighbor_lp_info_or =
      FindNeighbor(route_sections_info, lp_infos, lc_state.lc_left(),
                   lp_info.start_lane_id());

  const bool target_switched =
      prev_target_lane_path_from_start.front().lane_id() !=
      drive_passage.lane_path().front().lane_id();

  if (!neighbor_lp_info_or.ok() ||
      (neighbor_lp_info_or->max_reach_length() < kEpsilon &&
       plan_start_point.v() < kForceMergeMaxSpeed)) {
    // No lane path before lc can be found, have to force merge.
    QLOG(WARNING) << "Applying force merge to lane " << lp_info.start_lane_id()
                  << ", no safety check is applied!";
    lc_state.set_force_merge(true);
  }

  ASSIGN_OR_RETURN(auto path_boundary,
                   BuildPathBoundaryFromPose(
                       psmm, drive_passage, plan_start_point, vehicle_geom,
                       st_traj_mgr, lc_state, smooth_result_map, borrow,
                       should_smooth, /*unsafe_object_ids=*/{}),
                   _ << "Fail to build path boundary.");

  blocked_abreast &=
      target_switched && lc_state.stage() == LaneChangeStage::LCS_EXECUTING;
  mapping::LanePath lane_path_before_lc;
  if (target_switched && lc_state.stage() == LaneChangeStage::LCS_EXECUTING) {
    lane_path_before_lc = neighbor_lp_info_or.ok()
                              ? neighbor_lp_info_or->lane_path()
                              : mapping::LanePath();
  } else if (!prev_lane_path_before_lc_from_start.IsEmpty()) {
    lane_path_before_lc = UpdatePrevLanePathBeforeLc(
        psmm, route_sections_info, prev_lane_path_before_lc_from_start,
        route_navi_info);
  }

  return SchedulerOutput{
      .drive_passage = std::move(drive_passage),
      .sl_boundary = std::move(path_boundary),
      .lane_change_state = std::move(lc_state),
      .lane_path_before_lc = std::move(lane_path_before_lc),
      .length_along_route = lp_info.length_along_route(),
      .max_reach_length = lp_info.max_reach_length(),
      .standard_congestion_factor = standard_congestion_factor,
      .traffic_congestion_factor = traffic_congestion_factor,
      .blocked_abreast = blocked_abreast,  // only for LCS_EXECUTING
      .should_smooth = should_smooth,
      .borrow_lane = borrow,
      .is_solid_lane_change = lp_info.is_solid_lane_change(),
      .av_frenet_box_on_drive_passage = ego_frenet_box,
      .request_help_lane_change_by_route = request_help_lane_change_by_route,
      .switch_alternate_route = switch_alternate_route,
      .planner_turn_signal = planner_turn_signal,
      .turn_signal_reason = turn_signal_reason,
  };
}

absl::StatusOr<std::vector<SchedulerOutput>> ScheduleMultiplePlanTasks(
    const MultiTasksSchedulerInput& input,
    const std::vector<LanePathInfo>& target_lp_infos, ThreadPool* thread_pool) {
  SCOPED_QTRACE("ScheduleMultiplePlanTasks");

  // Construct vision map, if online semantic is not empty.
  std::shared_ptr<PlannerSemanticMapManager> vision_map = nullptr;
  if (input.online_semantic_map != nullptr) {
    auto vision_map_or =
        BuildOnlineMapPsmm(*input.online_semantic_map, thread_pool);
    if (vision_map_or.ok()) {
      vision_map = std::move(vision_map_or).value();
    }
  }

  std::vector<SchedulerOutput> multi_tasks;
  if (target_lp_infos.size() == 1) {
    const auto& lp_info = target_lp_infos.front();
    ASSIGN_OR_RETURN(
        const auto clamped_route_section,
        ClampRouteSectionsBeforeArcLength(
            *input.psmm, *input.prev_route_sections,
            kDrivePassageKeepBehindLength + kMaxTravelDistanceBetweenFrames));
    ASSIGN_OR_RETURN(const auto backward_extended_lane_path,
                     BackwardExtendLanePathOnRouteSections(
                         *input.psmm, clamped_route_section,
                         lp_info.lane_path(), kDrivePassageKeepBehindLength));
    ASSIGN_OR_RETURN(
        auto drive_passage,
        BuildDrivePassage(*input.psmm, vision_map, lp_info.lane_path(),
                          backward_extended_lane_path, *input.station_anchor,
                          input.sections_info_from_current->planning_horizon(),
                          input.sections_info_from_current->destination(),
                          FLAGS_planner_consider_all_lanes_virtual,
                          input.cruising_speed_limit),
        _ << "Failed to build drive passage on single target lane path.");
    if (FLAGS_planner_drive_passage_debug_level) {
      SendDrivePassageToCanvas(drive_passage, "drive_passage");
    }

    const bool should_smooth = ShouldSmoothRefLane(
        *input.tl_info_map, drive_passage, input.prev_smooth_state);

    const std::vector<bool> borrow_branches =
        FLAGS_planner_est_scheduler_allow_borrow
            ? std::vector<bool>{false, true}
            : std::vector<bool>{false};
    for (bool borrow : borrow_branches) {
      auto output_or = MakeSchedulerOutput(
          *input.psmm, *input.sections_info_from_current,
          *input.lane_path_infos, drive_passage, lp_info, *input.vehicle_geom,
          *input.st_traj_mgr, *input.plan_start_point, *input.smooth_result_map,
          *input.prev_target_lane_path_from_start,
          *input.prev_lane_path_before_lc_from_start, *input.prev_lc_state,
          *input.route_navi_info, borrow, should_smooth,
          input.planner_is_l4_mode, input.autonomy_state);
      if (output_or.ok()) {
        multi_tasks.emplace_back(std::move(output_or).value());
      } else {
        LOG(INFO) << "Building scheduler output from "
                  << lp_info.start_lane_id() << "("
                  << (borrow ? "borrow" : "no borrow")
                  << ") failed: " << output_or.status().message();
      }
    }
  } else {
    std::vector<absl::StatusOr<SchedulerOutput>> outputs(
        target_lp_infos.size());
    ParallelFor(0, target_lp_infos.size(), thread_pool, [&](int i) {
      const auto& lp_info = target_lp_infos[i];
      const auto clamped_route_section = ClampRouteSectionsBeforeArcLength(
          *input.psmm, *input.prev_route_sections,
          kDrivePassageKeepBehindLength + kMaxTravelDistanceBetweenFrames);
      if (!clamped_route_section.ok()) return;
      const auto backward_extended_lane_path =
          BackwardExtendLanePathOnRouteSections(
              *input.psmm, *clamped_route_section, lp_info.lane_path(),
              kDrivePassageKeepBehindLength);
      if (!backward_extended_lane_path.ok()) return;
      auto drive_passage_or = BuildDrivePassage(
          *input.psmm, vision_map, lp_info.lane_path(),
          *backward_extended_lane_path, *input.station_anchor,
          input.sections_info_from_current->planning_horizon(),
          input.sections_info_from_current->destination(),
          FLAGS_planner_consider_all_lanes_virtual, input.cruising_speed_limit);
      if (!drive_passage_or.ok()) return;

      if (FLAGS_planner_drive_passage_debug_level) {
        SendDrivePassageToCanvas(*drive_passage_or,
                                 absl::StrCat("drive_passage_", i));
      }

      const bool should_smooth = ShouldSmoothRefLane(
          *input.tl_info_map, *drive_passage_or, input.prev_smooth_state);

      outputs[i] = MakeSchedulerOutput(
          *input.psmm, *input.sections_info_from_current,
          *input.lane_path_infos, std::move(drive_passage_or).value(), lp_info,
          *input.vehicle_geom, *input.st_traj_mgr, *input.plan_start_point,
          *input.smooth_result_map, *input.prev_target_lane_path_from_start,
          *input.prev_lane_path_before_lc_from_start, *input.prev_lc_state,
          *input.route_navi_info, /*borrow=*/false, should_smooth,
          input.planner_is_l4_mode, input.autonomy_state);
    });

    for (int i = 0; i < outputs.size(); ++i) {
      if (outputs[i].ok()) {
        multi_tasks.emplace_back(std::move(outputs[i]).value());
      } else {
        LOG(INFO) << "Building scheduler output from "
                  << target_lp_infos[i].start_lane_id()
                  << " failed: " << outputs[i].status().message();
      }
    }
  }

  if (multi_tasks.empty()) {
    QLOG(ERROR) << "Unable to schedule multiple tasks.";
    return absl::NotFoundError(
        "Fail to build drive passage on each lane path.");
  }

  if (FLAGS_planner_est_scheduler_seperate_lc_pause &&
      input.prev_lc_state->stage() != LaneChangeStage::LCS_NONE) {
    absl::StatusOr<SchedulerOutput> lc_pause_scheduler;
    for (const auto& output : multi_tasks) {
      if (!output.borrow_lane &&
          output.lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING) {
        lc_pause_scheduler = MakeLcPauseSchedulerOutput(
            *input.psmm, output, *input.vehicle_geom, *input.st_traj_mgr,
            *input.plan_start_point, *input.smooth_result_map);
        break;
      }
    }
    if (lc_pause_scheduler.ok()) {
      multi_tasks.push_back(std::move(lc_pause_scheduler).value());
    }
  }

  return multi_tasks;
}

}  // namespace qcraft::planner
