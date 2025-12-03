#include "onboard/planner/selector/candidate_stats.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <float.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/util/scene_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

constexpr double kFollowAccelDecel = 0.6;         // m/s^2.
constexpr double kMinFollowTime = 3.0;            // s.
constexpr double kPreviewTimeForDecel = 5.0;      // s.
constexpr double kPreviewTimeForLeaderVel = 4.0;  // s.
constexpr double kPreviewMinLaneSizeDist = 60.0;  // m.
constexpr int kMinLaneSizeDiscourgeRightMost = 3;
constexpr double kMinDrivingDistDiscourgeRightMost = 2000.0;  // m
constexpr double kMinDrivingDistEncourgeRightMost = 500.0;    // m
constexpr double kPreviewLengthForRightMostLane = 150.0;      // m
constexpr double kSingleLaneStalledFactor = 5.0;
constexpr double kLengthEpsilon = 1.0;      // m.
constexpr double kInvalidLength = 10000.0;  // m.

double CalculateProgressSpeed(const double ego_v, const double leader_v,
                              const double leader_s,
                              const double min_front_dist) {
  const double target_follow_distance =
      std::max(min_front_dist, leader_v * kMinFollowTime);
  const double init_obj_v_lim =
      leader_v +
      std::sqrt(kFollowAccelDecel *
                    std::max(0.0, leader_s - target_follow_distance) +
                0.5 * (Sqr(ego_v) + Sqr(leader_v)) - ego_v * leader_v);
  // fix when leader v is far smaller than ego v
  const double deceleration_time = std::max(
      kPreviewTimeForDecel - (init_obj_v_lim - ego_v) / kFollowAccelDecel,
      kPreviewTimeForDecel);
  const double deceleration = std::clamp(
      (ego_v - leader_v) / kTrajectoryTimeHorizon, 0.0, kFollowAccelDecel);
  const double obj_v_lim =
      std::max(leader_v, init_obj_v_lim - deceleration_time * deceleration);
  return obj_v_lim;
}

bool IsNonOccludingLeader(ObjectType object_type) {
  switch (object_type) {
    case ObjectType::OT_VEHICLE:
    case ObjectType::OT_LARGE_VEHICLE:
    case ObjectType::OT_UNKNOWN_MOVABLE:
      return false;
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_CYCLIST:
    case ObjectType::OT_TRICYCLIST:
    case ObjectType::OT_PEDESTRIAN:
      return true;
    case ObjectType::OT_CONE:
    case ObjectType::OT_WARNING_TRIANGLE:
    case ObjectType::OT_UNKNOWN_STATIC:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_BUCKET:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_POST:
    case ObjectType::OT_FOD:
    case ObjectType::OT_VEGETATION:
      return false;
  }
}

inline mapping::ElementId GetBranchStartLaneId(const EstPlannerOutput& result) {
  return result.scheduler_output.drive_passage.lane_path().front().lane_id();
}

absl::flat_hash_map<mapping::ElementId, bool> GenerateValidMergeMap(
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const SelectorCommonFeature& common_feature) {
  absl::flat_hash_map<mapping::ElementId, bool> is_valid_merge_map;

  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok() &&
        est_status[idx].status_code() !=
            PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
      continue;
    }
    const auto& scheduler_output = results[idx].scheduler_output;
    bool is_valid = false;
    const auto& lane_feature_info = FindOrDieNoPrint(
        common_feature.lane_feature_infos, scheduler_output.Hash());
    for (int other_idx = 0; other_idx < est_status.size(); ++other_idx) {
      if (!est_status[other_idx].ok() &&
          est_status[other_idx].status_code() !=
              PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
        continue;
      }
      if (idx == other_idx) continue;
      if (lane_feature_info.merge_targets.contains(
              GetBranchStartLaneId(results[other_idx]))) {
        is_valid = true;
        break;
      }
    }
    is_valid_merge_map[GetBranchStartLaneId(results[idx])] = is_valid;
  }

  return is_valid_merge_map;
}

}  // namespace

ProgressStats::ProgressStats(
    const SelectorCommonFeature& common_feature,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results)
    : ego_v(plan_start_point.v()) {
  FUNC_QTRACE();
  constexpr double kEgoFrontBuffer = 5.0;  // m.
  const double min_front_dist =
      vehicle_geom.front_edge_to_center() + kEgoFrontBuffer;
  const auto is_valid_merge_map =
      GenerateValidMergeMap(results, est_status, common_feature);

  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok()) continue;

    const auto& result = results[idx];
    const auto& passage = result.scheduler_output.drive_passage;

    const auto& lane_feature_infos = common_feature.lane_feature_infos;
    if (!lane_feature_infos.contains(result.scheduler_output.Hash())) continue;
    const auto& lane_feature_info =
        FindOrDieNoPrint(lane_feature_infos, result.scheduler_output.Hash());
    const bool is_valid_merge =
        FindOrDie(is_valid_merge_map, GetBranchStartLaneId(result));
    const double lane_speed_limit = lane_feature_info.speed_limit;
    const auto& block_obj_ids = lane_feature_info.block_obj_ids;

    std::optional<std::string> block_obj = std::nullopt;
    double lowest_v = lane_speed_limit;
    double leader_v = lowest_v;
    double leader_init_v = leader_v;
    double init_leader_dist = DBL_MAX;

    // Set merge lane as stop line
    std::optional<double> merge_lane_s = std::nullopt;
    if (is_valid_merge) {
      for (const auto& station : passage.stations()) {
        if (station.accumulated_s() < 0.0) continue;
        if (station.is_in_intersection() ||
            station.accumulated_s() >
                result.scheduler_output.length_along_route) {
          break;
        }
        if (station.is_merging()) {
          merge_lane_s = station.accumulated_s();
          break;
        }
      }
    }

    double nearest_stop_s = std::min(result.first_stop_s.value_or(DBL_MAX),
                                     merge_lane_s.value_or(DBL_MAX));
    const double stop_line_ref_v = CalculateProgressSpeed(
        ego_v, 0.0, nearest_stop_s, vehicle_geom.front_edge_to_center());
    if (stop_line_ref_v < lowest_v) {
      init_leader_dist = nearest_stop_s;
      leader_v = 0.0;
      leader_init_v = 0.0;
      block_obj = "stopline";
      lowest_v = std::min(lowest_v, stop_line_ref_v);
    }

    absl::flat_hash_set<std::string> checked_set;
    std::vector<std::pair<FrenetBox, const SpacetimeObjectTrajectory*>>
        block_objs;
    for (const auto& traj : st_traj_mgr_list[idx].trajectories()) {
      if (checked_set.contains(traj.object_id())) continue;
      checked_set.emplace(traj.object_id());
      if (!block_obj_ids.contains(traj.object_id())) continue;

      // Ignore not nearest unknown obstacles.
      if (!lane_feature_info.nearest_leader.has_value() ||
          lane_feature_info.nearest_leader->obj_id != traj.object_id()) {
        if (traj.object_type() == ObjectType::OT_UNKNOWN_MOVABLE) {
          continue;
        }
      }

      ASSIGN_OR_CONTINUE(const auto aabbox,
                         passage.QueryFrenetBoxAtContour(traj.contour()));
      block_objs.push_back(std::make_pair(aabbox, &traj));
      // consider when leader is accelerating when start up
      const double obs_perception_v = traj.planner_object().pose().v();
      double max_obj_prediction_v = obs_perception_v;
      for (const auto& traj_state : traj.states()) {
        if (traj_state.traj_point->t() > kPreviewTimeForLeaderVel) break;
        max_obj_prediction_v =
            std::max(max_obj_prediction_v, traj_state.traj_point->v());
      }
      const double obj_ref_v = max_obj_prediction_v;
      const double obj_v_lim = CalculateProgressSpeed(
          ego_v, obj_ref_v, aabbox.s_min, min_front_dist);

      if (obj_v_lim < lowest_v) {
        init_leader_dist = aabbox.s_min;
        leader_init_v = obs_perception_v;
        leader_v = obj_ref_v;
        lowest_v = obj_v_lim;
        block_obj = traj.object_id();
      }
    }

    if (!lane_speed_map.contains(result.scheduler_output.Hash()) ||
        lane_speed_map[result.scheduler_output.Hash()]
                .lane_speed_limit_by_leader > lowest_v) {
      lane_speed_map[result.scheduler_output.Hash()] =
          LaneSpeedInfo{.init_leader_dist = init_leader_dist,
                        .init_leader_speed = leader_init_v,
                        .max_leader_speed = leader_v,
                        .lane_speed_limit_by_leader = lowest_v,
                        .lane_speed_limit = lane_speed_limit,
                        .block_obj = block_obj};
      max_lane_speed = std::max(max_lane_speed, lowest_v);
      min_leader_dist = std::min(min_leader_dist, init_leader_dist);
    }
  }
}

RouteLookAheadStats::RouteLookAheadStats(
    const SelectorCommonFeature& common_feature,
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const ApolloTrajectoryPointProto& plan_start_point,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results) {
  // 1. get parameters from inputs
  const double length_before_intersection =
      common_feature.length_before_intersection;
  const bool in_high_way = common_feature.in_high_way;
  ego_v = plan_start_point.v();
  is_valid_merge_lane_map =
      GenerateValidMergeMap(results, est_status, common_feature);
  is_left_turn = common_feature.is_left_turn;
  is_right_turn = common_feature.is_right_turn;

  // 2. fill drive_dist and lane change number for all lanes
  double min_driving_dist = DBL_MAX;
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok() &&
        est_status[idx].status_code() !=
            PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
      continue;
    }
    const auto& passage = results[idx].scheduler_output.drive_passage;
    const auto start_lane_id = passage.lane_path().front().lane_id();
    const auto& lane_feature_info =
        FindOrDieNoPrint(common_feature.lane_feature_infos,
                         results[idx].scheduler_output.Hash());
    const int lc_num_to_targets = lane_feature_info.lc_num_to_targets;
    const int lc_num_within_driving_dist =
        lane_feature_info.lc_num_within_driving_dist;
    const double driving_dist = lane_feature_info.driving_dist;
    const double len_before_merge_lane =
        lane_feature_info.len_before_merge_lane;

    min_lc_num = std::min(min_lc_num, lc_num_to_targets);
    max_lc_num = std::max(max_lc_num, lc_num_to_targets);
    min_driving_dist = std::min(min_driving_dist, driving_dist);

    lc_num_to_targets_map[start_lane_id] = lc_num_to_targets;
    lc_num_within_driving_dist_map[start_lane_id] = lc_num_within_driving_dist;
    driving_dist_map[start_lane_id] = driving_dist;
    len_before_merge_lane_map[start_lane_id] = len_before_merge_lane;

    // Calculate right most lane map.
    bool is_right_most_drivable_lane =
        IsRightMostDrivableLane(psmm, passage.lane_path().back().lane_id());
    bool is_continuous_right_most_lane = true;
    for (int i = 0; i < passage.lane_path().size(); ++i) {
      const auto start_length = passage.lane_path().start_s(i);
      if (start_length > kPreviewLengthForRightMostLane) {
        break;
      }
      is_continuous_right_most_lane =
          is_continuous_right_most_lane &&
          IsRightMostDrivableLane(psmm, passage.lane_path().lane_id(i));
    }
    is_right_most_lane_map[start_lane_id] =
        is_right_most_drivable_lane || is_continuous_right_most_lane;
  }

  enable_discourage_right_most_cost =
      (PreviewMinDrivableLanes(psmm, sections_info, kPreviewMinLaneSizeDist,
                               /*start_index=*/0) >=
           kMinLaneSizeDiscourgeRightMost &&
       in_high_way && min_driving_dist > kMinDrivingDistDiscourgeRightMost);
  enable_encourage_right_most_cost =
      (!in_high_way &&
       (min_driving_dist > kMinDrivingDistEncourgeRightMost || is_right_turn));

  // 4. adjust the length along route according to merge lane
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok() &&
        est_status[idx].status_code() !=
            PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
      continue;
    }

    const auto& passage = results[idx].scheduler_output.drive_passage;
    const auto start_lane_id = passage.lane_path().front().lane_id();
    const auto scheduler_hash = results[idx].scheduler_output.Hash();
    double length_along_route =
        results[idx].scheduler_output.length_along_route;
    raw_len_along_route_map[scheduler_hash] = length_along_route;

    const int lc_num_to_targets =
        FindOrDie(lc_num_to_targets_map, start_lane_id);

    if (lc_num_to_targets == min_lc_num && min_lc_num != max_lc_num) {
      // When congestion_factor is greater than zero, it means the road has
      // heavy traffic.
      traffic_congestion_factor =
          results[idx].scheduler_output.traffic_congestion_factor -
          results[idx].scheduler_output.standard_congestion_factor;
    }

    double len_before_merge_lane =
        FindOrDie(len_before_merge_lane_map, start_lane_id);
    if (len_before_merge_lane > length_along_route - kLengthEpsilon ||
        len_before_merge_lane > length_before_intersection - kLengthEpsilon) {
      // Ignore merge lane after intersection or length along route
      len_before_merge_lane = kInvalidLength;
    }

    const auto& lane_feature_infos = common_feature.lane_feature_infos;
    if (!lane_feature_infos.contains(scheduler_hash)) continue;
    const auto& lane_feature_info =
        FindOrDieNoPrint(lane_feature_infos, scheduler_hash);
    const auto& block_obj_ids = lane_feature_info.block_obj_ids;

    if (lc_num_to_targets == min_lc_num && min_lc_num != max_lc_num) {
      // Only consider nearest leader in target lane.
      if (lane_feature_info.nearest_leader.has_value() &&
          IsNonOccludingLeader(lane_feature_info.nearest_leader->obj_type)) {
        is_non_occluding_leader = true;
      }
    }

    std::optional<StalledObjInfo> stalled_obj_info = std::nullopt;
    constexpr double kDoubleEpsilon = 1e-6;
    for (const auto& traj : st_traj_mgr_list[idx].trajectories()) {
      if (!block_obj_ids.contains(traj.object_id())) continue;
      if (!stalled_objects.contains(traj.object_id())) continue;
      ASSIGN_OR_CONTINUE(const auto aabbox,
                         passage.QueryFrenetBoxAtContour(traj.contour()));
      const auto obj_station = passage.FindNearestStationAtS(aabbox.center_s());
      const auto obj_lane_pt = obj_station.GetLanePoint();
      const auto* obj_sec =
          sections_info.FindSegmentContainingLanePointOrNull(obj_lane_pt);
      if (obj_sec == nullptr) continue;

      const double punish_factor =
          (obj_sec->lane_ids.size() > 1 || obj_station.is_in_intersection())
              ? 1.0
              : kSingleLaneStalledFactor;
      if (!stalled_obj_info.has_value() ||
          stalled_obj_info->punish_factor < punish_factor ||
          (std::abs(stalled_obj_info->punish_factor - punish_factor) <
               kDoubleEpsilon &&
           stalled_obj_info->stalled_obj_s > aabbox.s_min)) {
        // Choose the biggest punish factor, then choose nearest obj
        stalled_obj_info =
            StalledObjInfo{.stalled_obj_id = std::string(traj.object_id()),
                           .stalled_obj_s = aabbox.s_min,
                           .punish_factor = punish_factor};
      }

      length_along_route = std::min(
          length_along_route, std::max(0.0, aabbox.s_min - kMinLcLaneLength));
    }

    const double curr_len_before_intersection =
        std::min(length_along_route, length_before_intersection);
    len_along_route_map[scheduler_hash] = length_along_route;
    len_before_intersection_map[start_lane_id] = curr_len_before_intersection;
    front_stalled_obj_map[scheduler_hash] = std::move(stalled_obj_info);
    len_before_merge_lane_map[start_lane_id] = len_before_merge_lane;
    if (length_along_route > max_length_along_route) {
      max_length_along_route = length_along_route;
    }
    if (length_along_route < min_length_along_route) {
      min_length_along_route = length_along_route;
    }
  }
}

}  // namespace qcraft::planner
