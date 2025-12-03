#include "onboard/planner/decision/leading_groups_builder.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

struct LeadingObjectTrajectoryInfo {
  // Project potential leading object trajectory onto drive passage.
  absl::string_view traj_id;
  double front_s, rear_s;
  double front_s_future, rear_s_future;
  absl::Span<const SpacetimeObjectState> states;
};

bool HasEnteredSlBoundary(const PathSlBoundary& path_boundary,
                          const FrenetBox& fbox, bool lc_left) {
  const auto [boundary_left_l, boundary_right_l] =
      CalcSlBoundaries(path_boundary, fbox);
  const double l_center = path_boundary.QueryReferenceCenterL(fbox.center_s());

  constexpr double kLateralEnterThres = 0.5;  // m.
  return lc_left ? fbox.l_min < boundary_left_l - kLateralEnterThres &&
                       fbox.l_max > l_center
                 : fbox.l_max > boundary_right_l + kLateralEnterThres &&
                       fbox.l_min < l_center;
}

bool ShouldConsiderInteraction(const SpacetimeObjectTrajectory& traj,
                               double ego_heading,
                               const FrenetBox& ego_frenet_box,
                               const PathSlBoundary& path_boundary,
                               const DrivePassage& drive_passage, bool lc_left,
                               LeadingObjectTrajectoryInfo* traj_info) {
  FUNC_QTRACE();

  const auto& states = traj.states();
  if (std::abs(NormalizeAngle(ego_heading -
                              states.front().traj_point->theta())) > M_PI_2) {
    return false;
  }

  // First check the current position.
  ASSIGN_OR_RETURN(const auto fbox,
                   drive_passage.QueryFrenetBoxAt(states.front().box), false);
  if (fbox.s_min <= ego_frenet_box.s_max) return false;
  if (HasEnteredSlBoundary(path_boundary, fbox, lc_left)) {
    traj_info->traj_id = traj.traj_id();
    traj_info->front_s = fbox.s_max;
    traj_info->rear_s = fbox.s_min;
    traj_info->states = states;
    return true;
  }

  // Then check the whole trajectory to find possible interaction by ratio.
  constexpr int kEvalStep = 1;
  constexpr int kEnterSRangeStep =
      static_cast<int>(1.0 / prediction::kPredictionTimeStep);
  bool is_valid = false;
  int projected_states = 1, along_path_states = 0;
  double front_s, rear_s;
  for (int i = 1; i < states.size(); i += kEvalStep) {
    if (!is_valid && i > kEnterSRangeStep) {
      // Not entering drive passage in the early seconds means the object is too
      // far behind, so we ignore it.
      return false;
    }

    ASSIGN_OR_CONTINUE(const auto fbox,
                       drive_passage.QueryFrenetBoxAt(states[i].box));
    ++projected_states;
    if (!is_valid) {
      // Record the first projectable state onto drive passage.
      is_valid = true;
      front_s = fbox.s_max;
      rear_s = fbox.s_min;
    }
    if (HasEnteredSlBoundary(path_boundary, fbox, lc_left)) {
      ++along_path_states;
    }
  }

  constexpr double kMoveAlongPathPercentageThreshold = 0.5;
  const double on_path_ratio = static_cast<double>(along_path_states) /
                               static_cast<double>(projected_states);
  if (on_path_ratio > kMoveAlongPathPercentageThreshold) {
    traj_info->traj_id = traj.traj_id();
    traj_info->front_s = front_s;
    traj_info->rear_s = rear_s;
    traj_info->states = states;
    return true;
  }
  return false;
}

std::vector<LeadingGroup> SeparateGroupsConsiderFuturePosition(
    const DrivePassage& drive_passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    std::vector<LeadingObjectTrajectoryInfo> traj_group_info,
    const VehicleGeometryParamsProto& vehicle_geom,
    const FrenetBox& ego_frenet_box, const double cur_ego_v,
    const int expected_new_leading_groups_size) {
  std::vector<LeadingGroup> new_leading_groups;
  constexpr double kMinFutureTimeConsidered = 2.0;  // s

  const double future_time_considered = std::max(
      (traj_group_info.front().rear_s - ego_frenet_box.s_max) / cur_ego_v,
      kMinFutureTimeConsidered);

  if (future_time_considered > prediction::kPredictionDuration) {
    // the group is far and consider them as a group
    LeadingGroup second_level_traj_group;
    for (auto& traj : traj_group_info) {
      second_level_traj_group.emplace(
          traj.traj_id,
          CreateLeadingObject(
              *st_traj_mgr.FindTrajectoryById(traj.traj_id), drive_passage,
              ConstraintProto::LeadingObjectProto::LANE_CHANGE_TARGET));
    }
    second_level_traj_group.swap(new_leading_groups.emplace_back());
    return new_leading_groups;
  }

  const int future_state_index = static_cast<int>(
      future_time_considered / prediction::kPredictionTimeStep);

  // further filter trajs in a group and assign future s
  std::vector<LeadingObjectTrajectoryInfo> filtered_traj_info;
  for (auto& traj : traj_group_info) {
    if (future_state_index < traj.states.size()) {
      ASSIGN_OR_CONTINUE(
          const auto fbox_future,
          drive_passage.QueryFrenetBoxAt(traj.states[future_state_index].box));
      traj.front_s_future = fbox_future.s_max;
      traj.rear_s_future = fbox_future.s_min;
      filtered_traj_info.push_back(traj);
    }
  }

  std::stable_sort(filtered_traj_info.begin(), filtered_traj_info.end(),
                   [](const LeadingObjectTrajectoryInfo& traj1,
                      const LeadingObjectTrajectoryInfo& traj2) {
                     return traj1.rear_s_future < traj2.rear_s_future;
                   });

  LeadingGroup second_level_traj_group;
  const double min_gap = vehicle_geom.length() * 2.0;
  double previous_s = std::numeric_limits<double>::lowest();
  for (const auto& traj : filtered_traj_info) {
    if (!second_level_traj_group.empty()) {
      const double current_gap = traj.rear_s_future - previous_s;
      if (current_gap >= min_gap) {
        second_level_traj_group.swap(new_leading_groups.emplace_back());
      }
    }

    second_level_traj_group.emplace(
        traj.traj_id,
        CreateLeadingObject(
            *st_traj_mgr.FindTrajectoryById(traj.traj_id), drive_passage,
            ConstraintProto::LeadingObjectProto::LANE_CHANGE_TARGET));

    previous_s = traj.front_s_future;

    if (new_leading_groups.size() == expected_new_leading_groups_size - 1) {
      break;
    }
  }

  if (!second_level_traj_group.empty()) {
    new_leading_groups.push_back(std::move(second_level_traj_group));
  }
  return new_leading_groups;
}

}  // namespace

std::vector<LeadingGroup> FindMultipleLeadingGroups(
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    bool lc_left, const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects, double ego_heading,
    const FrenetBox& ego_frenet_box,
    const VehicleGeometryParamsProto& vehicle_geom, const double cur_ego_v) {
  std::vector<LeadingGroup> leading_groups;
  // If current lateral offset greater than a value, consider we need lane
  // change or lane borrow.
  const auto& considered_trajectories = st_traj_mgr.trajectories();
  std::vector<LeadingObjectTrajectoryInfo> filtered_trajs;
  for (const auto& traj : considered_trajectories) {
    VLOG(3) << "Consider traj: " << traj.traj_id();
    const auto object_type = traj.planner_object().type();
    LeadingObjectTrajectoryInfo traj_info;
    if (IsLeadingObjectType(object_type) &&
        ShouldConsiderInteraction(traj, ego_heading, ego_frenet_box,
                                  path_boundary, drive_passage, lc_left,
                                  &traj_info)) {
      filtered_trajs.push_back(traj_info);
    }
  }
  std::stable_sort(filtered_trajs.begin(), filtered_trajs.end(),
                   [](const LeadingObjectTrajectoryInfo& traj1,
                      const LeadingObjectTrajectoryInfo& traj2) {
                     return traj1.rear_s < traj2.rear_s;
                   });

  // Group potential leading objects to groups. Separate groups according to
  // sufficient longitudinal gap.
  std::vector<LeadingObjectTrajectoryInfo> traj_group_info;

  const double min_gap = vehicle_geom.length() * 2.0;
  double previous_s = std::numeric_limits<double>::lowest();
  for (const auto& traj : filtered_trajs) {
    if (traj.front_s >= path_boundary.end_s()) break;

    if (!traj_group_info.empty()) {
      const double current_gap = traj.rear_s - previous_s;
      if (current_gap >= min_gap) {
        std::vector<LeadingGroup> new_leading_groups =
            SeparateGroupsConsiderFuturePosition(
                drive_passage, st_traj_mgr, std::move(traj_group_info),
                vehicle_geom, ego_frenet_box, cur_ego_v,
                FLAGS_planner_initializer_max_multi_traj_num -
                    leading_groups.size());

        traj_group_info.clear();

        leading_groups.insert(
            leading_groups.end(),
            std::make_move_iterator(new_leading_groups.begin()),
            std::make_move_iterator(new_leading_groups.end()));

        if (leading_groups.size() ==
            FLAGS_planner_initializer_max_multi_traj_num) {
          return leading_groups;
        }
      }
    }

    if (!stalled_objects.contains(
            SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
                traj.traj_id))) {
      traj_group_info.push_back(traj);
    }
    previous_s = traj.front_s;

    if (leading_groups.size() ==
        FLAGS_planner_initializer_max_multi_traj_num - 1) {
      break;
    }
  }

  if (!traj_group_info.empty()) {
    LeadingGroup one_more_traj_group;
    one_more_traj_group.emplace(
        traj_group_info.front().traj_id,
        CreateLeadingObject(
            *st_traj_mgr.FindTrajectoryById(traj_group_info.front().traj_id),
            drive_passage,
            ConstraintProto::LeadingObjectProto::LANE_CHANGE_TARGET));
    leading_groups.push_back(std::move(one_more_traj_group));
  }
  return leading_groups;
}

std::vector<LeadingGroup> DeriveMultipleLeadingGroupsFromCaptainNetTrajectory(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DrivePassage& passage,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<ApolloTrajectoryPointProto>& traj_points) {
  std::vector<LeadingGroup> leading_groups;
  LeadingGroup leading_group;

  // Build frenet frame
  std::vector<Vec2d> ref_points;
  ref_points.reserve(traj_points.size());
  for (const auto& traj_pt : traj_points) {
    ref_points.push_back(Vec2dFromApolloTrajectoryPointProto(traj_pt));
  }
  ASSIGN_OR_RETURN(const auto ref_frame,
                   BuildBruteForceFrenetFrame(ref_points,
                                              /*down_sample_raw_points=*/true),
                   leading_groups);

  for (const auto& st_traj : st_traj_mgr.trajectories()) {
    // Filter objects not leading type
    if (!IsLeadingObjectType(st_traj.planner_object().type())) {
      continue;
    }

    // Stall object is not leading object.
    if (stalled_objects.contains(st_traj.planner_object().id())) {
      continue;
    }

    // Not behind the ego at t = 0
    ASSIGN_OR_CONTINUE(
        const auto object_frenet_box,
        ref_frame.QueryFrenetBoxAtContour(st_traj.planner_object().contour()));
    if (object_frenet_box.s_min <
        ref_frame.start_s() + vehicle_geometry_params.front_edge_to_center()) {
      continue;
    }

    // Filter object with large diff heading from capnet trajectory
    const auto angle_vec_ref =
        ref_frame.InterpolateTangentByXY(st_traj.pose().pos());
    const double angle_ref = NormalizeAngle2D(angle_vec_ref);
    const double angle_leading = st_traj.pose().theta();
    if (std::abs(NormalizeAngle(angle_leading - angle_ref)) > M_PI_2) {
      continue;
    }

    // Get the time when object is close to capnet, and check time difference
    const double k_max_l_away_from_captain_net_traj =
        0.5 * vehicle_geometry_params.width();
    const double k_max_s_beyond_captain_net_traj_end_s =
        2.0 * vehicle_geometry_params.width();
    double intersect_time_object = -1.0;
    double intersect_time_capnet = -1.0;
    for (int i = 0; i < st_traj.states().size(); ++i) {
      ASSIGN_OR_CONTINUE(const auto object_frenet_box_future,
                         ref_frame.QueryFrenetBoxAt(st_traj.states()[i].box));
      const double l_diff = std::min(std::abs(object_frenet_box_future.l_max),
                                     std::abs(object_frenet_box_future.l_min));
      if (l_diff < k_max_l_away_from_captain_net_traj &&
          object_frenet_box_future.s_max <
              ref_frame.end_s() + k_max_s_beyond_captain_net_traj_end_s) {
        // close to capnet trajectory
        intersect_time_object = i * prediction::kPredictionTimeStep;
        intersect_time_capnet = ref_frame.GetNearestPointIndex(
                                    st_traj.states()[i].traj_point->pos()) *
                                ml::captain_net::kTimeStep;
        break;
      }
    }

    if (intersect_time_object < 0 ||
        intersect_time_object > intersect_time_capnet) {
      // As leading, object should reach some point earlier than ego
      continue;
    }

    // Create object and add to leading group
    leading_group.emplace(
        st_traj.traj_id(),
        CreateLeadingObject(
            *st_traj_mgr.FindTrajectoryById(st_traj.traj_id()), passage,
            ConstraintProto::LeadingObjectProto::LANE_CHANGE_TARGET));
  }

  leading_group.swap(leading_groups.emplace_back());

  return leading_groups;
}

}  // namespace qcraft::planner
