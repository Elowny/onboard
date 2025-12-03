#include "onboard/planner/initializer/multi_traj_selector.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <float.h>

#include <algorithm>
#include <ostream>
#include <string_view>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/common/lane_change_safety.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

struct TrajEvalInfo {
  double eval_cost;
  absl::flat_hash_set<std::string> follower_set;
  double follower_max_decel = 0.0;

  std::string unsafe_object_id;
  LaneChangeSafetyDebugProto debug_proto;
};

absl::Status EvaluateSingleTrajectory(const SingleTrajInfo& traj_info,
                                      TrajEvalInfo* eval_info) {
  eval_info->eval_cost = traj_info.total_cost;
  return absl::OkStatus();
}

absl::Status EvaluateSingleTrajectoryAndCheckSafety(
    const SingleTrajInfo& traj_info, const FrenetFrame& target_frenet_frame,
    double speed_limit, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geom, LaneChangeStyle lc_style,
    absl::Duration path_look_ahead_duration, TrajEvalInfo* eval_info) {
  auto lc_safety_status = CheckLaneChangeSafety(
      traj_info.traj_points, target_frenet_frame, speed_limit, st_traj_mgr,
      vehicle_geom, lc_style, path_look_ahead_duration,
      &eval_info->follower_set, &eval_info->follower_max_decel,
      &eval_info->unsafe_object_id, &eval_info->debug_proto);
  eval_info->eval_cost = lc_safety_status.ok() ? traj_info.total_cost : DBL_MAX;

  return lc_safety_status;
}

}  // namespace

absl::StatusOr<int> EvaluateMultiTrajs(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<SingleTrajInfo>& multi_trajs,
    const VehicleGeometryParamsProto& vehicle_geom, bool eval_safety,
    LaneChangeStyle lc_style, absl::Duration path_look_ahead_duration,
    absl::flat_hash_set<std::string>* follower_set, double* follower_max_decel,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    LaneChangeSafetyDebugProto* lane_change_safety_debug_proto,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("EvaluateMultiTrajs");
  VLOG(3) << "Evaluating " << multi_trajs.size() << " trajectories.";

  std::vector<absl::Status> traj_status_list(multi_trajs.size());
  std::vector<TrajEvalInfo> traj_eval_info_list(multi_trajs.size());

  if (!eval_safety) {
    for (int i = 0; i < multi_trajs.size(); ++i) {
      traj_status_list[i] =
          EvaluateSingleTrajectory(multi_trajs[i], &traj_eval_info_list[i]);
    }
  } else {
    const auto target_lane_path_ext =
        BackwardExtendLanePath(psmm,
                               drive_passage.extend_lane_path().BeforeArclength(
                                   kLaneChangeCheckForwardLength),
                               kLaneChangeCheckBackwardLength);
    ASSIGN_OR_RETURN(
        const auto target_frenet_frame,
        BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, target_lane_path_ext),
                               /*down_sample_raw_points=*/true));

    constexpr double kLaneSpeedLimitPreviewTime = 6.0;  // s.
    const auto lp = drive_passage.lane_path().ArclengthToLanePoint(
        multi_trajs[0].traj_points[0].v() * kLaneSpeedLimitPreviewTime);
    const double speed_limit =
        psmm.QueryLaneSpeedLimitByFraction(lp.lane_id(), lp.fraction());

    ParallelFor(0, multi_trajs.size(), thread_pool, [&](int i) {
      traj_status_list[i] = EvaluateSingleTrajectoryAndCheckSafety(
          multi_trajs[i], target_frenet_frame, speed_limit, st_traj_mgr,
          vehicle_geom, lc_style, path_look_ahead_duration,
          &traj_eval_info_list[i]);
    });
  }
  const int choice =
      std::min_element(traj_eval_info_list.begin(), traj_eval_info_list.end(),
                       [](const auto& a, const auto& b) {
                         return a.eval_cost < b.eval_cost;
                       }) -
      traj_eval_info_list.begin();
  *lane_change_safety_debug_proto =
      std::move(traj_eval_info_list[choice].debug_proto);

  if (!traj_status_list[choice].ok()) {
    std::string fail_msg;
    for (int i = 0; i < traj_status_list.size(); ++i) {
      fail_msg += absl::StrFormat("%d. %s ", i, traj_status_list[i].message());
      if (!traj_eval_info_list[i].unsafe_object_id.empty()) {
        unsafe_object_ids->insert(traj_eval_info_list[i].unsafe_object_id);
      }
    }
    return absl::CancelledError(fail_msg);
  }
  *follower_set = std::move(traj_eval_info_list[choice].follower_set);
  *follower_max_decel = traj_eval_info_list[choice].follower_max_decel;

  return choice;
}

}  // namespace qcraft::planner
