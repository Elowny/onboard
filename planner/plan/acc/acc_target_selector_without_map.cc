#include "onboard/planner/plan/acc/acc_target_selector_without_map.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_target.h"
#include "onboard/planner/plan/acc/acc_target_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

namespace {

constexpr double kAccTargetWithoutMapOverlapThreshold = 0.3;

}  // namespace

AccTargetPerCorridor SelectAccTargetWithoutMap(const AccTargetInput& input) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(input.corridor);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.av_pose);
  QCHECK_NOTNULL(input.vehicle_geom);
  QCHECK_NOTNULL(input.plan_start_point);

  const auto& st_traj_mgr = *input.st_traj_mgr;
  const auto& plan_start_point = *input.plan_start_point;
  const auto& vehicle_geom = *input.vehicle_geom;
  const auto& corridor = *input.corridor;
  const auto& path_ff = corridor.frenet_frame;

  const auto curr_time = Clock::Now();
  AccTargetPerCorridor output;

  // 1. Use object current motion state(position) to filter out some objects.
  const auto av_pos = Vec2d(plan_start_point.path_point().x(),
                            plan_start_point.path_point().y());
  const auto& av_heading = plan_start_point.path_point().theta();

  const auto av_box = ComputeAvBox(av_pos, av_heading, vehicle_geom);
  const auto av_front_center =
      ComputeAvFrontCenter(av_pos, av_heading, vehicle_geom);

  const auto av_front_center_sl = path_ff.XYToSL(av_front_center);
  const auto av_box_or = path_ff.QueryFrenetBoxAt(av_box);

  const double av_front_s =
      av_box_or.ok() ? av_box_or->s_max : av_front_center_sl.s;
  const auto potential_targets =
      FindPotentialTargets(st_traj_mgr, corridor, av_front_s,
                           /*enable_filter_oncoming_objects=*/true);

  // 2. Make leader/crossing leader, and cutin desicions.
  const auto leader_traj_ids = MakeLeaderDecisions(
      potential_targets, kAccTargetWithoutMapOverlapThreshold);

  // 3. Rebuild SpacetimeTrajectoryManager.
  const absl::flat_hash_set<std::string> targets_need_alignment = {};
  output.acc_st_traj_mgr =
      BuildSpacetimeTrajectoryManagerWithAlignmentAndRemoveLateralGap(
          targets_need_alignment, potential_targets, corridor, st_traj_mgr);
  auto target_decisions =
      BuildAccTargetDecisions(leader_traj_ids, *output.acc_st_traj_mgr);
  output.leaders = std::move(target_decisions.first);
  output.potential_targets = std::move(target_decisions.second);
  output.timestamp = curr_time;
  output.source = AccCorridorSource::ESTIMATED;

  return output;
}

}  // namespace qcraft::planner
