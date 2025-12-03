#include "onboard/planner/mapless/mapless_scheduler.h"

#include <algorithm>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/status/status.h"

#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

absl::StatusOr<MaplessSchedulerOutput> ScheduleOnLanePath(
    const PlannerSemanticMapManager& psmm,
    const VehicleGeometryParamsProto& vehicle_geom, const Vec2d& ego_pos,
    const Box2d& ego_box, const ApolloTrajectoryPointProto& plan_start_point,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const mapping::LanePath& lane_path,
    const mapping::LanePath& prev_target_lane_path,
    const LaneChangeStateProto& prev_lc_state,
    std::optional<double> cruising_speed_mps, bool borrow,
    AutonomyStateProto::State autonomy_state) {
  if (lane_path.IsEmpty()) {
    return absl::InternalError("Empty lane path.");
  }

  // Drive passage.
  const double required_length =
      cruising_speed_mps.has_value()
          ? *cruising_speed_mps * kPlanningTimeHorizon
          : kMaplessReferenceLineRequiredLength;
  ASSIGN_OR_RETURN(
      auto drive_passage,
      BuildDrivePassageFromLanePath(
          psmm, lane_path,
          /*step_s=*/1.0, /*avoid_loop=*/true, kDrivePassageKeepBehindLength,
          required_length, kDrivePassageKeepBehindLength, cruising_speed_mps),
      _ << "Building drive passage failed.");

  ASSIGN_OR_RETURN(
      auto ego_frenet_box, drive_passage.QueryFrenetBoxAt(ego_box),
      _ << "Ego box " << ego_box.DebugString() << " is out of drive passage!");

  // Lane change state.
  const SmoothedReferenceLineResultMap empty_smooth_result_map;

  const double ref_center_l =
      CalcAvhRefCenterL(psmm, drive_passage, ego_frenet_box,
                        empty_smooth_result_map, /*should_smooth=*/false);
  ASSIGN_OR_RETURN(
      auto lc_state,
      MakeLaneChangeState(
          drive_passage, ego_pos, ego_frenet_box, prev_target_lane_path,
          /*prev_lane_path_before_lc_from_start=*/mapping::LanePath(),
          prev_lc_state, ref_center_l, autonomy_state),
      _ << "Making lane change state failed.");

  // Path boundary.
  ASSIGN_OR_RETURN(auto path_boundary,
                   BuildPathBoundaryFromPose(
                       psmm, drive_passage, plan_start_point, vehicle_geom,
                       st_traj_mgr, lc_state, empty_smooth_result_map, borrow,
                       /*should_smooth=*/false, /*unsafe_object_ids=*/{}),
                   _ << "Fail to build path boundary.");

  return MaplessSchedulerOutput{
      .drive_passage = std::move(drive_passage),
      .sl_boundary = std::move(path_boundary),
      .lane_change_state = std::move(lc_state),
      .av_frenet_box_on_drive_passage = ego_frenet_box,
      .borrow_lane = borrow};
}

}  // namespace

absl::StatusOr<std::vector<MaplessSchedulerOutput>> RunMaplessScheduler(
    const MaplessSchedulerInput& input, ThreadPool* thread_pool) {
  FUNC_QTRACE();

  const Vec2d ego_pos =
      Vec2dFromApolloTrajectoryPointProto(*input.plan_start_point);
  const Box2d ego_box =
      ComputeAvBox(ego_pos, input.plan_start_point->path_point().theta(),
                   *input.vehicle_geom);

  const auto& target_lp_vec = *input.target_lp_vec;
  const int target_lp_num = target_lp_vec.size();

  std::vector<MaplessSchedulerOutput> multi_tasks;
  multi_tasks.reserve(std::max(target_lp_num, 2));

  if (target_lp_num == 1) {
    const std::vector<bool> borrow_branches = std::vector<bool>{false, true};
    std::vector<absl::StatusOr<MaplessSchedulerOutput>> outputs(
        borrow_branches.size());

    ParallelFor(0, borrow_branches.size(), thread_pool, [&](int i) {
      outputs[i] = ScheduleOnLanePath(
          *input.psmm, *input.vehicle_geom, ego_pos, ego_box,
          *input.plan_start_point, *input.st_traj_mgr, target_lp_vec.front(),
          *input.prev_target_lane_path, *input.prev_lc_state,
          input.cruising_speed_mps, borrow_branches[i], input.autonomy_state);
    });

    for (int i = 0; i < outputs.size(); ++i) {
      if (outputs[i].ok()) {
        multi_tasks.emplace_back(std::move(outputs[i]).value());
      } else {
        QLOG(INFO) << "Building mapless scheduler output from "
                   << target_lp_vec.front().DebugString() << "("
                   << (borrow_branches[i] ? "borrow" : "no borrow")
                   << ") failed: " << outputs[i].status().message();
      }
    }
  } else {
    std::vector<absl::StatusOr<MaplessSchedulerOutput>> outputs(target_lp_num);

    ParallelFor(0, target_lp_num, thread_pool, [&](int i) {
      outputs[i] = ScheduleOnLanePath(
          *input.psmm, *input.vehicle_geom, ego_pos, ego_box,
          *input.plan_start_point, *input.st_traj_mgr, target_lp_vec[i],
          *input.prev_target_lane_path, *input.prev_lc_state,
          input.cruising_speed_mps, /*borrow=*/false, input.autonomy_state);
    });

    for (int i = 0; i < outputs.size(); ++i) {
      if (outputs[i].ok()) {
        multi_tasks.emplace_back(std::move(outputs[i]).value());
      } else {
        QLOG(INFO) << "Building mapless scheduler output from "
                   << target_lp_vec[i].DebugString()
                   << " failed: " << outputs[i].status().message();
      }
    }
  }

  if (multi_tasks.empty()) {
    return absl::NotFoundError("Empty mapless scheduler.");
  }

  return multi_tasks;
}

}  // namespace qcraft::planner
