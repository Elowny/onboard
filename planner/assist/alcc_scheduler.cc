#include "onboard/planner/assist/alcc_scheduler.h"

// IWYU pragma: no_include <cxxabi.h>  // for __forced_unwind
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"

#include "onboard/async/async_util.h"
#include "onboard/async/future.h"
#include "onboard/global/trace.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/assist/assist_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

DEFINE_bool(planner_alcc_smooth_lane_path, true,
            "Whether to smooth lane path when building drive passage.");

namespace qcraft::planner {

namespace {

struct AlcResult {
  const mapping::LanePath* origin_lane_path;
  const mapping::LanePath* target_lane_path;
  QALCState origin_alc_state;
  QALCState target_alc_state;
  LaneChangeDirection origin_lc_direction = LaneChangeDirection::LCD_NONE;
  LaneChangeDirection target_lc_direction = LaneChangeDirection::LCD_NONE;
  std::string reason;

  std::string DebugString() const {
    return absl::StrCat(
        "origin lane path:",
        origin_lane_path ? origin_lane_path->DebugString() : "null", "\n",
        "target lane path:",
        target_lane_path ? target_lane_path->DebugString() : "null", "\n",
        "origin alc state:", origin_alc_state, "\n",
        "target alc state:", target_alc_state, "\n",
        "origin lc direction:", origin_lc_direction, "\n",
        "target lc direction:", target_lc_direction);
  }
};

absl::Status CheckLanePathValidity(const mapping::LanePath& lane_path) {
  // TODO(jiayu): Check length, type, merging. May need to provide smm or lane
  // path info.

  if (lane_path.IsEmpty()) {
    return absl::NotFoundError("Empty target lane path.");
  }

  return absl::OkStatus();
}

AlcResult InitiateAlc(const std::array<mapping::LanePath, 3>& candidate_lanes,
                      DriverAction::LaneChangeCommand lc_cmd) {
  switch (lc_cmd) {
    case DriverAction::LC_CMD_NONE:
    case DriverAction::LC_CMD_CANCEL:
    case DriverAction::LC_CMD_STRAIGHT:  // Should not receive this here.
      return AlcResult{.origin_lane_path = &candidate_lanes[1],
                       .target_lane_path = nullptr,
                       .origin_alc_state = ALC_STANDBY_ENABLE};

    case DriverAction::LC_CMD_LEFT: {
      const auto status = CheckLanePathValidity(candidate_lanes[0]);

      if (status.ok()) {
        return AlcResult{.origin_lane_path = &candidate_lanes[1],
                         .target_lane_path = &candidate_lanes[0],
                         .origin_alc_state = ALC_PREPARE,
                         .target_alc_state = ALC_ONGOING,
                         .origin_lc_direction = LaneChangeDirection::LCD_NONE,
                         .target_lc_direction = LaneChangeDirection::LCD_LEFT};
      } else {
        return AlcResult{.origin_lane_path = &candidate_lanes[1],
                         .target_lane_path = nullptr,
                         .origin_alc_state = ALC_PREPARE,
                         .reason = std::string(status.message())};
      }
    }
    case DriverAction::LC_CMD_RIGHT: {
      const auto status = CheckLanePathValidity(candidate_lanes[2]);

      if (status.ok()) {
        return AlcResult{.origin_lane_path = &candidate_lanes[1],
                         .target_lane_path = &candidate_lanes[2],
                         .origin_alc_state = ALC_PREPARE,
                         .target_alc_state = ALC_ONGOING,
                         .origin_lc_direction = LaneChangeDirection::LCD_NONE,
                         .target_lc_direction = LaneChangeDirection::LCD_RIGHT};
      } else {
        return AlcResult{.origin_lane_path = &candidate_lanes[1],
                         .target_lane_path = nullptr,
                         .origin_alc_state = ALC_PREPARE,
                         .reason = std::string(status.message())};
      }
    }
  }
}

AlcResult PrepareAlc(const std::array<mapping::LanePath, 3>& candidate_lanes,
                     DriverAction::LaneChangeCommand lc_cmd) {
  switch (lc_cmd) {
    case DriverAction::LC_CMD_NONE:
    case DriverAction::LC_CMD_CANCEL:
    case DriverAction::LC_CMD_STRAIGHT:  // Should not receive this here.
      return AlcResult{.origin_lane_path = &candidate_lanes[1],
                       .target_lane_path = nullptr,
                       .origin_alc_state = ALC_STANDBY_ENABLE};

    case DriverAction::LC_CMD_LEFT: {
      const auto status = CheckLanePathValidity(candidate_lanes[0]);

      if (status.ok()) {
        return AlcResult{.origin_lane_path = &candidate_lanes[1],
                         .target_lane_path = &candidate_lanes[0],
                         .origin_alc_state = ALC_PREPARE,
                         .target_alc_state = ALC_ONGOING,
                         .origin_lc_direction = LaneChangeDirection::LCD_NONE,
                         .target_lc_direction = LaneChangeDirection::LCD_LEFT};
      } else {
        return AlcResult{.origin_lane_path = &candidate_lanes[1],
                         .target_lane_path = nullptr,
                         .origin_alc_state = ALC_PREPARE,
                         .reason = std::string(status.message())};
      }
    }
    case DriverAction::LC_CMD_RIGHT: {
      const auto status = CheckLanePathValidity(candidate_lanes[2]);

      if (status.ok()) {
        return AlcResult{.origin_lane_path = &candidate_lanes[1],
                         .target_lane_path = &candidate_lanes[2],
                         .origin_alc_state = ALC_PREPARE,
                         .target_alc_state = ALC_ONGOING,
                         .origin_lc_direction = LaneChangeDirection::LCD_NONE,
                         .target_lc_direction = LaneChangeDirection::LCD_RIGHT};
      } else {
        return AlcResult{.origin_lane_path = &candidate_lanes[1],
                         .target_lane_path = nullptr,
                         .origin_alc_state = ALC_PREPARE,
                         .reason = std::string(status.message())};
      }
    }
  }
}

AlcResult ContinueAlc(const std::array<mapping::LanePath, 3>& candidate_lanes,
                      DriverAction::LaneChangeCommand lc_cmd) {
  switch (lc_cmd) {
    case DriverAction::LC_CMD_NONE:
    case DriverAction::LC_CMD_CANCEL:
    case DriverAction::LC_CMD_STRAIGHT:  // Should not receive this here.
      return AlcResult{
          .origin_lane_path = &candidate_lanes[1],
          .target_lane_path = nullptr,
          .origin_alc_state = ALC_RETURNING,
      };

    case DriverAction::LC_CMD_LEFT: {
      return AlcResult{.origin_lane_path = &candidate_lanes[1],
                       .target_lane_path = &candidate_lanes[0],
                       .origin_alc_state = ALC_RETURNING,
                       .target_alc_state = ALC_ONGOING,
                       .target_lc_direction = LaneChangeDirection::LCD_LEFT};
    }
    case DriverAction::LC_CMD_RIGHT: {
      return AlcResult{.origin_lane_path = &candidate_lanes[1],
                       .target_lane_path = &candidate_lanes[2],
                       .origin_alc_state = ALC_RETURNING,
                       .target_alc_state = ALC_ONGOING,
                       .target_lc_direction = LaneChangeDirection::LCD_RIGHT};
    }
  }
}

AlcResult CrossingLaneAlc(
    const std::array<mapping::LanePath, 3>& candidate_lanes,
    QALCState prev_alc_state, bool left) {
  if (left) {
    return AlcResult{.origin_lane_path = nullptr,
                     .target_lane_path = &candidate_lanes[0],
                     .target_alc_state = prev_alc_state,
                     .target_lc_direction = LaneChangeDirection::LCD_LEFT};
  } else {
    return AlcResult{.origin_lane_path = nullptr,
                     .target_lane_path = &candidate_lanes[2],
                     .target_alc_state = prev_alc_state,
                     .target_lc_direction = LaneChangeDirection::LCD_RIGHT};
  }
}

AlcResult ProcessAlcRequest(
    const std::array<mapping::LanePath, 3>& candidate_lanes,
    DriverAction::LaneChangeCommand lc_cmd, QALCState prev_alc_state) {
  switch (prev_alc_state) {
    case ALC_STANDBY_ENABLE:
      return InitiateAlc(candidate_lanes, lc_cmd);

    case ALC_PREPARE:
    case ALC_RETURN_COMPLETED:
      return PrepareAlc(candidate_lanes, lc_cmd);

    case ALC_ONGOING:
      return ContinueAlc(candidate_lanes, lc_cmd);

    case ALC_COMPLETED:
      return AlcResult{.origin_lane_path = &candidate_lanes[1],
                       .target_lane_path = nullptr,
                       .origin_alc_state = ALC_STANDBY_ENABLE,
                       .target_lc_direction = LaneChangeDirection::LCD_NONE};

    case ALC_CROSSING_LANE:
      return CrossingLaneAlc(candidate_lanes, prev_alc_state,
                             lc_cmd == DriverAction::LC_CMD_LEFT);

    case ALC_OFF:
    case ALC_STANDBY:
    case ALC_RETURNING:
      return AlcResult{.origin_lane_path = &candidate_lanes[1],
                       .target_lane_path = nullptr,
                       .origin_alc_state = prev_alc_state};
  }
}

absl::Status ScheduleOnLanePath(
    const VehicleGeometryParamsProto& vehicle_geom, const Box2d& ego_box,
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const HistoryBufferAbslTime<PiecewiseLinearFunction<double>>&
        online_map_drift_buffer,
    const mapping::LanePath* lane_path, QALCState alc_state,
    LaneChangeDirection lc_direction,
    std::optional<double> lcc_cruising_speed_mps, AlccSchedulerOutput* output) {
  if (lane_path == nullptr || lane_path->IsEmpty()) {
    return absl::InternalError("empty lane path");
  }

  const double required_length =
      lcc_cruising_speed_mps.has_value()
          ? *lcc_cruising_speed_mps * kPlanningTimeHorizon
          : kAlccReferenceLineRequiredLength;
  ASSIGN_OR_RETURN(
      output->drive_passage,
      BuildDrivePassageFromLanePath(
          psmm, *lane_path,
          /*step_s=*/1.0, /*avoid_loop=*/true, kDrivePassageKeepBehindLength,
          required_length, kDrivePassageKeepBehindLength,
          lcc_cruising_speed_mps, FrenetFrameType::kQtfmKdTree,
          /*smooth_lane_path=*/FLAGS_planner_alcc_smooth_lane_path),
      _ << "Building drive passage failed.");
  ASSIGN_OR_RETURN(
      output->av_frenet_box_on_drive_passage,
      output->drive_passage.QueryFrenetBoxAt(ego_box),
      _ << "Ego box " << ego_box.DebugString() << " is out of drive passage!");

  if (std::fabs(output->av_frenet_box_on_drive_passage.center_l()) >
      2.0 * kMaxHalfLaneWidth) {
    return absl::InternalError("ego box is too far from target lane");
  }

  const bool lane_change =
      (alc_state == ALC_ONGOING || alc_state == ALC_CROSSING_LANE ||
       alc_state == ALC_RETURNING);

  const auto lc_state = CalculateLaneChangeState(
      output->av_frenet_box_on_drive_passage, alc_state, lc_direction);

  ASSIGN_OR_RETURN(
      output->sl_boundary,
      lane_change
          ? BuildPathBoundaryForLaneChange(
                psmm, output->drive_passage, plan_start_point, vehicle_geom,
                st_traj_mgr, lc_state, online_map_drift_buffer)
          : BuildPathBoundaryForLaneKeeping(psmm, output->drive_passage,
                                            plan_start_point, vehicle_geom,
                                            online_map_drift_buffer),
      _ << "Fail to build path boundary.");

  output->alc_state = alc_state;
  output->lc_direction = lc_direction;

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<std::vector<AlccSchedulerOutput>> RunAlccScheduler(
    const AlccSchedulerInput& input, ThreadPool* thread_pool) {
  SCOPED_QTRACE("Schedule ALCC Task");

  const auto alc_result = ProcessAlcRequest(*input.candidate_lanes,
                                            input.lc_cmd, input.prev_alc_state);

  const Vec2d ego_pos =
      Vec2dFromApolloTrajectoryPointProto(*input.plan_start_point);
  const Box2d ego_box =
      ComputeAvBox(ego_pos, input.plan_start_point->path_point().theta(),
                   *input.vehicle_geom);

  AlccSchedulerOutput origin_output;
  Future<absl::Status> future_origin_status =
      ScheduleFuture(thread_pool, [&]() {
        return ScheduleOnLanePath(
            *input.vehicle_geom, ego_box, *input.psmm, *input.plan_start_point,
            *input.st_traj_mgr, *input.online_map_drift_buffer,
            alc_result.origin_lane_path, alc_result.origin_alc_state,
            alc_result.origin_lc_direction, input.lcc_cruising_speed_mps,
            &origin_output);
      });

  AlccSchedulerOutput target_output;
  auto target_status = ScheduleOnLanePath(
      *input.vehicle_geom, ego_box, *input.psmm, *input.plan_start_point,
      *input.st_traj_mgr, *input.online_map_drift_buffer,
      alc_result.target_lane_path, alc_result.target_alc_state,
      alc_result.target_lc_direction, input.lcc_cruising_speed_mps,
      &target_output);

  auto origin_status = future_origin_status.Get();
  std::vector<AlccSchedulerOutput> res;
  res.reserve(2);
  if (origin_status.ok()) {
    res.emplace_back(std::move(origin_output));
  }
  if (target_status.ok()) {
    res.emplace_back(std::move(target_output));
  }
  if (res.empty()) {
    return absl::NotFoundError("Empty scheduler output.");
  }

  return res;
}

}  // namespace qcraft::planner
