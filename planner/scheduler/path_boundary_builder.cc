#include "onboard/planner/scheduler/path_boundary_builder.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/scheduler/path_boundary_builder_helper.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

constexpr double kMaxInnerBoundLaneChangeLatAccel = 0.09;  // m/s^2.
constexpr double kMaxOuterBoundLaneChangeLatAccel = 0.08;  // m/s^2.
constexpr double kMaxLaneChangeCancelLatAccel = 0.5;       // m/s^2.
constexpr double kExtendOuterBoundWidth = kDefaultHalfLaneWidth;
constexpr double kAutoCorrectionRatio = 5.0;
constexpr double kAutoCorrectionMaxLatOffset = 1.0;  // m.
constexpr double kAutoCorrectionMaxStep = 0.01;      // m.

PathBoundary ExtendBoundaryBy(const DrivePassage& drive_passage,
                              PathBoundary boundary, bool is_lane_changing,
                              bool lc_left, double extend_width) {
  FUNC_QTRACE();
  const double left_extend_path_boundary_width =
      is_lane_changing && lc_left ? 0.0 : extend_width;
  const double right_extend_path_boundary_width =
      is_lane_changing && !lc_left ? 0.0 : extend_width;

  // Get first uturn middle s.
  std::optional<double> uturn_middle_s;
  bool u_turn_passed = false;
  double u_turn_start_s = 0.0;
  double u_turn_end_s = 0.0;
  for (const auto& station : drive_passage.stations()) {
    if (station.direction() == mapping::LaneProto::UTURN) {
      if (!u_turn_passed) {
        u_turn_start_s = station.accumulated_s();
        u_turn_end_s = station.accumulated_s();
        u_turn_passed = true;
      } else {
        u_turn_end_s = station.accumulated_s();
      }
    } else if (u_turn_passed) {
      break;
    }
  }
  if (u_turn_passed) {
    uturn_middle_s = 0.5 * (u_turn_start_s + u_turn_end_s);
  }

  for (int i = 0; i < boundary.size(); ++i) {
    const auto& station = drive_passage.station(StationIndex(i));
    boundary.ShiftLeftByIndex(i, left_extend_path_boundary_width);
    if (station.direction() == mapping::LaneProto::UTURN &&
        uturn_middle_s.has_value() &&
        station.accumulated_s() < *uturn_middle_s) {
      continue;
    }
    boundary.ShiftRightByIndex(i, -right_extend_path_boundary_width);
  }

  return boundary;
}

std::vector<double> PostprocessOuterBoundary(absl::Span<const double> s_vec,
                                             absl::Span<const double> inner_vec,
                                             std::vector<double> outer_vec) {
  FUNC_QTRACE();
  constexpr double kEpsilon = 0.1;
  constexpr double kMinContinuousExtendedBoundLength = 3.0;  // m.
  int first_extended_idx = -1;
  for (int i = 1; i < inner_vec.size(); ++i) {
    bool curr_is_extended = std::fabs(inner_vec[i] - outer_vec[i]) >
                            (kExtendOuterBoundWidth - kEpsilon);
    bool prev_is_extended = std::fabs(inner_vec[i - 1] - outer_vec[i - 1]) >
                            (kExtendOuterBoundWidth - kEpsilon);
    if (first_extended_idx == -1) {
      if (curr_is_extended && !prev_is_extended) {
        first_extended_idx = i;
      }
    } else if (!curr_is_extended) {
      if ((s_vec[i - 1] - s_vec[first_extended_idx]) <=
          kMinContinuousExtendedBoundLength) {
        for (int j = first_extended_idx; j < i; ++j) {
          outer_vec[j] = outer_vec[first_extended_idx - 1];
        }
      }
      first_extended_idx = -1;
    }
  }

  return outer_vec;
}

}  // namespace

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromDrivePassage(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage) {
  FUNC_QTRACE();
  const int n = drive_passage.size();
  std::vector<double> s_vec, center_l(n, 0.0);
  s_vec.reserve(n);
  for (const auto& station : drive_passage.stations()) {
    s_vec.push_back(station.accumulated_s());
  }
  auto inner_boundary = BuildPathBoundaryFromTargetLane(
      psmm, drive_passage, 0.5 * kMinLaneWidth, /*borrow_lane_boundary=*/false);

  const auto curb_boundary = BuildCurbPathBoundary(drive_passage);
  inner_boundary.OuterClampBy(curb_boundary);
  PathBoundary outer_boundary = inner_boundary;

  return BuildPathSlBoundary(drive_passage, std::move(s_vec),
                             std::move(center_l), std::move(inner_boundary),
                             std::move(outer_boundary));
}

absl::StatusOr<PathSlBoundary> BuildPathBoundaryForLaneKeeping(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const HistoryBufferAbslTime<PiecewiseLinearFunction<double>>&
        online_map_drift_buffer) {
  FUNC_QTRACE();

  const int n = drive_passage.size();
  std::vector<double> s_vec, center_l;
  s_vec.reserve(n);
  center_l.reserve(n);
  for (const auto& station : drive_passage.stations()) {
    s_vec.push_back(station.accumulated_s());

    double avg_l = 0.0;
    if (!online_map_drift_buffer.empty() && s_vec.back() >= 0.0) {
      for (const auto& [_, plf] : online_map_drift_buffer) {
        avg_l += plf(station.accumulated_s());
      }
      avg_l /= online_map_drift_buffer.size();
    }

    double prev_l = center_l.empty() ? 0.0 : center_l.back();
    avg_l =
        std::clamp(avg_l * kAutoCorrectionRatio, -kAutoCorrectionMaxLatOffset,
                   kAutoCorrectionMaxLatOffset);
    avg_l = std::clamp(avg_l, prev_l - kAutoCorrectionMaxStep,
                       prev_l + kAutoCorrectionMaxStep);
    center_l.push_back(avg_l);
  }

  const double half_av_width = vehicle_geom.width() * 0.5;

  auto boundary = psmm.IsOnVisionMap() ? BuildPathBoundaryFromOnlineTargetLane(
                                             psmm, drive_passage, half_av_width)
                                       : BuildPathBoundaryFromTargetLane(
                                             psmm, drive_passage, half_av_width,
                                             /*borrow_lane_boundary=*/false);

  const Box2d ego_box =
      ComputeAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
                   plan_start_point.path_point().theta(), vehicle_geom);

  ASSIGN_OR_RETURN(const auto sl_box, drive_passage.QueryFrenetBoxAt(ego_box),
                   _ << "BuildPathBoundaryForLaneKeeping: Fail to project ego "
                        "box on drive passage.");

  ASSIGN_OR_RETURN(const auto cur_sl,
                   drive_passage.QueryFrenetCoordinateAt(
                       Vec2dFromApolloTrajectoryPointProto(plan_start_point)),
                   _ << "BuildPathBoundaryForLaneKeeping: Fail to project ego "
                        "pos on drive passage.");

  // Kinematic boundaries assuming constant lateral acceleration.
  const auto kinematic_boundary = BuildPathBoundaryFromAvKinematics(
      drive_passage, plan_start_point, vehicle_geom, cur_sl, sl_box, s_vec,
      /*target_lane_offset=*/0.0, kMaxLaneChangeCancelLatAccel,
      /*lane_change_pause=*/false);
  PathBoundary outer_boundary = boundary;
  outer_boundary.InnerClampBy(kinematic_boundary);

  // Add ego protection buffer
  const auto protect_boundary = BuildPathBoundaryFromEgoProtectBuffer(
      drive_passage, plan_start_point, vehicle_geom);
  outer_boundary.InnerClampBy(protect_boundary);

  const auto curb_boundary = BuildCurbPathBoundary(drive_passage);
  boundary.OuterClampBy(curb_boundary);
  outer_boundary.OuterClampBy(curb_boundary);

  // Post-process center_l to make sure it is within path boundary.
  for (int i = 0; i < n; ++i) {
    if (boundary.left(i) - boundary.right(i) <= vehicle_geom.width()) {
      if (i <= 1) {
        return absl::InternalError("Path boundary only has one point.");
      }
      s_vec.resize(i);
      center_l.resize(i);
      boundary.EraseFrom(i);
      outer_boundary.EraseFrom(i);
      break;
    } else {
      center_l[i] = std::clamp(center_l[i], boundary.right(i) + half_av_width,
                               boundary.left(i) - half_av_width);
    }
  }

  return BuildPathSlBoundary(drive_passage, std::move(s_vec),
                             std::move(center_l), std::move(boundary),
                             std::move(outer_boundary));
}

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromPose(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const LaneChangeStateProto& lc_state,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    bool borrow_lane_boundary, bool should_smooth,
    const absl::flat_hash_set<std::string>& unsafe_object_ids) {
  FUNC_QTRACE();
  const Box2d ego_box =
      ComputeAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
                   plan_start_point.path_point().theta(), vehicle_geom);

  ASSIGN_OR_RETURN(const auto sl_box, drive_passage.QueryFrenetBoxAt(ego_box),
                   _ << "BuildPathBoundaryFromPose: Fail to project ego box on "
                        "drive passage.");

  ASSIGN_OR_RETURN(const auto cur_sl,
                   drive_passage.QueryFrenetCoordinateAt(
                       Vec2dFromApolloTrajectoryPointProto(plan_start_point)),
                   _ << "BuildPathBoundaryFromPose: Fail to project ego pos on "
                        "drive passage.");

  const int n = drive_passage.stations().size();
  std::vector<double> s_vec;
  s_vec.reserve(n);
  for (const auto& station : drive_passage.stations()) {
    s_vec.push_back(station.accumulated_s());
  }

  std::vector<double> center_l(n, 0.0);
  if (should_smooth) {
    center_l =
        ComputeSmoothedReferenceLine(psmm, drive_passage, smooth_result_map);
  }

  const auto target_lane_offset = ComputeTargetLaneOffset(
      drive_passage, cur_sl, lc_state, st_traj_mgr, unsafe_object_ids,
      plan_start_point, vehicle_geom.width() * 0.5);
  bool lane_change_pause = lc_state.stage() == LaneChangeStage::LCS_PAUSE;
  if (lane_change_pause) {
    const auto cur_station_index =
        drive_passage
            .FindNearestStationIndex(
                Vec2dFromApolloTrajectoryPointProto(plan_start_point))
            .value();
    const auto smoothed_center_offset =
        target_lane_offset - center_l[cur_station_index];
    for (int i = 0; i < n; ++i) {
      center_l[i] += smoothed_center_offset;
    }
  }

  auto boundary = BuildPathBoundaryFromTargetLane(
      psmm, drive_passage, 0.5 * kMinLaneWidth, borrow_lane_boundary);

  const auto solid_boundary = BuildSolidPathBoundary(
      drive_passage, cur_sl, vehicle_geom, target_lane_offset);

  const auto curb_boundary = BuildCurbPathBoundary(drive_passage);

  const bool is_lane_changing =
      (lc_state.stage() == LaneChangeStage::LCS_EXECUTING ||
       lc_state.stage() == LaneChangeStage::LCS_RETURN) &&
      !lc_state.entered_target_lane();

  // Kinematic boundaries assuming constant lateral acceleration.
  const double inner_bound_lat_acc =
      lc_state.stage() == LaneChangeStage::LCS_EXECUTING
          ? kMaxInnerBoundLaneChangeLatAccel
          : kMaxLaneChangeCancelLatAccel;
  const double outer_bound_lat_acc =
      lc_state.stage() == LaneChangeStage::LCS_EXECUTING
          ? kMaxOuterBoundLaneChangeLatAccel
          : kMaxLaneChangeCancelLatAccel;
  const auto inner_kinematic_boundary = BuildPathBoundaryFromAvKinematics(
      drive_passage, plan_start_point, vehicle_geom, cur_sl, sl_box, s_vec,
      target_lane_offset, inner_bound_lat_acc, lane_change_pause);
  const auto outer_kinematic_boundary = BuildPathBoundaryFromAvKinematics(
      drive_passage, plan_start_point, vehicle_geom, cur_sl, sl_box, s_vec,
      target_lane_offset, outer_bound_lat_acc, lane_change_pause);
  PathBoundary outer_boundary = boundary;
  outer_boundary.InnerClampBy(outer_kinematic_boundary);

  // Extend path boundary by half lane width.
  if (!(borrow_lane_boundary || lane_change_pause)) {
    const bool lc_left = lc_state.lc_left();
    outer_boundary =
        ExtendBoundaryBy(drive_passage, std::move(outer_boundary),
                         is_lane_changing, lc_left, kExtendOuterBoundWidth);

    if (is_lane_changing) {
      boundary.InnerClampBy(outer_boundary);
    }
  }

  // Shrink when lane change pause.
  if (lane_change_pause) {
    outer_boundary = ShrinkPathBoundaryForLaneChangePause(
        vehicle_geom, sl_box, lc_state, std::move(outer_boundary),
        target_lane_offset);
    outer_boundary.InnerClampBy(inner_kinematic_boundary);
    boundary.OuterClampBy(outer_boundary);
  }

  // Clamp boundary by curb and solid line.
  if (!borrow_lane_boundary) {
    boundary.OuterClampBy(solid_boundary);
    outer_boundary.OuterClampBy(solid_boundary);
    outer_boundary.InnerClampBy(outer_kinematic_boundary);
    if (is_lane_changing || lane_change_pause) {
      boundary.InnerClampBy(inner_kinematic_boundary);
    }
  }
  boundary.OuterClampBy(curb_boundary);
  outer_boundary.OuterClampBy(curb_boundary);

  // Post-process center_l to make sure it is within path boundary.
  for (int i = 0; i < n; ++i) {
    if (boundary.left(i) - boundary.right(i) <= vehicle_geom.width()) {
      if (i <= 1) {
        return absl::InternalError("Path boundary only has one point.");
      }
      s_vec.resize(i);
      center_l.resize(i);
      boundary.EraseFrom(i);
      outer_boundary.EraseFrom(i);
      break;
    } else {
      center_l[i] = std::clamp(center_l[i],
                               boundary.right(i) + vehicle_geom.width() * 0.5,
                               boundary.left(i) - vehicle_geom.width() * 0.5);
    }
  }

  // Shrink to fit objects.
  std::vector<Vec2d> center_xy;
  for (int i = 0; i < boundary.size(); ++i) {
    const auto& station = drive_passage.station(StationIndex(i));
    center_xy.push_back(station.lat_point(center_l[i]));
  }
  if (!is_lane_changing) {
    outer_boundary = ShrinkPathBoundaryForObject(
        drive_passage, st_traj_mgr, plan_start_point, s_vec, center_l,
        center_xy, boundary, curb_boundary, std::move(outer_boundary));
    outer_boundary.InnerClampBy(outer_kinematic_boundary);
    outer_boundary.OuterClampBy(curb_boundary);
  }

  // Post-process outer boundary to avoid zigzags.
  outer_boundary =
      PathBoundary(PostprocessOuterBoundary(s_vec, boundary.right_vec(),
                                            outer_boundary.right_vec()),
                   PostprocessOuterBoundary(s_vec, boundary.left_vec(),
                                            outer_boundary.left_vec()));
  outer_boundary.InnerClampBy(boundary);

  return BuildPathSlBoundary(drive_passage, std::move(s_vec),
                             std::move(center_l), std::move(boundary),
                             std::move(outer_boundary));
}

// TODO(zixuan): Try to combine 2 alcc path boundary builders.
absl::StatusOr<PathSlBoundary> BuildPathBoundaryForLaneChange(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const LaneChangeStateProto& lc_state,
    const HistoryBufferAbslTime<PiecewiseLinearFunction<double>>&
        online_map_drift_buffer) {
  FUNC_QTRACE();
  const Box2d ego_box =
      ComputeAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
                   plan_start_point.path_point().theta(), vehicle_geom);

  ASSIGN_OR_RETURN(const auto sl_box, drive_passage.QueryFrenetBoxAt(ego_box),
                   _ << "BuildPathBoundaryFromPose: Fail to project ego box on "
                        "drive passage.");

  ASSIGN_OR_RETURN(const auto cur_sl,
                   drive_passage.QueryFrenetCoordinateAt(
                       Vec2dFromApolloTrajectoryPointProto(plan_start_point)),
                   _ << "BuildPathBoundaryFromPose: Fail to project ego pos on "
                        "drive passage.");

  const int n = drive_passage.stations().size();

  const bool is_lane_changing =
      lc_state.stage() == LaneChangeStage::LCS_EXECUTING ||
      lc_state.stage() == LaneChangeStage::LCS_RETURN;
  const bool lane_change_pause = lc_state.stage() == LaneChangeStage::LCS_PAUSE;

  double target_lane_offset = 0.0;
  if (is_lane_changing && !lc_state.entered_target_lane()) {
    constexpr double kMoveCenterDist = 0.2;  // m.;
    target_lane_offset =
        lc_state.lc_left() ? -kMoveCenterDist : kMoveCenterDist;
  } else if (lane_change_pause) {
    target_lane_offset = ComputeTargetLaneOffset(
        drive_passage, cur_sl, lc_state, st_traj_mgr, /*unsafe_object_ids=*/{},
        plan_start_point, vehicle_geom.width() * 0.5);
  }

  std::vector<double> s_vec, center_l;
  s_vec.reserve(n);
  center_l.reserve(n);
  for (const auto& station : drive_passage.stations()) {
    s_vec.push_back(station.accumulated_s());

    double avg_l = 0.0;
    if (!online_map_drift_buffer.empty() && s_vec.back() >= 0.0) {
      for (const auto& [_, plf] : online_map_drift_buffer) {
        avg_l += plf(station.accumulated_s());
      }
      avg_l /= online_map_drift_buffer.size();
    }

    double prev_l = center_l.empty() ? target_lane_offset : center_l.back();
    avg_l = target_lane_offset + std::clamp(avg_l * kAutoCorrectionRatio,
                                            -kAutoCorrectionMaxLatOffset,
                                            kAutoCorrectionMaxLatOffset);
    avg_l = std::clamp(avg_l, prev_l - kAutoCorrectionMaxStep,
                       prev_l + kAutoCorrectionMaxStep);
    center_l.push_back(avg_l);
  }

  const double half_av_width = vehicle_geom.width() * 0.5;
  auto boundary = psmm.IsOnVisionMap() ? BuildPathBoundaryFromOnlineTargetLane(
                                             psmm, drive_passage, half_av_width)
                                       : BuildPathBoundaryFromTargetLane(
                                             psmm, drive_passage, half_av_width,
                                             /*borrow_lane_boundary=*/false);

  const auto solid_boundary = BuildSolidPathBoundary(
      drive_passage, cur_sl, vehicle_geom, target_lane_offset);

  const auto curb_boundary = BuildCurbPathBoundary(drive_passage);

  // Kinematic boundaries assuming constant lateral acceleration.
  const double inner_bound_lat_acc =
      lc_state.stage() == LaneChangeStage::LCS_EXECUTING
          ? kMaxInnerBoundLaneChangeLatAccel
          : kMaxLaneChangeCancelLatAccel;
  const double outer_bound_lat_acc =
      lc_state.stage() == LaneChangeStage::LCS_EXECUTING
          ? kMaxOuterBoundLaneChangeLatAccel
          : kMaxLaneChangeCancelLatAccel;
  const auto inner_kinematic_boundary = BuildPathBoundaryFromAvKinematics(
      drive_passage, plan_start_point, vehicle_geom, cur_sl, sl_box, s_vec,
      target_lane_offset, inner_bound_lat_acc, lane_change_pause);
  const auto outer_kinematic_boundary = BuildPathBoundaryFromAvKinematics(
      drive_passage, plan_start_point, vehicle_geom, cur_sl, sl_box, s_vec,
      target_lane_offset, outer_bound_lat_acc, lane_change_pause);
  PathBoundary outer_boundary = boundary;
  outer_boundary.InnerClampBy(outer_kinematic_boundary);

  // Extend path boundary by half lane width.
  if (!lane_change_pause) {
    // Add ego protection buffer
    const auto protect_boundary = BuildPathBoundaryFromEgoProtectBuffer(
        drive_passage, plan_start_point, vehicle_geom);
    outer_boundary.InnerClampBy(protect_boundary);

    const bool lc_left = lc_state.lc_left();
    outer_boundary =
        ExtendBoundaryBy(drive_passage, std::move(outer_boundary),
                         is_lane_changing, lc_left, kExtendOuterBoundWidth);

    if (is_lane_changing) {
      boundary.InnerClampBy(outer_boundary);
    }
  }

  // Shrink when lane change pause.
  if (lane_change_pause) {
    outer_boundary = ShrinkPathBoundaryForLaneChangePause(
        vehicle_geom, sl_box, lc_state, std::move(outer_boundary),
        target_lane_offset);
    outer_boundary.InnerClampBy(inner_kinematic_boundary);
    boundary.OuterClampBy(outer_boundary);
  }

  // Clamp boundary by curb and solid line.
  boundary.OuterClampBy(solid_boundary);
  outer_boundary.OuterClampBy(solid_boundary);
  outer_boundary.InnerClampBy(outer_kinematic_boundary);
  if (is_lane_changing || lane_change_pause) {
    boundary.InnerClampBy(inner_kinematic_boundary);
  }
  boundary.OuterClampBy(curb_boundary);
  outer_boundary.OuterClampBy(curb_boundary);

  // Post-process center_l to make sure it is within path boundary.
  for (int i = 0; i < n; ++i) {
    if (boundary.left(i) - boundary.right(i) <= vehicle_geom.width()) {
      if (i <= 1) {
        return absl::InternalError("Path boundary only has one point.");
      }
      s_vec.resize(i);
      center_l.resize(i);
      boundary.EraseFrom(i);
      outer_boundary.EraseFrom(i);
      break;
    } else {
      center_l[i] = std::clamp(center_l[i], boundary.right(i) + half_av_width,
                               boundary.left(i) - half_av_width);
    }
  }

  // Shrink to fit objects.
  std::vector<Vec2d> center_xy;
  for (int i = 0; i < boundary.size(); ++i) {
    const auto& station = drive_passage.station(StationIndex(i));
    center_xy.push_back(station.lat_point(center_l[i]));
  }
  if (!is_lane_changing) {
    outer_boundary = ShrinkPathBoundaryForObject(
        drive_passage, st_traj_mgr, plan_start_point, s_vec, center_l,
        center_xy, boundary, curb_boundary, std::move(outer_boundary));
    outer_boundary.InnerClampBy(outer_kinematic_boundary);
    outer_boundary.OuterClampBy(curb_boundary);
  }

  // Post-process outer boundary to avoid zigzags.
  outer_boundary =
      PathBoundary(PostprocessOuterBoundary(s_vec, boundary.right_vec(),
                                            outer_boundary.right_vec()),
                   PostprocessOuterBoundary(s_vec, boundary.left_vec(),
                                            outer_boundary.left_vec()));
  outer_boundary.InnerClampBy(boundary);

  return BuildPathSlBoundary(drive_passage, std::move(s_vec),
                             std::move(center_l), std::move(boundary),
                             std::move(outer_boundary));
}

}  // namespace qcraft::planner
