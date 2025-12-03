#include "onboard/planner/assist/lcc_map_builder.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

inline bool IsValidNeighborLaneLatOffset(double lat_offset, bool is_left) {
  constexpr double kMaxLatOffset = 6.5;  // m.
  if (is_left) {
    return lat_offset >= kMinLaneWidth && lat_offset <= kMaxLatOffset;
  } else {
    return lat_offset <= -kMinLaneWidth && lat_offset >= -kMaxLatOffset;
  }
}

bool IsLaterallyTooClose(double offset_a, double offset_b) {
  constexpr double kBoundLatOffsetEpsilon = 1.0;  // m.
  return std::fabs(offset_a - offset_b) < kBoundLatOffsetEpsilon;
}

int FindBoundaryNumInBetween(absl::Span<const StationBoundary> boundaries,
                             double lat_offset) {
  const auto [min_l, max_l] = std::minmax(0.0, lat_offset);
  double prev_bound_l = std::numeric_limits<double>::infinity();
  int bound_num = 0;
  for (const auto& bound : boundaries) {
    const double bound_l = bound.lat_offset;
    if (bound_l < min_l || bound_l > max_l ||
        IsLaterallyTooClose(bound_l, min_l) ||
        IsLaterallyTooClose(bound_l, max_l) ||
        IsLaterallyTooClose(bound_l, prev_bound_l)) {
      continue;
    }
    bound_num++;
    prev_bound_l = bound_l;
  }

  return bound_num;
}

absl::StatusOr<mapping::LanePath> FindNeighborLaneByDrivingMap(
    const PlannerSemanticMapManager& psmm, const DrivingMapTopo& driving_map,
    const DrivePassage& drive_passage, bool is_left) {
  constexpr double kValidNghbrLaneLen = 100.0;  // m.
  std::vector<mapping::LanePath> candidate_lane_paths;
  for (const auto start_id : driving_map.starting_lane_ids()) {
    const auto* start_lane = driving_map.GetLaneById(start_id);
    if (start_lane == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("FindNeighborLaneIdByDrivingMap: Can not find lane ",
                       start_id, " in driving map."));
    }

    ASSIGN_OR_CONTINUE(
        const auto lane_path,
        BuildLanePathFromData(
            mapping::LanePathData(start_lane->start_fraction,
                                  /*end_fraction=*/1.0, {start_id}),
            psmm));

    auto new_lane_path_vec =
        CollectAllLanePathFromStartLane(psmm, lane_path, kValidNghbrLaneLen);
    candidate_lane_paths.insert(
        candidate_lane_paths.end(),
        std::make_move_iterator(new_lane_path_vec.begin()),
        std::make_move_iterator(new_lane_path_vec.end()));
  }

  constexpr double kMinCheckPreviewLength = 5.0;  // m.
  constexpr double kMinCheckLaneLength = 10.0;    // m.
  constexpr double kCheckLaneLength = 40.0;       // m.
  constexpr double kSampleStep = 2.0;             // m.

  mapping::LanePath nghbr_lane_path;
  double min_end_lat_offset = std::numeric_limits<double>::infinity();
  for (const auto& lane_path : candidate_lane_paths) {
    const double start_check_s =
        std::clamp(lane_path.length() - kCheckLaneLength,
                   kMinCheckPreviewLength, kAlccPlcPreviewDistance);
    const double end_check_s =
        std::min(lane_path.length(), start_check_s + kCheckLaneLength);

    if (end_check_s - start_check_s < kMinCheckLaneLength) continue;

    const int start_idx = FloorToInt(start_check_s / kSampleStep);
    const int end_idx = FloorToInt(end_check_s / kSampleStep);
    double lat_offset = 0.0;
    int boundary_num = 0;
    for (int i = start_idx; i <= end_idx; ++i) {
      const auto xy = ArclengthToPos(psmm, lane_path, i * kSampleStep);
      const auto& station = drive_passage.FindNearestStation(xy);
      const double lat_l = station.lat_offset(xy);
      lat_offset += lat_l;

      if (boundary_num == 0) {
        boundary_num = FindBoundaryNumInBetween(station.boundaries(), lat_l);
      }
    }

    lat_offset /= (end_idx - start_idx + 1);

    const auto nghbr_end_point = ComputeLanePointPos(psmm, lane_path.back());
    ASSIGN_OR_CONTINUE(const auto end_sl,
                       drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                           nghbr_end_point));

    if (IsValidNeighborLaneLatOffset(lat_offset, is_left) &&
        std::fabs(end_sl.l) < min_end_lat_offset && boundary_num < 2) {
      nghbr_lane_path = lane_path;
      min_end_lat_offset = std::fabs(end_sl.l);
    }
  }

  if (!nghbr_lane_path.IsEmpty()) {
    return BuildLanePathFromData(
        mapping::LanePathData(nghbr_lane_path.start_fraction(),
                              /*end_fraction=*/1.0, nghbr_lane_path.lane_ids()),
        psmm);
  }

  return absl::NotFoundError(absl::StrCat(
      "Can not find ", (is_left ? "left" : "right"),
      " neighbor lane near lane:\t", drive_passage.lane_path().DebugString()));
}

absl::StatusOr<mapping::ElementId> FindOutgoingLaneWithMinimumHeadingDiff(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const mapping::ElementId> outgoing_lanes,
    mapping::ElementId id) {
  SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, id);

  constexpr double kSampleLen = 4.0;  // m.
  const Vec2d origin_pt = lane_info.points_smooth.back();
  const Vec2d prev_pt = lane_info.LerpPointFromFraction(
      std::max(0.0, 1.0 - kSampleLen / lane_info.length()));
  const double heading = (origin_pt - prev_pt).FastAngle();

  double min_heading_diff = std::numeric_limits<double>::infinity();
  mapping::ElementId outgoing_lane_id = mapping::kInvalidElementId;
  // Calculate proj value for each outgoing lane.
  for (auto temp_id : outgoing_lanes) {
    SMM_ASSIGN_LANE_OR_CONTINUE(temp_lane_info, psmm, temp_id);

    const Vec2d next_pt = temp_lane_info.LerpPointFromFraction(
        std::min(1.0, kSampleLen / temp_lane_info.length()));
    const double tmp_heading = (next_pt - origin_pt).FastAngle();
    const double heading_diff =
        std::fabs(NormalizeAngle(tmp_heading - heading));
    if (heading_diff < min_heading_diff) {
      outgoing_lane_id = temp_id;
      min_heading_diff = heading_diff;
    }
  }

  if (outgoing_lane_id == mapping::kInvalidElementId) {
    return absl::NotFoundError("Cannot find any outgoing lane.");
  }

  return outgoing_lane_id;
}

absl::StatusOr<mapping::LanePath> ExtendLanePathOnDrivingMap(
    const PlannerSemanticMapManager& psmm, const DrivingMapTopo& dm,
    const mapping::LanePath& lane_path) {
  std::vector<mapping::ElementId> lane_ids;
  double end_fraction = 1.0;
  for (const auto& seg : lane_path) {
    const auto* lane_ptr = dm.GetLaneById(seg.lane_id);
    if (lane_ptr == nullptr) {
      break;
    }

    lane_ids.push_back(seg.lane_id);

    end_fraction = std::min(lane_ptr->end_fraction, seg.end_fraction);
  }

  if (lane_ids.empty()) {
    return absl::InternalError("Should not happen.");
  }

  // Extend lane path on driving map as long as possible.
  auto* last_lane_ptr = dm.GetLaneById(lane_ids.back());
  if (last_lane_ptr == nullptr) {
    return absl::InternalError("Should not happen.");
  }

  while (!last_lane_ptr->outgoing_lane_ids.empty()) {
    ASSIGN_OR_BREAK(
        const auto next_id,
        FindOutgoingLaneWithMinimumHeadingDiff(
            psmm, last_lane_ptr->outgoing_lane_ids, last_lane_ptr->id));
    lane_ids.push_back(next_id);

    const auto* next_lane_ptr = dm.GetLaneById(next_id);
    if (next_lane_ptr == nullptr) {
      return absl::InternalError("Should not happen.");
    }

    end_fraction = next_lane_ptr->end_fraction;
    last_lane_ptr = next_lane_ptr;
  }

  return BuildLanePathFromData(
      mapping::LanePathData(lane_path.start_fraction(), end_fraction,
                            std::move(lane_ids)),
      psmm);
}

absl::StatusOr<mapping::LanePath> ComputeNeighborLanePath(
    const PlannerSemanticMapManager& psmm, const DrivingMapTopo& driving_map,
    const DrivePassage& drive_passage, bool is_left) {
  ASSIGN_OR_RETURN(
      const auto nghbr_lane,
      FindNeighborLaneByDrivingMap(psmm, driving_map, drive_passage, is_left));

  return ExtendLanePathOnDrivingMap(psmm, driving_map, nghbr_lane);
}

absl::StatusOr<std::array<mapping::LanePath, 3>> BuildLKLocalLaneMap(
    const PlannerSemanticMapManager& psmm, const DrivingMapTopo& driving_map,
    const mapping::LanePath& lane_path, double cut_off_length) {
  if (lane_path.IsEmpty()) {
    return absl::InvalidArgumentError(
        "Lane path is empty, build local lane map failed.");
  }

  ASSIGN_OR_RETURN(
      const auto drive_passage,
      BuildDrivePassageFromLanePath(
          psmm, lane_path,
          /*step_s=*/2.0, /*avoid_loop=*/false, /*backward_extend_len=*/0.0,
          /*required_planning_horizon=*/0.0, /*required_backward_len=*/0.0,
          /*override_speed_limit_mps=*/std::nullopt,
          /*type=*/FrenetFrameType::kKdTree,
          /*smooth_lane_path=*/false));

  // Get init neighbor lane path according driving map.
  auto left_lp_or = ComputeNeighborLanePath(psmm, driving_map, drive_passage,
                                            /*is_left=*/true);
  auto left_lane_path = left_lp_or.ok() && left_lp_or->length() > cut_off_length
                            ? std::move(*left_lp_or)
                            : mapping::LanePath();

  auto right_lp_or = ComputeNeighborLanePath(psmm, driving_map, drive_passage,
                                             /*is_left=*/false);
  auto right_lane_path =
      right_lp_or.ok() && right_lp_or->length() > cut_off_length
          ? std::move(*right_lp_or)
          : mapping::LanePath();

  return std::array<mapping::LanePath, 3>{std::move(left_lane_path), lane_path,
                                          std::move(right_lane_path)};
}

absl::StatusOr<std::array<mapping::LanePath, 3>> BuildLCLocalLaneMap(
    mapping::LanePath origin_lane_path, mapping::LanePath target_lane_path,
    LaneChangeDirection lc_direction) {
  switch (lc_direction) {
    case LaneChangeDirection::LCD_LEFT:
      return std::array<mapping::LanePath, 3>{
          std::move(target_lane_path), origin_lane_path, mapping::LanePath()};
    case LaneChangeDirection::LCD_RIGHT:
      return std::array<mapping::LanePath, 3>{mapping::LanePath(),
                                              std::move(origin_lane_path),
                                              std::move(target_lane_path)};

    case LaneChangeDirection::LCD_NONE:
      return absl::InternalError("should not reach here.");
  }
}

absl::StatusOr<std::array<mapping::LanePath, 3>> BuildReturnLocalLaneMap(
    mapping::LanePath origin_lane_path) {
  return std::array<mapping::LanePath, 3>{
      mapping::LanePath(), std::move(origin_lane_path), mapping::LanePath()};
}

}  // namespace

absl::StatusOr<mapping::LanePath> ProjectLanePathToDrivingMap(
    const mapping::LanePath& lane_path, const DrivingMapTopo& dm,
    const PlannerSemanticMapManager& psmm) {
  bool found_start_lane = false;
  mapping::LanePath aligned_lane_path;
  for (const auto id : dm.starting_lane_ids()) {
    // NOTE: starting lane must be found.
    if (id == lane_path.front().lane_id()) {
      found_start_lane = true;
    }

    const auto* lane_ptr = dm.GetLaneById(id);
    if (lane_ptr == nullptr) continue;

    const mapping::LanePoint lane_point =
        mapping::LanePoint(id, lane_ptr->start_fraction);
    if (lane_path.ContainsLanePoint(lane_point)) {
      aligned_lane_path = lane_path.AfterFirstOccurrenceOfLanePoint(lane_point);
      found_start_lane = true;
      break;
    }
  }

  if (!found_start_lane) {
    return absl::NotFoundError(absl::StrFormat(
        "start lanes [%s] can not be found on lane path %s.",
        absl::StrJoin(dm.starting_lane_ids(), ","), lane_path.DebugString()));
  }

  return ExtendLanePathOnDrivingMap(
      psmm, dm, aligned_lane_path.IsEmpty() ? lane_path : aligned_lane_path);
}

absl::StatusOr<LccDrivingMapUpdateResult> UpdateLccDrivingMapByOnlineMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& origin_lane_path,
    const mapping::LanePath& target_lane_path,
    const mapping::OnlineSemanticMapProto& online_map, const Vec2d& ego_pos) {
  ASSIGN_OR_RETURN(auto dm,
                   BuildDrivingMapByOnlineMap(psmm, online_map, ego_pos));

  mapping::LanePath aligned_origin_lane_path;
  if (!origin_lane_path.IsEmpty()) {
    ASSIGN_OR_RETURN(aligned_origin_lane_path,
                     ProjectLanePathToDrivingMap(origin_lane_path, dm, psmm));
  }

  mapping::LanePath aligned_target_lane_path;
  if (!target_lane_path.IsEmpty()) {
    ASSIGN_OR_RETURN(aligned_target_lane_path,
                     ProjectLanePathToDrivingMap(target_lane_path, dm, psmm));
  }

  return LccDrivingMapUpdateResult{
      .aligned_origin_lane_path = std::move(aligned_origin_lane_path),
      .aligned_target_lane_path = std::move(aligned_target_lane_path),
      .driving_map = std::move(dm)};
}

absl::StatusOr<LccDrivingMapUpdateResult> UpdateLccDrivingMapByOfflineMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& origin_lane_path,
    const mapping::LanePath& target_lane_path, const Vec2d& ego_pos) {
  const auto back_extend_origin_lp = BackwardExtendLanePath(
      psmm, origin_lane_path, kDrivePassageKeepBehindLength);

  ASSIGN_OR_RETURN(
      const auto ff,
      BuildBruteForceFrenetFrame(
          SampleLanePathPoints(psmm, back_extend_origin_lp.BeforeArclength(
                                         kDrivePassageKeepBehindLength +
                                         kMaxTravelDistanceBetweenFrames)),
          /*down_sample_raw_points=*/true));

  const auto sl = ff.XYToSL(ego_pos);

  if (sl.s < 0.0) {
    return absl::NotFoundError("Can not project ego pos to origin lane path.");
  }

  const auto proj_lane_path = back_extend_origin_lp.AfterArclength(sl.s);

  ASSIGN_OR_RETURN(
      auto final_origin_lane_path,
      ForwardExtendLanePathWithMinimumHeadingDiff(
          psmm, proj_lane_path,
          kAlccReferenceLineRequiredLength - proj_lane_path.length(),
          /*allow_virtual=*/true));

  ASSIGN_OR_RETURN(auto dm, BuildDrivingMapByRouteOnOfflineMap(
                                psmm, RouteSections::BuildFromLanePath(
                                          psmm, final_origin_lane_path)));

  mapping::LanePath final_target_lane_path;
  if (!target_lane_path.IsEmpty()) {
    ASSIGN_OR_RETURN(final_target_lane_path,
                     ProjectLanePathToDrivingMap(target_lane_path, dm, psmm));
  }

  return LccDrivingMapUpdateResult{
      .aligned_origin_lane_path = std::move(final_origin_lane_path),
      .aligned_target_lane_path = std::move(final_target_lane_path),
      .driving_map = std::move(dm)};
}

absl::StatusOr<std::array<mapping::LanePath, 3>> BuildLocalLaneMap(
    const BuildLocalMapInput& input) {
  const auto build_lk_map = [&input](const mapping::LanePath& lane_path)
      -> absl::StatusOr<std::array<mapping::LanePath, 3>> {
    return BuildLKLocalLaneMap(*input.psmm, *input.driving_map_topo, lane_path,
                               input.cut_off_length);
  };

  switch (input.alc_state) {
    case ALC_STANDBY_ENABLE:
    case ALC_PREPARE:
    case ALC_RETURN_COMPLETED: {
      return build_lk_map(*input.origin_lane_path);
    }

    case ALC_COMPLETED: {
      return build_lk_map(*input.target_lane_path);
    }

    case ALC_ONGOING:
    case ALC_CROSSING_LANE: {
      return BuildLCLocalLaneMap(*input.origin_lane_path,
                                 *input.target_lane_path, input.lc_direction);
    }

    case ALC_RETURNING: {
      return BuildReturnLocalLaneMap(*input.origin_lane_path);
    }
    case ALC_OFF:
    case ALC_STANDBY:
      return absl::InternalError("should not reach here.");
  }
}

}  // namespace qcraft::planner
