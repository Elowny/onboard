#include "onboard/planner/common/path_generator_util.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {

using mapping::LanePath;

constexpr double kBackwardExtendLength = 20.0;  // m.
constexpr double kDrivePassageStepS = 1.0;      // m.
constexpr double kBoundaryDistanceLimit = 2.5;  // m.

absl::StatusOr<mapping::LanePoint> FindOutgoingLanePointWithoutSpliting(
    const PlannerSemanticMapManager& psmm, mapping::ElementId id) {
  SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, id);
  auto target_lane_id = mapping::kInvalidElementId;
  for (const auto& lane_id : lane_info.outgoing_lanes()) {
    if (lane_id == mapping::kInvalidElementId) {
      continue;
    }
    const auto* outgoing_lane_info = psmm.FindLaneInfoOrNull(lane_id);
    if (nullptr == outgoing_lane_info ||
        (outgoing_lane_info->proto->has_is_virtual() &&
         outgoing_lane_info->proto->is_virtual())) {
      continue;
    }
    if (target_lane_id != mapping::kInvalidElementId) {
      return absl::NotFoundError(absl::StrCat(
          "Outgoing lane is not unqiue. Lane id:", lane_info.id.value(), "."));
    }
    target_lane_id = lane_id;
  }

  if (target_lane_id == mapping::kInvalidElementId) {
    return absl::NotFoundError(absl::StrCat(
        "Can not find outgoing lane. Lane id:", lane_info.id.value(), "."));
  }
  return mapping::LanePoint(target_lane_id, 0.0);
}

mapping::LanePath ForwardExtendLanePathUntilSpliting(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_length) {
  if (extend_length <= 0.0) return raw_lane_path;
  mapping::LanePoint start_lp = raw_lane_path.back();
  mapping::LanePath forward_path(psmm.semantic_map_manager(), start_lp);
  while (extend_length > 0.0) {
    if (start_lp.fraction() == 1.0) {
      ASSIGN_OR_BREAK(start_lp, FindOutgoingLanePointWithoutSpliting(
                                    psmm, start_lp.lane_id()));
    } else {
      SMM_ASSIGN_LANE_OR_BREAK_ISSUE(lane_info, psmm, start_lp.lane_id());
      const double len = lane_info.length();
      if (const double fraction = extend_length / len + start_lp.fraction();
          fraction < 1.0) {
        forward_path = forward_path.Connect(
            psmm.semantic_map_manager(),
            mapping::LanePath(psmm.semantic_map_manager(), {lane_info.id},
                              start_lp.fraction(), fraction));
        break;
      } else {
        extend_length -= len * (1.0 - start_lp.fraction());
        forward_path = forward_path.Connect(
            psmm.semantic_map_manager(),
            mapping::LanePath(psmm.semantic_map_manager(), {lane_info.id},
                              start_lp.fraction(), 1.0));
        start_lp = mapping::LanePoint(start_lp.lane_id(), 1.0);
      }
    }
  }
  return raw_lane_path.Connect(psmm.semantic_map_manager(), forward_path);
}

}  // namespace

bool IsPointInLeftLaneBoundary(const DrivePassage& drive_passage,
                               const Vec2d& point) {
  const auto frenet_or =
      drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(point);
  if (!frenet_or.ok()) return false;

  const auto boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(frenet_or->s);
  if (!boundary.left.has_value()) return false;

  const double left_boundary_lat_offset =
      boundary.left->type == StationBoundaryType::VIRTUAL_CURB
          ? std::min(boundary.left->lat_offset, kBoundaryDistanceLimit)
          : boundary.left->lat_offset;
  return frenet_or->l < left_boundary_lat_offset;
}

bool IsPointInRightLaneBoundary(const DrivePassage& drive_passage,
                                const Vec2d& point) {
  const auto frenet_or =
      drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(point);
  if (!frenet_or.ok()) return false;

  const auto boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(frenet_or->s);
  if (!boundary.right.has_value()) return false;

  const double right_boundary_lat_offset =
      boundary.right->type == StationBoundaryType::VIRTUAL_CURB
          ? std::max(boundary.right->lat_offset, -kBoundaryDistanceLimit)
          : boundary.right->lat_offset;
  return frenet_or->l > right_boundary_lat_offset;
}

bool IsPointInLaneBoundaries(const DrivePassage& drive_passage,
                             const Vec2d& point) {
  const auto frenet_or =
      drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(point);
  if (!frenet_or.ok()) return false;

  const auto boundary =
      drive_passage.QueryEnclosingLaneBoundariesAtS(frenet_or->s);
  if (!boundary.right.has_value() || !boundary.left.has_value()) {
    return false;
  }

  // Check if point is in left lane boundary.
  const double left_boundary_lat_offset =
      boundary.left->type == StationBoundaryType::VIRTUAL_CURB
          ? std::min(boundary.left->lat_offset, kBoundaryDistanceLimit)
          : boundary.left->lat_offset;
  if (frenet_or->l >= left_boundary_lat_offset) return false;
  // Check if point is in right lane boundary.
  const double right_boundary_lat_offset =
      boundary.right->type == StationBoundaryType::VIRTUAL_CURB
          ? std::max(boundary.right->lat_offset, -kBoundaryDistanceLimit)
          : boundary.right->lat_offset;
  return frenet_or->l > right_boundary_lat_offset;
}

std::optional<DrivePassage> BuildDrivePassageFromLaneId(
    const PlannerSemanticMapManager& psmm, const mapping::ElementId& lane_id,
    const PathPoint& start_path_point, double max_length) {
  const Vec2d start_point = ToVec2d(start_path_point);
  Vec2d closest_point;

  std::vector<mapping::ElementId> candidate_lanes{lane_id};
  // Prevent dead loop.
  constexpr int kMaxIterationNum = 10000;
  int iter = 0;
  while (iter < kMaxIterationNum) {
    ASSIGN_OR_BREAK(const auto lane_point, FindOutgoingLanePointWithoutSpliting(
                                               psmm, candidate_lanes.back()));
    candidate_lanes.push_back(lane_point.lane_id());
    ++iter;
  }
  const auto lane_point_or =
      FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
          psmm.GetLevel(), psmm, start_point, candidate_lanes,
          start_path_point.theta(),
          /*heading_penalty_weight=*/0.0, &closest_point,
          /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  if (lane_point_or.ok()) {
    const auto tmp_lane_path_or =
        BuildLanePathFromData(mapping::LanePathData(*lane_point_or), psmm);
    if (!tmp_lane_path_or.ok()) {
      return std::nullopt;
    }
    const auto lane_path =
        ForwardExtendLanePathUntilSpliting(psmm, *tmp_lane_path_or, max_length);
    auto dp_or = BuildDrivePassageFromLanePath(
        psmm, lane_path, kDrivePassageStepS,
        /*avoid_loop=*/true, kBackwardExtendLength,
        /*required_planning_horizon=*/0.0, /*required_backward_len=*/0.0,
        std::nullopt);
    if (!dp_or.ok()) return std::nullopt;
    return std::move(*dp_or);
  }
  return std::nullopt;
}

}  // namespace planner
}  // namespace qcraft
