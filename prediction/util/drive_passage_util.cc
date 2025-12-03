#include "onboard/prediction/util/drive_passage_util.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/utils/status_macros.h"
namespace qcraft {
namespace prediction {
mapping::LanePath PruneLanePathByLength(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const mapping::LanePath& lane_path, const mapping::LanePoint& lane_point,
    double front_length, double back_length) {
  const int lane_size = lane_path.lane_ids().size();
  double front_cut_len = front_length, back_cut_len = back_length;

  int lane_index = -1;

  for (int i = 0; i < lane_size; ++i) {
    if (lane_path.lane_id(i) == lane_point.lane_id()) {
      lane_index = i;
      break;
    }
  }
  QCHECK_GT(lane_index, -1);

  double start_fraction = 0.0, end_fraction = 1.0;
  int start_lane_index = 0, end_lane_index = lane_size - 1;

  // Find start lane and start fraction
  for (int i = lane_index; i >= 0; --i) {
    const auto* lane_info_ptr =
        semantic_map_mgr.FindLaneInfoOrNull(lane_path.lane_id(i));
    if (lane_info_ptr == nullptr) return lane_path;
    const double lane_length = lane_info_ptr->length();
    if (lane_length < kEpsilon) continue;
    if (i == lane_index) {
      // Find start lane in point lane
      const double lane_back_length = lane_point.fraction() * lane_length;
      if (lane_back_length < back_cut_len) {
        back_cut_len -= lane_back_length;
      } else {
        start_fraction = (lane_back_length - back_cut_len) / lane_length;
        start_lane_index = i;
        break;
      }
    } else {
      // Find start lane in back lane
      if (lane_length < back_cut_len) {
        back_cut_len -= lane_length;
      } else {
        start_fraction = (lane_length - back_cut_len) / lane_length;
        start_lane_index = i;
        break;
      }
    }
  }

  // Find end lane and end fraction
  for (int i = lane_index; i < lane_size; ++i) {
    const auto* lane_info_ptr =
        semantic_map_mgr.FindLaneInfoOrNull(lane_path.lane_id(i));
    if (lane_info_ptr == nullptr) return lane_path;
    const double lane_length = lane_info_ptr->length();
    if (lane_length < kEpsilon) continue;
    if (i == lane_index) {
      // Find end lane in point lane
      const double lane_front_length =
          (1.0 - lane_point.fraction()) * lane_length;
      const double lane_back_length = lane_length - lane_front_length;
      if (lane_front_length < front_cut_len) {
        front_cut_len -= lane_front_length;
      } else {
        end_fraction = (lane_back_length + front_cut_len) / lane_length;
        end_lane_index = i;
        break;
      }
    } else {
      // Find end lane in front lane
      if (lane_length < front_cut_len) {
        front_cut_len -= lane_length;
      } else {
        end_fraction = front_cut_len / lane_length;
        end_lane_index = i;
        break;
      }
    }
  }

  start_fraction = std::clamp(start_fraction, 0.0, 1.0);
  end_fraction = std::clamp(end_fraction, 0.0, 1.0);

  std::vector<mapping::ElementId> lane_ids;
  for (int i = start_lane_index; i <= end_lane_index; ++i) {
    lane_ids.push_back(lane_path.lane_id(i));
  }
  return mapping::LanePath(semantic_map_mgr.semantic_map_manager(), lane_ids,
                           start_fraction, end_fraction);
}

std::vector<mapping::LanePath> FilterLanePathByDistance(
    const std::vector<mapping::LanePath>& lane_paths,
    const planner::PlannerSemanticMapManager& psmm, const Vec2d& pos,
    double heading, int max_num) {
  std::vector<std::pair<int, double>> idx_to_dist_map;
  idx_to_dist_map.reserve(lane_paths.size());
  for (int i = 0; i < lane_paths.size(); ++i) {
    const auto& lane_path = lane_paths[i];
    const auto closest_lane_point_or = planner::
        FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
            psmm.GetLevel(), psmm, pos, lane_path, heading);
    if (closest_lane_point_or.ok()) {
      idx_to_dist_map.push_back(std::make_pair(
          i,
          pos.DistanceTo(ComputeLanePointPos(psmm, *closest_lane_point_or))));
    }
  }

  std::sort(
      idx_to_dist_map.begin(), idx_to_dist_map.end(),
      [](const std::pair<int, double>& x, const std::pair<int, double>& y) {
        return x.second < y.second;
      });

  std::vector<mapping::LanePath> filtered_lane_paths;
  filtered_lane_paths.reserve(max_num);
  for (int i = 0; i < std::min<int>(max_num, idx_to_dist_map.size()); ++i) {
    if (std::fabs(idx_to_dist_map[i].second) > 3.0 * kDefaultHalfLaneWidth) {
      continue;
    }
    filtered_lane_paths.push_back(lane_paths[idx_to_dist_map[i].first]);
  }
  return filtered_lane_paths;
}

absl::StatusOr<planner::DrivePassage> BuildAvDrivePassageWithNearestLane(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const planner::LaneBoundaryCache& lane_boundary_cache, const Vec2d& av_pos,
    double av_heading, double max_heading_diff, double backward_extend_len,
    double front_length, double back_length) {
  const auto lane_id_or =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          semantic_map_mgr, av_pos, av_heading,
          /*boundary_distance_limit=*/0.0, max_heading_diff);

  const auto lps =
      SearchLanePath(av_pos, semantic_map_mgr, *lane_id_or, front_length,
                     /*is_reverse_driving=*/false);
  if (lps.empty()) {
    return absl::FailedPreconditionError("No enough lane path.");
  }
  const auto closest_lane_point_or = planner::
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          semantic_map_mgr.GetLevel(), semantic_map_mgr, av_pos, lps[0],
          /*heading=*/M_PI);
  if (!closest_lane_point_or.ok()) {
    return absl::FailedPreconditionError("No valid closest lane point.");
  }
  const auto pruned_lane_path =
      PruneLanePathByLength(semantic_map_mgr, lps[0], *closest_lane_point_or,
                            front_length, back_length);
  const double current_backward_len =
      pruned_lane_path.FirstOccurrenceOfLanePointToArclength(
          *closest_lane_point_or);

  return planner::BuildDrivePassageForPredictionWithLaneBoundaryCache(
      semantic_map_mgr, pruned_lane_path, lane_boundary_cache, /*step_s=*/3.0,
      /*avoid_loop=*/true,
      std::max(backward_extend_len - current_backward_len, 0.0),
      FrenetFrameType::kKdTree);
}

absl::StatusOr<planner::DrivePassage> BuildAvDrivePassageWithRouting(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const planner::LaneBoundaryCache& lane_boundary_cache,
    const planner::RouteSections& sections, const Vec2d& query_point,
    double backward_extend_len, double front_length, double back_length) {
  ASSIGN_OR_RETURN(const auto clamp_sections,
                   ClampRouteSectionsBeforeArcLength(semantic_map_mgr, sections,
                                                     front_length));

  const auto nearest_lane_path =
      FindClosestLanePathOnRouteSectionsToSmoothPoint(
          semantic_map_mgr, clamp_sections, query_point);

  if (!nearest_lane_path.ok()) {
    return absl::FailedPreconditionError("No valid nearest lane path.");
  }
  const auto closest_lane_point_or = planner::
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          semantic_map_mgr.GetLevel(), semantic_map_mgr, query_point,
          *nearest_lane_path, /*heading=*/M_PI);
  if (!closest_lane_point_or.ok()) {
    return absl::FailedPreconditionError("No valid closest lane point.");
  }
  const auto pruned_lane_path =
      PruneLanePathByLength(semantic_map_mgr, *nearest_lane_path,
                            *closest_lane_point_or, front_length, back_length);
  const double current_backward_len =
      pruned_lane_path.FirstOccurrenceOfLanePointToArclength(
          *closest_lane_point_or);
  return planner::BuildDrivePassageForPredictionWithLaneBoundaryCache(
      semantic_map_mgr, pruned_lane_path, lane_boundary_cache, /*step_s=*/3.0,
      /*avoid_loop=*/true,
      std::max(backward_extend_len - current_backward_len, 0.0),
      FrenetFrameType::kKdTree);
}
}  // namespace prediction
}  // namespace qcraft
