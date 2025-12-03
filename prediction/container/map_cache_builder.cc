#include "onboard/prediction/container/map_cache_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/util/drive_passage_util.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {
namespace {
using planner::PredictionLaneBoundary;
using planner::PredictionLaneBoundaryLerper;
using DrivePassages = std::vector<std::unique_ptr<planner::DrivePassage>>;
constexpr double kMinHalfLaneWidthForPrediction = 0.5;
constexpr double kMaxHalfLaneWidthForPrediction = 2.5;
constexpr double kDpCacheFrontLength = 180;
constexpr double kDpCacheBackLength = 80;
Vec2d LatPoint(const Vec2d& pt, const Vec2d& tangent, double offset) {
  return pt + tangent.Perp() * offset;
}

PiecewiseLinearFunction<PredictionLaneBoundary, double,
                        PredictionLaneBoundaryLerper>
GetLaneBoundaryPlf(const planner::PlannerSemanticMapManager& psmm,
                   const mapping::LaneInfo& lane, double max_lateral_offset,
                   int num_step) {
  std::vector<double> vec_fraction;
  std::vector<PredictionLaneBoundary> vec_bounds;
  vec_fraction.reserve(num_step);
  vec_bounds.reserve(num_step);

  // Create lane boundary Plf.
  const double num_step_inv = 1 / static_cast<double>(num_step);
  for (int i = 0; i < num_step; ++i) {
    const double fraction = i * num_step_inv;
    vec_fraction.push_back(fraction);
    const auto xy = lane.LerpPointFromFraction(fraction);
    const auto tangent = lane.GetTangent(fraction);
    const Segment2d normal_line(LatPoint(xy, tangent, -max_lateral_offset),
                                LatPoint(xy, tangent, max_lateral_offset));
    const auto candidate_boundaries = psmm.GetLaneBoundariesInfoAtLevel(
        psmm.GetLevel(), xy, max_lateral_offset);
    PredictionLaneBoundary current_boundary;
    current_boundary.right_bound = -max_lateral_offset;
    current_boundary.left_bound = max_lateral_offset;
    for (const auto& boundary : candidate_boundaries) {
      // Compute cross point of boundary and normal line
      Vec2d closest_intersection;
      double min_sqr_dis = std::numeric_limits<double>::max();
      int closest_index;
      for (int k = 0; k + 1 < boundary->points_smooth.size(); ++k) {
        const Vec2d& p0 = boundary->points_smooth[k];
        const Vec2d& p1 = boundary->points_smooth[k + 1];
        const Segment2d boundary_segment(p0, p1);

        Vec2d intersection;
        if (!normal_line.GetIntersect(boundary_segment, &intersection))
          continue;
        const double sqr_dis = xy.DistanceSquareTo(intersection);
        if (sqr_dis < min_sqr_dis) {
          closest_index = k;
          min_sqr_dis = sqr_dis;
          closest_intersection = intersection;
        }
      }
      // Assign to left or right boundary.
      if (min_sqr_dis < std::numeric_limits<double>::max()) {
        const double signed_dis = tangent.CrossProd(closest_intersection - xy);
        if (signed_dis < 0) {
          if (current_boundary.right_bound < signed_dis) {
            current_boundary.right_bound = signed_dis;
            current_boundary.right_type =
                planner::MapBoundaryTypeToStationBoundaryType(
                    planner::GetBoundaryType(*boundary, closest_index));
          }
        } else {
          if (current_boundary.left_bound > signed_dis) {
            current_boundary.left_bound = signed_dis;
            current_boundary.left_type =
                planner::MapBoundaryTypeToStationBoundaryType(
                    planner::GetBoundaryType(*boundary, closest_index));
          }
        }
      }
    }
    current_boundary.right_bound = std::clamp(current_boundary.right_bound,
                                              -kMaxHalfLaneWidthForPrediction,
                                              -kMinHalfLaneWidthForPrediction);
    current_boundary.left_bound =
        std::clamp(current_boundary.left_bound, kMinHalfLaneWidthForPrediction,
                   kMaxHalfLaneWidthForPrediction);

    vec_bounds.push_back(current_boundary);
  }
  return PiecewiseLinearFunction<PredictionLaneBoundary, double,
                                 PredictionLaneBoundaryLerper>(vec_fraction,
                                                               vec_bounds);
}
std::vector<mapping::LanePath> FilterAndTruncateLanePaths(
    absl::Span<const mapping::LanePath> lps,
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const Vec2d& ego_pos, double ego_heading, int max_num) {
  struct LanePathInfo {
    double square_dis;
    const mapping::LanePath* lp;
    mapping::LanePoint lane_point;
    Vec2d closest_point_on_lane;
  };
  std::vector<LanePathInfo> infos;
  infos.reserve(lps.size());
  constexpr double kDpSearchRadiusThreshold = 200.0;
  for (const auto& lp : lps) {
    Vec2d closest_pt;
    auto closest_lane_point_or = planner::
        FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
            semantic_map_manager.GetLevel(), semantic_map_manager, ego_pos, lp,
            ego_heading,
            /*heading_penalty_weight=*/0.0,
            /*closest_point_on_lane=*/&closest_pt,
            /*cutoff_distance=*/kDpSearchRadiusThreshold);
    // The projection must be valid.
    if (!closest_lane_point_or.ok()) {
      continue;
    }
    const double square_dis = ego_pos.DistanceSquareTo(closest_pt);
    infos.push_back(
        LanePathInfo{.square_dis = square_dis,
                     .lp = &lp,
                     .lane_point = std::move(closest_lane_point_or).value(),
                     .closest_point_on_lane = std::move(closest_pt)});
  }
  std::sort(infos.begin(), infos.end(),
            [](const LanePathInfo& info1, const LanePathInfo& info2) {
              return info1.square_dis < info2.square_dis;
            });
  if (infos.size() > max_num) {
    infos.resize(max_num);
  }
  std::vector<mapping::LanePath> res;
  res.reserve(infos.size());
  for (const auto& info : infos) {
    const double heading_diff =
        NormalizeAngle(ego_heading - ComputeLanePointTangent(
                                         semantic_map_manager, info.lane_point)
                                         .FastAngle());
    if (std::fabs(heading_diff) < M_PI_2) {
      res.push_back(PruneLanePathByLength(semantic_map_manager, *info.lp,
                                          info.lane_point, kDpCacheFrontLength,
                                          kDpCacheBackLength));
    } else {
      res.push_back(PruneLanePathByLength(semantic_map_manager, *info.lp,
                                          info.lane_point, kDpCacheBackLength,
                                          kDpCacheFrontLength));
    }
  }
  return res;
}
}  // namespace
planner::LaneBoundaryCache BuildLaneBoundaryCache(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const Vec2d& ego_pos, double ego_heading, double front_dist,
    double back_dist, double side_dist, double desire_sample_step) {
  SCOPED_QTRACE("MapCacheBuilder::BuildLaneBoundaryCache");
  const auto region_box =
      GetRegionBox(ego_pos, ego_heading, front_dist, back_dist, side_dist);
  const auto level_id = semantic_map_manager.GetLevel();
  const auto lanes = semantic_map_manager.GetLanesInfoAtLevel(
      level_id, region_box.center(), region_box.diagonal() * 0.5);
  constexpr int kMaxLaneNum = 64;
  const auto filtered_lanes =
      GetElementsInBox2d(lanes, region_box, kMaxLaneNum);
  constexpr double kMaxLateralOffset = 4.0;
  planner::LaneBoundaryCache cache;
  constexpr int kMaxNumStep = 50;
  const double desire_sample_step_inv = 1.0 / desire_sample_step;
  for (const auto& lane : filtered_lanes) {
    const int num_step = std::clamp(
        static_cast<int>(lane->length() * desire_sample_step_inv) + 1, 2,
        kMaxNumStep);
    cache[lane->id] = GetLaneBoundaryPlf(semantic_map_manager, *lane,
                                         kMaxLateralOffset, num_step);
  }
  return cache;
}
std::pair<DrivePassages, DrivePassageCache> BuildDrivePassageCache(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const planner::LaneBoundaryCache& lane_boundary_cache, const Vec2d& ego_pos,
    double ego_heading, double front_dist, double back_dist, double side_dist,
    double desire_sample_step, int max_dp_num, bool filter_virtual) {
  SCOPED_QTRACE("MapCacheBuilder::BuildDrivePassageCache");
  const auto region_box =
      GetRegionBox(ego_pos, ego_heading, front_dist, back_dist, side_dist);
  const auto level_id = semantic_map_manager.GetLevel();
  const auto lanes = semantic_map_manager.GetLanesInfoAtLevel(
      level_id, region_box.center(), region_box.diagonal() * 0.5);
  constexpr int kMaxLaneNum = 128;
  const auto filtered_lanes =
      GetElementsInBox2d(lanes, region_box, kMaxLaneNum);
  std::set<mapping::ElementId> lane_ids;
  for (const auto* lane : filtered_lanes) {
    QCHECK(lane->id != mapping::kInvalidElementId);
    lane_ids.insert(lane->id);
  }
  const auto filtered_ids = FilterLanesByConsecutiveRelationship(
      semantic_map_manager, lane_ids, filter_virtual);
  VLOG(2) << "Lane numbers: "
          << "Before filtering " << lane_ids.size() << " ,after filtering "
          << filtered_ids.size();
  VLOG(2) << "Original Lane Ids: " << absl::StrJoin(lane_ids, ",");
  VLOG(2) << "Filtered Lane Ids: " << absl::StrJoin(filtered_ids, ",");

  std::vector<mapping::LanePath> lane_paths;
  lane_paths.reserve(2 * filtered_ids.size());
  for (const auto& lane_id : filtered_ids) {
    auto lps = SearchLanePath(ego_pos, semantic_map_manager, lane_id,
                              kDpCacheFrontLength,
                              /*is_reverse_driving=*/false, filter_virtual);
    for (auto& lp : lps) {
      lane_paths.push_back(std::move(lp));
    }
  }
  VLOG(2) << "Feasible lane path total num: " << lane_paths.size();
  lane_paths = FilterAndTruncateLanePaths(lane_paths, semantic_map_manager,
                                          ego_pos, ego_heading, max_dp_num);
  VLOG(2) << "Filtered feasible lane path total num: " << lane_paths.size();

  DrivePassageCache cache;
  DrivePassages dps;
  dps.reserve(lane_paths.size());
  for (const auto& lp : lane_paths) {
    auto dp_or = planner::BuildDrivePassageForPredictionWithLaneBoundaryCache(
        semantic_map_manager, lp, lane_boundary_cache, desire_sample_step,
        /*avoid_loop=*/true,
        /*backward_extend_len=*/0.0, /*type=*/FrenetFrameType::kKdTree);
    if (dp_or.ok()) {
      auto dp_ptr =
          std::make_unique<planner::DrivePassage>(std::move(dp_or).value());
      for (const auto& cur_id : lp.lane_ids()) {
        auto* ref_dps = FindOrNull(cache, cur_id);
        if (ref_dps == nullptr) {
          cache[cur_id] = std::vector<const planner::DrivePassage*>();
        }
        cache[cur_id].push_back(dp_ptr.get());
      }
      dps.push_back(std::move(dp_ptr));
    }
  }
  return std::make_pair(std::move(dps), std::move(cache));
}

}  // namespace prediction
}  // namespace qcraft
