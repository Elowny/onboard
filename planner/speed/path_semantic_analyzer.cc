#include "onboard/planner/speed/path_semantic_analyzer.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_set>

#include "absl/status/status.h"
#include "absl/types/span.h"

#include "onboard/async/parallel_for.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kMinDistSqrThreshold = Sqr(0.5 * kPathSampleInterval);  // m^2.
constexpr double kEps = 1e-3;

inline LaneSemantic QueryLaneSemantic(const mapping::LaneInfo& lane_info) {
  if (lane_info.is_in_intersection) {
    switch (lane_info.direction) {
      case mapping::LaneProto::STRAIGHT:
        return LaneSemantic::INTERSECTION_STRAIGHT;
      case mapping::LaneProto::LEFT_TURN:
        return LaneSemantic::INTERSECTION_LEFT_TURN;
      case mapping::LaneProto::RIGHT_TURN:
        return LaneSemantic::INTERSECTION_RIGHT_TURN;
      case mapping::LaneProto::UTURN:
        return LaneSemantic::INTERSECTION_UTURN;
    }
  } else {
    switch (lane_info.direction) {
      case mapping::LaneProto::STRAIGHT:
        return LaneSemantic::ROAD;
      case mapping::LaneProto::LEFT_TURN:
      case mapping::LaneProto::RIGHT_TURN:
      case mapping::LaneProto::UTURN:
        QLOG_EVERY_N_SEC(ERROR, 2.0)
            << "Lane " << lane_info.id << " direction is "
            << mapping::LaneProto::Direction_Name(lane_info.direction)
            << " but not in intersection.";
        return LaneSemantic::ROAD;
    }
  }
}

void AnalyzePrimaryPathSemantic(
    const PathPoint& path_point,
    const absl::Span<const DrivingMapTopo::Lane>& lanes,
    const PlannerSemanticMapManager& psmm, PathPointSemantic* path_semantic) {
  const Vec2d path_point_xy = ToVec2d(path_point);
  double min_dist_sqr = std::numeric_limits<double>::infinity();
  mapping::LanePoint closest_lane_point(
      /*lane_id=*/mapping::kInvalidElementId,
      /*fraction=*/0.0);
  Vec2d closest_lane_point_pos;
  for (const auto& lane : lanes) {
    Vec2d closest_point;
    const auto lane_point_opt =
        FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
            psmm.GetLevel(), psmm, path_point_xy, lane.id, path_point.theta(),
            /*heading_penalty_weight=*/0.0, &closest_point, lane.start_fraction,
            lane.end_fraction);
    if (!lane_point_opt.has_value()) continue;
    const double dist_sqr = path_point_xy.DistanceSquareTo(closest_point);
    if (dist_sqr < min_dist_sqr) {
      min_dist_sqr = dist_sqr;
      closest_lane_point = *lane_point_opt;
      closest_lane_point_pos = closest_point;
    }
    // Since path point interval is about 0.2m, if minimum distance is less
    // than 0.1m, we don't need to continue find closest point on other lanes.
    if (min_dist_sqr < kMinDistSqrThreshold) {
      break;
    }
  }
  if (!closest_lane_point.Valid()) {
    return;
  }
  path_semantic->closest_lane_point = closest_lane_point;
  path_semantic->closest_lane_point_pos = closest_lane_point_pos;
  path_semantic->deviation_distance = std::sqrt(min_dist_sqr);
  const auto* lane_info_ptr =
      psmm.FindLaneInfoOrNull(closest_lane_point.lane_id());
  if (UNLIKELY(lane_info_ptr == nullptr)) {
    QLOG(ERROR) << "Cannot find semantic id: " << closest_lane_point.lane_id();
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                      QIssueSubType::QIST_SEMANTIC_MAP,
                      "It is a map issue, not a planner issue. ",
                      std::to_string(closest_lane_point.lane_id()));
    return;
  }
  path_semantic->lane_semantic = QueryLaneSemantic(*lane_info_ptr);
  path_semantic->lane_info = lane_info_ptr;
  return;
}

void AnalyzeSecondaryPathSemantic(
    const PathPoint& path_point,
    const absl::Span<const DrivingMapTopo::Lane>& lanes,
    const PlannerSemanticMapManager& psmm,
    const PathPointSemantic& prev_primary_path_semantic,
    const PathPointSemantic& next_primary_path_semantic, double ratio,
    PathPointSemantic* path_semantic) {
  if (!prev_primary_path_semantic.closest_lane_point.Valid() ||
      !next_primary_path_semantic.closest_lane_point.Valid()) {
    AnalyzePrimaryPathSemantic(path_point, lanes, psmm, path_semantic);
    return;
  }
  if (prev_primary_path_semantic.closest_lane_point.lane_id() !=
      next_primary_path_semantic.closest_lane_point.lane_id()) {
    AnalyzePrimaryPathSemantic(path_point, lanes, psmm, path_semantic);
    return;
  }
  const mapping::LanePoint closest_lane_point(
      prev_primary_path_semantic.closest_lane_point.lane_id(),
      Lerp(prev_primary_path_semantic.closest_lane_point.fraction(),
           next_primary_path_semantic.closest_lane_point.fraction(), ratio));
  path_semantic->deviation_distance =
      Lerp(prev_primary_path_semantic.deviation_distance,
           next_primary_path_semantic.deviation_distance, ratio);
  const auto closest_lane_point_pos =
      ComputeLanePointPos(psmm, closest_lane_point);
  path_semantic->closest_lane_point = closest_lane_point;
  path_semantic->closest_lane_point_pos = closest_lane_point_pos;
  path_semantic->lane_semantic = prev_primary_path_semantic.lane_semantic;
  path_semantic->lane_info = prev_primary_path_semantic.lane_info;
  return;
}

}  // namespace

absl::StatusOr<std::vector<PathPointSemantic>> AnalyzePathSemantics(
    const DiscretizedPath& path, int max_analyze_path_index,
    const PlannerSemanticMapManager& psmm,
    const DrivingMapTopo* driving_map_topo, ThreadPool* thread_pool) {
  SCOPED_QTRACE("AnalyzePathSemantics");

  QCHECK(!path.empty());
  QCHECK_LT(max_analyze_path_index, path.size());

  if (driving_map_topo == nullptr) {
    return absl::NotFoundError("driving_map_topo is nullptr!");
  }
  std::atomic_int path_semantic_size = max_analyze_path_index + 1;
  std::vector<PathPointSemantic> path_semantics(path_semantic_size);

  const auto& lanes = driving_map_topo->lanes();
  constexpr double kPrimaryPathSampleInterval = 1.0;  // m.
  std::vector<int> primary_indices;
  std::unordered_set<int> primary_index_set;
  primary_indices.reserve(
      CeilToInt(path[max_analyze_path_index].s() / kPrimaryPathSampleInterval) +
      1);
  primary_index_set.reserve(primary_indices.capacity());
  double last_primary_s = 0.0;
  for (int i = 0; i < path_semantic_size; ++i) {
    if (i == 0 || i + 1 == path_semantic_size ||
        path[i].s() - last_primary_s > kPrimaryPathSampleInterval - kEps) {
      last_primary_s = path[i].s();
      primary_indices.push_back(i);
      primary_index_set.insert(i);
    }
  }

  // Analyze primary path semantics by finding closest lane point.
  ParallelFor(0, primary_indices.size(), thread_pool, [&](int i) {
    // TODO(renjie): Consider to use lane boundary and AV box info to tell which
    // lane the path point is located in exactly.
    const int idx = primary_indices[i];
    int current_size = path_semantic_size.load(std::memory_order_relaxed);
    if (idx > current_size) return;
    AnalyzePrimaryPathSemantic(path[idx], lanes, psmm, &path_semantics[idx]);
    if (!path_semantics[idx].closest_lane_point.Valid()) {
      do {
        if (idx > current_size) break;
      } while (!path_semantic_size.compare_exchange_weak(
          current_size, idx, std::memory_order_relaxed));
    }
    return;
  });

  // Analyze secondary path semantics by interpolation with primary path
  // semantics.
  ParallelFor(0, path_semantic_size, thread_pool, [&](int i) {
    int current_size = path_semantic_size.load(std::memory_order_relaxed);
    if (i > current_size) return;
    if (ContainsKey(primary_index_set, i)) {
      return;
    }
    int idx = FloorToInt(path[i].s() / kPrimaryPathSampleInterval);
    while (idx + 1 > primary_indices.size() || primary_indices[idx] > i) {
      --idx;
    }
    while (idx + 1 < primary_indices.size() && primary_indices[idx + 1] < i) {
      ++idx;
    }
    const int prev_primary_idx = primary_indices[idx];
    const int next_primary_idx = primary_indices[idx + 1];
    const double ratio =
        (path[i].s() - path[prev_primary_idx].s()) /
        (path[next_primary_idx].s() - path[prev_primary_idx].s());
    AnalyzeSecondaryPathSemantic(
        path[i], lanes, psmm, path_semantics[prev_primary_idx],
        path_semantics[next_primary_idx], ratio, &path_semantics[i]);
    if (!path_semantics[i].closest_lane_point.Valid()) {
      do {
        if (i > current_size) break;
      } while (!path_semantic_size.compare_exchange_weak(
          current_size, i, std::memory_order_relaxed));
    }
    return;
  });

  if (path_semantic_size == 0) {
    return absl::NotFoundError("Failed to analyze path semantics!");
  }
  path_semantics.erase(path_semantics.begin() + path_semantic_size,
                       path_semantics.end());
  // Set lane path id. If two adjacent closest lane points have a larger squared
  // distance than this, we consider that a lane change happens.
  constexpr double kClosestLanePointSqrDistThres = Sqr(2.8);  // m^2.
  std::vector<int> curr_lane_path_id_history = {0};
  path_semantics[0].lane_path_id_history = curr_lane_path_id_history;
  for (int i = 1; i < path_semantic_size; ++i) {
    if (path_semantics[i].closest_lane_point.lane_id() !=
            path_semantics[i - 1].closest_lane_point.lane_id() &&
        path_semantics[i].closest_lane_point_pos.DistanceSquareTo(
            path_semantics[i - 1].closest_lane_point_pos) >
            kClosestLanePointSqrDistThres) {
      const auto closest_lane_point_tangent =
          ComputeLanePointTangent(psmm, path_semantics[i].closest_lane_point);

      if (closest_lane_point_tangent.CrossProd(
              path_semantics[i].closest_lane_point_pos -
              path_semantics[i - 1].closest_lane_point_pos) > 0.0) {
        // Lane change left.
        curr_lane_path_id_history.push_back(curr_lane_path_id_history.back() +
                                            1);
      } else {
        // Lane change right.
        curr_lane_path_id_history.push_back(curr_lane_path_id_history.back() -
                                            1);
      }
    }
    path_semantics[i].lane_path_id_history = curr_lane_path_id_history;
  }

  return path_semantics;
}

}  // namespace planner
}  // namespace qcraft
