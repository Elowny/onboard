#include "onboard/prediction/util/lane_path_finder.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <ostream>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kMaxSearchRadiusForNearestLane = 5.0;  // m.
constexpr int kMaxChildrenSearchLayer = 5;

// Approximate a remaining length to search lane path.
constexpr double kPathLengthRelaxationFactor = 2.0;
constexpr int kMaxPathSamplePoints = 10;

std::set<mapping::ElementId> FindConsecutiveLaneIds(
    mapping::ElementId id,
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    bool is_reversed, int max_level, bool ignore_virtual) {
  std::set<mapping::ElementId> res;
  if (id == mapping::kInvalidElementId) {
    return res;
  }
  res.insert(id);
  if (max_level <= 0) {
    return res;
  }
  const auto* lane_info_ptr = semantic_map_mgr.FindLaneInfoOrNull(id);
  if (lane_info_ptr == nullptr) return res;

  auto next_ids = lane_info_ptr->outgoing_lanes();
  if (is_reversed) {
    next_ids = lane_info_ptr->incoming_lanes();
  }
  for (const auto& next_id : next_ids) {
    if (next_id == mapping::kInvalidElementId) {
      continue;
    }
    // Ignore virtual lane in consecutive lane id search.
    if (ignore_virtual) {
      const auto* next_info_ptr = semantic_map_mgr.FindLaneInfoOrNull(next_id);
      if (next_info_ptr == nullptr) {
        continue;
      }
      if (next_info_ptr->IsVirtual()) {
        continue;
      }
    }

    const auto next_set = FindConsecutiveLaneIds(
        next_id, semantic_map_mgr, is_reversed, max_level - 1, ignore_virtual);
    for (const auto& next_next_id : next_set) {
      res.insert(next_next_id);
    }
  }
  return res;
}

struct LaneSequence {
  std::vector<mapping::ElementId> ids;
  double accumulated_len;
};

LaneSequence SearchMostStraightLaneSequence(
    const mapping::ElementId id,
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    double start_len, double forward_len, bool is_reversed) {
  LaneSequence cur_sequence{{id}, start_len};
  while (cur_sequence.ids.size() != kMaxChildrenSearchLayer &&
         cur_sequence.accumulated_len <= forward_len) {
    const auto& cur_lane_id = cur_sequence.ids.back();
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, semantic_map_mgr, cur_lane_id);
    auto next_ids = lane_info.outgoing_lanes();
    if (is_reversed) {
      next_ids = lane_info.incoming_lanes();
    }
    if (next_ids.empty()) {
      break;
    }
    double max_unit_dot = -1.0;
    auto min_angle_diff_id = next_ids.at(0);
    for (const auto& next_id : next_ids) {
      const auto* next_lane_info_ptr =
          semantic_map_mgr.FindLaneInfoOrNull(next_id);
      if (next_lane_info_ptr == nullptr) continue;
      Vec2d start_tangent, end_tangent;
      next_lane_info_ptr->GetTangent(0.0, &start_tangent);
      next_lane_info_ptr->GetTangent(1.0, &end_tangent);
      const double dot_prod = start_tangent.Dot(end_tangent);
      if (max_unit_dot < dot_prod) {
        max_unit_dot = dot_prod;
        min_angle_diff_id = next_id;
      }
    }
    cur_sequence.ids.push_back(min_angle_diff_id);
    const auto lane_info_ptr =
        semantic_map_mgr.FindLaneInfoOrNull(min_angle_diff_id);
    if (lane_info_ptr == nullptr) continue;
    cur_sequence.accumulated_len += lane_info_ptr->length();
  }
  if (is_reversed) {
    std::reverse(cur_sequence.ids.begin(), cur_sequence.ids.end());
  }
  return cur_sequence;
}

bool isChild(const planner::PlannerSemanticMapManager& smm,
             const mapping::ElementId& curr,
             const mapping::ElementId& child_maybe, bool is_reversed) {
  SMM_ASSIGN_LANE_OR_RETURN(lane_info, smm, curr, false);
  auto candidates = lane_info.outgoing_lanes();
  if (is_reversed) {
    candidates = lane_info.incoming_lanes();
  }
  for (const auto& lane_id : candidates) {
    if (child_maybe == lane_id) {
      return true;
    }
  }
  return false;
}

std::vector<const mapping::LaneInfo*> PruneChildren(
    const planner::PlannerSemanticMapManager& smm,
    const std::vector<const mapping::LaneInfo*>& lanes, bool is_reversed) {
  std::set<mapping::ElementId> to_delete;
  for (int i = 0; i < lanes.size(); ++i) {
    for (int j = i + 1; j < lanes.size(); ++j) {
      const auto* curr = lanes[i];
      const auto* child_maybe = lanes[j];
      if (isChild(smm, curr->id, child_maybe->id, is_reversed)) {
        to_delete.insert(child_maybe->id);
      }

      if (isChild(smm, child_maybe->id, curr->id, is_reversed)) {
        to_delete.insert(curr->id);
      }
    }
  }
  std::vector<const mapping::LaneInfo*> ret;
  for (const auto* lane_info : lanes) {
    if (to_delete.count(lane_info->id) == 0) {
      ret.push_back(lane_info);
    }
  }

  return ret;
}

void AddNextLayer(const planner::PlannerSemanticMapManager& smm,
                  bool is_reversed,
                  std::vector<std::set<mapping::ElementId>>* graph,
                  int remain_level, double max_remain_s) {
  if (max_remain_s < 0.0) {
    return;
  }

  if (remain_level <= 0) {
    return;
  }
  const auto& prev_layer = graph->back();
  std::set<mapping::ElementId> layer_to_add;
  double min_length_cur_layer = std::numeric_limits<double>::infinity();
  for (const auto& lane_id : prev_layer) {
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, smm, lane_id);
    auto children_lane_ids = lane_info.outgoing_lanes();
    if (is_reversed) {
      children_lane_ids = lane_info.incoming_lanes();
    }

    for (const auto& child_lane_id : children_lane_ids) {
      if (child_lane_id == mapping::kInvalidElementId) {
        continue;
      }
      layer_to_add.insert(child_lane_id);
      min_length_cur_layer = std::min(min_length_cur_layer, lane_info.length());
    }
  }

  graph->push_back(std::move(layer_to_add));
  AddNextLayer(smm, is_reversed, graph, remain_level - 1,
               max_remain_s - min_length_cur_layer);
}

}  // namespace

std::optional<mapping::ElementId> FindNearestLaneIdWithBoundaryDistanceLimit(
    const planner::PlannerSemanticMapManager& semantic_map_mgr, const Vec2d& pt,
    double boundary_distance_limit) {
  const auto* lane_info = semantic_map_mgr.GetNearestLaneInfoAtLevel(
      semantic_map_mgr.GetLevel(), pt);
  if (lane_info && lane_info->id != mapping::kInvalidElementId) {
    if (planner::GetDistOfPointInvasionLaneSupport(
            pt, semantic_map_mgr, lane_info->id) > -boundary_distance_limit) {
      return lane_info->id;
    }
  }
  return std::nullopt;
}

std::optional<mapping::ElementId>
FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
    const planner::PlannerSemanticMapManager& semantic_map_mgr, const Vec2d& pt,
    double heading, double boundary_distance_limit, double max_heading_diff) {
  const auto* lane_info = semantic_map_mgr.GetNearestLaneInfoWithHeadingAtLevel(
      semantic_map_mgr.GetLevel(), pt, heading, kMaxSearchRadiusForNearestLane,
      max_heading_diff);
  if (lane_info && lane_info->id != mapping::kInvalidElementId) {
    if (planner::GetDistOfPointInvasionLaneSupport(
            pt, semantic_map_mgr, lane_info->id) > -boundary_distance_limit) {
      return lane_info->id;
    }
  }
  return std::nullopt;
}

std::set<mapping::ElementId>
FindNearestLaneIdsByBBoxWithBoundaryDistLimitAndHeadingDiffLimit(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const Box2d& box, double boundary_distance_limit, double max_heading_diff) {
  std::set<mapping::ElementId> id_sets;
  const double heading = box.heading();
  for (const auto& pt : box.GetCornersCounterClockwise()) {
    const auto id_or =
        FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
            semantic_map_mgr, pt, heading, boundary_distance_limit,
            max_heading_diff);
    if (id_or.has_value()) {
      id_sets.insert(*id_or);
    }
  }
  return id_sets;
}

std::set<mapping::ElementId> FilterLanesByConsecutiveRelationship(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const std::set<mapping::ElementId>& lane_ids, bool filter_virtual) {
  std::set<mapping::ElementId> filtered_ids;
  // Filter out virtual lanes.
  if (filter_virtual) {
    for (const auto& id : lane_ids) {
      const auto* lane_info_ptr = semantic_map_mgr.FindLaneInfoOrNull(id);
      if (lane_info_ptr == nullptr) {
        continue;
      }
      if (lane_info_ptr->IsVirtual()) {
        continue;
      }
      filtered_ids.insert(id);
    }
  } else {
    filtered_ids = lane_ids;
  }
  absl::flat_hash_map<mapping::ElementId, std::set<mapping::ElementId>>
      id_children_map;
  for (const auto& id : filtered_ids) {
    id_children_map[id] =
        FindConsecutiveLaneIds(id, semantic_map_mgr, /*is_reversed=*/false,
                               kMaxChildrenSearchLayer, filter_virtual);
  }
  std::set<mapping::ElementId> res;
  for (const auto& id : filtered_ids) {
    bool is_child = false;
    for (const auto& other_id : filtered_ids) {
      if (other_id == id) {
        continue;
      }
      if (id_children_map[other_id].count(id) != 0) {
        is_child = true;
        break;
      }
    }
    if (!is_child) {
      res.insert(id);
    }
  }
  return res;
}

// NOLINTNEXTLINE
std::vector<mapping::LanePath> SearchLanePath(
    const Vec2d& pos,
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    mapping::ElementId lane_id, double forward_len, bool is_reverse_driving,
    bool filter_virtual) {
  struct LaneSequence {
    std::vector<mapping::ElementId> ids;
    double accumulated_len;
  };
  SMM_ASSIGN_LANE_OR_RETURN(lane_info, semantic_map_mgr, lane_id, {});
  auto start_lane_length = lane_info.length();
  if (const auto closest_lane_pt_or =
          FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
              semantic_map_mgr.GetLevel(), semantic_map_mgr, pos,
              std::vector<mapping::ElementId>({lane_id}), /*heading=*/0.0,
              /*heading_penalty_weight=*/0.0);
      closest_lane_pt_or.ok()) {
    start_lane_length -= closest_lane_pt_or->fraction() * start_lane_length;
  }
  std::vector<LaneSequence> cur_sequences{
      LaneSequence{{lane_id}, start_lane_length}};
  std::vector<LaneSequence> final_seqs;
  while (!cur_sequences.empty()) {
    std::vector<LaneSequence> new_seqs;
    for (const auto& lane_seq : cur_sequences) {
      const auto& last_lane = lane_seq.ids.back();
      SMM_ASSIGN_LANE_OR_CONTINUE(last_lane_info, semantic_map_mgr, last_lane);
      auto next_ids = last_lane_info.outgoing_lanes();
      if (is_reverse_driving) {
        next_ids = last_lane_info.incoming_lanes();
      }
      bool has_virtual = false;
      for (const auto& next_id : next_ids) {
        if (next_id == mapping::kInvalidElementId) {
          continue;
        }
        SMM_ASSIGN_LANE_OR_CONTINUE(next_lane_info, semantic_map_mgr, next_id);
        if (filter_virtual && next_lane_info.IsVirtual()) {
          has_virtual = true;
          continue;
        }
        LaneSequence new_seq = lane_seq;
        new_seq.ids.push_back(next_id);
        new_seq.accumulated_len += next_lane_info.length();
        if (new_seq.ids.size() == kMaxChildrenSearchLayer ||
            new_seq.accumulated_len > forward_len) {
          final_seqs.push_back(new_seq);
          continue;
        }
        new_seqs.push_back(std::move(new_seq));
      }
      // If the lane is a terminal lane or it has consecutive virtual
      // connections, we keep a lane path terminated at this lane.
      if (next_ids.empty() || has_virtual) {
        final_seqs.push_back(lane_seq);
        continue;
      }
    }
    cur_sequences = std::move(new_seqs);
  }
  std::vector<mapping::LanePath> ret;
  for (auto& seq : final_seqs) {
    if (is_reverse_driving) {
      std::reverse(seq.ids.begin(), seq.ids.end());
    }
    auto lane_path = BuildLanePathFromData(
        mapping::LanePathData(0.0, 1.0, seq.ids), semantic_map_mgr);
    if (lane_path.ok()) {
      ret.push_back(std::move(*lane_path));
    }
  }
  return ret;
}

mapping::LanePath SearchMostStraightLanePath(
    const Vec2d& pos,
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    mapping::ElementId lane_id, double forward_len, bool is_reverse_driving) {
  SMM_ASSIGN_LANE_OR_RETURN(lane_info, semantic_map_mgr, lane_id,
                            mapping::LanePath());
  auto start_lane_length = lane_info.length();
  if (const auto closest_lane_pt_or =
          FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
              semantic_map_mgr.GetLevel(), semantic_map_mgr, pos,
              std::vector<mapping::ElementId>({lane_id}), /*heading=*/0.0,
              /*heading_penalty_weight=*/0.0);
      closest_lane_pt_or.ok()) {
    if (is_reverse_driving) {
      start_lane_length = closest_lane_pt_or->fraction() * start_lane_length;
    } else {
      start_lane_length -= closest_lane_pt_or->fraction() * start_lane_length;
    }
  } else {
    return mapping::LanePath();
  }

  const auto cur_sequence = SearchMostStraightLaneSequence(
      lane_id, semantic_map_mgr, start_lane_length, forward_len,
      is_reverse_driving);
  const auto lane_path = BuildLanePathFromData(
      mapping::LanePathData(0.0, 1.0, cur_sequence.ids), semantic_map_mgr);
  if (lane_path.ok()) {
    return *lane_path;
  } else {
    return mapping::LanePath();
  }
}

mapping::LanePath ExtendMostStraightLanePath(
    const mapping::LanePath& lp,
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    double forward_len, bool is_reversed) {
  const auto& ids = lp.lane_ids();
  QCHECK_GT(ids.size(), 0);
  mapping::ElementId start_id = mapping::kInvalidElementId;
  if (is_reversed) {
    start_id = ids.front();
  } else {
    start_id = ids.back();
  }
  auto cur_sequence = SearchMostStraightLaneSequence(start_id, semantic_map_mgr,
                                                     /*start_len=*/0.0,
                                                     forward_len, is_reversed);
  std::vector<mapping::ElementId> final_ids;
  if (is_reversed) {
    std::copy(cur_sequence.ids.begin(), cur_sequence.ids.end(),
              std::back_inserter(final_ids));
    std::copy(std::next(ids.begin(), 1), ids.end(),
              std::back_inserter(final_ids));
  } else {
    std::copy(ids.begin(), ids.end(), std::back_inserter(final_ids));
    std::copy(std::next(cur_sequence.ids.begin(), 1), cur_sequence.ids.end(),
              std::back_inserter(final_ids));
  }
  const auto lane_path = BuildLanePathFromData(
      mapping::LanePathData(0.0, 1.0, final_ids), semantic_map_mgr);

  if (lane_path.ok()) {
    return *lane_path;
  } else {
    return mapping::LanePath();
  }
}

absl::StatusOr<std::vector<mapping::ElementId>>
FindConsecutiveLanesForDiscretizedPath(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const planner::DiscretizedPath& path, bool is_reversed) {
  FUNC_QTRACE();
  QCHECK_GE(path.size(), 1);
  std::vector<mapping::ElementId> ret;
  const auto& cur_point = path.front();

  // Collect layers of lanes and the number of sampled path points closest to
  // it.
  std::vector<std::set<mapping::ElementId>> graph;
  std::map<mapping::ElementId, int> point_lane_match_count;

  auto start_lanes = semantic_map_mgr.GetLanesInfoWithHeadingAtLevel(
      semantic_map_mgr.GetLevel(), Vec2d(cur_point.x(), cur_point.y()),
      cur_point.theta(), kMaxSearchRadiusForNearestLane, M_PI_2);

  if (start_lanes.empty()) {
    return absl::NotFoundError(
        "Cannot find nearest lane for the start of the path!");
  }

  start_lanes = PruneChildren(semantic_map_mgr, start_lanes, is_reversed);

  std::set<mapping::ElementId> first_layer;
  for (const auto* start_lane_info : start_lanes) {
    first_layer.insert(start_lane_info->id);
  }

  graph.push_back(std::move(first_layer));
  AddNextLayer(semantic_map_mgr, is_reversed, &graph,
               kMaxChildrenSearchLayer - 1,
               path.back().s() * kPathLengthRelaxationFactor);

  VLOG(5) << "Create lane graph with count mapping:";
  for (const auto& layer : graph) {
    VLOG(5) << "-----------";
    for (const auto& id : layer) {
      VLOG(5) << id;
    }
  }

  // Taking samples of points and add to point_lane_match_count counter.
  const int min_points = std::min<int>(kMaxPathSamplePoints, path.size());
  const int step = FloorToInt(path.size() / static_cast<float>(min_points));
  for (int i = 0; i < path.size(); i += step) {
    const auto& cur_point = path[i];
    const mapping::LaneInfo* lane = semantic_map_mgr.GetNearestLaneInfoAtLevel(
        semantic_map_mgr.GetLevel(), Vec2d(cur_point.x(), cur_point.y()));
    if (lane == nullptr) {
      continue;
    }
    point_lane_match_count[lane->id]++;
  }

  VLOG(5) << "Points on lane counter: ";
  for (const auto& pair : point_lane_match_count) {
    VLOG(5) << absl::StrFormat("Lane %d has %d point(s).", pair.first,
                               pair.second);
  }

  // Build lane path for this discretized path. Do not require semantic lane
  // connection correctness since agents might move differently.
  for (const auto& layer : graph) {
    mapping::ElementId max_id = mapping::kInvalidElementId;
    int max = 0;
    for (const auto& lane : layer) {
      auto* ptr_second = FindOrNull(point_lane_match_count, lane);
      if (ptr_second == nullptr) {
        continue;
      }
      const auto count = *ptr_second;
      if (count > max) {
        max = count;
        max_id = lane;
      }
    }
    if (max_id != mapping::kInvalidElementId) {
      ret.push_back(max_id);
    }
  }
  // Returned path size might be less than the graph size.
  if (ret.empty()) {
    return absl::NotFoundError(
        "Cannot project any other sampled points on path to this local lane "
        "graph!");
  }
  return ret;
}

std::optional<const mapping::IntersectionInfo*>
FindIntersectionsAlongTheLaneByDistance(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const double forward_len, const mapping::ElementId lane_id,
    const double start_lane_fraction) {
  SMM_ASSIGN_LANE_OR_RETURN(lane_info, semantic_map_mgr, lane_id, std::nullopt);
  for (const auto& intersection_info : lane_info.Intersections()) {
    if (start_lane_fraction <= intersection_info.second.y()) {
      if (std::max(0.0, intersection_info.second.x() - start_lane_fraction) *
              lane_info.length() <
          forward_len) {
        auto* intersection = semantic_map_mgr.FindIntersectionByIdOrNull(
            intersection_info.first);
        if (intersection != nullptr) return intersection;
      }
    }
  }
  return std::nullopt;
}
}  // namespace prediction
}  // namespace qcraft
