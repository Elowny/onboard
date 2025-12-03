#include "onboard/planner/scheduler/target_lane_path_filter.h"

#include <float.h>
#include <limits.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <type_traits>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/scene_util.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner {

namespace {

// Should both be much smaller than -LaneGraph::kDeadEndToTargetCost.
constexpr double kLastTargetLanePathReward = -1e10;
constexpr double kPreferredLanePathReward = -1e12;
constexpr double kLcPreviewLanePathReward = -1e9;
constexpr int kInvalidIndexDiff = 1000;
constexpr double kHighwayPreviewDist = 100.0;         // m.
constexpr double kDiscourageRightMostMinDist = 2000;  // m.
constexpr int kMinLaneSizeDiscourageRightMost = 3;
constexpr double kRightMostLanePathCost = 1000;
constexpr double kIgnoreLaneBoundaryMaxDist = 1.0;  // m.

bool IsValidMergeLane(
    const RouteSectionsInfo::RouteSectionSegmentInfo& section_seg,
    const RouteNaviInfo::RouteLaneInfo* target_navi_info_ptr) {
  if (target_navi_info_ptr == nullptr) return false;
  if (target_navi_info_ptr->merge_targets.empty()) return false;

  for (const auto lane_id : section_seg.lane_ids) {
    if (target_navi_info_ptr->merge_targets.contains(lane_id)) return true;
  }
  return false;
}

double ComputeLanePathCost(const LanePathInfo& lp_info,
                           const RouteSectionsInfo& route_sections_info,
                           const RouteNaviInfo& route_navi_info) {
  double cost = lp_info.path_cost();

  constexpr double kLcNumToTargetsWeight = 1.0;
  constexpr double kMergeLaneCostBase = 100.0;
  constexpr double kConsiderMaxDrivingDistance = 4000.0;  // m.
  constexpr double kLengthEpsilon = 0.1;                  // m.
  // Minor difference based on global route.
  const auto* lane_navi_info_ptr =
      FindOrNull(route_navi_info.route_lane_info_map, lp_info.start_lane_id());
  if (lane_navi_info_ptr == nullptr) return DBL_MAX;
  cost += route_sections_info.planning_horizon() /
          (std::min(lane_navi_info_ptr->max_driving_distance,
                    kConsiderMaxDrivingDistance) +
           kLengthEpsilon);

  int lc_num = lane_navi_info_ptr->min_lc_num_to_target;
  double len_before_merge = DBL_MAX;
  bool valid_merge = false;
  const auto check_lane_path =
      lp_info.lane_path().BeforeArclength(lp_info.max_reach_length());
  for (int i = 0; i < check_lane_path.size(); ++i) {
    const auto lane_seg = check_lane_path.lane_segment(i);
    const auto* lane_navi_info_ptr =
        FindOrNull(route_navi_info.route_lane_info_map, lane_seg.lane_id);
    lc_num = std::max(lc_num, lane_navi_info_ptr == nullptr
                                  ? INT_MAX
                                  : lane_navi_info_ptr->min_lc_num_to_target);
    valid_merge |= IsValidMergeLane(route_sections_info.section_segment(i),
                                    lane_navi_info_ptr);
    if (valid_merge)
      len_before_merge = std::min(
          len_before_merge,
          lane_seg.start_s + (lane_navi_info_ptr == nullptr
                                  ? 0.0
                                  : lane_navi_info_ptr->len_before_merge_lane));
  }
  cost += kLcNumToTargetsWeight * lc_num;
  if (valid_merge) {
    cost += kMergeLaneCostBase / (len_before_merge + kMergeLaneCostBase);
  }

  return cost;
}

std::vector<double> CollectLaneLatOffsets(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& cur_lane_ids, double start_frac,
    double end_frac, const Vec2d& ego_pos, int* neutral_next_idx,
    mapping::ElementId* closest_lane_id) {
  *neutral_next_idx = -1;
  std::vector<double> l_offsets(cur_lane_ids.size());
  for (int i = 0; i < cur_lane_ids.size(); ++i) {
    const mapping::LanePath lane_path(psmm.semantic_map_manager(),
                                      {cur_lane_ids[i]}, start_frac, end_frac);
    const auto frenet_frame_or =
        BuildBruteForceFrenetFrame(SampleLanePathPoints(psmm, lane_path),
                                   /*down_sample_raw_points=*/true);
    if (frenet_frame_or.ok()) {
      l_offsets[i] = frenet_frame_or->XYToSL(ego_pos).l;
    } else {
      const mapping::LanePoint lane_pt(cur_lane_ids[i], start_frac);
      l_offsets[i] =
          ComputeLanePointTangent(psmm, lane_pt)
              .CrossProd(ego_pos - ComputeLanePointPos(psmm, lane_pt));
    }
    if (i > 0 && l_offsets[i - 1] * l_offsets[i] < 0.0) {
      *neutral_next_idx = i;
      *closest_lane_id = std::abs(l_offsets[i - 1]) < std::abs(l_offsets[i])
                             ? cur_lane_ids[i - 1]
                             : cur_lane_ids[i];
    }
  }
  if (*neutral_next_idx == -1) {
    *neutral_next_idx = l_offsets[0] > 0.0 ? 0 : cur_lane_ids.size();
    *closest_lane_id =
        l_offsets[0] > 0.0 ? cur_lane_ids.front() : cur_lane_ids.back();
  }
  return l_offsets;
}

absl::flat_hash_map<mapping::ElementId, int> CollectLaneIndexDiff(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo::RouteSectionSegmentInfo& front_sec_info,
    const std::vector<double>& l_offsets,
    const std::vector<LanePathInfo>& lp_infos, int neutral_next_idx,
    mapping::ElementId closest_lane_id) {
  // NOLINTBEGIN(readability-function-cognitive-complexity)
  const auto& cur_lane_ids = front_sec_info.lane_ids;
  const int lane_id_size = cur_lane_ids.size();

  absl::flat_hash_map<mapping::ElementId, double> reach_length_map;
  for (const auto& lp_info : lp_infos) {
    const auto start_id = lp_info.start_lane_id();
    const auto start_id_iter = reach_length_map.find(start_id);
    if (start_id_iter != reach_length_map.end()) {
      start_id_iter->second =
          std::max(start_id_iter->second, lp_info.max_reach_length());
    } else {
      reach_length_map.insert({start_id, lp_info.max_reach_length()});
    }
  }

  // Allow continuous lane change if the rest distance is not enough or
  // currently beyond the leftmost/rightmost turning lane.
  const bool allow_cont_lc =
      FindWithDefault(reach_length_map, closest_lane_id, 0.0) <
          0.5 * kMinLcLaneLength ||
      (IsTurningLane(psmm, cur_lane_ids[0]) &&
       (neutral_next_idx == 0 || neutral_next_idx == lane_id_size));
  const double lat_dist_thres =
      allow_cont_lc ? 1.5 * kDefaultHalfLaneWidth : kMaxLaneKeepLateralOffset;

  constexpr double kMinReachLengthContLc = 2.0 * kMinLcLaneLength;
  absl::flat_hash_map<mapping::ElementId, int> results;
  if (neutral_next_idx > 0) {  // From ego to the left side.
    int index_diff =
        std::abs(l_offsets[neutral_next_idx - 1]) < lat_dist_thres ? 0 : -1;
    for (int i = neutral_next_idx - 1; i > 0; --i) {
      results[cur_lane_ids[i]] = index_diff;
      if (l_offsets[i] - l_offsets[i - 1] < kMaxLaneKeepLateralOffset) continue;

      if (allow_cont_lc && index_diff == 0 &&
          FindWithDefault(reach_length_map, cur_lane_ids[i], 0.0) >
              kMinReachLengthContLc) {
        --index_diff;
      }
      SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, psmm, cur_lane_ids[i - 1], {});
      if (!lane_info.IsMerging() ||
          l_offsets[i] - l_offsets[i - 1] > kDefaultHalfLaneWidth) {
        --index_diff;
      }
    }
    results[cur_lane_ids[0]] = index_diff;
  }
  if (neutral_next_idx < lane_id_size) {  // From ego to the right side.
    int index_diff =
        std::abs(l_offsets[neutral_next_idx]) < lat_dist_thres ? 0 : 1;
    for (int i = neutral_next_idx; i + 1 < lane_id_size; ++i) {
      results[cur_lane_ids[i]] = index_diff;
      if (l_offsets[i + 1] - l_offsets[i] < kMaxLaneKeepLateralOffset) continue;

      if (allow_cont_lc && index_diff == 0 &&
          FindWithDefault(reach_length_map, cur_lane_ids[i], 0.0) >
              kMinReachLengthContLc) {
        ++index_diff;
      }
      SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, psmm, cur_lane_ids[i + 1], {});
      if (!lane_info.IsMerging() ||
          l_offsets[i + 1] - l_offsets[i] > kDefaultHalfLaneWidth) {
        ++index_diff;
      }
    }
    results[cur_lane_ids[lane_id_size - 1]] = index_diff;
  }

  return results;
}  // NOLINTEND(readability-function-cognitive-complexity)

bool HasSolidBoundaryFromCurrent(bool left_boundary,
                                 const PlannerSemanticMapManager& psmm,
                                 const mapping::LanePath& lane_path,
                                 double preview_length) {
  const auto check_side = left_boundary ? mapping::BoundarySide::kLeft
                                        : mapping::BoundarySide::kRight;

  constexpr double kCrossingBoundaryRatio = 0.4;
  const auto crossing_lane_pt =
      lane_path.ArclengthToLanePoint(kCrossingBoundaryRatio * preview_length);
  const auto crossing_type_or =
      QueryLanePointBoundaryType(psmm, crossing_lane_pt, left_boundary);
  if (crossing_type_or.ok() && *crossing_type_or != std::nullopt &&
      IsBoundarySolid(**crossing_type_or, check_side)) {
    return true;
  }
  const auto preview_lane_pt = lane_path.ArclengthToLanePoint(preview_length);
  const auto preview_type_or =
      QueryLanePointBoundaryType(psmm, preview_lane_pt, left_boundary);
  if (preview_type_or.ok() && *preview_type_or != std::nullopt &&
      IsBoundarySolid(**preview_type_or, check_side)) {
    return true;
  }
  return false;
}

absl::flat_hash_map<mapping::ElementId, bool> CollectSolidBoundaryInfo(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids,
    const std::vector<LanePathInfo>& lp_infos, double preview_length,
    const absl::flat_hash_map<mapping::ElementId, int>& index_diff_map,
    const std::vector<double>& l_offsets) {
  absl::flat_hash_map<mapping::ElementId, bool> solid_boundary_map;
  for (const auto lane_id : lane_ids) solid_boundary_map[lane_id] = false;

  absl::flat_hash_map<mapping::ElementId, double> l_offsets_map;
  l_offsets_map.reserve(lane_ids.size());
  for (int i = 0; i < lane_ids.size(); ++i) {
    l_offsets_map[lane_ids[i]] = l_offsets[i];
  }

  for (const auto& lp_info : lp_infos) {
    const auto start_lane_id = lp_info.start_lane_id();
    const int index_diff = FindOrDie(index_diff_map, start_lane_id);
    if (std::fabs(FindWithDefault(l_offsets_map, start_lane_id, 0.0)) <
        kIgnoreLaneBoundaryMaxDist) {
      continue;
    }

    solid_boundary_map[start_lane_id] = HasSolidBoundaryFromCurrent(
        /*left_boundary=*/index_diff > 0, psmm, lp_info.lane_path(),
        preview_length);
  }
  // From current to right.
  for (int i = 1; i < lane_ids.size(); ++i) {
    const int cur_index_diff = FindOrDie(index_diff_map, lane_ids[i]);
    if (cur_index_diff <= 0 ||
        cur_index_diff * FindOrDie(index_diff_map, lane_ids[i - 1]) < 0)
      continue;
    solid_boundary_map[lane_ids[i]] |= solid_boundary_map[lane_ids[i - 1]];
  }
  // From current to left.
  for (int i = lane_ids.size() - 2; i >= 0; --i) {
    const int cur_index_diff = FindOrDie(index_diff_map, lane_ids[i]);
    if (cur_index_diff >= 0 ||
        cur_index_diff * FindOrDie(index_diff_map, lane_ids[i + 1]) < 0)
      continue;
    solid_boundary_map[lane_ids[i]] |= solid_boundary_map[lane_ids[i + 1]];
  }

  return solid_boundary_map;
}

struct LanePathCost {
  int index_diff;
  bool solid_boundary;
  double cost;

  bool operator<(const LanePathCost& other) const {
    const int abs_idx_diff = std::abs(index_diff);
    const int other_abs_idx_diff = std::abs(other.index_diff);
    return abs_idx_diff < other_abs_idx_diff ||
           (abs_idx_diff == other_abs_idx_diff &&
            solid_boundary < other.solid_boundary) ||
           (abs_idx_diff == other_abs_idx_diff &&
            solid_boundary == other.solid_boundary && cost < other.cost);
  }

  bool operator==(const LanePathCost& other) const {
    constexpr double kEpsilon = 1e-12;
    const int abs_idx_diff = std::abs(index_diff);
    const int other_abs_idx_diff = std::abs(other.index_diff);
    return abs_idx_diff == other_abs_idx_diff &&
           solid_boundary == other.solid_boundary &&
           std::fabs(cost - other.cost) < kEpsilon;
  }
};

int FindMostSimilarLanePathIndexToLastTargetLanePath(
    const std::vector<LanePathInfo>& lp_infos,
    const std::vector<LanePathCost>& lane_path_costs,
    const mapping::LanePath& last_target_lane_path, double ego_v) {
  if (last_target_lane_path.IsEmpty()) return -1;

  constexpr double kCostEpsilon = 1e-10;
  constexpr double kMinSharedTimeOnLanePath = 2.0;  // s.
  const double shared_len_thres = kMinSharedTimeOnLanePath * ego_v;

  int best_index = -1;
  double best_cost = DBL_MAX;
  double best_shared_len = 0.0;
  for (int i = 0; i < lp_infos.size(); ++i) {
    if (lp_infos[i].start_lane_id() !=
        last_target_lane_path.front().lane_id()) {
      // Assume already aligned.
      continue;
    }

    const auto lane_path =
        lp_infos[i].lane_path().BeforeArclength(lp_infos[i].max_reach_length());
    const int lp_size =
        std::min(lane_path.size(), last_target_lane_path.size());
    int last_shared_lane_idx = 0;
    while (last_shared_lane_idx + 1 < lp_size &&
           lane_path.lane_id(last_shared_lane_idx + 1) ==
               last_target_lane_path.lane_id(last_shared_lane_idx + 1)) {
      ++last_shared_lane_idx;
    }
    const double shared_len = lane_path.LaneIndexPointToArclength(
        last_shared_lane_idx,
        std::min(lane_path.lane_segment(last_shared_lane_idx).end_fraction,
                 last_target_lane_path.lane_segment(last_shared_lane_idx)
                     .end_fraction));

    if (shared_len >= shared_len_thres &&
        (lane_path_costs[i].cost < best_cost - kCostEpsilon ||
         (lane_path_costs[i].cost < best_cost + kCostEpsilon &&
          shared_len > best_shared_len))) {
      best_index = i;
      best_cost = lane_path_costs[i].cost;
      best_shared_len = shared_len;
    }
  }
  return best_index;
}

std::pair<mapping::LanePoint, mapping::LanePoint> FindNeighborLanePoints(
    const RouteSectionsInfo& route_sections_info,
    const mapping::LanePoint& lane_pt) {
  const auto& cur_sec =
      *route_sections_info.FindSegmentContainingLanePointOrNull(lane_pt);
  const int cur_idx = FindOrDie(cur_sec.id_idx_map, lane_pt.lane_id());

  auto left_lane_pt = cur_idx > 0
                          ? mapping::LanePoint(cur_sec.lane_ids[cur_idx - 1],
                                               lane_pt.fraction())
                          : mapping::LanePoint();
  auto right_lane_pt = cur_idx + 1 < cur_sec.lane_ids.size()
                           ? mapping::LanePoint(cur_sec.lane_ids[cur_idx + 1],
                                                lane_pt.fraction())
                           : mapping::LanePoint();
  return {left_lane_pt, right_lane_pt};
}

bool ShouldChooseLhsLanePathInfo(const PlannerSemanticMapManager& psmm,
                                 const LanePathInfo& lhs,
                                 const LanePathInfo& rhs) {
  const auto& left_lane_ids = lhs.lane_path().lane_ids();
  const auto& right_lane_ids = rhs.lane_path().lane_ids();
  int i = 0;
  mapping::ElementId last_shared_lane_id = mapping::kInvalidElementId;
  for (; i < std::min(left_lane_ids.size(), right_lane_ids.size()); ++i) {
    if (left_lane_ids[i] != right_lane_ids[i]) {
      if (i > 0) {
        last_shared_lane_id = left_lane_ids[i - 1];
      }
      break;
    }
  }
  constexpr double kEpsilon = 1e-9;
  const bool default_result = lhs.path_cost() + kEpsilon < rhs.path_cost();
  if (last_shared_lane_id == mapping::kInvalidElementId ||
      i + 1 == left_lane_ids.size() || i + 1 == right_lane_ids.size()) {
    return default_result;
  }

  SMM_ASSIGN_LANE_OR_RETURN(shared_lane_info, psmm, last_shared_lane_id,
                            default_result);
  SMM_ASSIGN_LANE_OR_RETURN(left_lane_info, psmm, left_lane_ids[i],
                            default_result);
  SMM_ASSIGN_LANE_OR_RETURN(right_lane_info, psmm, right_lane_ids[i],
                            default_result);
  constexpr double kQueryDistance = 5.0;  // m.
  const double frac =
      std::max(0.0, (shared_lane_info.length() - kQueryDistance) /
                        shared_lane_info.length());
  const Vec2d shared_point = shared_lane_info.LerpPointFromFraction(frac);
  const Vec2d tangent = shared_lane_info.GetTangent(frac);

  const Vec2d left_point = left_lane_info.LerpPointFromFraction(
      std::min(1.0, kQueryDistance / left_lane_info.length()));
  const Vec2d right_point = right_lane_info.LerpPointFromFraction(
      std::min(1.0, kQueryDistance / right_lane_info.length()));

  const double left_abs_cross_prod =
      std::fabs((left_point - shared_point).CrossProd(tangent));
  const double right_abs_cross_prod =
      std::fabs((right_point - shared_point).CrossProd(tangent));

  const bool both_are_turning =
      left_lane_info.proto->direction() != mapping::LaneProto::STRAIGHT &&
      right_lane_info.proto->direction() != mapping::LaneProto::STRAIGHT;
  return both_are_turning ? left_abs_cross_prod > right_abs_cross_prod
                          : left_abs_cross_prod < right_abs_cross_prod;
}

std::optional<int> FindDiscouragedForkRightMostLanePathInHighWay(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& route_sections_info,
    const std::vector<LanePathInfo>& lp_infos,
    const RouteNaviInfo& route_navi_info) {
  if (!route_navi_info.in_highway) {
    return std::nullopt;
  }
  absl::flat_hash_map<mapping::ElementId, std::vector<int>>
      start_lane_index_map;
  const int lp_size = lp_infos.size();
  start_lane_index_map.reserve(lp_size);
  for (int i = 0; i < lp_size; ++i) {
    start_lane_index_map[lp_infos[i].start_lane_id()].push_back(i);
  }
  if (start_lane_index_map.size() == lp_size) return std::nullopt;

  int right_most_index = -1;
  for (const auto& [start_lane, index_vec] : start_lane_index_map) {
    if (index_vec.size() == 1) {
      continue;
    }
    double min_recommend_length = DBL_MAX;
    for (const int index : index_vec) {
      const auto first_fork_lk_lane_id =
          lp_infos[index].first_fork_lk_lane_id();
      const auto* fork_lane_info_ptr = FindOrNull(
          route_navi_info.route_lane_info_map, first_fork_lk_lane_id);
      const double cur_recommend_length =
          fork_lane_info_ptr == nullptr
              ? 0.0
              : fork_lane_info_ptr->recommend_reach_length;
      min_recommend_length =
          std::min(min_recommend_length, cur_recommend_length);
      if (right_most_index == -1 &&
          IsRightMostDrivableLane(psmm, first_fork_lk_lane_id)) {
        right_most_index = index;
      }
    }
    if (min_recommend_length < kDiscourageRightMostMinDist) {
      right_most_index = -1;
      continue;
    }

    if (right_most_index != -1) {
      const auto& lane_ids = lp_infos[right_most_index].lane_path().lane_ids();
      const auto iter =
          std::find(lane_ids.begin(), lane_ids.end(),
                    lp_infos[right_most_index].first_fork_lk_lane_id());
      if (iter != lane_ids.end() &&
          PreviewMinDrivableLanes(
              psmm, route_sections_info, kHighwayPreviewDist,
              iter - lane_ids.begin()) >= kMinLaneSizeDiscourageRightMost) {
        return std::make_optional<int>(right_most_index);
      } else {
        right_most_index = -1;
      }
    }
  }

  return std::nullopt;
}
}  // namespace

std::vector<LanePathInfo> FilterMultipleTargetLanePath(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& route_sections_info,
    const RouteNaviInfo& route_navi_info,
    const mapping::LanePath& last_target_lane_path,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& preferred_lane_path,
    const mapping::LanePath& lc_preview_lane_path,
    std::vector<LanePathInfo>* mutable_lp_infos) {
  // NOLINTBEGIN(readability-function-cognitive-complexity)
  SCOPED_QTRACE("FilterMultipleTargetLanePath");

  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const double ego_v = plan_start_point.v();

  constexpr double kLcPreviewTime = 3.0;     // s.
  constexpr double kMinLcPreviewLen = 20.0;  // m.
  const double lc_preview_length =
      std::max(kMinLcPreviewLen, kLcPreviewTime * ego_v);

  const auto& front_sec_info = route_sections_info.front();
  // Left -> right: negative -> positive.
  int neutral_next_idx = -1;
  mapping::ElementId closest_lane_id = mapping::kInvalidElementId;
  const auto l_offsets = CollectLaneLatOffsets(
      psmm, front_sec_info.lane_ids, front_sec_info.start_fraction,
      front_sec_info.end_fraction, ego_pos, &neutral_next_idx,
      &closest_lane_id);
  const auto lane_idx_diff_map =
      CollectLaneIndexDiff(psmm, front_sec_info, l_offsets, *mutable_lp_infos,
                           neutral_next_idx, closest_lane_id);
  const auto solid_boundary_map =
      CollectSolidBoundaryInfo(psmm, front_sec_info.lane_ids, *mutable_lp_infos,
                               lc_preview_length, lane_idx_diff_map, l_offsets);

  for (auto& lp_info : *mutable_lp_infos) {
    lp_info.set_is_solid_lane_change(
        FindWithDefault(solid_boundary_map, lp_info.start_lane_id(), false));
  }
  const auto lp_infos = std::move(*mutable_lp_infos);
  const int n_lps = lp_infos.size();
  // Vector indices correspond to start lanes' indices in the current section.
  std::vector<LanePathCost> lane_path_costs(n_lps);
  for (int i = 0; i < n_lps; ++i) {
    lane_path_costs[i] = {
        FindOrDie(lane_idx_diff_map, lp_infos[i].start_lane_id()),
        FindOrDie(solid_boundary_map, lp_infos[i].start_lane_id()),
        ComputeLanePathCost(lp_infos[i], route_sections_info, route_navi_info)};
  }

  const auto right_most_idx_opt = FindDiscouragedForkRightMostLanePathInHighWay(
      psmm, route_sections_info, lp_infos, route_navi_info);
  if (right_most_idx_opt.has_value()) {
    lane_path_costs[*right_most_idx_opt].cost += kRightMostLanePathCost;
  }
  // If the last target lane path is still viable, choose it as one candidate.
  const int last_target_index =
      FindMostSimilarLanePathIndexToLastTargetLanePath(
          lp_infos, lane_path_costs, last_target_lane_path, ego_v);
  if (last_target_index != -1) {
    if (std::abs(lane_path_costs[last_target_index].index_diff) > 1) {
      lane_path_costs[last_target_index].index_diff =
          std::copysign(1, lane_path_costs[last_target_index].index_diff);
    }
    lane_path_costs[last_target_index].solid_boundary = false;
    lane_path_costs[last_target_index].cost += kLastTargetLanePathReward;

    const auto& front_sec_idx_map = front_sec_info.id_idx_map;
    const int last_start_id_idx = FindOrDie(
        front_sec_idx_map, lp_infos[last_target_index].start_lane_id());

    const auto preview_lane_pt =
        lp_infos[last_target_index].lane_path().ArclengthToLanePoint(
            lc_preview_length);
    const auto [left_lane_pt, right_lane_pt] =
        FindNeighborLanePoints(route_sections_info, preview_lane_pt);
    for (int i = 0; i < n_lps; ++i) {
      if (i == last_target_index) continue;

      if (std::abs(FindOrDie(front_sec_idx_map, lp_infos[i].start_lane_id()) -
                   last_start_id_idx) > 1 ||
          (!lp_infos[i].lane_path().ContainsLanePoint(preview_lane_pt) &&
           ((i < last_target_index &&
             !lp_infos[i].lane_path().ContainsLanePoint(left_lane_pt)) ||
            (i > last_target_index &&
             !lp_infos[i].lane_path().ContainsLanePoint(right_lane_pt))))) {
        // For lane paths that split later, do not consider them now.
        lane_path_costs[i].index_diff = kInvalidIndexDiff;
      }
    }
  }  // NOLINTEND(readability-function-cognitive-complexity)

  absl::flat_hash_set<mapping::ElementId> preferred_lanes(
      preferred_lane_path.lane_ids().begin(),
      preferred_lane_path.lane_ids().end());
  for (int i = 0; i < n_lps; ++i) {
    if (preferred_lanes.contains(lp_infos[i].start_lane_id())) {
      // Guarantee the closest preferred lane is selected.
      lane_path_costs[i].index_diff =
          std::copysign(1, lane_path_costs[i].index_diff);
      lane_path_costs[i].solid_boundary = false;
      lane_path_costs[i].cost += kPreferredLanePathReward;
    }
  }

  if (!lc_preview_lane_path.IsEmpty()) {
    const int lc_preview_index =
        FindMostSimilarLanePathIndexToLastTargetLanePath(
            lp_infos, lane_path_costs, lc_preview_lane_path, ego_v);
    if (lc_preview_index != -1) {
      lane_path_costs[lc_preview_index].index_diff =
          std::copysign(1, lane_path_costs[lc_preview_index].index_diff);
      lane_path_costs[lc_preview_index].solid_boundary = false;
      lane_path_costs[lc_preview_index].cost += kLcPreviewLanePathReward;
    }
  }

  std::vector<int> idx_vec(n_lps);
  std::iota(idx_vec.begin(), idx_vec.end(), 0);
  std::stable_sort(idx_vec.begin(), idx_vec.end(),
                   [&lane_path_costs, &lp_infos, &psmm](int i1, int i2) {
                     if (lane_path_costs[i1] == lane_path_costs[i2]) {
                       return ShouldChooseLhsLanePathInfo(psmm, lp_infos[i1],
                                                          lp_infos[i2]);
                     }
                     return lane_path_costs[i1] < lane_path_costs[i2];
                   });

  // Keep only the best one for each start lane id.
  absl::flat_hash_set<mapping::ElementId> start_lane_set;
  for (int i = 0; i < n_lps; ++i) {
    const int lane_idx = idx_vec[i];
    if (!start_lane_set.contains(lp_infos[lane_idx].start_lane_id())) {
      start_lane_set.insert(lp_infos[lane_idx].start_lane_id());
      mutable_lp_infos->emplace_back(lp_infos[lane_idx]);
    }
  }

  const int res_size = std::min(n_lps, FLAGS_planner_est_parallel_branch_num);
  std::vector<LanePathInfo> results;
  results.reserve(res_size);
  for (int i = 0, prev_lane_diff = kInvalidIndexDiff; i < n_lps; ++i) {
    const int lane_idx = idx_vec[i];
    // Discourage one single lane change over multiple lanes.
    if (std::abs(lane_path_costs[lane_idx].index_diff) > 1) break;

    // Same index diff, choose only the better one.
    if (lane_path_costs[lane_idx].index_diff == prev_lane_diff) continue;

    results.push_back(lp_infos[lane_idx]);
    prev_lane_diff = lane_path_costs[lane_idx].index_diff;
    if (results.size() == res_size) break;
  }

  return results;
}

}  // namespace qcraft::planner
