#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
// IWYU pragma: no_include <boost/container/vector.hpp>

#include <limits.h>

#include <algorithm>
#include <ostream>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

#include "onboard/async/parallel_for.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/lane_graph/v2/dijkstra.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph_util.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

constexpr double kEpsilon = 0.1;

mapping::LanePath ExtendLanePath(const PlannerSemanticMapManager& psmm,
                                 const RouteSectionsInfo& sections_info,
                                 const RouteNaviInfo& route_navi_info,
                                 double local_horizon, int last_sec_idx,
                                 std::vector<mapping::ElementId> lane_ids,
                                 double start_frac, double end_frac) {
  const auto& sections = sections_info.section_segments();
  const mapping::LanePath raw_lane_path(psmm.semantic_map_manager(), lane_ids,
                                        start_frac, end_frac);
  if (raw_lane_path.length() > local_horizon) {
    return raw_lane_path.BeforeArclength(local_horizon);
  }

  double accum_len = raw_lane_path.length();
  mapping::ElementId cur_lane_id = lane_ids.back();
  if (end_frac < 1.0) lane_ids.pop_back();
  while (accum_len < local_horizon) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, cur_lane_id);
    if (end_frac == 1.0) {
      if (last_sec_idx + 1 == sections.size()) break;
      const auto& next_sec = sections[++last_sec_idx];
      mapping::ElementId outgoing_lane_id = mapping::kInvalidElementId;
      int min_lc_num = INT_MAX;
      for (const auto& next_lane_id : lane_info.outgoing_lanes()) {
        if (next_sec.id_idx_map.contains(next_lane_id)) {
          const auto* lane_navi_info_ptr =
              FindOrNull(route_navi_info.route_lane_info_map, next_lane_id);
          const double lc_num = lane_navi_info_ptr == nullptr
                                    ? INT_MAX
                                    : lane_navi_info_ptr->min_lc_num_to_target;
          if (lc_num < min_lc_num) {
            outgoing_lane_id = next_lane_id;
            min_lc_num = lc_num;
          }
        }
      }
      if (outgoing_lane_id == mapping::kInvalidElementId) break;
      cur_lane_id = outgoing_lane_id;
      end_frac = 0.0;
    } else {
      lane_ids.emplace_back(cur_lane_id);
      const double full_len = lane_info.length();
      const double len =
          full_len * (sections[last_sec_idx].end_fraction - end_frac);
      if (accum_len + len > local_horizon) {
        end_frac += (local_horizon - accum_len) / full_len;
        break;
      }
      if (last_sec_idx + 1 == sections.size()) {
        end_frac = sections_info.end_fraction();
        break;
      }
      accum_len += len;
      end_frac = 1.0;
    }
  }

  return mapping::LanePath(psmm.semantic_map_manager(), std::move(lane_ids),
                           start_frac, end_frac);
}

LanePathInfo RecoverLanePathInfo(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, const v2::LaneGraph& lane_graph,
    const v2::TravelPath<v2::LaneGraph::VertexId>& travel_path,
    int* last_lk_idx_ptr = nullptr) {
  const auto& layers = lane_graph.layers;
  QCHECK(!layers.empty()) << "The input lane graph is empty!";
  const double local_horizon = sections_info.planning_horizon();
  const double start_frac = layers[0].second;
  const auto& vertices = travel_path.vertices;
  const auto& fork_lk_edges = lane_graph.fork_lk_edges;
  const int vertices_size = vertices.size();
  std::vector<mapping::ElementId> lane_ids;
  double end_frac = start_frac;
  auto [cur_layer, cur_id] = v2::FromVertId(vertices[0]);
  lane_ids.push_back(cur_id);

  int last_index = vertices_size;
  mapping::ElementId first_fork_lk_lane_id = mapping::kInvalidElementId;
  for (int i = 0; i + 1 < vertices_size; ++i) {
    if (first_fork_lk_lane_id == mapping::kInvalidElementId &&
        std::find_if(fork_lk_edges.begin(), fork_lk_edges.end(),
                     [&vertices, i](const auto& edge) {
                       return edge.second == vertices[i];
                     }) != fork_lk_edges.end()) {
      first_fork_lk_lane_id = mapping::ElementId(vertices[i].second);
    }
    if (lane_graph.graph.edge_cost(vertices[i], vertices[i + 1]) >
        v2::LaneGraph::kDeadEndToTargetCost - kEpsilon) {
      last_index = i + 1 == vertices_size - 1 ? i + 1 : i;
      break;
    }
  }

  int last_lk_idx = 0;
  bool end_vert_found = false;
  for (int i = 1; i < last_index;) {
    auto [next_layer, next_id] = v2::FromVertId(vertices[i]);
    while (i < last_index && next_id == cur_id) {
      std::tie(next_layer, next_id) = v2::FromVertId(vertices[++i]);
    }

    const auto lanes_between = v2::DirectlyConnectedLanesBetween(
        *psmm.semantic_map_manager(), sections_info, layers, cur_layer,
        next_layer, cur_id, next_id);
    if (lanes_between.empty()) {
      end_frac = layers[v2::FromVertId(vertices[i - 1]).first].second;
      last_lk_idx = i - 1;
      end_vert_found = true;
      break;
    }
    lane_ids.insert(lane_ids.end(), lanes_between.begin(), lanes_between.end());
    cur_layer = next_layer;
    cur_id = next_id;
  }
  if (!end_vert_found) {
    last_lk_idx = std::max(0, std::min(last_index - 1, vertices_size - 2));
    end_frac = layers[v2::FromVertId(vertices[last_lk_idx]).first].second;
  }
  if (last_lk_idx_ptr != nullptr) *last_lk_idx_ptr = last_lk_idx;

  const int last_lk_layer = v2::FromVertId(vertices[last_lk_idx]).first;
  mapping::LanePath lane_path = ExtendLanePath(
      psmm, sections_info, route_navi_info, local_horizon,
      layers[last_lk_layer].first, lane_ids, start_frac, end_frac);
  const double length_along_lane_path =
      std::min(kPlannerLaneGraphLength,
               v2::GetSectionsLength(*psmm.semantic_map_manager(), lane_ids,
                                     start_frac, end_frac));
  const double actual_cost =
      travel_path.total_cost - FindWithDefault(lane_graph.to_target_extra_cost,
                                               *(vertices.rbegin() + 1), 1.0);
  LanePathInfo lp_info(std::move(lane_path), length_along_lane_path,
                       actual_cost, psmm);
  lp_info.set_first_fork_lk_lane_id(first_fork_lk_lane_id);
  return lp_info;
}

std::vector<LanePathInfo> FindBestLanePathsFrom(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, const v2::LaneGraph& lane_graph,
    mapping::ElementId lane_id) {
  ASSIGN_OR_RETURN(const auto path,
                   v2::Dijkstra(lane_graph.graph, v2::ToVertId(0, lane_id),
                                v2::LaneGraph::kTargetVertex),
                   std::vector<LanePathInfo>());
  int last_lk_idx = 0;
  std::vector<LanePathInfo> lp_infos{RecoverLanePathInfo(
      psmm, sections_info, route_navi_info, lane_graph, path, &last_lk_idx)};

  const auto& vertices = path.vertices;
  for (int i = 0; i < last_lk_idx; ++i) {
    if (lane_graph.fork_lk_edges.contains({vertices[i], vertices[i + 1]})) {
      // Try to find the second-best path from the first diverging point.
      auto new_graph = lane_graph.graph;
      new_graph.RemoveEdge(vertices[i], vertices[i + 1]);
      ASSIGN_OR_RETURN(
          auto fork_path,
          v2::Dijkstra(new_graph, vertices[i], v2::LaneGraph::kTargetVertex),
          lp_infos);

      auto& fork_vertices = fork_path.vertices;
      fork_vertices.insert(fork_vertices.begin(), vertices.begin(),
                           vertices.begin() + i);
      for (int j = 0; j < i; ++j) {
        fork_path.total_cost +=
            lane_graph.graph.edge_cost(vertices[j], vertices[j + 1]);
      }
      lp_infos.emplace_back(RecoverLanePathInfo(
          psmm, sections_info, route_navi_info, lane_graph, fork_path));
      break;  // Only consider the first diverging point.
    }
  }
  return lp_infos;
}

}  // namespace

// Find the best lane paths starting from each lane in the first section.
absl::StatusOr<std::vector<LanePathInfo>> FindBestLanePathsFromStart(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, const v2::LaneGraph& lane_graph,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("FindBestLanePathsFromStart");

  const auto& start_ids = sections_info.front().lane_ids;
  std::vector<std::vector<LanePathInfo>> results(start_ids.size());
  ParallelFor(0, start_ids.size(), thread_pool,
              [&psmm, &sections_info, &route_navi_info, &lane_graph, &start_ids,
               &results](int i) {
                results[i] =
                    FindBestLanePathsFrom(psmm, sections_info, route_navi_info,
                                          lane_graph, start_ids[i]);
              });

  constexpr double kGraphLayerResolution = 10.0;  // m.
  std::vector<LanePathInfo> lp_infos;
  lp_infos.reserve(start_ids.size() * 2);  // At most one fork per start lane.
  for (auto& result : results) {
    for (auto& lp_info_from_lane : result) {
      if (lp_info_from_lane.empty()) continue;

      const auto* lane_navi_info_ptr =
          FindOrNull(route_navi_info.route_lane_info_map,
                     lp_info_from_lane.start_lane_id());
      if (lane_navi_info_ptr == nullptr) {
        return absl::NotFoundError(
            absl::StrCat("Lane id ", lp_info_from_lane.start_lane_id(),
                         " not found in route navi info!"));
      }
      if (lp_info_from_lane.length_along_route() >
          kPlannerLaneGraphLength - kEpsilon) {
        lp_info_from_lane.set_length_along_route(
            lane_navi_info_ptr->recommend_reach_length);
      }
      lp_info_from_lane.set_max_reach_length(
          lp_info_from_lane.length_along_route() <
                  lane_navi_info_ptr->recommend_reach_length -
                      kGraphLayerResolution
              ? lp_info_from_lane.length_along_route()
              : lane_navi_info_ptr->max_reach_length);

      lp_infos.emplace_back(std::move(lp_info_from_lane));
    }
  }

  return lp_infos;
}

}  // namespace qcraft::planner
