#include "onboard/planner/router/navi/route_navi_info_builder.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
// IWYU pragma: no_include <float.h>
// IWYU pragma: no_include <map>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "boost/container/vector.hpp"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/planner/router/lane_graph/v2/dijkstra.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph_util.h"
#include "onboard/planner/router/lane_graph/v2/route_lane_graph_builder.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/navi/route_navi_util.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/router_defs.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/planner/util/scene_util.h"
#include "onboard/proto/route.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner {

namespace {
constexpr double kOutNaviInfoDist = 1000.0;          // m.
constexpr double kLaneGraphInfoDistNormal = 50.0;    // m.
constexpr double kLaneGraphInfoDistHighWay = 100.0;  // m.
constexpr double kExtendDrivingDist = 200.0;         // m.
constexpr double kExtendDrivingDistHighWay = 500.0;  // m.
constexpr double kPreviewMergeLaneDist = 3000.0;     // m.
constexpr double kMaxDeduceDist = 500.0;             //  m.

struct RecommendPathInfo {
  std::vector<mapping::ElementId> lane_ids;
  double start_fraction = 0.0;
  double end_fraction = 0.0;
};

struct LaneNaviInfo {
  double max_reach_length = 0.0;
  double recommend_reach_length = 0.0;
  RecommendPathInfo recommend_path_info;
};

struct LaneGraphResultInfo {
  double recommend_reach_length = 0.0;
  RecommendPathInfo path_info;
};

bool IsTwoLanesConnected(const mapping::v2::SemanticMapManager& v2smm,
                         mapping::ElementId source_id,
                         mapping::ElementId target_id) {
  constexpr int kExpandLayers = 5;
  std::queue<std::pair<mapping::ElementId, int>> que;
  que.push({source_id, 0});

  while (!que.empty()) {
    const auto [cur_id, layer] = que.front();
    que.pop();

    if (layer > kExpandLayers) break;

    SMM2_ASSIGN_LANE_OR_CONTINUE(lane_info, v2smm, cur_id);
    for (const auto next_id : lane_info.proto().outgoing_lanes()) {
      const auto out_lane_id = mapping::ElementId(next_id);
      if (out_lane_id == target_id) return true;
      que.push({out_lane_id, layer + 1});
    }
  }

  return false;
}

void DeduceLaneNaviInfo(
    const mapping::v2::SemanticMapManager& v2smm,
    const LaneNaviInfo& base_lane_info,
    absl::flat_hash_map<mapping::ElementId, RouteNaviInfo::RouteLaneInfo>*
        out) {
  double reduced_len = 0.0;
  const auto& path_info = base_lane_info.recommend_path_info;
  for (int i = 1; i < path_info.lane_ids.size(); ++i) {
    SMM2_ASSIGN_LANE_OR_BREAK(prev_lane_info, v2smm, path_info.lane_ids[i - 1]);
    SMM2_ASSIGN_SECTION_OR_BREAK(
        prev_sec_info, v2smm,
        mapping::SectionId(prev_lane_info.proto().section_id()));
    reduced_len += prev_sec_info.proto().average_length();
    if (reduced_len > kMaxDeduceDist) return;
    auto& cur_lane_navi_info = (*out)[path_info.lane_ids[i]];
    cur_lane_navi_info.max_reach_length =
        std::max(0.0, base_lane_info.max_reach_length - reduced_len);
    cur_lane_navi_info.recommend_reach_length =
        std::max(0.0, base_lane_info.recommend_reach_length - reduced_len);
  }
}

LaneGraphResultInfo RecoverLaneGraphResult(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info, const v2::LaneGraph& lane_graph,
    const v2::TravelPath<v2::LaneGraph::VertexId>& path, int layer) {
  FUNC_QTRACE();
  const auto& layers = lane_graph.layers;
  QCHECK(!layers.empty()) << "Input lane graph is empty.";
  const auto& vertices = path.vertices;

  std::vector<mapping::ElementId> lane_ids;
  const double start_frac = layers[layer].second;
  auto [cur_layer, cur_id] = v2::FromVertId(vertices[0]);
  lane_ids.push_back(cur_id);

  int last_index = vertices.size();
  constexpr double kEpsilon = 0.1;
  for (int i = 0; i + 1 < vertices.size(); ++i) {
    if (lane_graph.graph.edge_cost(vertices[i], vertices[i + 1]) >
        v2::LaneGraph::kDeadEndToTargetCost - kEpsilon) {
      last_index = i;
      break;
    }
  }

  double end_frac = 0.0;
  for (int i = 1; i + 1 < last_index;) {
    auto [next_layer, next_id] = v2::FromVertId(vertices[i]);
    while (i + 1 < last_index && next_id == cur_id) {
      std::tie(next_layer, next_id) = v2::FromVertId(vertices[++i]);
    }
    const auto lanes_between = v2::DirectlyConnectedLanesBetween(
        v2smm, sections_info, layers, cur_layer, next_layer, cur_id, next_id);

    if (lanes_between.empty()) {
      end_frac = layers[v2::FromVertId(vertices[i - 1]).first].second;
      break;
    }
    lane_ids.insert(lane_ids.end(), lanes_between.begin(), lanes_between.end());
    cur_layer = next_layer;
    cur_id = next_id;
  }
  LaneGraphResultInfo lane_graph_result_info;
  lane_graph_result_info.recommend_reach_length =
      v2::GetSectionsLength(v2smm, lane_ids, /*start_frac=*/0.0, end_frac);
  lane_graph_result_info.path_info = {.lane_ids = std::move(lane_ids),
                                      .start_fraction = start_frac,
                                      .end_fraction = end_frac};

  return lane_graph_result_info;
}

LaneNaviInfo CalcLaneNaviInfo(const mapping::v2::SemanticMapManager& v2smm,
                              const RouteSectionsInfo& sections_info,
                              const v2::LaneGraph& lane_graph,
                              mapping::ElementId lane_id, int layer) {
  FUNC_QTRACE();
  QCHECK(!lane_graph.layers.empty()) << "Input lane graph is empty.";

  const auto path_or = Dijkstra(lane_graph.graph, v2::ToVertId(layer, lane_id),
                                v2::LaneGraph::kTargetVertex);
  LaneNaviInfo lane_navi_info;
  if (!path_or.ok()) {
    return lane_navi_info;
  }
  auto fork_graph_result_info =
      RecoverLaneGraphResult(v2smm, sections_info, lane_graph, *path_or, layer);
  lane_navi_info.recommend_reach_length =
      fork_graph_result_info.recommend_reach_length;
  lane_navi_info.max_reach_length =
      fork_graph_result_info.recommend_reach_length;
  lane_navi_info.recommend_path_info =
      std::move(fork_graph_result_info.path_info);

  if (!lane_graph.fork_lc_verts.empty()) {
    auto new_graph = lane_graph.graph;
    for (const auto& vert_id : lane_graph.fork_lc_verts) {
      new_graph.RemoveEdgesFrom(vert_id);
    }
    const auto new_path_or = v2::Dijkstra(
        new_graph, v2::ToVertId(layer, lane_id), v2::LaneGraph::kTargetVertex);
    if (new_path_or.ok()) {
      const auto graph_result_info = RecoverLaneGraphResult(
          v2smm, sections_info, lane_graph, *new_path_or, layer);
      if (graph_result_info.recommend_reach_length >
          lane_navi_info.max_reach_length) {
        lane_navi_info.max_reach_length =
            graph_result_info.recommend_reach_length;
      }
    }
  }

  return lane_navi_info;
}

RouteNaviInfo::NaviSectionInfo CalculateNaviSectionInfo(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info, double preview_length,
    int start_section_idx) {
  FUNC_QTRACE();
  std::optional<NaviSectionInfoProto::Direction> direction = std::nullopt;
  RouteNaviInfo::NaviSectionInfo navi_section_info;
  for (int index = start_section_idx; index < sections_info.size(); ++index) {
    if (preview_length <= 0.0) break;
    const auto section_length =
        sections_info.section_segment(index).average_length;
    preview_length -= section_length;
    const auto& lane_ids = sections_info.section_segment(index).lane_ids;
    if (lane_ids.empty()) break;
    SMM2_ASSIGN_LANE_OR_CONTINUE(lane_info, v2smm, lane_ids.front());

    if (lane_info.proto().is_in_intersection()) {
      // check intersection direction
      if (!direction.has_value()) {
        switch (lane_info.proto().direction()) {
          case mapping::LaneProto::LEFT_TURN:
            direction = NaviSectionInfoProto::LEFT_TURN;
            break;
          case mapping::LaneProto::RIGHT_TURN:
            direction = NaviSectionInfoProto::RIGHT_TURN;
            break;
          case mapping::LaneProto::UTURN:
            direction = NaviSectionInfoProto::LEFT_TURN;
            break;
          case mapping::LaneProto::STRAIGHT:
            direction = NaviSectionInfoProto::STRAIGHT;
            break;
        }
        navi_section_info.intersection_direction = *direction;
      }
      // Check if the car in traffic light controlled intersection.
      for (const auto& geo_intersect_info :
           lane_info.proto().intersected_intersections()) {
        const auto intersection = v2smm.FindIntersection(
            mapping::ElementId(geo_intersect_info.other_id()));
        if (intersection != nullptr &&
            intersection->proto().traffic_light_controlled()) {
          return navi_section_info;
        }
      }
    }
    navi_section_info.length_before_intersection += section_length;
  }
  return navi_section_info;
}

void CalcForkLaneNaviInfo(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info, const v2::LaneGraph& lane_graph,
    absl::flat_hash_map<mapping::ElementId, RouteNaviInfo::RouteLaneInfo>*
        lane_navi_info_map) {
  absl::flat_hash_map<mapping::ElementId, int> fork_lane_begin_layer;
  for (const auto& [_, target_vertex] : lane_graph.fork_lk_edges) {
    auto iter =
        fork_lane_begin_layer.find(mapping::ElementId(target_vertex.second));
    if (iter == fork_lane_begin_layer.end() ||
        iter->second > target_vertex.first) {
      fork_lane_begin_layer[mapping::ElementId(target_vertex.second)] =
          target_vertex.first;
    }
  }

  for (const auto& [lane_id, layer] : fork_lane_begin_layer) {
    if (layer * kVertexSampleDist > kMaxDeduceDist) continue;
    auto iter = lane_navi_info_map->find(lane_id);
    if (iter != lane_navi_info_map->end()) {
      const auto lane_navi_info =
          CalcLaneNaviInfo(v2smm, sections_info, lane_graph, lane_id, layer);
      iter->second.max_reach_length = lane_navi_info.max_reach_length;
      iter->second.recommend_reach_length =
          lane_navi_info.recommend_reach_length;
      DeduceLaneNaviInfo(v2smm, lane_navi_info, lane_navi_info_map);
    }
  }
}
}  // namespace

absl::StatusOr<RouteNaviInfo> CalcNaviInfoByLaneGraph(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_dist) {
  SCOPED_QTRACE("CalcNaviInfoByLaneGraph");

  if (sections_info.length() < 0.0) {
    return absl::InvalidArgumentError("Input sections are invalid.");
  }

  SMM2_ASSIGN_SECTION_OR_RETURN(front_section_info, v2smm,
                                sections_info.front().id,
                                absl::NotFoundError(absl::StrFormat(
                                    "Can not find the first section info "
                                    "to calculate route navi info, id: %d",
                                    sections_info.front().id)));
  bool in_high_way = false;

  const auto road_class = front_section_info.proto().road_class();
  in_high_way = road_class == mapping::SectionProto_RoadClass_CITY_EXPRESS ||
                road_class == mapping::SectionProto_RoadClass_HIGHWAY;

  const double lane_graph_info_dist =
      in_high_way ? kLaneGraphInfoDistHighWay : kLaneGraphInfoDistNormal;

  const auto lane_graph =
      v2::BuildRouteLaneGraph(v2smm, sections_info, avoid_lanes, preview_dist);

  if (FLAGS_route_send_lane_graph_to_canvas) {
    SendRouteLaneGraphToCanvas(lane_graph, v2smm, sections_info,
                               "route_lane_graph");
  }

  const auto driving_dist_map = CalculateMaxDrivingDistance(
      v2smm, sections_info, avoid_lanes, /*from_lane_beginning=*/true);
  const auto merge_lane_info_map = CalculateLengthBeforeMergeLane(
      v2smm, sections_info, avoid_lanes, kPreviewMergeLaneDist);
  const auto min_lc_map =
      FindLcNumToTargets(v2smm, sections_info, avoid_lanes, preview_dist, 0);
  absl::flat_hash_map<mapping::ElementId, int> lc_within_driving_dist_map;

  RouteNaviInfo out_navi_info;
  auto& lane_navi_info_map = out_navi_info.route_lane_info_map;

  const auto fill_navi_info = [&out_navi_info, &driving_dist_map, &min_lc_map,
                               &lc_within_driving_dist_map,
                               &merge_lane_info_map](
                                  mapping::ElementId lane_id,
                                  const LaneNaviInfo& input_navi_info) {
    auto& lane_navi_info = out_navi_info.route_lane_info_map[lane_id];
    lane_navi_info.max_driving_distance =
        FindWithDefault(driving_dist_map, lane_id, 0.0);
    lane_navi_info.max_reach_length = input_navi_info.max_reach_length;
    lane_navi_info.recommend_reach_length =
        input_navi_info.recommend_reach_length;
    lane_navi_info.min_lc_num_to_target =
        FindWithDefault(min_lc_map, lane_id, std::numeric_limits<int>::max());
    lane_navi_info.lc_num_within_driving_dist = FindWithDefault(
        lc_within_driving_dist_map, lane_id, std::numeric_limits<int>::max());
    auto merge_lane_info =
        FindWithDefault(merge_lane_info_map, lane_id, MergeLaneInfo());
    lane_navi_info.len_before_merge_lane =
        merge_lane_info.len_before_merge_lane;
    lane_navi_info.merge_targets = std::move(merge_lane_info.merge_targets);
  };

  double accum_dist = 0.0;
  int layer_idx = 0;
  for (int i = 0; i < sections_info.size(); ++i) {
    LaneNaviInfo lane_navi_info;
    if (accum_dist < lane_graph_info_dist) {
      while (lane_graph.layers[layer_idx].first < i) {
        ++layer_idx;
      }

      for (const auto& lane_id : sections_info.section_segment(i).lane_ids) {
        const double max_driving_dist =
            FindWithDefault(driving_dist_map, lane_id, 0.0);
        const double extend_len =
            in_high_way ? kExtendDrivingDistHighWay : kExtendDrivingDist;
        const double preview_distance =
            std::min(max_driving_dist + extend_len, preview_dist - accum_dist);
        const auto lc_num_map = FindLcNumToTargets(
            v2smm, sections_info, avoid_lanes, preview_distance, i);
        lc_within_driving_dist_map[lane_id] = FindWithDefault(
            lc_num_map, lane_id, std::numeric_limits<int>::max());

        // Calculate route_lane_info if not deduced.
        if (!lane_navi_info_map.contains(lane_id)) {
          // Cross short section.
          if (lane_graph.layers[layer_idx].first != i && layer_idx > 0) {
            const auto& last_sec = sections_info.section_segment(
                lane_graph.layers[layer_idx - 1].first);
            for (const auto& last_lane_id : last_sec.lane_ids) {
              if (IsTwoLanesConnected(v2smm, last_lane_id, lane_id)) {
                lane_navi_info =
                    CalcLaneNaviInfo(v2smm, sections_info, lane_graph,
                                     last_lane_id, layer_idx - 1);
                const double length =
                    sections_info.length_between(
                        lane_graph.layers[layer_idx - 1].first, i) -
                    last_sec.length() * lane_graph.layers[layer_idx - 1].second;
                lane_navi_info.max_reach_length =
                    std::max(0.0, lane_navi_info.max_reach_length - length);
                lane_navi_info.recommend_reach_length = std::max(
                    0.0, lane_navi_info.recommend_reach_length - length);
                break;
              }
            }
          } else {
            lane_navi_info = CalcLaneNaviInfo(v2smm, sections_info, lane_graph,
                                              lane_id, layer_idx);
          }
          fill_navi_info(lane_id, lane_navi_info);
          DeduceLaneNaviInfo(v2smm, lane_navi_info, &lane_navi_info_map);
        } else {
          // Fill messages which are not computed by lane graph.
          // Todo(chengyang): unify with fill_navi_info func.
          lane_navi_info_map[lane_id].max_driving_distance =
              FindWithDefault(driving_dist_map, lane_id, 0.0);
          lane_navi_info_map[lane_id].min_lc_num_to_target = FindWithDefault(
              min_lc_map, lane_id, std::numeric_limits<int>::max());
          lane_navi_info_map[lane_id].lc_num_within_driving_dist =
              FindWithDefault(lc_within_driving_dist_map, lane_id,
                              std::numeric_limits<int>::max());
          auto merge_lane_info =
              FindWithDefault(merge_lane_info_map, lane_id, MergeLaneInfo());
          lane_navi_info_map[lane_id].len_before_merge_lane =
              merge_lane_info.len_before_merge_lane;
          lane_navi_info_map[lane_id].merge_targets =
              std::move(merge_lane_info.merge_targets);
        }
      }
    } else if (lane_graph_info_dist <= accum_dist &&
               accum_dist < kOutNaviInfoDist) {
      for (const auto& lane_id : sections_info.section_segment(i).lane_ids) {
        if (lane_navi_info_map.contains(lane_id)) {
          // Fill messages which can not be deduced.
          // Todo(chengyang): unify with fill_navi_info func.
          lane_navi_info_map[lane_id].max_driving_distance =
              FindWithDefault(driving_dist_map, lane_id, 0.0);
          lane_navi_info_map[lane_id].min_lc_num_to_target = FindWithDefault(
              min_lc_map, lane_id, std::numeric_limits<int>::max());
          lane_navi_info_map[lane_id].lc_num_within_driving_dist =
              FindWithDefault(lc_within_driving_dist_map, lane_id,
                              std::numeric_limits<int>::max());
          auto merge_lane_info =
              FindWithDefault(merge_lane_info_map, lane_id, MergeLaneInfo());
          lane_navi_info_map[lane_id].len_before_merge_lane =
              merge_lane_info.len_before_merge_lane;
          lane_navi_info_map[lane_id].merge_targets =
              std::move(merge_lane_info.merge_targets);
        } else {
          fill_navi_info(lane_id, lane_navi_info);
        }
      }
    } else if (accum_dist >= kOutNaviInfoDist) {
      break;
    }

    out_navi_info.navi_section_info_map[sections_info.section_segment(i).id] =
        CalculateNaviSectionInfo(v2smm, sections_info,
                                 preview_dist - accum_dist, i);

    accum_dist += sections_info.section_segment(i).length();
  }

  // TODO(zuowei): Separate lane graph calculation and global information
  // calculation.
  // Supply fork lanes lane graph related information.
  CalcForkLaneNaviInfo(v2smm, sections_info, lane_graph, &lane_navi_info_map);
  out_navi_info.in_highway =
      IsInHighWay(v2smm, sections_info, kPreviewEgoInHighwayDist);
  return out_navi_info;
}
}  // namespace qcraft::planner
