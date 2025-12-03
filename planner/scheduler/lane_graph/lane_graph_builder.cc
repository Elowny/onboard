#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"

#include <algorithm>
#include <utility>
#include <vector>

// IWYU pragma: no_include <boost/container/vector.hpp>
#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

// Extend local map here to prevent sudden change of route look ahead cost.
constexpr double kExtendedLocalHorizon =
    kPlannerLaneGraphLength + 5.0 * kMinLcLaneLength;
constexpr int kLayerNumPerLcLen = 2;  // Should be larger than 1.
constexpr double kToTargetExtraCostCoeff = 1e-3;

using AvoidLanes = absl::flat_hash_set<mapping::ElementId>;

void ProjectStationaryObjects(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info, bool target_in_horizon,
    const PlannerObjectManager& obj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const v2::LaneGraph::LaneGraphLayers& layers,
    v2::VertexGraph<v2::LaneGraph::VertexId>* graph) {
  std::vector<Box2d> obj_boxes;
  obj_boxes.reserve(stalled_objects.size());
  for (const auto obj_ptr : obj_mgr.stationary_objects()) {
    if (!stalled_objects.contains(obj_ptr->id())) continue;

    constexpr double kObjectBoxBuffer = 0.8;  // m.
    auto obj_box = obj_ptr->contour().MinAreaBoundingBox();
    obj_box.LongitudinalExtend(kObjectBoxBuffer);
    obj_box.LateralExtend(kObjectBoxBuffer);
    ASSIGN_OR_CONTINUE(const auto obj_proj,
                       FindSmoothPointOnRouteSectionsByLateralOffset(
                           psmm, sections_info, obj_box.center()));
    ASSIGN_OR_CONTINUE(
        const auto tmp_lane_path,
        BuildLanePathFromData(
            mapping::LanePathData(/*start_fraction=*/0.0, /*end_fraction=*/1.0,
                                  {obj_proj.lane_id}),
            psmm));
    if (!IsLanePathBlockedByBox2dAtLevel(psmm.GetLevel(), psmm, obj_box,
                                         tmp_lane_path, /*lat_thres=*/0.0)) {
      continue;
    }

    obj_boxes.push_back(std::move(obj_box));
  }
  if (obj_boxes.empty()) return;

  const auto& sections = sections_info.section_segments();
  absl::flat_hash_map<v2::LaneGraph::VertexId, Vec2d> vertex_pos;
  // If destination is within horizon, leave the last several layers to handle
  // stalled objects on route end that may block all lane paths.
  const int last_layer_idx =
      target_in_horizon ? layers.size() - kLayerNumPerLcLen : layers.size();
  for (int i = 0; i < last_layer_idx; ++i) {
    const auto [sec_idx, frac] = layers[i];
    for (const auto lane_id : sections[sec_idx].lane_ids) {
      vertex_pos[v2::ToVertId(i, lane_id)] =
          ComputeLanePointPos(psmm, mapping::LanePoint(lane_id, frac));
    }
  }

  for (const auto& from_vert : graph->vertices()) {
    if (!vertex_pos.contains(from_vert)) continue;

    for (const auto& [to_vert, edge_cost] : graph->edges_from(from_vert)) {
      if (!vertex_pos.contains(to_vert)) continue;

      const Segment2d edge_seg(vertex_pos[from_vert], vertex_pos[to_vert]);
      for (const auto& box : obj_boxes) {
        if (box.HasOverlap(edge_seg)) {
          graph->ModifyEdge(from_vert, to_vert,
                            v2::LaneGraph::kDeadEndToTargetCost);
          break;
        }
      }
    }
  }
}

}  // namespace

v2::LaneGraph BuildLaneGraph(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info, const PlannerObjectManager& obj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const AvoidLanes& avoid_lanes, const RouteNaviInfo& route_navi_info) {
  SCOPED_QTRACE("BuildLaneGraph");

  const auto& sections = sections_info.section_segments();
  // Find the last section and its end fraction.
  // Psmm is updated async and need cross patch.
  const auto* destination_section_ptr =
      psmm.FindSectionInfoOrNull(sections.back().id);
  const bool target_in_horizon =
      (sections_info.length() <= kExtendedLocalHorizon) &&
      destination_section_ptr != nullptr;
  int last_sec_idx = 0;
  double last_sec_frac = 0.0;
  if (target_in_horizon) {
    last_sec_idx = sections_info.size() - 1;
    last_sec_frac = sections_info.end_fraction();
  } else {
    double accum_len = 0.0;
    bool not_loaded_yet = false;
    for (; last_sec_idx < sections_info.size(); ++last_sec_idx) {
      const auto& section = sections[last_sec_idx];
      if (section.lane_ids.empty()) {
        not_loaded_yet = true;
        break;
      }
      if (accum_len + section.length() >= kExtendedLocalHorizon) {
        last_sec_frac =
            section.start_fraction +
            (kExtendedLocalHorizon - accum_len) / section.average_length;
        break;
      }
      accum_len += section.length();
    }
    if (not_loaded_yet) {
      last_sec_frac = 1.0;
      --last_sec_idx;
    }
  }

  v2::VertexGraph<v2::LaneGraph::VertexId> graph;
  auto layers =
      v2::SplitSectionsAsVertices(sections_info, last_sec_idx, last_sec_frac);
  // Add vertices.
  for (int i = 0; i < layers.size(); ++i) {
    for (const auto lane_id : sections[layers[i].first].lane_ids) {
      graph.AddVertex(v2::ToVertId(i, lane_id.value()));
    }
  }
  graph.AddVertex(v2::LaneGraph::kTargetVertex);

  v2::ConnectEdgesWithinSections(*psmm.semantic_map_manager(), sections_info,
                                 avoid_lanes, layers, &graph);

  absl::flat_hash_set<v2::LaneGraph::VertexId> fork_lc_verts;
  absl::flat_hash_set<v2::LaneGraph::EdgeType> fork_lk_edges;
  v2::ConnectEdgesAcrossSections(*psmm.semantic_map_manager(), sections_info,
                                 avoid_lanes, layers, last_sec_idx, &graph,
                                 &fork_lc_verts, &fork_lk_edges);

  // Project stationary objects onto lane graph and modify edge costs.
  ProjectStationaryObjects(psmm, sections_info, target_in_horizon, obj_mgr,
                           stalled_objects, layers, &graph);

  // Add normal connection edges to the target vertex.
  const auto last_layer = layers.size() - 1;
  const bool destination_in_horizon =
      target_in_horizon &&
      sections_info.destination().lane_id() != mapping::kInvalidElementId;
  absl::flat_hash_map<v2::LaneGraph::VertexId, double> to_target_extra_cost;
  if (destination_in_horizon) {
    // From the target point.
    for (const auto lane_id : sections.back().lane_ids) {
      if (lane_id == sections_info.destination().lane_id()) {
        graph.AddEdge(v2::ToVertId(last_layer, lane_id),
                      v2::LaneGraph::kTargetVertex, 0.0);
      } else {
        // Avoid kickout when can not connect to target vertex.
        graph.AddEdge(v2::ToVertId(last_layer, lane_id),
                      v2::LaneGraph::kTargetVertex,
                      v2::LaneGraph::kDeadEndToTargetCost);
      }
    }
  } else {
    // From vertices of the last layer that extends beyond horizon.
    const auto& last_sec = sections[last_sec_idx];
    const auto& lane_info_map = route_navi_info.route_lane_info_map;
    for (const auto lane_id : last_sec.lane_ids) {
      const auto lane_info_iter = lane_info_map.find(lane_id);
      const double target_extra_cost =
          lane_info_iter == lane_info_map.end()
              ? 1.0
              : 1.0 / (lane_info_iter->second.max_driving_distance + 1e-3) *
                    kToTargetExtraCostCoeff;
      graph.AddEdge(v2::ToVertId(last_layer, lane_id),
                    v2::LaneGraph::kTargetVertex, target_extra_cost);
      to_target_extra_cost[v2::ToVertId(last_layer, lane_id)] =
          target_extra_cost;
    }
  }

  return v2::LaneGraph{.layers = std::move(layers),
                       .graph = std::move(graph),
                       .fork_lc_verts = std::move(fork_lc_verts),
                       .fork_lk_edges = std::move(fork_lk_edges),
                       .to_target_extra_cost = std::move(to_target_extra_cost)};
}

}  // namespace qcraft::planner
