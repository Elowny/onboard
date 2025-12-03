#include "onboard/planner/initializer/reference_line_searcher.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {
const PiecewiseLinearFunction<double, double> kActivationBoundaryWidthRatioPlf(
    {0.0, 10.0, 20.0, 30.0, 50.0, 100.0, 10000.0},
    {0.6, 0.6, 0.8, 0.6, 0.3, 0.25, 0.25});  // (LaneKeeping)
constexpr int kDownSampleStep = 5;

std::vector<double> AddCost(absl::Span<const double> vec1,
                            absl::Span<const double> vec2) {
  const int vec1_size = vec1.size();
  std::vector<double> res(vec1_size);
  QCHECK_EQ(vec1_size, vec2.size());
  for (int i = 0; i < vec1_size; ++i) {
    res[i] = vec1[i] + vec2[i];
  }
  return res;
}

ReferenceLineSearcherOutput::EdgeInfo ComputeEdgeCost(
    const RefLineCostProvider* cost_provider, const GeometryEdge& edge,
    bool terminating) {
  ReferenceLineSearcherOutput::EdgeInfo edge_cost_info;
  edge_cost_info.feature_costs.resize(cost_provider->weights().size());
  cost_provider->ComputeRefLineCost(
      edge.geometry, terminating, absl::MakeSpan(edge_cost_info.feature_costs));
  edge_cost_info.sum_cost =
      absl::c_accumulate(edge_cost_info.feature_costs, 0.0);

  return edge_cost_info;
}

void Initialize(
    const GeometryGraph& graph,
    GeometryNodeVector<ReferenceLineSearcherOutput::NodeInfo>* ptr_cost_nodes) {
  // If the node has no outgoing edges, it means that
  // it is a terminating node on the graph.
  auto& cost_nodes = *ptr_cost_nodes;
  for (const auto& node : graph.nodes()) {
    if (!node.reachable) {
      VLOG(5) << node.index.value() << " unreachable";
      continue;  // Ignore nodes that are not reachable.
    }
    const auto& outgoing_edges = graph.GetOutgoingEdges(node.index);
    if (outgoing_edges.size() == 0) {
      VLOG(5) << "set " << node.index.value() << "min_cost to 0.0";
      cost_nodes[node.index].min_cost = 0.0;
    }
  }
}

absl::Status BuildConnection(
    const GeometryGraph& graph,
    const GeometryNodeVector<ReferenceLineSearcherOutput::NodeInfo>& cost_nodes,
    const GeometryEdgeVector<ReferenceLineSearcherOutput::EdgeInfo>& cost_edges,
    const RefLineCostProvider& cost_provider,
    std::vector<GeometryNodeIndex>* ptr_nodes_list,
    std::vector<GeometryEdgeIndex>* ptr_edges_list,
    std::vector<double>* ptr_feature_costs) {
  const auto& start_node = graph.GetStartNode();
  GeometryNodeIndex cur_node_idx = start_node.index;
  auto& nodes = *ptr_nodes_list;
  auto& edges = *ptr_edges_list;
  auto& feature_costs = *ptr_feature_costs;
  feature_costs.resize(cost_provider.cost_names().size());
  std::fill(feature_costs.begin(), feature_costs.end(), 0.0);
  while (cur_node_idx != GeometryNodeVector<GeometryNode>::kInvalidIndex) {
    nodes.push_back(cur_node_idx);
    const auto& node_info = cost_nodes[cur_node_idx];
    const auto& edge_idx = node_info.outgoing_edge_idx;
    if (edge_idx != GeometryEdgeVector<GeometryEdge>::kInvalidIndex) {
      edges.push_back(edge_idx);
      feature_costs =
          AddCost(feature_costs, cost_edges[edge_idx].feature_costs);
    }
    cur_node_idx = node_info.prev_node_idx;
    if (nodes.size() > graph.nodes_layers().size()) {
      return absl::InternalError(
          "Linkage of nodes has size greater than layer size of geometry "
          "graph, linkage error! Check for loops.");
    }
  }
  return absl::OkStatus();
}

std::vector<Vec2d> GenerateRefLinePoints(
    const GeometryGraph& graph,
    const std::vector<GeometryEdgeIndex>& edges_list) {
  std::vector<Vec2d> ref_line_points;
  for (const auto& edge_idx : edges_list) {
    if (edge_idx == GeometryEdgeVector<GeometryEdge>::kInvalidIndex) continue;
    const auto& edge = graph.GetEdge(edge_idx);
    const auto* geom_form = edge.geometry;
    const auto states = geom_form->Sample(0.5);
    for (const auto& state : states) {
      ref_line_points.emplace_back(state.xy);
    }
  }
  return ref_line_points;
}

PiecewiseLinearFunction<double, double> GetActivationRadiusPlf(
    const PathSlBoundary& path_sl, const GeometryGraph& geom_graph,
    const std::vector<GeometryNodeIndex>& node_idx_list) {
  // Generate Plf: accumulated_s <-> activation radius (for each layer) since sl
  // boundary width can be large within intersection.
  std::vector<double> accum_s;
  std::vector<double> activate_r;
  accum_s.reserve(node_idx_list.size());
  activate_r.reserve(node_idx_list.size());

  for (const auto& index : node_idx_list) {
    const auto& node = geom_graph.GetNode(index);
    accum_s.push_back(node.accumulated_s);
    // <right, left>
    const auto boundary_l = path_sl.QueryBoundaryL(node.accumulated_s);
    // Open up certain percent of width with respect to the outer boundary.
    const double activate_radius =
        0.5 * kActivationBoundaryWidthRatioPlf(node.accumulated_s) *
        (boundary_l.second - boundary_l.first);
    activate_r.push_back(activate_radius);
  }
  VLOG(1) << "Activate width plf:";
  VLOG(1) << "accum_s: " << absl::StrJoin(accum_s, ", ");
  VLOG(1) << "r: " << absl::StrJoin(activate_r, ", ");

  return PiecewiseLinearFunction<double, double>(accum_s, activate_r);
}

}  // namespace

absl::StatusOr<ReferenceLineSearcherOutput> SearchReferenceLine(
    const ReferenceLineSearcherInput& input,
    InitializerDebugProto* /*debug_proto*/, ThreadPool* /*thread_pool*/) {
  SCOPED_QTRACE("SearchReferenceLine");

  QCHECK_NOTNULL(input.geometry_graph);
  QCHECK_NOTNULL(input.drive_passage);
  QCHECK_NOTNULL(input.sl_boundary);
  QCHECK_NOTNULL(input.initializer_params);
  QCHECK_NOTNULL(input.vehicle_drive);
  QCHECK_NOTNULL(input.vehicle_geom);
  QCHECK_NOTNULL(input.st_planner_object_traj);

  const GeometryGraph& graph = *input.geometry_graph;
  const DrivePassage& drive_passage = *input.drive_passage;
  const InitializerConfig& initializer_params = *input.initializer_params;
  const PathSlBoundary& path_sl = *input.sl_boundary;
  const VehicleDriveParamsProto& vehicle_drive = *input.vehicle_drive;
  const VehicleGeometryParamsProto& vehicle_geom = *input.vehicle_geom;
  const auto& st_planner_object_traj = *input.st_planner_object_traj;

  // Create reference line cost provider.
  const double relaxed_center_max_curvature =
      ComputeRelaxedCenterMaxCurvature(vehicle_geom, vehicle_drive);

  std::unique_ptr<RefLineCostProvider> cost_provider =
      std::make_unique<RefLineCostProvider>(
          &st_planner_object_traj, &drive_passage, &path_sl,
          graph.GetMaxAccumulatedS(), relaxed_center_max_curvature,
          initializer_params);

  // Start from last layer of geometry graph. Update min_cost to each node layer
  // by layer.
  GeometryNodeVector<ReferenceLineSearcherOutput::NodeInfo> cost_nodes;
  GeometryEdgeVector<ReferenceLineSearcherOutput::EdgeInfo> cost_edges;
  cost_nodes.resize(graph.nodes().size());
  cost_edges.resize(graph.edges().size());
  Initialize(graph, &cost_nodes);
  // Searching.
  const auto& nodes_layers = graph.nodes_layers();
  for (int layer_idx = nodes_layers.size() - 1; layer_idx >= 0; --layer_idx) {
    VLOG(5) << "layer_idx: " << layer_idx;
    const auto& nodes_on_cur_layer = nodes_layers[layer_idx];

    for (const auto& node_idx : nodes_on_cur_layer) {
      // Calculate all outgoing edge costs for this node.
      VLOG(5) << ">>>>>>>>> node_idx: " << node_idx.value();
      const auto& geom_node = graph.GetNode(node_idx);
      if (!geom_node.reachable) {
        VLOG(5) << "should be unreachable?";
        continue;  // Ignore unreachable nodes.
      }
      const auto& outgoing_edges = graph.GetOutgoingEdges(node_idx);

      // Update min_cost for this node.
      for (const auto& edge_idx : outgoing_edges) {
        const auto& edge = graph.GetEdge(edge_idx);
        const bool terminating = graph.GetOutgoingEdges(edge.end).empty();
        cost_edges[edge_idx] =
            ComputeEdgeCost(cost_provider.get(), edge, terminating);
        const double edge_sum_cost = cost_edges[edge_idx].sum_cost;

        const auto& prev_node_cost_info = cost_nodes[edge.end];
        VLOG(5) << ">>> edge: " << edge_idx.value()
                << " has cost: " << edge_sum_cost;
        if (std::isinf(prev_node_cost_info.min_cost)) {
          VLOG(5) << edge.end.value()
                  << " node has inf cost, terminating edge idx "
                  << edge_idx.value() << "edge_sum_cost:" << edge_sum_cost;
          return absl::InternalError(
              "prev_node_cost infinity, initialization on geometry node cost "
              "might have failed!");
        }
        const double cost_through_this_edge =
            edge_sum_cost + prev_node_cost_info.min_cost;
        if (cost_through_this_edge < cost_nodes[node_idx].min_cost) {
          // Update current node cost infos.
          VLOG(5) << "prev_node: " << edge.end.value() << "has min_cost"
                  << prev_node_cost_info.min_cost;
          VLOG(5) << "cost through this edge: " << cost_through_this_edge;
          VLOG(5) << "cur node min cost : " << cost_nodes[node_idx].min_cost
                  << " so update.";

          auto& cur_node_cost_info = cost_nodes[node_idx];
          cur_node_cost_info.min_cost = cost_through_this_edge;
          cur_node_cost_info.outgoing_edge_idx = edge_idx;
          cur_node_cost_info.prev_node_idx = edge.end;
        }
      }
    }
  }

  // Build output.
  std::vector<GeometryNodeIndex> nodes_list;
  std::vector<GeometryEdgeIndex> edges_list;
  std::vector<double> feature_costs;

  RETURN_IF_ERROR(BuildConnection(graph, cost_nodes, cost_edges, *cost_provider,
                                  &nodes_list, &edges_list, &feature_costs));
  auto ref_line_points = GenerateRefLinePoints(graph, edges_list);

  if (ref_line_points.size() <= kDownSampleStep) {
    return absl::NotFoundError(
        "Reference line points size less than down sample size.");
  }

  const double min_cost = cost_nodes[nodes_list.front()].min_cost;
  ReferenceLineSearcherOutput search_result{
      .nodes_list = std::move(nodes_list),
      .edges_list = std::move(edges_list),
      .ref_line_points = std::move(ref_line_points),
      .total_cost = min_cost,
      .feature_costs = std::move(feature_costs),
      .ptr_cost_provider = std::move(cost_provider),
      .cost_edges = std::move(cost_edges),
  };
  return search_result;
}

absl::Status DeactivateFarGeometries(
    const ReferenceLineSearcherOutput& search_result,
    const PathSlBoundary& path_sl, GeometryGraph* mutable_geometry_graph) {
  SCOPED_QTRACE("DeactivateFarGeometries");
  const auto& ref_points = search_result.ref_line_points;
  std::vector<Vec2d> downsampled_ref_points;
  downsampled_ref_points.reserve(
      CeilToInt(static_cast<double>(ref_points.size()) / kDownSampleStep));
  for (int i = 0; i < ref_points.size(); i += kDownSampleStep) {
    downsampled_ref_points.push_back(ref_points[i]);
  }
  ASSIGN_OR_RETURN(const auto ref_frame,
                   BuildBruteForceFrenetFrame(downsampled_ref_points,
                                              /*down_sample_raw_points=*/true),
                   _ << "Failed to build frenet frame from ref line.");

  // Get Plf.
  const auto accum_s_r_plf = GetActivationRadiusPlf(
      path_sl, *mutable_geometry_graph, search_result.nodes_list);

  for (const auto& node : mutable_geometry_graph->nodes()) {
    if (!node.reachable) continue;
    const double l = std::fabs(ref_frame.XYToSL(node.xy).l);
    if (l > accum_s_r_plf(node.accumulated_s)) {
      mutable_geometry_graph->DeactivateNode(node.index);
    }
  }

  // Deactivate geometry edges if the start or the end node is not active.
  for (const auto& edge : mutable_geometry_graph->edges()) {
    if (!mutable_geometry_graph->IsActive(edge.start) ||
        !mutable_geometry_graph->IsActive(edge.end)) {
      mutable_geometry_graph->DeactivateEdge(edge.index);
    }
  }

  return absl::OkStatus();
}

absl::Status DeactivateFarGeometries(
    const std::vector<ApolloTrajectoryPointProto>& traj_points,
    const PathSlBoundary& path_sl, GeometryGraph* mutable_geometry_graph) {
  SCOPED_QTRACE("DeactivateFarGeometries");
  std::vector<Vec2d> ref_points;
  ref_points.reserve(traj_points.size());
  for (const auto& traj_pt : traj_points) {
    ref_points.push_back(Vec2dFromApolloTrajectoryPointProto(traj_pt));
  }
  ASSIGN_OR_RETURN(const auto ref_frame,
                   BuildBruteForceFrenetFrame(ref_points,
                                              /*down_sample_raw_points=*/true),
                   _ << "Failed to build frenet frame from ref line.");

  // Get radius threshold plf.
  constexpr double kSampleRadiusInterval = 5.0;  // m.
  const double max_accum_s = mutable_geometry_graph->GetMaxAccumulatedS();
  const int radius_anchor_num =
      CeilToInt(max_accum_s / kSampleRadiusInterval) + 1;
  std::vector<double> accum_s(radius_anchor_num);
  std::vector<double> activate_r(radius_anchor_num);
  for (int i = 0; i < radius_anchor_num; ++i) {
    accum_s[i] = i * kSampleRadiusInterval;
    const auto [right_boundary_l, left_boundary_l] =
        path_sl.QueryBoundaryL(accum_s[i]);
    activate_r[i] = 0.5 * kActivationBoundaryWidthRatioPlf(accum_s[i]) *
                    (left_boundary_l - right_boundary_l);
  }
  const PiecewiseLinearFunction<double, double> accum_s_r_plf(accum_s,
                                                              activate_r);

  for (const auto& node : mutable_geometry_graph->nodes()) {
    if (!node.reachable) continue;
    const double l = std::abs(ref_frame.XYToSL(node.xy).l);
    if (l > accum_s_r_plf(node.accumulated_s)) {
      mutable_geometry_graph->DeactivateNode(node.index);
    }
  }

  // Deactivate geometry edges if the start or the end node is not active.
  for (const auto& edge : mutable_geometry_graph->edges()) {
    if (!mutable_geometry_graph->IsActive(edge.start) ||
        !mutable_geometry_graph->IsActive(edge.end)) {
      mutable_geometry_graph->DeactivateEdge(edge.index);
    }
  }

  return absl::OkStatus();
}

void ParseReferenceLineResultToProto(const ReferenceLineSearcherOutput& result,
                                     GeometryGraphProto* graph_proto) {
  auto* proto = graph_proto->mutable_reference_line();
  for (const auto& node_idx : result.nodes_list) {
    proto->add_node_idxes(node_idx.value());
  }
  for (const auto& edge_idx : result.edges_list) {
    proto->add_edge_idxes(edge_idx.value());
  }
  for (const auto& name : result.ptr_cost_provider->cost_names()) {
    proto->add_cost_names(name);
  }
  proto->set_total_cost(result.total_cost);
  for (const auto& feature_cost : result.feature_costs) {
    proto->add_feature_costs(feature_cost);
  }

  if (FLAGS_planner_initializer_debug_level >= 1) {
    for (const auto& point : result.ref_line_points) {
      point.ToProto(proto->add_ref_line_points());
    }
    const auto& cost_edges = result.cost_edges;
    for (const auto index : result.cost_edges.index_range()) {
      const auto& edge_info = cost_edges[index];
      auto* new_edge_costs = proto->add_edge_costs();
      GeometryGraphProto::ReferenceLine::EdgeCost info_proto;
      info_proto.set_edge_idx(index.value());
      for (const auto& feature_cost : edge_info.feature_costs) {
        info_proto.add_feature_costs(feature_cost);
      }
      info_proto.set_total_cost(edge_info.sum_cost);
      *new_edge_costs = std::move(info_proto);
    }
  }
}
}  // namespace qcraft::planner
