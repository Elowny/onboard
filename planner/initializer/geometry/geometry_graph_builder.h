#ifndef ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_BUILDER_H_
#define ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_BUILDER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/leading_groups_builder.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/geometry/geometry_graph_cache.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {
// Sample params
struct GeometryGraphSamplingParams {
  double layer_gap;
  double lateral_resolution;
  int cross_layer_connection;
  double unit_length_lateral_span;
  bool is_lane_change;
};

// Sample strategy class & method for finding the sampling params
struct GeometryGraphSamplingStrategy {
  // Range anchors for finding params.
  std::vector<double> range_list;
  // Stations between two neighboring layers of node.
  // TODO(boqian): use absolute distance to replace station numbers here.
  std::vector<double> layer_gap_list;
  // Lateral resolution between neighboring nodes in one layer.
  std::vector<double> lateral_resolution_list;
  // Max allowed number of layers in cross-layer connections.
  std::vector<int> cross_layer_connection_list;
  // Max allowed l-s tangent of a forward connection.
  std::vector<double> unit_length_lateral_span_list;
  bool is_lane_change = false;

  GeometryGraphSamplingParams FindSamplingParams(double dist_to_start) const {
    // dist_to_start is the relative distance to the first station.
    QCHECK_GT(range_list.size(), 1);
    QCHECK_EQ(range_list.size(), layer_gap_list.size());
    QCHECK_EQ(range_list.size(), lateral_resolution_list.size());
    QCHECK_EQ(range_list.size(), cross_layer_connection_list.size());
    QCHECK_EQ(range_list.size(), unit_length_lateral_span_list.size());
    for (int i = 0, n = range_list.size(); i < n; ++i) {
      if (dist_to_start <= range_list[i]) {
        return GeometryGraphSamplingParams{
            .layer_gap = layer_gap_list[i],
            .lateral_resolution = lateral_resolution_list[i],
            .cross_layer_connection = cross_layer_connection_list[i],
            .unit_length_lateral_span = unit_length_lateral_span_list[i],
            .is_lane_change = is_lane_change};
      }
    }
    return GeometryGraphSamplingParams{
        .layer_gap = layer_gap_list.back(),
        .lateral_resolution = lateral_resolution_list.back(),
        .cross_layer_connection = cross_layer_connection_list.back(),
        .unit_length_lateral_span = unit_length_lateral_span_list.back(),
        .is_lane_change = is_lane_change};
  }
};

struct CurvyGeometryGraphBuilderInput {
  const DrivePassage* passage = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const std::vector<double>* stop_s_vec = nullptr;
  const std::vector<LeadingGroup>* leading_groups = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  double s_from_start = 0.0;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const CollisionChecker* collision_checker = nullptr;
  const GeometryGraphSamplingStrategy* sampling_params = nullptr;
  const VehicleDriveParamsProto* vehicle_drive = nullptr;
  const GeometryFormBuilder* form_builder = nullptr;
  bool lc_multiple_traj = false;
};

// CurvyGeometryGraphBuilder performs the following tasks:
// 1. Sample drive passage and build geometry graph & edges;
// 2. Filter out edges that collide with STATIC objects;
// 3. Filter out edges that collide with curb (or hard boundary constraints).
// Note that we preposition static collision check at geometry graph instead of
// motion graph to reduce computation time, better explore parallelism and
// provide (static) blocking information to downstream modules.
absl::StatusOr<XYGeometryGraph> BuildCurvyGeometryGraph(
    const CurvyGeometryGraphBuilderInput& input, bool retry_collision_checker,
    GeometryGraphCache* graph_cache, ThreadPool* thread_pool,
    InitializerDebugProto* debug_proto);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_BUILDER_H_
