#include "onboard/planner/initializer/geometry/geometry_graph_builder.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_graph_cache.h"
#include "onboard/planner/initializer/geometry/geometry_graph_debug.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {
// For curvy geometry form.
constexpr double kAccumulatedSEpsilon = 0.1;             // meters.
constexpr double kGeometryFormSamplingS = 1.0;           // meters.
constexpr double kEpsilon = 0.1;                         // meters.
constexpr double kHalfMinSamplingWidth = 0.5;            // meters.
constexpr double kStartLayerMinDistFromSDCLayerS = 1.0;  // meters;
constexpr double kStartLayerMultipleCurvatureVelocityThreshold = 0.5;  // m/s.
constexpr std::array<double, 11> kSampleCurvature = {
    0.0, -0.01, 0.01, -0.02, 0.02, -0.03, 0.03, -0.04, 0.04, -0.05, 0.05};
// From motion search acceleration sampling.
constexpr double kNearCurbRelaxationFactor = 1.11;
constexpr double kCurvatureRelaxRatio = 1.3;
constexpr double kInitializerMaxLKSamplingWidth = 4.0;  // meters.
// Consider the width of two neighboring lanes when perform lane change.
constexpr double kInitializerMaxLCSamplingWidth = 8.0;  // meters.
// The struct stores the collision information of the most far-reaching edge
// among edges that expand from this node and colliding with some static
// objects.
struct NodeCollisionInfo {
  double max_collision_s = std::numeric_limits<double>::lowest();
  std::optional<std::string> collision_obj_id_with_max_s = std::nullopt;

  std::string Debug() const {
    if (collision_obj_id_with_max_s.has_value()) {
      return absl::StrCat("Collide with obj ",
                          collision_obj_id_with_max_s.value(), " at ",
                          max_collision_s, ".");
    } else {
      return "No collision.";
    }
  }
};

struct DpEdgeInfo {
  GeometryGraphCacheKey key;
  GeometryNodeIndex start_node_index;
  GeometryNodeIndex end_node_index;
  bool truncate;
};

struct ConnectEdgesInput {
  const ApolloTrajectoryPointProto* plan_start_point;
  const GeometryFormBuilder* form_builder;
  const CollisionChecker* collision_checker;
  bool retry_collision_checker;
  const PiecewiseLinearFunction<double, double>* max_l_s;
  const PiecewiseLinearFunction<double, double>* min_l_s;
  double max_curvature;
  const std::vector<GeometryGraphSamplingParams>* ptr_layer_params;
};

std::vector<GeometryState> SampleGeometryStates(
    const GeometryForm* ptr_geometry_form) {
  double sampling_ds = kGeometryFormSamplingS;
  constexpr int kMinEvalSteps = 5;
  if (sampling_ds * kMinEvalSteps > ptr_geometry_form->length()) {
    sampling_ds = ptr_geometry_form->length() / kMinEvalSteps;
  }
  return ptr_geometry_form->Sample(sampling_ds);
}

GeometryNodeVector<GeometryNode> SampleOneLayer(
    const PathSlBoundary& sl_boundary, const Station& station,
    const int station_index, double ego_width, double lat_step,
    bool is_lane_change, int* nodes_cnt) {
  const double accum_s = station.accumulated_s();
  const double ref_center_l = sl_boundary.QueryReferenceCenterL(accum_s);
  const auto [boundary_right_l, boundary_left_l] =
      sl_boundary.QueryBoundaryL(accum_s);
  const double right_width =
      std::max(0.0, ref_center_l - boundary_right_l - ego_width * 0.5);
  const double left_width =
      std::max(0.0, boundary_left_l - ref_center_l - ego_width * 0.5);

  // Limit the number of sampling points to default_sampling_width /
  // lateral_step.
  const double default_sampling_width =
      is_lane_change ? kInitializerMaxLCSamplingWidth - ego_width
                     : kInitializerMaxLKSamplingWidth - ego_width;
  QCHECK_GT(default_sampling_width, 0.0);
  const double half_default_sampling_width = 0.5 * default_sampling_width;
  int right_n =
      CeilToInt(std::min(right_width, half_default_sampling_width) / lat_step);
  int left_n =
      CeilToInt(std::min(left_width, half_default_sampling_width) / lat_step);

  // Resample with a resolution close to the desired one and fit to the
  // actual boundary at the same time. Use different step sizes on left and
  // right sides to guarantee samples exactly on the leftmost border, the ref
  // center and the rightmost border.
  double resampled_right_lat_step = 0.0, resampled_left_lat_step = 0.0;
  if (right_n > 0) {
    resampled_right_lat_step = right_width / right_n;
    right_n = RoundToInt(right_width / resampled_right_lat_step);
  }
  if (left_n > 0) {
    resampled_left_lat_step = left_width / left_n;
    left_n = RoundToInt(left_width / resampled_left_lat_step);
  }

  GeometryNodeVector<GeometryNode> nodes_layer;
  nodes_layer.reserve(left_n + right_n + 1);
  for (int i = -right_n; i <= left_n; ++i) {
    const double lat_offset =
        ref_center_l +
        (i < 0 ? resampled_right_lat_step : resampled_left_lat_step) * i;
    nodes_layer.emplace_back(GeometryNode{
        .index = GeometryNodeIndex((*nodes_cnt)++),
        .xy = station.lat_point(lat_offset),
        .k = 0.0,  // Placeholder for multiple curvature feature later.
        .station_index = station_index,
        .lateral_offset = lat_offset,
        .accumulated_s = accum_s,
        .reachable = false});
  }

  return nodes_layer;
}

void ResetReachableStatus(
    std::vector<GeometryNodeVector<GeometryNode>>* ptr_nodes_layers) {
  auto& nodes_layers = *ptr_nodes_layers;
  // Reset all nodes to unreachable from the second layers.
  for (int i = 1; i < nodes_layers.size(); ++i) {
    auto& nodes = nodes_layers[i];
    for (auto& node : nodes) {
      node.reachable = false;
    }
  }
}

absl::StatusOr<GeometryNodeVector<GeometryNode>> CreateStartNodeLayer(
    const DrivePassage& passage, const GeometryState& plan_start_state,
    double sdc_v, StationIndex* ptr_next_index) {
  // Add plan start state to geometry nodes.
  ASSIGN_OR_RETURN(const auto ego_waypoint,
                   passage.QueryFrenetLonOffsetAt(plan_start_state.xy),
                   _.SetPrepend()
                       << "Cannot find frenet mapping of ego vehicle state: ");
  ASSIGN_OR_RETURN(const auto lat_offset,
                   passage.QueryFrenetLatOffsetAt(plan_start_state.xy),
                   _.SetPrepend()
                       << "Cannot find frenet mapping of ego vehicle state: ");

  if (ego_waypoint.lon_offset >= 0) {
    *ptr_next_index = ego_waypoint.station_index.next();
  } else {
    *ptr_next_index = ego_waypoint.station_index;
  }
  GeometryNodeVector<GeometryNode> start_node_layer;
  const double accumulated_s =
      passage.station(ego_waypoint.station_index).accumulated_s() +
      ego_waypoint.lon_offset;
  const int station_index = ptr_next_index->value() - 1;
  start_node_layer.emplace_back(GeometryNode{.index = GeometryNodeIndex(0),
                                             .xy = plan_start_state.xy,
                                             .k = plan_start_state.k,
                                             .station_index = station_index,
                                             .lateral_offset = lat_offset,
                                             .accumulated_s = accumulated_s,
                                             .reachable = true});

  // If vehicle is under a low speed threshold, create start node layer with
  // different curvatures. Otherwise just generate one start node.
  if (sdc_v < kStartLayerMultipleCurvatureVelocityThreshold) {
    start_node_layer.reserve(kSampleCurvature.size() + 1);
    // Get other samples.
    for (int i = 0; i < kSampleCurvature.size(); ++i) {
      if (kSampleCurvature[i] == plan_start_state.k) {
        // Already added to the layer, skip.
        continue;
      }
      const int curr_node_cnt = start_node_layer.size();
      start_node_layer.emplace_back(
          GeometryNode{.index = GeometryNodeIndex(curr_node_cnt),
                       .xy = plan_start_state.xy,
                       .k = kSampleCurvature[i],
                       .station_index = station_index,
                       .lateral_offset = lat_offset,
                       .accumulated_s = accumulated_s,
                       .reachable = true});
    }
  }
  return start_node_layer;
}

double FindCurbBuffer(const DrivePassage& passage, const Vec2d& ego_pos,
                      StationIndex next_station_idx, double ego_half_width) {
  ASSIGN_OR_RETURN(const auto ego_pos_curb_offset,
                   passage.QueryCurbOffsetAt(ego_pos), 0.0);
  ASSIGN_OR_RETURN(
      const auto next_station_curb_offset,
      passage.station(next_station_idx).QueryCurbOffsetAt(/*signed_lat=*/0.0),
      0.0);

  // Use curb width to check shrinking.
  const double ego_pos_curb_width =
      ego_pos_curb_offset.second - ego_pos_curb_offset.first;
  const double next_station_curb_width =
      next_station_curb_offset.second - next_station_curb_offset.first;
  // If curb is shrinking, need to make sure start position is within curb
  // when sdc is already very close to curb.
  const double relax_factor = ego_pos_curb_width > next_station_curb_width
                                  ? kNearCurbRelaxationFactor
                                  : 1.0;

  const double min_dist_to_curb =
      std::min(std::abs(ego_pos_curb_offset.first),
               std::abs(ego_pos_curb_offset.second));
  return std::clamp(min_dist_to_curb - ego_half_width * relax_factor, 0.0,
                    ego_half_width);
}

std::pair<PiecewiseLinearFunction<double>, PiecewiseLinearFunction<double>>
FindLateralBoundaryPlf(const DrivePassage& passage, double curb_buffer) {
  std::vector<double> station_s, max_l, min_l;
  station_s.reserve(passage.size());
  max_l.reserve(passage.size());
  min_l.reserve(passage.size());
  for (const auto& station : passage.stations()) {
    const auto station_curboffset = station.QueryCurbOffsetAt(0.0);
    if (station_curboffset.ok()) {
      // (first, second) = (right_offset, left_offset).
      max_l.push_back(station_curboffset->second - curb_buffer);
      min_l.push_back(station_curboffset->first + curb_buffer);
      station_s.push_back(station.accumulated_s());
    }
  }
  return {PiecewiseLinearFunction<double>(station_s, std::move(max_l)),
          PiecewiseLinearFunction<double>(station_s, std::move(min_l))};
}

bool IsInvalidCurvature(absl::Span<const GeometryState> states,
                        double max_curvature) {
  for (const auto& state : states) {
    if (std::fabs(state.k) > max_curvature) {
      return true;
    }
  }
  return false;
}

// If performing lane change, we only allow edges directing towards the lane
// center until we reach the target lane. Inside the target lane, we allow
// edge of any direction.
bool IsEdgeConvergingToLaneCenter(double cur_offset, double future_offset) {
  if (cur_offset > 0.0 && future_offset < cur_offset + kEpsilon &&
      future_offset > -kEpsilon) {
    return true;
  }
  if (cur_offset <= 0.0 && future_offset > cur_offset - kEpsilon &&
      future_offset < kEpsilon) {
    return true;
  }
  // If geometry edge is inside the targe lane zone, we allow the edges to
  // nudge potential objects inside the zone.
  if (std::fabs(future_offset) < kDefaultHalfLaneWidth &&
      std::fabs(cur_offset) < kDefaultHalfLaneWidth) {
    return true;
  }
  return false;
}

// Check if an edge (represented by a sequence of states) is inside the curb
bool IsEdgeHittingCurb(absl::Span<const GeometryState> states,
                       const PiecewiseLinearFunction<double, double>& max_l_s,
                       const PiecewiseLinearFunction<double, double>& min_l_s) {
  for (int i = 0; i < states.size(); ++i) {
    const auto& state = states[i];
    // Ego width already considered in the plf.
    if (state.l >= max_l_s(state.accumulated_s) ||
        state.l <= min_l_s(state.accumulated_s)) {
      return true;
    }
  }
  return false;
}

// Check if an edge (represented by a sequence of states) collides with static
// objects.
bool IsEdgeCollidingWithStaticObject(absl::Span<const GeometryState> states,
                                     const CollisionChecker& collision_checker,
                                     int* ptr_collide_state_idx,
                                     std::string* ptr_obj_id) {
  QCHECK_GT(states.size(), 0);
  CollisionInfo collision_info;
  collision_checker.CheckCollisionWithStationaryObjects(states,
                                                        &collision_info);
  if (!collision_info.collision_objects.empty()) {
    const auto min_it =
        std::min_element(collision_info.collision_objects.begin(),
                         collision_info.collision_objects.end(),
                         [](const CollisionInfo::ObjectCollision& lhs,
                            const CollisionInfo::ObjectCollision& rhs) {
                           if (lhs.time == rhs.time)
                             return lhs.traj->traj_id() < rhs.traj->traj_id();
                           return lhs.time < rhs.time;
                         });
    *ptr_collide_state_idx = min_it->time;
    *ptr_obj_id = min_it->traj->object_id();
    return true;
  }
  return false;
}

bool PassPreGeometryFormBuildCheck(const GeometryGraphSamplingParams& params,
                                   const GeometryNode& this_node,
                                   const GeometryNode& next_node,
                                   double layer_idx,
                                   GeometryEdgeCache* edge_info) {
  // Cover connection check before polynomials or spirals are solved.

  const double dl = next_node.lateral_offset - this_node.lateral_offset;
  const double ds = next_node.accumulated_s - this_node.accumulated_s;
  if (ds < kAccumulatedSEpsilon) {
    // This node is too close to the next node, do not expand.
    edge_info->connection_result = ConnectionResult::OMIT_TOO_SHORT;
    return false;
  }
  if (std::fabs(dl / ds) >= params.unit_length_lateral_span) {
    // Opening angle is too large.
    edge_info->connection_result = ConnectionResult::OMIT_LATERAL_OFFSET;
    return false;
  }
  // If not the first layer & performing lane change, then force the
  // edges to converge to lane center.
  if (layer_idx != 0 && params.is_lane_change &&
      (!IsEdgeConvergingToLaneCenter(this_node.lateral_offset,
                                     next_node.lateral_offset))) {
    edge_info->connection_result =
        ConnectionResult::OMIT_NOT_CONVERGE_TO_CENTER;
    return false;
  }
  return true;
}

absl::StatusOr<PiecewiseLinearGeometry> BuildGeometryForm(
    const GeometryFormBuilder& form_builder, const GeometryNode& this_node,
    const GeometryNode& next_node,
    const ApolloTrajectoryPointProto& plan_start_point, int layer_idx) {
  const DrivePassageSamplePoint next_point = {
      next_node.xy, next_node.lateral_offset, next_node.accumulated_s,
      next_node.station_index};
  if (layer_idx == 0) {
    double dk = plan_start_point.path_point().lambda();
    if (this_node.k != plan_start_point.path_point().kappa()) {
      // Set lambda to zero.
      dk = 0.0;
    }
    const GeometryState start_state = {
        .xy = this_node.xy,
        .h = plan_start_point.path_point().theta(),
        .k = this_node.k,
        .dk = dk};
    const auto& quintic_spiral_or =
        form_builder.BuildQuinticSpiralGeometry(start_state, next_point);
    if (quintic_spiral_or.ok()) {
      return *quintic_spiral_or;
    } else {
      const auto& cubic_spiral_or =
          form_builder.BuildCubicSpiralGeometry(start_state, next_point);
      if (cubic_spiral_or.ok()) {
        return *cubic_spiral_or;
      }
    }
  }
  // Use lateral polynomial.
  const DrivePassageSamplePoint this_point = {
      this_node.xy, this_node.lateral_offset, this_node.accumulated_s,
      this_node.station_index};
  return form_builder.BuildLateralQuinticPolyGeometry(this_point, next_point);
}

bool PassCurvatureAndCurbCheck(
    const std::vector<GeometryState>& states, const double max_curvature,
    const PiecewiseLinearFunction<double, double>& max_l_s,
    const PiecewiseLinearFunction<double, double>& min_l_s,
    GeometryEdgeCache* edge_info) {
  if (IsInvalidCurvature(states, max_curvature)) {
    edge_info->connection_result = ConnectionResult::FAIL_INVALID_CURVATURE;
    return false;
  }
  if (IsEdgeHittingCurb(states, max_l_s, min_l_s)) {
    edge_info->connection_result = ConnectionResult::FAIL_CURB_COLLISION;
    return false;
  }
  return true;
}

// Separate check collision and generate geometry form.
ConnectionResult CheckCollision(const CollisionChecker& collision_checker,
                                const std::vector<GeometryState>& states,
                                NodeCollisionInfo* ptr_node_collision_info,
                                int* colliding_index, std::string* obj_id) {
  if (IsEdgeCollidingWithStaticObject(states, collision_checker,
                                      colliding_index, obj_id)) {
    const double last_s = states[*colliding_index].accumulated_s;
    // Update node_collision_info for collision case.
    if (last_s >= ptr_node_collision_info->max_collision_s) {
      ptr_node_collision_info->max_collision_s = last_s;
      ptr_node_collision_info->collision_obj_id_with_max_s = *obj_id;
    }
    constexpr double kMinTruncatedEdgeLength = 1.0;  // m.
    if (*colliding_index > 1 &&
        states[*colliding_index].accumulated_s - states[0].accumulated_s >
            kMinTruncatedEdgeLength) {
      return ConnectionResult::COLLIDE_TRUNCATE;
    } else {
      return ConnectionResult::COLLIDE_NO_EDGE;
    }
  }
  // If connection is success, do nothing. NodeCollisionInfo should only be
  // updated when an edge from a node has collision with some obstacle.
  return ConnectionResult::SUCCESS;
}

void UpdateEdgeInfoInCache(const GeometryGraphCacheKey& key,
                           const ConnectionResult& new_result,
                           const std::vector<GeometryState>& states,
                           const GeometryNode& this_node, int colliding_index,
                           const GeometryNode& next_node,
                           const std::string& obj_id,
                           std::vector<DpEdgeInfo>* this_node_outgoing_edges,
                           GeometryGraphCache* graph_cache) {
  // Case 1: New result is Success.
  if (new_result == ConnectionResult::SUCCESS) {
    graph_cache->UpdateConnectionResult(key, new_result);
    this_node_outgoing_edges->emplace_back(DpEdgeInfo{
        .key = key,
        .start_node_index = this_node.index,
        .end_node_index = next_node.index,
        .truncate = false,
    });
    return;
  }

  const double collision_s = states[colliding_index].accumulated_s;
  // Case 2: new result is truncated edge. Need to update collision info
  // in the cache for this edge as well as the truncated geometry and
  // final state.
  if (new_result == ConnectionResult::COLLIDE_TRUNCATE) {
    std::vector<GeometryState> truncated_states(
        states.begin(), states.begin() + colliding_index);
    std::unique_ptr<GeometryForm> ptr_geometry_form_truncated =
        std::make_unique<PiecewiseLinearGeometry>(truncated_states);
    const auto& final_state = truncated_states.back();
    graph_cache->UpdateTruncatedGeometryForm(
        key, final_state, std::move(ptr_geometry_form_truncated), obj_id,
        collision_s);
    this_node_outgoing_edges->emplace_back(DpEdgeInfo{
        .key = key,
        .start_node_index = this_node.index,
        .end_node_index = GeometryNodeVector<GeometryNode>::kInvalidIndex,
        .truncate = true,
    });
    return;
  }

  // Case 3: new result still collide and has no edge. Update result and
  // collision info.
  if (new_result == ConnectionResult::COLLIDE_NO_EDGE) {
    graph_cache->UpdateConnectionResult(key, new_result);
    graph_cache->UpdateCollisionInfo(key, obj_id, collision_s);
    return;
  }
}

std::pair<double, std::string> GetLeadingObjectAccumulatedS(
    const DrivePassage& passage, const LeadingGroup& leading_group,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  double min_accumulated_s = std::numeric_limits<double>::max();
  std::string obj_id;
  for (const auto& [traj_id, _] : leading_group) {
    const auto* traj_ptr = st_traj_mgr.FindTrajectoryById(traj_id);
    // Only consider stationary leading object to ensure that sdc does try to
    // nudge a queueing object in front of traffic light for example.
    if (!traj_ptr->is_stationary()) continue;

    ASSIGN_OR_CONTINUE(
        const auto frenet_box,
        passage.QueryFrenetBoxAt(traj_ptr->planner_object().bounding_box()));
    if (min_accumulated_s > frenet_box.s_min) {
      min_accumulated_s = frenet_box.s_min;
      obj_id = traj_ptr->planner_object().id();
    }
  }
  return {min_accumulated_s, obj_id};
}

void FillTruncateEdgeInfo(const std::vector<GeometryState>& states,
                          int colliding_index, const std::string& obj_id,
                          GeometryEdgeCache* edge_info) {
  std::vector<GeometryState> truncated_states(states.begin(),
                                              states.begin() + colliding_index);
  edge_info->collision_accum_s.push_back(states[colliding_index].accumulated_s);
  edge_info->collision_ids.push_back(obj_id);
  edge_info->ptr_geometry_form_truncated =
      std::make_unique<PiecewiseLinearGeometry>(truncated_states);
  edge_info->final_state = truncated_states.back();
}

absl::Status SampleGeometryNodesWithVaryingResolutionStrategy(
    const DrivePassage& passage, const PathSlBoundary& sl_boundary,
    const GeometryGraphSamplingStrategy& sampling_params,
    double max_sampling_accum_s, double constraint_stop_sampling_s,
    double s_from_start, double /*ego_v*/, double ego_width, double ego_accum_s,
    int start_index,
    std::vector<GeometryNodeVector<GeometryNode>>* ptr_nodes_layers,
    std::vector<GeometryGraphSamplingParams>* ptr_layer_params,
    int* total_nodes) {
  QCHECK_GT(ptr_nodes_layers->size(), 0);
  QCHECK_GT(ptr_nodes_layers->at(0).size(), 0);

  const int num_stations = passage.stations().size();
  QCHECK_GT(num_stations, start_index);

  // 1. Find the start sampling station index.
  const int nodes_cnt_before = *total_nodes;
  int& nodes_cnt = *total_nodes;
  // A useful trick: pin the geometry graph to regular intervals to reduce
  // variation in geometry graph across frames.
  const double s_from_start_with_diff = s_from_start - ego_accum_s;
  const auto start_param = sampling_params.FindSamplingParams(0.0);
  double accum_s = passage.station(StationIndex(start_index)).accumulated_s();
  double route_s = s_from_start_with_diff + accum_s;
  while (start_index + 1 < num_stations &&
         accum_s < constraint_stop_sampling_s &&
         (route_s < start_param.layer_gap - kEpsilon ||
          accum_s - ego_accum_s < kStartLayerMinDistFromSDCLayerS)) {
    accum_s = passage.station(StationIndex(++start_index)).accumulated_s();
    route_s = s_from_start_with_diff + accum_s;
  }

  // 2. Iteratively sample layers.
  double prev_layer_s = ego_accum_s;
  for (int cur_index = start_index; cur_index < num_stations; ++cur_index) {
    const double dist2start =
        passage.station(StationIndex(cur_index)).accumulated_s() -
        passage.station(StationIndex(start_index)).accumulated_s();
    // If we reach the maximal sampling range, stop.
    if (dist2start > max_sampling_accum_s) break;

    const auto cur_param = sampling_params.FindSamplingParams(dist2start);
    const auto& station = passage.station(StationIndex(cur_index));
    if (station.accumulated_s() - prev_layer_s >=
        cur_param.layer_gap - kEpsilon) {
      ptr_nodes_layers->emplace_back(SampleOneLayer(
          sl_boundary, station, cur_index, ego_width,
          cur_param.lateral_resolution, cur_param.is_lane_change, &nodes_cnt));
      ptr_layer_params->push_back(cur_param);
      prev_layer_s = station.accumulated_s();

      // Quit the loop if we reach a stop line and the last layer of nodes is
      // at a sampling buffer distance away from the stopline.
      if (station.accumulated_s() >=
          constraint_stop_sampling_s + cur_param.layer_gap) {
        break;
      }
    }
  }

  if (nodes_cnt - nodes_cnt_before == 0) {
    return absl::InternalError(
        "Zero nodes sampled! Path boundary width < sdc_width?");
  }

  return absl::OkStatus();
}

void ExpandGeometryEdgesFromLayerNode(
    const ApolloTrajectoryPointProto& plan_start_point,
    const CollisionChecker& collision_checker, bool retry_collision_checker,
    const GeometryGraphSamplingParams& params,
    const GeometryFormBuilder& form_builder,
    const PiecewiseLinearFunction<double, double>& max_l_s,
    const PiecewiseLinearFunction<double, double>& min_l_s, int layer_idx,
    int this_layer_node_idx, double max_curvature,
    const std::vector<GeometryNodeVector<GeometryNode>>& nodes_layers,
    std::vector<DpEdgeInfo>* ptr_edge_container,
    NodeCollisionInfo* ptr_node_collision_info,
    GeometryGraphCache* graph_cache) {
  auto& this_node_outgoing_edges = *ptr_edge_container;
  const auto& this_node =
      nodes_layers[layer_idx][GeometryNodeIndex(this_layer_node_idx)];
  const int total_layer_num = nodes_layers.size();

  ptr_node_collision_info->max_collision_s =
      std::numeric_limits<double>::lowest();
  ptr_node_collision_info->collision_obj_id_with_max_s = std::nullopt;

  // Iterate through all connectible layers.
  const int cross_layer_connection_num =
      std::min(params.cross_layer_connection, total_layer_num - 1 - layer_idx);
  for (int i = 1; i <= cross_layer_connection_num; ++i) {
    const int to_be_connected_layer_index = layer_idx + i;
    // Iterate through all connectible nodes.
    for (const auto& next_node : nodes_layers[to_be_connected_layer_index]) {
      GeometryGraphCacheKey key(this_node, next_node);
      const auto connection_result_or = graph_cache->GetConnectionResult(key);
      if (!connection_result_or.ok()) {
        // Cache do not contains current pair of connection, create a new
        // one.
        GeometryEdgeCache edge_info;

        if (FLAGS_planner_initializer_debug_level >= 1) {
          edge_info.debug_info = EdgeDebugInfo{
              .start_layer_idx = layer_idx,
              .end_layer_idx = to_be_connected_layer_index,
              .start_station_idx = this_node.station_index,
              .end_station_idx = next_node.station_index,
          };
        }

        // Check if edge is too short, lateral span too large, or the edge is
        // not converging to center when lane keeping.
        if (!PassPreGeometryFormBuildCheck(params, this_node, next_node,
                                           layer_idx, &edge_info)) {
          graph_cache->AddEdge(key, std::move(edge_info));
          continue;
        }

        // Build geometry form and check if it's successful.
        auto poly_form_or = BuildGeometryForm(
            form_builder, this_node, next_node, plan_start_point, layer_idx);
        if (!poly_form_or.ok()) {
          edge_info.connection_result = ConnectionResult::FAIL_NO_POLY;
          graph_cache->AddEdge(key, std::move(edge_info));
          continue;
        }
        std::unique_ptr<GeometryForm> ptr_geometry_form =
            std::make_unique<PiecewiseLinearGeometry>(
                std::move(poly_form_or).value());
        const auto states = SampleGeometryStates(ptr_geometry_form.get());

        // Check if the geometry form hit curb or has invalid curvature.
        const double relaxed_allowed_curvature =
            layer_idx == 0 && i == 1 ? 2.0 * max_curvature : max_curvature;
        if (!PassCurvatureAndCurbCheck(states, relaxed_allowed_curvature,
                                       max_l_s, min_l_s, &edge_info)) {
          graph_cache->AddEdge(key, std::move(edge_info));
          continue;
        }

        // After pre-check of the geometry edge (hitting curbs etc.), record
        // the full geometry form in cache.
        edge_info.ptr_geometry_form = std::move(ptr_geometry_form);
        int colliding_index = 0;
        std::string obj_id;
        edge_info.connection_result =
            CheckCollision(collision_checker, states, ptr_node_collision_info,
                           &colliding_index, &obj_id);
        const auto& result = edge_info.connection_result;
        if (result == ConnectionResult::COLLIDE_TRUNCATE) {
          FillTruncateEdgeInfo(states, colliding_index, obj_id, &edge_info);
          if (graph_cache->AddEdge(key, std::move(edge_info))) {
            this_node_outgoing_edges.emplace_back(
                DpEdgeInfo{.key = key,
                           .start_node_index = this_node.index,
                           .end_node_index =
                               GeometryNodeVector<GeometryNode>::kInvalidIndex,
                           .truncate = true});
          }
        } else if (result == ConnectionResult::SUCCESS) {
          if (graph_cache->AddEdge(key, std::move(edge_info))) {
            this_node_outgoing_edges.push_back(
                DpEdgeInfo{.key = key,
                           .start_node_index = this_node.index,
                           .end_node_index = next_node.index,
                           .truncate = false});
          }
        } else if (result == ConnectionResult::COLLIDE_NO_EDGE) {
          // Full geometry form already in edge_info. Need to record collision
          // info.
          edge_info.collision_accum_s.push_back(
              states[colliding_index].accumulated_s);
          edge_info.collision_ids.push_back(obj_id);
          graph_cache->AddEdge(key, std::move(edge_info));
        }
      } else {
        // If connection is stored in cache, get the connection result.
        const auto& connection_result = *connection_result_or;

        if (connection_result == ConnectionResult::SUCCESS) {
          this_node_outgoing_edges.push_back(
              DpEdgeInfo{.key = key,
                         .start_node_index = this_node.index,
                         .end_node_index = next_node.index,
                         .truncate = false});
          continue;
        }

        if (!retry_collision_checker &&
            connection_result == ConnectionResult::COLLIDE_TRUNCATE) {
          this_node_outgoing_edges.emplace_back(DpEdgeInfo{
              .key = key,
              .start_node_index = this_node.index,
              .end_node_index = GeometryNodeVector<GeometryNode>::kInvalidIndex,
              .truncate = true});
          const auto [collision_accum_s, collision_id] =
              *graph_cache->GetCollisionInfo(key);
          if (collision_accum_s >= ptr_node_collision_info->max_collision_s) {
            ptr_node_collision_info->max_collision_s = collision_accum_s;
            ptr_node_collision_info->collision_obj_id_with_max_s = collision_id;
          }
          continue;
        }

        if (retry_collision_checker &&
            (connection_result == ConnectionResult::COLLIDE_TRUNCATE ||
             connection_result == ConnectionResult::COLLIDE_NO_EDGE)) {
          // Collision Checker buffer updated, so recheck collision.
          ASSIGN_OR_DIE(const auto geom_form,
                        graph_cache->GetFullGeometryForm(key));
          const auto states = SampleGeometryStates(geom_form);
          int colliding_index = 0;
          std::string obj_id;
          const ConnectionResult new_result =
              CheckCollision(collision_checker, states, ptr_node_collision_info,
                             &colliding_index, &obj_id);
          UpdateEdgeInfoInCache(key, new_result, states, this_node,
                                colliding_index, next_node, obj_id,
                                &this_node_outgoing_edges, graph_cache);
        }
      }
    }
  }
}

void AddResampleNodes(
    const DrivePassage& drive_passage,
    std::vector<GeometryNodeVector<GeometryNode>>* ptr_nodes_layers,
    int* total_nodes, GeometryGraphCache* graph_cache) {
  auto& nodes_layers = *ptr_nodes_layers;
  QCHECK_GE(nodes_layers.size(), 2);
  std::vector<ResampleReason> resample_result(nodes_layers.size() - 1,
                                              ResampleReason::NOT_INITIALIZED);
  for (int i = 1, n = nodes_layers.size(); i < n; ++i) {
    double max_reachable_l = std::numeric_limits<double>::lowest();
    double min_reachable_l = std::numeric_limits<double>::max();
    int n_reachable = 0;
    for (const auto& node : nodes_layers[i]) {
      if (node.reachable) {
        n_reachable++;
        // Update the reachable range for current geometry graph layer.
        max_reachable_l = std::max(max_reachable_l, node.lateral_offset);
        min_reachable_l = std::min(min_reachable_l, node.lateral_offset);
      }
    }
    const int n_unreachable = nodes_layers[i].size() - n_reachable;
    if (n_unreachable == 0) {
      // We do not have to resample this layer if all the nodes are
      // reachable.
      resample_result[i - 1] = ResampleReason::NR_ALL_REACHABLE;
      continue;
    }
    if (n_reachable == 0) {
      // If all the nodes are unreachable, we do not resample this layer.
      // Maybe add more strategies later.
      resample_result[i - 1] = ResampleReason::NR_ZERO_REACHABLE;
      continue;
    }
    if (max_reachable_l < min_reachable_l) {
      // Weird range.
      resample_result[i - 1] = ResampleReason::NR_INVALID_RANGE;
      continue;
    }
    if (max_reachable_l == min_reachable_l) {
      // Create a sampling range when only one node is reachable.
      max_reachable_l += kHalfMinSamplingWidth;
      min_reachable_l -= kHalfMinSamplingWidth;
    }
    // For now, only consider the cases where the path is half-blocked.
    const double resample_lat_step =
        (max_reachable_l - min_reachable_l) / (n_unreachable + 1);
    resample_result[i - 1] = ResampleReason::RESAMPLED;
    const double accum_s = nodes_layers[i].front().accumulated_s;
    const int station_index = nodes_layers[i].front().station_index;
    const auto& station = drive_passage.station(StationIndex(station_index));
    for (int count = 0; count < n_unreachable; ++count) {
      // Add same number of nodes as the n_unreachable to this layer so that
      // we can maintain the number of sampled nodes on this layer.
      const double lateral_offset =
          min_reachable_l + (count + 1) * resample_lat_step;
      nodes_layers[i].push_back(GeometryNode{
          .index = GeometryNodeIndex((*total_nodes)++),
          .xy = station.lat_point(lateral_offset),
          .k = 0.0,  // Placeholder.
          .station_index = station_index,
          .lateral_offset = lateral_offset,
          .accumulated_s = accum_s,
          .reachable = false,  // Default.
          .resampled = true,
      });
    }
  }
  graph_cache->AddResampleReasons(resample_result);
}

void SetEdgeEndNodesReachable(
    const GeometryNodeVector<std::vector<DpEdgeInfo>>& outgoing_edges,
    int layer_index,
    std::vector<GeometryNodeVector<GeometryNode>>* ptr_nodes_layers) {
  FUNC_QTRACE();

  auto& nodes_layers = *ptr_nodes_layers;
  absl::flat_hash_set<GeometryNodeIndex> reachable_nodes;
  for (const auto& node : nodes_layers[layer_index]) {
    for (const auto& edge : outgoing_edges[node.index]) {
      if (edge.end_node_index !=
          GeometryNodeVector<GeometryNode>::kInvalidIndex) {
        reachable_nodes.insert(edge.end_node_index);
      }
    }
  }
  // The edge nodes should only appear in the layers with larger index.
  for (int i = layer_index + 1; i < nodes_layers.size(); ++i) {
    for (auto& node : nodes_layers[i]) {
      if (reachable_nodes.contains(node.index)) {
        node.reachable = true;
      }
    }
  }
}

absl::Status ConnectEdges(
    const ConnectEdgesInput& connect_edges_input, int nodes_cnt,
    std::vector<GeometryNodeVector<GeometryNode>>* ptr_nodes_layers,
    GeometryNodeVector<std::vector<DpEdgeInfo>>* ptr_outgoing_edges,
    GeometryNodeVector<NodeCollisionInfo>* ptr_node_collision_infos,
    ThreadPool* thread_pool, GeometryGraphCache* graph_cache) {
  SCOPED_QTRACE("ConnectEdges");
  const auto& plan_start_point = *connect_edges_input.plan_start_point;
  const auto& form_builder = *connect_edges_input.form_builder;
  const auto& collision_checker = *connect_edges_input.collision_checker;
  const bool retry_collision_checker =
      connect_edges_input.retry_collision_checker;
  const auto& layer_params = *connect_edges_input.ptr_layer_params;
  const auto& max_l_s = *connect_edges_input.max_l_s;
  const auto& min_l_s = *connect_edges_input.min_l_s;
  graph_cache->reset_debug();

  auto& nodes_layers = *ptr_nodes_layers;
  auto& outgoing_edges = *ptr_outgoing_edges;
  auto& node_collision_infos = *ptr_node_collision_infos;
  outgoing_edges.resize(nodes_cnt);
  node_collision_infos.resize(nodes_cnt);

  // First layer - choose only one start node.
  auto& first_layer = nodes_layers[0];
  int count_zero_out_edge_node = 0;
  for (int this_layer_node_idx = 0; this_layer_node_idx < first_layer.size();
       this_layer_node_idx++) {
    auto& this_node = first_layer[GeometryNodeIndex(this_layer_node_idx)];
    ExpandGeometryEdgesFromLayerNode(
        plan_start_point, collision_checker, retry_collision_checker,
        layer_params[0], form_builder, max_l_s, min_l_s, /*layer_idx=*/0,
        this_layer_node_idx, connect_edges_input.max_curvature, nodes_layers,
        &outgoing_edges[this_node.index],
        &node_collision_infos[this_node.index], graph_cache);
    if (outgoing_edges[this_node.index].size() > 0) {
      // Found a feasible start node with min curvature, ignore all other
      // GeometryNode at this layer.
      for (int i = this_layer_node_idx + 1; i < first_layer.size(); ++i) {
        first_layer[GeometryNodeIndex(i)].reachable = false;
      }
      break;
    }

    ++count_zero_out_edge_node;
    if (this_layer_node_idx != 0) {
      // The first node of the first layer is the current ego state, which is
      // always reachable.
      this_node.reachable = false;
    }
  }
  if (count_zero_out_edge_node == first_layer.size()) {
    QEVENT("changqing", "first_layer_zero_outgoing_edges", [](QEvent* qevent) {
      qevent->AddField("Geometry graph builder",
                       "First layer zero outgoing edges.");
    });
    return absl::NotFoundError(
        "No valid geometry edge found from the current ego state!");
  }
  SetEdgeEndNodesReachable(outgoing_edges, /*layer_index=*/0, &nodes_layers);

  // The other layers.
  for (int layer_idx = 1; layer_idx + 1 < nodes_layers.size(); ++layer_idx) {
    if (nodes_layers[layer_idx].size() == 0) continue;

    ParallelFor(
        0, nodes_layers[layer_idx].size(), thread_pool,
        [&](int this_layer_node_idx) {
          const auto& this_node =
              nodes_layers[layer_idx][GeometryNodeIndex(this_layer_node_idx)];
          if (!this_node.reachable) return;

          ExpandGeometryEdgesFromLayerNode(
              plan_start_point, collision_checker, retry_collision_checker,
              layer_params[layer_idx], form_builder, max_l_s, min_l_s,
              layer_idx, this_layer_node_idx, connect_edges_input.max_curvature,
              nodes_layers, &outgoing_edges[this_node.index],
              &node_collision_infos[this_node.index], graph_cache);
        });
    // Set this layer's edges' end nodes reachable.
    SetEdgeEndNodesReachable(outgoing_edges, layer_idx, &nodes_layers);
  }
  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<XYGeometryGraph> BuildCurvyGeometryGraph(
    const CurvyGeometryGraphBuilderInput& input, bool retry_collision_checker,
    GeometryGraphCache* graph_cache, ThreadPool* thread_pool,
    InitializerDebugProto* debug_proto) {
  SCOPED_QTRACE("BuildCurvyGeometryGraph");
  const auto start_time = absl::Now();
  VLOG(2) << "------ BuildCurvyGeometryGraph ------";
  if (retry_collision_checker) {
    VLOG(2) << "retry_collision_checker:\t true.";
  } else {
    VLOG(2) << "retry_collision_checker:\t false.";
  }

  QCHECK_NOTNULL(input.passage);
  QCHECK_NOTNULL(input.sl_boundary);
  QCHECK_NOTNULL(input.stop_s_vec);
  QCHECK_NOTNULL(input.leading_groups);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.plan_start_point);
  QCHECK_NOTNULL(input.vehicle_geom);
  QCHECK_NOTNULL(input.collision_checker);
  QCHECK_NOTNULL(input.sampling_params);
  QCHECK_NOTNULL(input.vehicle_drive);
  QCHECK_NOTNULL(input.form_builder);
  // thread pool can be null

  const auto& drive_passage = *input.passage;
  const auto& plan_start_point = *input.plan_start_point;
  const auto& stop_s_vec = *input.stop_s_vec;
  const auto& vehicle_geom = *input.vehicle_geom;
  const auto& sampling_params = *input.sampling_params;

  // 0. Create start node layer
  const Vec2d plan_start_point_pos =
      Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const GeometryState plan_start_state(
      {.xy = plan_start_point_pos,
       .h = plan_start_point.path_point().theta(),
       .k = plan_start_point.path_point().kappa()});
  StationIndex sdc_next_station_idx;
  ASSIGN_OR_RETURN(
      auto start_node_layer,
      CreateStartNodeLayer(drive_passage, plan_start_state,
                           plan_start_point.v(), &sdc_next_station_idx));
  const double sdc_accumulated_s = start_node_layer.front().accumulated_s;

  std::vector<GeometryNodeVector<GeometryNode>> nodes_layers;
  nodes_layers.reserve(CeilToInt((drive_passage.end_s() - sdc_accumulated_s) /
                                 sampling_params.layer_gap_list[0]));
  nodes_layers.push_back(std::move(start_node_layer));

  std::vector<GeometryGraphSamplingParams> layer_params;
  layer_params.push_back(
      sampling_params.FindSamplingParams(/*dist_to_start=*/0.0));

  // 1. Decide curb based on start node and its next station.
  const double curb_buffer =
      FindCurbBuffer(drive_passage, plan_start_point_pos, sdc_next_station_idx,
                     0.5 * vehicle_geom.width());
  const auto [max_l_s, min_l_s] =
      FindLateralBoundaryPlf(drive_passage, curb_buffer);

  // 2. Determine the maximal sampling distance.
  const double max_sampling_accum_s =
      input.form_builder->smooth_dp_sampling_acc_s();
  // 2.1 We consider the stationary leading object, do not sample over it,
  // adjust stop sampling distance.
  double leading_obj_caused_target_s = std::numeric_limits<double>::max();
  std::optional<std::string> static_leading_id = std::nullopt;
  if (!input.lc_multiple_traj && input.leading_groups->size() == 1) {
    // If not generating multiple traj and has only one leading group, cut off
    // the geometry graph behind the nearest stationary leading object.
    const auto [nearest_leading_s, nearest_leading_id] =
        GetLeadingObjectAccumulatedS(
            drive_passage, input.leading_groups->front(), *input.st_traj_mgr);
    static_leading_id = nearest_leading_id;
    // Should at least include one station forward.
    leading_obj_caused_target_s = std::max(
        drive_passage.station(sdc_next_station_idx).accumulated_s() + kEpsilon,
        nearest_leading_s - vehicle_geom.front_edge_to_center() -
            kInitializerMinFollowDistance);
  }
  // 2.2 We consider the stop line information.
  const double stop_line_accumulated_s =
      stop_s_vec.empty() ? std::numeric_limits<double>::max()
                         : std::max(sdc_accumulated_s, stop_s_vec[0]);
  // Construct a list of constraints that we might use when deciding the
  // sampling node layers.
  const double constraint_stop_sampling_s =
      std::min(leading_obj_caused_target_s, stop_line_accumulated_s);

  // 3. Sample nodes according to the sampling strategy.
  int nodes_cnt = nodes_layers.front().size();
  RETURN_IF_ERROR(SampleGeometryNodesWithVaryingResolutionStrategy(
      drive_passage, *input.sl_boundary, sampling_params, max_sampling_accum_s,
      constraint_stop_sampling_s, input.s_from_start, plan_start_point.v(),
      vehicle_geom.width(), sdc_accumulated_s, sdc_next_station_idx.value(),
      &nodes_layers, &layer_params, &nodes_cnt));

  // 4. Create edge info cache and start to connect edges.
  GeometryNodeVector<std::vector<DpEdgeInfo>> outgoing_edges;
  GeometryNodeVector<NodeCollisionInfo> node_collision_infos;
  const double max_curvature =
      ComputeRelaxedCenterMaxCurvature(vehicle_geom, *input.vehicle_drive) *
      kCurvatureRelaxRatio;
  ConnectEdgesInput connect_edge_input{
      .plan_start_point = input.plan_start_point,
      .form_builder = input.form_builder,
      .collision_checker = input.collision_checker,
      .retry_collision_checker = retry_collision_checker,
      .max_l_s = &max_l_s,
      .min_l_s = &min_l_s,
      .max_curvature = max_curvature,
      .ptr_layer_params = &layer_params,
  };
  RETURN_IF_ERROR(ConnectEdges(connect_edge_input, nodes_cnt, &nodes_layers,
                               &outgoing_edges, &node_collision_infos,
                               thread_pool, graph_cache));
  if (FLAGS_planner_initializer_debug_level >= 2) {
    QLOG(INFO) << "Initial geometry cache:\n" << graph_cache->DebugString();
  }

  // 5. After first connection between geometry graph nodes. We check all the
  // unreachable nodes to decide whether to resample a node grid.
  const int nodes_cnt_pre_resample = nodes_cnt;
  AddResampleNodes(drive_passage, &nodes_layers, &nodes_cnt, graph_cache);

  if (nodes_cnt_pre_resample != nodes_cnt) {
    outgoing_edges.clear();
    node_collision_infos.clear();
    ResetReachableStatus(&nodes_layers);
    RETURN_IF_ERROR(ConnectEdges(connect_edge_input, nodes_cnt, &nodes_layers,
                                 &outgoing_edges, &node_collision_infos,
                                 thread_pool, graph_cache));
    if (FLAGS_planner_initializer_debug_level >= 2) {
      QLOG(INFO) << "Resampled geometry cache after adding "
                 << nodes_cnt - nodes_cnt_pre_resample << " nodes:\n"
                 << graph_cache->DebugString();
    }
  }

  // 6. Collect all edges & nodes and build geometry graph. (moving cache
  // result to final geometry graph)
  GeometryEdgeVector<GeometryEdge> edges;
  GeometryNodeVector<std::vector<GeometryEdgeIndex>> outgoing_edges_idxs;
  GeometryNodeVector<GeometryNode> blocked_nodes;
  outgoing_edges_idxs.reserve(nodes_cnt);
  int edge_idx = 0;
  for (auto& out_edges_per_node : outgoing_edges) {
    std::vector<GeometryEdgeIndex> edge_idxs_per_node;
    edge_idxs_per_node.reserve(out_edges_per_node.size());
    for (const auto& out_edge : out_edges_per_node) {
      // out_edge type is DpEdgeInfo (key, start_node, end_node, truncate).
      const GeometryEdgeIndex cur_idx(edge_idx);
      // If we are processing a complete edge.
      if (!out_edge.truncate) {
        ASSIGN_OR_DIE(const auto ptr_geometry_form,
                      graph_cache->GetGeometryForm(out_edge.key));
        edges.push_back(GeometryEdge{.index = cur_idx,
                                     .start = out_edge.start_node_index,
                                     .end = out_edge.end_node_index,
                                     .geometry = ptr_geometry_form});
      } else {
        // If we are processing a truncated edge due to collision, create an
        // end node.
        const auto& final_state = graph_cache->GetFinalState(out_edge.key);
        ASSIGN_OR_CONTINUE(
            const auto station_waypoint,
            drive_passage.QueryFrenetLonOffsetAt(final_state.xy));
        GeometryNode end_node = GeometryNode{
            .index = GeometryNodeIndex(nodes_cnt++),
            .xy = final_state.xy,
            .k = 0.0,
            .station_index = station_waypoint.station_index.value(),
            .lateral_offset = final_state.l,
            .accumulated_s = final_state.accumulated_s,
            .reachable = true};
        blocked_nodes.push_back(end_node);
        ASSIGN_OR_DIE(const auto ptr_geometry_form,
                      graph_cache->GetGeometryForm(out_edge.key));
        edges.push_back(GeometryEdge{.index = cur_idx,
                                     .start = out_edge.start_node_index,
                                     .end = end_node.index,
                                     .geometry = ptr_geometry_form,
                                     .truncated = true});
      }
      edge_idxs_per_node.push_back(cur_idx);
      edge_idx++;
    }
    outgoing_edges_idxs.push_back(std::move(edge_idxs_per_node));
  }
  for (int i = 0; i < blocked_nodes.size(); ++i) {
    outgoing_edges_idxs.emplace_back(std::vector<GeometryEdgeIndex>());
  }
  nodes_layers.push_back(std::move(blocked_nodes));
  VLOG(2) << "Total edges " << edges.size();

  // 7. Build layers.
  GeometryNodeVector<GeometryNode> nodes;
  nodes.resize(nodes_cnt);
  std::vector<std::vector<GeometryNodeIndex>> nodes_layers_idx;
  nodes_layers_idx.reserve(nodes_layers.size());
  for (int i = 0; i < nodes_layers.size(); ++i) {
    auto& nodes_layer = nodes_layers[i];
    std::vector<GeometryNodeIndex> nodes_layer_indices;
    nodes_layer_indices.reserve(nodes_layer.size());
    for (auto& node : nodes_layer) {
      nodes_layer_indices.push_back(node.index);
      nodes[node.index] = std::move(node);
    }
    nodes_layers_idx.push_back(std::move(nodes_layer_indices));
  }

  // 8. Build end reason.
  double max_geom_graph_len = std::numeric_limits<double>::lowest();
  for (const auto& node : nodes) {
    if (node.reachable) {
      max_geom_graph_len = std::max(node.accumulated_s, max_geom_graph_len);
    }
  }
  max_geom_graph_len = std::min(max_geom_graph_len, constraint_stop_sampling_s);
  GeometryGraphProto::EndInfo end_info;
  std::optional<std::string> colliding_obj_id;
  double max_collision_distance = std::numeric_limits<double>::lowest();
  for (const auto& node_collision_info : node_collision_infos) {
    if (node_collision_info.collision_obj_id_with_max_s.has_value() &&
        node_collision_info.max_collision_s > max_collision_distance &&
        node_collision_info.max_collision_s < constraint_stop_sampling_s) {
      max_collision_distance = node_collision_info.max_collision_s;
      colliding_obj_id = node_collision_info.collision_obj_id_with_max_s;
    }
  }
  VLOG(2) << "max_collision_distance: (within reachable accum_s_limit) "
          << max_collision_distance;
  VLOG(2) << "stop_line_accumualted_s: " << stop_line_accumulated_s;
  VLOG(2) << "leading obj caused: " << leading_obj_caused_target_s;
  VLOG(2) << "reachable accumu s limit: " << constraint_stop_sampling_s;
  VLOG(2) << "geometry_graph_len: " << max_geom_graph_len;
  // If geometry graph is shorter than the maximal collision distance, means
  // that some obj blocked the entire path.
  const double route_s_acc_s_diff = input.s_from_start - sdc_accumulated_s;
  if (colliding_obj_id.has_value() &&
      max_geom_graph_len <= max_collision_distance) {
    end_info.set_end_reason(GeometryGraphProto::END_STATIC_OBJ);
    end_info.set_object_id(colliding_obj_id.value());
    VLOG(2) << "End_info: static_obj: " << colliding_obj_id.value();
    // Set route s for cross frame consistency
    end_info.set_end_route_s(max_collision_distance + route_s_acc_s_diff);
    end_info.set_end_accumulated_s(max_collision_distance);
  } else {
    // If reach maximal sampling distance.
    if (max_sampling_accum_s < constraint_stop_sampling_s) {
      end_info.set_end_reason(GeometryGraphProto::END_REACH_MAX_SAMPLING_DIS);
      end_info.set_end_route_s(max_sampling_accum_s + route_s_acc_s_diff);
      end_info.set_end_accumulated_s(max_sampling_accum_s);
      VLOG(2) << "End_info: max_sampling_accum_s";
    } else {
      // If leading object causes end of sampling.
      if (leading_obj_caused_target_s < stop_line_accumulated_s) {
        end_info.set_end_reason(GeometryGraphProto::END_STATIC_LEADING_OBJ);
        end_info.set_object_id(*static_leading_id);
        end_info.set_end_route_s(leading_obj_caused_target_s +
                                 route_s_acc_s_diff);
        end_info.set_end_accumulated_s(leading_obj_caused_target_s);
        VLOG(2) << "End_info: static_leading_obj: " << *static_leading_id;
      } else {
        // If stop line causes end of sampling.
        end_info.set_end_reason(GeometryGraphProto::END_STOP_LINE);
        end_info.set_end_route_s(stop_line_accumulated_s + route_s_acc_s_diff);
        end_info.set_end_accumulated_s(stop_line_accumulated_s);
        VLOG(2) << "End_info: end_stop_line.";
      }
    }
  }

  if (FLAGS_planner_initializer_debug_level >= 1) {
    debug_proto->mutable_geometry_graph_debug()->Clear();
    graph_cache->ParseConnectionProcessDebugInfoToProto(
        debug_proto->mutable_geometry_graph_debug());
    graph_cache->ParseResampleReasonToProto(
        debug_proto->mutable_geometry_graph_debug());
    graph_cache->ParseCollisionInfoToProto(
        debug_proto->mutable_geometry_graph_debug());
  }
  VLOG(2) << "BuildCurvyGraph time: " << absl::Now() - start_time;
  VLOG(2) << "------------------------------------";
  return XYGeometryGraph(std::move(nodes), std::move(nodes_layers_idx),
                         std::move(edges), std::move(outgoing_edges_idxs),
                         std::move(end_info));
}

}  // namespace qcraft::planner
