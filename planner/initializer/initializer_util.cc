#include "onboard/planner/initializer/initializer_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <stddef.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "boost/intrusive/list.hpp"
#include "boost/iterator/iterator_facade.hpp"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/astar_motion_searcher_defs.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"
#include "onboard/vis/common/colormap.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft::planner {

namespace {

double FindRequiredLonDecel(
    double ego_front_to_ra, double ego_s, double ego_v,
    absl::Span<const ConstraintProto::StopLineProto> stop_lines,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    int traj_steps) {
  ego_s += ego_front_to_ra;
  const double traj_time = traj_steps * kTrajectoryTimeStep;
  double decel = 0.0;
  if (!stop_lines.empty()) {
    const double min_stop_s = stop_lines.front().s();
    decel = 2.0 * (min_stop_s - ego_s - ego_v * traj_time) / Sqr(traj_time);
  }
  constexpr double kLeadingLonBuffer = 0.3;  // m.
  ego_s += kLeadingLonBuffer;
  for (const auto& [traj_id, leading_traj] : leading_trajs) {
    for (const auto& st_constraint : leading_traj.st_constraints()) {
      decel = std::min(
          decel, 2.0 * (st_constraint.s() - ego_s - ego_v * st_constraint.t()) /
                     Sqr(st_constraint.t() - kTrajectoryTimeStep));
    }
  }
  return decel;
}

void FillAstarNodeInProto(const MotionSearchOutput& search_output,
                          const AstarNode& node,
                          AstarGraphProto::AstarNode* new_node_proto) {
  node.state.ToProto(new_node_proto->mutable_state());
  new_node_proto->set_visited(true);
  new_node_proto->set_cum_cost_g(node.cost_g);
  new_node_proto->set_cost_h(node.cost_h);
  new_node_proto->set_node_index(node.key);
  new_node_proto->set_parent_index(node.parent);
  if (node.motion != nullptr) {
    const auto cost_names = search_output.cost_provider->cost_names();
    for (size_t i = 0; i < cost_names.size(); ++i) {
      new_node_proto->add_feature_costs(cost_names[i] + ": " +
                                        std::to_string(node.feature_costs[i]));
    }
  }
}

void SendAstarGraphToProto(const MotionSearchOutput& search_output,
                           AstarGraphProto* proto) {
  const auto open_pq = search_output.astar_open_pq;
  const auto close_map = search_output.astar_close_map;
  proto->set_end_node_index(search_output.astar_end_node_index);

  // Fill close map
  absl::flat_hash_set<GeometryNodeIndex> close_set;
  for (const auto& close_pair : close_map) {
    const auto node = close_pair.second;
    close_set.insert(node->geom_index);
    // Set nodes
    auto* new_node_proto = proto->add_nodes();
    FillAstarNodeInProto(search_output, *node, new_node_proto);
    // Set edges
    if (node->motion == nullptr) {
      // Root node has no motion
      continue;
    }
    const auto edge = node->motion;
    auto* new_edge_proto = proto->add_edges();
    new_edge_proto->set_visited(true);
    // constexpr double kSampleS = 0.5;  // m.
    const auto states = edge->SampleEqualIntervalStates();
    for (const auto& state : states) {
      state.ToProto(new_edge_proto->add_states());
    }
  }
  // Fill open map
  for (const auto& node : open_pq) {
    if (close_set.find(node->geom_index) != close_set.end()) {
      continue;
    }
    // Set nodes
    auto* new_node_proto = proto->add_nodes();
    FillAstarNodeInProto(search_output, *node, new_node_proto);
    // Set edges
    const auto edge = node->motion;
    auto* new_edge_proto = proto->add_edges();
    new_edge_proto->set_visited(false);
    // constexpr double kSampleS = 0.5;  // m.
    const auto states = edge->SampleEqualIntervalStates();
    for (const auto& state : states) {
      state.ToProto(new_edge_proto->add_states());
    }
  }
  // // Fill final trajectory info
  auto node = close_map.at(search_output.astar_end_node_index);
  auto* trajectory_proto = proto->mutable_trajectory();
  while (node->motion != nullptr) {
    FillAstarNodeInProto(search_output, *node, trajectory_proto->add_nodes());
    // Set edges
    const auto edge = node->motion;
    auto* new_edge_proto = trajectory_proto->add_edges();
    new_edge_proto->set_visited(true);
    const auto states = edge->SampleEqualIntervalStates();
    for (const auto& state : states) {
      state.ToProto(new_edge_proto->add_states());
    }
    node = close_map.at(node->parent);
  }
  auto* root_node_proto = trajectory_proto->add_nodes();
  FillAstarNodeInProto(search_output, *node, root_node_proto);
}

}  // namespace

void SendSingleTrajectoryToCanvas(
    const MotionSearchOutput::MultiTrajCandidate& traj_info, int idx,
    int planner_id) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
      absl::StrCat("multi_traj_idx_", idx, "_planner_", planner_id));
  canvas.SetGroundZero(1);
  const auto& trajectory = traj_info.trajectory;
  std::vector<Vec3d> points;
  points.reserve(trajectory.size());
  for (const auto& pt : trajectory) {
    Vec3d point(pt.path_point().x(), pt.path_point().y(), pt.relative_time());
    points.emplace_back(std::move(point));
  }
  // Draw trajectory.
  canvas.DrawLineStrip(points, vis::Color::kMagenta, 3);
  const auto& last_point = points.back();
  // Write leading object info.
  canvas.DrawText(absl::StrJoin(traj_info.leading_traj_ids, ","), last_point,
                  trajectory.front().path_point().theta() - M_PI / 2.0, 0.8,
                  vis::Color::kWhite);
  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendGeometryGraphToCanvas(const GeometryGraph* graph,
                               const std::string& channel) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto& node : graph->nodes()) {
    const auto node_color =
        node.reachable ? (node.active ? vis::Color::kCyan : vis::Color::kOrange)
                       : vis::Color::kRed;
    canvas.DrawPoint(Vec3d(node.xy.x(), node.xy.y(), 0), node_color, 7);

    const auto text_color =
        node.resampled ? vis::Color::kLightYellow : vis::Color::kLightRed;
    canvas.DrawText(absl::StrCat(node.station_index, node.index.value()),
                    Vec3d(node.xy.x(), node.xy.y(), 0), 0, 0.1, text_color);

    for (const auto& out_edge_index : graph->GetOutgoingEdges(node.index)) {
      const auto& edge = graph->edges()[out_edge_index];
      const auto edge_color =
          edge.active ? vis::Color::kLightGray : vis::Color::kDarkGray;

      const auto samples = edge.geometry->Sample(0.1);
      std::vector<Vec3d> vertices;
      vertices.reserve(samples.size());
      for (const auto& sample : samples) {
        vertices.emplace_back(sample.xy.x(), sample.xy.y(), 0);
      }
      canvas.DrawLineStrip(vertices, edge_color, 1);
    }
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendRefSpeedTableToCanvas(const RefSpeedTable& ref_speed_table,
                               const DrivePassage& drive_passage) {
  constexpr double kSpanSampleStep = 0.5;
  const std::array<double, 11> sample_times = {0.0, 0.9, 1.9, 2.9, 3.9, 4.9,
                                               5.9, 6.9, 7.9, 8.9, 9.9};

  double max_speed_limit = 0.0;
  for (const auto& station : drive_passage.stations()) {
    max_speed_limit = std::max(max_speed_limit, station.speed_limit());
  }
  const vis::Colormap color_map("jet", 0.0, max_speed_limit);

  const double end_s = drive_passage.end_s();
  const int sample_num = FloorToInt(end_s / kSpanSampleStep) + 1;
  std::vector<Vec3d> points;
  points.reserve(sample_num);
  for (double sample_s = 0; sample_s < end_s; sample_s += kSpanSampleStep) {
    const Vec2d xy_pos = drive_passage.QueryPointXYAtS(sample_s).value();
    points.emplace_back(xy_pos, 0.0);
  }

  for (const double sample_time : sample_times) {
    int sample_index_end = sample_num;
    for (int j = 0; j < sample_num; ++j) {
      const double sample_s = j * kSpanSampleStep;
      points[j].z() =
          ref_speed_table.LookUpRefSpeed(sample_time, sample_s).second;

      if (points[j].z() < 1e-6) {  // Cut off at some early stop constraint.
        sample_index_end = j;
        break;
      }
    }

    auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
        absl::StrFormat("ref_speed_table/at %.1f", sample_time));
    canvas.SetGroundZero(1);
    for (int j = 0; j < sample_index_end - 1; ++j) {
      canvas.DrawLine(points[j], points[j + 1], color_map.Map(points[j].z()),
                      /*line_width=*/3);
    }
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>>
GenerateConstLateralAccelConstSpeedTraj(
    const DrivePassage& drive_passage, double ego_front_to_ra, double target_l,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    absl::Span<const ConstraintProto::StopLineProto> stop_lines,
    const ApolloTrajectoryPointProto& plan_start_point, int traj_steps) {
  QCHECK_GT(traj_steps, 0);
  constexpr double kStationarySpeedThres = 0.1;  // m/s.
  if (plan_start_point.v() < kStationarySpeedThres) {
    std::vector<ApolloTrajectoryPointProto> stationary_traj(traj_steps,
                                                            plan_start_point);
    for (int i = 1; i < traj_steps; ++i) {
      stationary_traj[i].set_relative_time(i * kTrajectoryTimeStep);
    }
    return stationary_traj;
  }

  const auto ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  ASSIGN_OR_RETURN(const auto ego_sl,
                   drive_passage.QueryFrenetCoordinateAt(ego_pos),
                   _ << "Unable to project ego position onto drive passage.");
  ASSIGN_OR_RETURN(const auto lane_tangent,
                   drive_passage.QueryTangentAtS(ego_sl.s),
                   _ << "Unable to find lane tangent at ego position.");

  const double required_lon_decel =
      FindRequiredLonDecel(ego_front_to_ra, ego_sl.s, plan_start_point.v(),
                           stop_lines, leading_trajs, traj_steps);

  constexpr double kMaxLaneChangeCancelLatAccel = 0.5;   // m/s^2.
  constexpr double kZeroLateralVelThreshold = 0.01;      // m/s.
  constexpr double kZeroLongitudinalVelThreshold = 1.0;  // m/s.
  constexpr double kDt = kTrajectoryTimeStep;

  const auto heading_tangent =
      Vec2d::FastUnitFromAngle(plan_start_point.path_point().theta());
  double lon_v = plan_start_point.v() * lane_tangent.Dot(heading_tangent);
  double lat_v = plan_start_point.v() * lane_tangent.CrossProd(heading_tangent);
  lat_v = std::fabs(lat_v) < kZeroLateralVelThreshold ? 0.0 : lat_v;
  const double lon_a =
      std::min(plan_start_point.a() * lane_tangent.Dot(heading_tangent),
               required_lon_decel);
  double lat_a =
      plan_start_point.a() * lane_tangent.CrossProd(heading_tangent) +
      Sqr(plan_start_point.v()) * lane_tangent.Dot(heading_tangent) *
          plan_start_point.path_point().kappa();

  std::vector<FrenetCoordinate> traj_sl;
  traj_sl.reserve(traj_steps);
  traj_sl.push_back(ego_sl);
  double s = ego_sl.s, l = ego_sl.l;
  // Assume const lateral jerk until lateral acceleration reaches expected const
  // accel value.
  constexpr double kMaxComfortLatJerk = 1.0;  // m/s^3.
  const double expect_lat_a =
      std::copysign(kMaxLaneChangeCancelLatAccel, lat_v);
  const int const_lat_jerk_steps =
      CeilToInt(std::abs(expect_lat_a - lat_a) / (kMaxComfortLatJerk * kDt));
  for (int i = 1; i < const_lat_jerk_steps + 1; ++i) {
    s += lon_v * kDt;
    l += lat_v * kDt;
    traj_sl.push_back(FrenetCoordinate{.s = s, .l = l});

    lon_v = std::max(0.0, lon_v + lon_a * kDt);
    lat_v += lat_a * kDt;
    lat_a += std::copysign(
        std::min(kMaxComfortLatJerk * kDt, std::abs(expect_lat_a - lat_a)),
        expect_lat_a - lat_a);
    if (std::abs(lon_v) < kZeroLongitudinalVelThreshold) break;
  }
  // Assume const lateral acceleration for the rest of trajectory.
  constexpr double kReachTargetLaneThreshold = 0.1;  // m.
  constexpr double kMaxLaneChangeLatAccel = 0.09;    // m/s^2.
  const double l_diff = l - target_l;
  constexpr double kNearTargetLaneThreshold = 0.3;  // m.
  lat_a = std::abs(l_diff) < kNearTargetLaneThreshold
              ? kMaxLaneChangeCancelLatAccel
              : (lat_v * l_diff < 0.0 ? kMaxLaneChangeLatAccel
                                      : kMaxLaneChangeCancelLatAccel);
  lat_a = std::copysign(lat_a, -l_diff);
  for (int i = traj_sl.size(); i < traj_steps; ++i) {
    if (std::abs(lon_v) < kZeroLongitudinalVelThreshold) break;
    const double new_l = l + lat_v * kDt;
    if ((std::fabs(new_l - target_l) < kReachTargetLaneThreshold &&
         std::fabs(lat_v) < kZeroLateralVelThreshold) ||
        (new_l - target_l) * (l - target_l) < 0.0) {
      break;
    }

    s += lon_v * kDt;
    l += lat_v * kDt;
    traj_sl.push_back(FrenetCoordinate{.s = s, .l = l});

    lon_v = std::max(0.0, lon_v + lon_a * kDt);
    lat_v += lat_a * kDt;
  }
  // Fill the rest trajectory points.
  for (int i = traj_sl.size(); i < traj_steps; ++i) {
    s += lon_v * kDt;
    lon_v = std::max(0.0, lon_v + lon_a * kDt);
    traj_sl.push_back(FrenetCoordinate{.s = s, .l = l});
  }
  const double max_accum_s = drive_passage.end_s();
  if (traj_sl.back().s > max_accum_s) {
    const double ratio = max_accum_s / traj_sl.back().s;
    for (auto& pt_sl : traj_sl) pt_sl.s *= ratio;
  }

  std::vector<Vec2d> traj_xy;
  traj_xy.reserve(traj_sl.size());
  for (const auto& pt_sl : traj_sl) {
    // Should not exceed drive passage since already post-processed.
    ASSIGN_OR_RETURN(const auto pt_xy,
                     drive_passage.QueryPointXYAtSL(pt_sl.s, pt_sl.l),
                     _ << "Lc pause trajectory out of drive passage.");
    traj_xy.push_back(pt_xy);
  }
  std::vector<double> traj_v(traj_steps), traj_a(traj_steps);
  for (int i = 1; i + 1 < traj_steps; ++i) {
    traj_v[i] = traj_xy[i + 1].DistanceTo(traj_xy[i]) / kDt;
  }
  traj_v[traj_steps - 1] = traj_v[traj_steps - 2];
  for (int i = 1; i + 1 < traj_steps; ++i) {
    traj_a[i] = (traj_v[i + 1] - traj_v[i]) / kDt;
  }
  traj_a[traj_steps - 1] = traj_a[traj_steps - 2];

  constexpr double kEpsilon = 1e-3;
  // Transform into trajectory proto.
  std::vector<ApolloTrajectoryPointProto> traj_pts;
  traj_pts.reserve(traj_steps);
  traj_pts.push_back(plan_start_point);
  for (int i = 1; i < traj_steps; ++i) {
    PathPoint path_pt;
    path_pt.set_x(traj_xy[i].x());
    path_pt.set_y(traj_xy[i].y());
    path_pt.set_z(0.0);
    const double dist2prev = traj_xy[i].DistanceTo(traj_xy[i - 1]);
    const double cur_theta =
        i + 1 == traj_steps || traj_xy[i + 1].DistanceTo(traj_xy[i]) < kEpsilon
            ? traj_pts[i - 1].path_point().theta()
            : (traj_xy[i + 1] - traj_xy[i]).FastAngle();
    path_pt.set_theta(cur_theta);
    path_pt.set_kappa(
        dist2prev < kEpsilon
            ? 0.0
            : NormalizeAngle(cur_theta - traj_pts[i - 1].path_point().theta()) /
                  dist2prev);
    path_pt.set_s(traj_pts[i - 1].path_point().s() + dist2prev);

    ApolloTrajectoryPointProto traj_point;
    *traj_point.mutable_path_point() = std::move(path_pt);
    traj_point.set_v(traj_v[i]);
    traj_point.set_a(traj_a[i]);
    traj_point.set_relative_time(i * kTrajectoryTimeStep);

    traj_pts.push_back(std::move(traj_point));
  }

  return traj_pts;
}

void ParseMotionSearchOutputToMotionSearchDebugProto(
    const MotionSearchOutput& search_output, MotionSearchDebugProto* proto) {
  proto->Clear();

  // Set Astar graph
  // Set Astar geometry graph
  if (!search_output.astar_close_map.empty()) {
    SendAstarGraphToProto(search_output, proto->mutable_astar_graph());
  }

  // Set cost_names.
  for (const auto& name : search_output.cost_provider->cost_names()) {
    auto* cost_name = proto->add_cost_names();
    *cost_name = name;
  }

  if (search_output.motion_graph == nullptr) return;

  search_output.motion_graph->ToProto(proto->mutable_motion_graph());

  // Set edge costs.
  proto->mutable_edge_costs()->Reserve(search_output.motion_graph->edge_size());
  for (const auto& edge_cost : search_output.search_costs) {
    auto* edge_cost_proto = proto->add_edge_costs();
    edge_cost_proto->mutable_costs()->Reserve(edge_cost.feature_cost.size());
    for (const double c : edge_cost.feature_cost) {
      edge_cost_proto->add_costs(c);
    }
    QCHECK_EQ(edge_cost_proto->costs_size(), proto->cost_names_size());
    edge_cost_proto->set_cum_cost(edge_cost.cost_to_come);
  }

  proto->set_best_last_edge_index(search_output.best_last_edge_index.value());

  // Get considered trajectory costs (compare with Set edge costs)
  // feature costs, final costs, final progress costs
  proto->mutable_top_k_trajs()->Reserve(search_output.top_k_trajs.size());
  for (int i = 0; i < search_output.top_k_trajs.size(); ++i) {
    const auto last_edge = search_output.top_k_edges[i];
    auto* traj_info_proto = proto->add_top_k_trajs();
    // Feature costs
    traj_info_proto->mutable_costs()->Reserve(
        search_output.cost_provider->cost_names().size());
    for (const double c : search_output.search_costs[last_edge].feature_cost) {
      traj_info_proto->add_costs(c);
    }
    traj_info_proto->set_total_cost(search_output.top_k_total_costs[i]);
    traj_info_proto->set_last_edge_idx(last_edge.value());
    for (const auto& point : search_output.top_k_trajs[i]) {
      *traj_info_proto->add_traj_points() = ToPoseTrajectoryPointProto(point);
    }
  }
}

void ParseMotionSearchOutputToMultiTrajDebugProto(
    const MotionSearchOutput& search_output, MultiTrajDebugProto* proto) {
  const auto& multi_trajs = search_output.multi_traj_candidates;
  proto->Clear();

  proto->mutable_multi_traj_candidates()->Reserve(multi_trajs.size());
  for (const auto& traj : multi_trajs) {
    auto* traj_proto = proto->add_multi_traj_candidates();
    // Set traj points.
    traj_proto->mutable_traj_points()->Reserve(traj.trajectory.size());
    for (const auto& point : traj.trajectory) {
      *traj_proto->add_traj_points() = ToPoseTrajectoryPointProto(point);
    }
    // Set leading trajs.
    traj_proto->mutable_leading_traj_ids()->Reserve(
        traj.leading_traj_ids.size());
    for (const auto& lead_traj_id : traj.leading_traj_ids) {
      traj_proto->add_leading_traj_ids(lead_traj_id);
    }
    traj_proto->mutable_ignored_trajs()->Reserve(traj.ignored_trajs.size());
    for (const auto& [traj_id, info] : traj.ignored_trajs) {
      MultiTrajDebugProto::IgnoredObjectTrajectoryProto ignored_traj;
      ignored_traj.set_traj_id(traj_id);
      ignored_traj.set_time_idx(info.time_idx);
      ignored_traj.set_collision_config(
          static_cast<int>(info.collision_config));
      *traj_proto->add_ignored_trajs() = std::move(ignored_traj);
    }

    // Set cost.
    traj_proto->set_total_cost(traj.total_cost);
  }
}

void ParseMotionSearchOutputToInitializerResult(
    const MotionSearchOutput& search_output, InitializerDebugProto* proto) {
  proto->set_lc_pause(search_output.is_lc_pause);

  switch (search_output.search_algorithm) {
    case MotionSearchOutput::SearchAlgorithm::kDP:
      proto->set_search_algorithm(InitializerDebugProto::DP);
      break;
    case MotionSearchOutput::SearchAlgorithm::kASTAR:
      proto->set_search_algorithm(InitializerDebugProto::ASTAR);
      break;
  }

  auto* follower_set = proto->mutable_follower_objects();
  follower_set->Clear();
  for (const auto& obj_id : search_output.follower_set) {
    *follower_set->Add() = obj_id;
  }

  auto* resampled_traj = proto->mutable_resampled_trajectory();
  resampled_traj->Clear();
  resampled_traj->mutable_trajectory_points()->Reserve(
      search_output.traj_points.size());
  for (const auto& point : search_output.traj_points) {
    *resampled_traj->add_trajectory_points() = point;
  }
}

void ExportMoionSpeedProfileToChart(const MotionSearchOutput& search_output,
                                    vis::vantage::ChartDataProto* chart) {
  QCHECK_NOTNULL(chart);
  const auto& traj_pts = search_output.traj_points;

  chart->set_title("initializer/speed_profile");
  auto* subchart = chart->add_subcharts();
  subchart->set_x_name("time(s)");
  auto* x_value_config = subchart->mutable_x_value_config();
  x_value_config->set_begin(traj_pts.front().relative_time());
  x_value_config->set_size(traj_pts.size());
  x_value_config->set_interval(traj_pts[1].relative_time() -
                               traj_pts[0].relative_time());

  auto* speed = subchart->add_y();
  speed->set_name("speed(m/s)");
  auto* a = subchart->add_y();
  a->set_name("acceleration");

  for (const auto& point : search_output.traj_points) {
    speed->add_values(point.v());
    a->add_values(point.a());
  }
}

InitializerOutput MakeAebInitializerOutput(
    int traj_steps, ApolloTrajectoryPointProto plan_start_point,
    InitializerStateProto new_state, absl::string_view message,
    InitializerDebugProto* debug_proto) {
  QLOG(WARNING) << message << "Initializer will output an AEB trajectory!";

  constexpr double kAebDecel = -10.0;  // m/s^2.
  std::vector<ApolloTrajectoryPointProto> aeb_traj;
  aeb_traj.reserve(traj_steps);
  ApolloTrajectoryPointProto cur_point = std::move(plan_start_point);
  for (int i = 0; i < traj_steps; ++i) {
    const double dist =
        std::max(0.0, (cur_point.v() + 0.5 * kAebDecel * kTrajectoryTimeStep) *
                          kTrajectoryTimeStep);
    ApolloTrajectoryPointProto next_point;
    *next_point.mutable_path_point() =
        GetPathPointAlongCircle(cur_point.path_point(), dist);
    next_point.set_relative_time((i + 1) * kTrajectoryTimeStep);
    next_point.set_v(
        std::max(0.0, cur_point.v() + kTrajectoryTimeStep * kAebDecel));
    next_point.set_a(
        std::max(kAebDecel, (0.0 - next_point.v()) / kTrajectoryTimeStep));
    cur_point.set_j((next_point.a() - cur_point.a()) / kTrajectoryTimeStep);
    aeb_traj.push_back(std::move(cur_point));
    cur_point = std::move(next_point);
  }

  debug_proto->mutable_multi_traj_debug()
      ->mutable_multi_traj_candidates()
      ->Clear();
  auto& debug_traj = *debug_proto->mutable_multi_traj_debug()
                          ->add_multi_traj_candidates()
                          ->mutable_traj_points();
  debug_traj.Clear();
  debug_traj.Reserve(aeb_traj.size());
  for (const auto& traj_pt : aeb_traj) {
    *debug_traj.Add() = ToPoseTrajectoryPointProto(traj_pt);
  }

  return InitializerOutput{.traj_points = std::move(aeb_traj),
                           .initializer_state = std::move(new_state)};
}

}  // namespace qcraft::planner
