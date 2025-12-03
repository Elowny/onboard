#include "onboard/planner/freespace/hybrid_a_star/hybrid_a_star.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "boost/heap/fibonacci_heap.hpp"
#include "boost/heap/policies.hpp"
#include "boost/intrusive/link_mode.hpp"
#include "boost/intrusive/list.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/vehicle_octagon_model.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/freespace/hybrid_a_star/a_star_heuristics.h"
#include "onboard/planner/freespace/hybrid_a_star/fast_reeds_shepp.h"
#include "onboard/planner/freespace/hybrid_a_star/node_3d.h"
#include "onboard/planner/freespace/hybrid_a_star/reeds_shepp.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DEFINE_bool(send_sampling_rs_extension_to_canvas, false,
            "Whether to send all sampling path to canvas.");
DEFINE_bool(send_hybrid_a_star_output_path_to_canvas, false,
            "Whether to send hybrid a star path to canvas.");
DEFINE_bool(enable_parking_reverse_plan, true,
            "Whether to exchange start and end of parallel parking or in "
            "narrow scenarios in ha*.");

namespace qcraft {
namespace planner {
namespace {

struct Motion {
  double delta_x;
  double delta_y;
  double delta_theta;
  double steer;
  double s;
  bool forward;
};

struct HybridAStarPath {
  std::vector<std::shared_ptr<Node3d>> nodes;
  double cost = 0.0;
  int ha_star_nodes_num = 0;
};

void SendHybridAStarPathToCanvas(const HybridAStarPath& path,
                                 bool exchange_start_end,
                                 const std::string& name) {
  vis::Canvas& canvas_path = vis::vantage::GetCanvasClient()->GetCanvas(name);
  for (int i = 0; i + 1 < path.nodes.size(); ++i) {
    const bool forward = path.nodes[i + 1]->forward();
    const auto color =
        (forward ^ exchange_start_end) ? vis::Color::kGreen : vis::Color::kRed;
    canvas_path.DrawLine(
        Vec3d(path.nodes[i]->x(), path.nodes[i]->y(), 0.0),
        Vec3d(path.nodes[i + 1]->x(), path.nodes[i + 1]->y(), 0.0), color);
  }
  for (int i = 0; i < path.ha_star_nodes_num; ++i) {
    canvas_path.DrawCircle(Vec3d(path.nodes[i]->x(), path.nodes[i]->y(), 0.0),
                           0.05, vis::Color::kViolet);
  }
  for (int i = path.ha_star_nodes_num; i < path.nodes.size(); ++i) {
    canvas_path.DrawCircle(Vec3d(path.nodes[i]->x(), path.nodes[i]->y(), 0.0),
                           0.05, vis::Color::kOrange);
  }
}

bool CheckNodeValidityWithKDTree(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const AABox2d& region, const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<SpecialBoundary>& special_boundaries,
    const Node3d& node) {
  if (node.x() < region.min_x() || node.y() < region.min_y() ||
      node.x() > region.max_x() || node.y() > region.max_y()) {
    return false;
  }
  if (!CheckPoseValidityWithKDTree(veh_geo_params, path_finder_params,
                                   vehicle_model_params, segments_kd_tree,
                                   objects_map, boundaries_map,
                                   Vec2d(node.x(), node.y()), node.theta(),
                                   Vec2d(node.cos_theta(), node.sin_theta()))) {
    return false;
  }
  for (const auto& boundary : special_boundaries) {
    switch (boundary.type) {
      case FreespaceMapProto::GEAR_REVERSE_STOPPER: {
        QCHECK_EQ(boundary.points.size(), 2);
        // Only enable when gear reverse.
        if (!node.forward()) {
          const auto vehicle_box = node.GetVehicleBoundingBoxWithBuffer(
              veh_geo_params, /*lateral_buffer=*/0.0,
              /*longitudinal_buffer=*/0.0);
          if (vehicle_box.HasOverlap(
                  Segment2d(boundary.points[0], boundary.points[1])))
            return false;
        }
      } break;
      case FreespaceMapProto::SOFT_PARKING_SPOT_LINE:
      case FreespaceMapProto::CROSSABLE_LANE_LINE:
        break;
    }
  }
  return true;
}

bool CheckRSExtensionValidityWithKDTree(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const AABox2d& region, const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<SpecialBoundary>& special_boundaries,
    const std::vector<std::shared_ptr<Node3d>>& rs_extension) {
  if (rs_extension.empty()) return false;
  for (const auto& node : rs_extension) {
    if (!CheckNodeValidityWithKDTree(
            path_finder_params, veh_geo_params, vehicle_model_params, region,
            segments_kd_tree, objects_map, boundaries_map, special_boundaries,
            *node)) {
      return false;
    }
  }
  return true;
}

// Returns a pair of <forward_motions, backward_motions>.
std::pair<std::vector<Motion>, std::vector<Motion>> GenerateMotions(
    const std::vector<double>& steers, double s) {
  std::vector<Motion> forward_motions;
  std::vector<Motion> backward_motions;
  forward_motions.reserve(steers.size() + 1);
  backward_motions.reserve(steers.size() + 1);
  constexpr double kEpsilon = 1e-6;
  for (const double steer : steers) {
    if (std::abs(steer) < kEpsilon) {
      forward_motions.push_back({s, 0.0, 0.0, 0.0, s, true});
      backward_motions.push_back({-s, 0.0, 0.0, 0.0, s, false});
      continue;
    }
    const double delta_theta = steer * s;
    double theta_cos_sin[2];
    fast_math::CosAndSin<7>(delta_theta * 0.5, theta_cos_sin);
    const double sin_theta = theta_cos_sin[1];
    const double cos_theta = theta_cos_sin[0];
    const double delta_x = 2.0 * sin_theta * cos_theta / steer;
    const double delta_y = 2.0 * sin_theta * sin_theta / steer;
    forward_motions.push_back({delta_x, delta_y, delta_theta, steer, s, true});
    backward_motions.push_back(
        {-delta_x, delta_y, -delta_theta, steer, s, false});
  }
  return std::make_pair(std::move(forward_motions),
                        std::move(backward_motions));
}

std::vector<std::shared_ptr<Node3d>> GenerateNextNodes(
    const std::pair<std::vector<Motion>, const std::vector<Motion>>&
        all_motions,
    const std::vector<double>& all_steers, bool enable_reverse_driving,
    const Node3d& node, double steer_step) {
  constexpr double kEpsilon = 1e-6;

  const auto select_motion = [&all_motions](double motion_steer,
                                            bool forward) -> Motion {
    if (forward) {
      for (const auto& motion : all_motions.first) {
        if (std::abs(motion.steer - motion_steer) < kEpsilon) {
          return motion;
        }
      }
    } else {
      for (const auto& motion : all_motions.second) {
        if (std::abs(motion.steer - motion_steer) < kEpsilon) {
          return motion;
        }
      }
    }
    QLOG(FATAL);  // Unexpected.
    return {0.0, 0.0, 0.0, 0.0, 0.0, true};
  };

  std::vector<Motion> possible_motions;
  for (const double next_steer : all_steers) {
    // If keep direction, the steer change should be no larger than steer_step.
    if (std::abs(node.steer() - next_steer) < steer_step + kEpsilon) {
      possible_motions.push_back(select_motion(next_steer, node.forward()));
    }
    if (!enable_reverse_driving) continue;
    // Don't allow change direction continuously.
    if (!node.prev_node() || node.forward() == node.prev_node()->forward()) {
      // If change direction, the steer change have no limit.
      possible_motions.push_back(select_motion(next_steer, !node.forward()));
    }
  }

  std::vector<std::shared_ptr<Node3d>> res;
  for (int i = 0; i < possible_motions.size(); ++i) {
    const double next_x = node.x() +
                          possible_motions[i].delta_x * node.cos_theta() -
                          possible_motions[i].delta_y * node.sin_theta();
    const double next_y = node.y() +
                          possible_motions[i].delta_x * node.sin_theta() +
                          possible_motions[i].delta_y * node.cos_theta();
    const double next_theta =
        NormalizeAngle(node.theta() + possible_motions[i].delta_theta);
    res.push_back(std::make_shared<Node3d>(next_x, next_y, next_theta));
    res.back()->set_forward(possible_motions[i].forward);
    res.back()->set_steer(possible_motions[i].steer);
  }
  return res;
}

double GetParkingSpotLineCost(
    const HybridAStarParamsProto& hybrid_a_star_params,
    const VehicleGeometryParamsProto& veh_geo_params, const Node3d& node,
    const std::vector<SpecialBoundary>& boundaries) {
  const auto vehicle_box = node.GetVehicleBoundingBoxWithBuffer(
      veh_geo_params, /*lateral_buffer=*/0.0,
      /*longitudinal_buffer=*/0.0);
  double cost = 0.0;
  for (const auto& boundary : boundaries) {
    // Currently only consider spot line
    if (boundary.type != FreespaceMapProto::SOFT_PARKING_SPOT_LINE) {
      continue;
    }
    QCHECK_EQ(boundary.points.size(), 2);
    if (vehicle_box.HasOverlap(
            Segment2d(boundary.points[0], boundary.points[1]))) {
      cost += hybrid_a_star_params.spot_line_weight();
    }
  }
  return cost;
}

double GetDistanceToObstacleCost(
    const HybridAStarParamsProto& hybrid_a_star_params, const Node3d& node) {
  return hybrid_a_star_params.a_star_distance_to_obstacle_weight() *
         std::exp(-node.dis_to_obs() *
                  hybrid_a_star_params.a_star_distance_to_obstacle_scale());
}

double GetTrajectoryCost(const HybridAStarParamsProto& hybrid_a_star_params,
                         const VehicleGeometryParamsProto& veh_geo_params,
                         const std::vector<SpecialBoundary>& boundaries,
                         double max_kappa, const Node3d& prev_node,
                         const Node3d& cur_node) {
  constexpr double kEpsilon = 1e-6;
  double res = prev_node.traj_cost();
  if (cur_node.forward()) {
    res += hybrid_a_star_params.search_step() *
           hybrid_a_star_params.forward_gear_weight();
  } else {
    res += hybrid_a_star_params.search_step() *
           hybrid_a_star_params.backward_gear_weight();
  }
  // If prev_node is the start, do not consider gear switch cost.
  if (cur_node.forward() != prev_node.forward() && prev_node.s() > kEpsilon) {
    res += hybrid_a_star_params.gear_switch_weight();
  }
  res += std::abs(cur_node.steer() - prev_node.steer()) / max_kappa *
         hybrid_a_star_params.steer_change_weight();
  res += std::abs(cur_node.steer()) * hybrid_a_star_params.steer_weight();
  res += GetParkingSpotLineCost(hybrid_a_star_params, veh_geo_params, cur_node,
                                boundaries);
  res += GetDistanceToObstacleCost(hybrid_a_star_params, cur_node);
  return res;
}

double GetReedsSheppCost(const HybridAStarParamsProto& hybrid_a_star_params,
                         const FastReedSheppPath& rs_path, bool current_gear) {
  // Currently doesn't distinguish forward and backward weight in heuristics.
  double res =
      rs_path.total_length * hybrid_a_star_params.forward_gear_weight();
  res += static_cast<double>(rs_path.gear_change_num) *
         hybrid_a_star_params.gear_switch_weight();
  if (rs_path.init_gear != current_gear) {
    res += hybrid_a_star_params.gear_switch_weight();
  }
  return res;
}

std::vector<std::shared_ptr<Node3d>> GetReedsSheppExtension(
    const HybridAStarParamsProto& hybrid_a_star_params, double max_steer,
    const Node3d& start, const Node3d& end) {
  std::vector<std::shared_ptr<Node3d>> res;
  const auto rs_path = GetShortestReedsShepp(start, end, max_steer);
  if (!rs_path.ok()) {
    return res;
  }
  // Check if has short path or reverse driving path.
  std::vector<double> segs_lengths;
  for (const double length : rs_path->segs_lengths) {
    // Two segs may have the same direction.
    if (!segs_lengths.empty() && length * segs_lengths.back() > 0.0) {
      segs_lengths.back() += length;
    } else {
      segs_lengths.push_back(length);
    }
    // TODO(Zhuang): Add enable_reverse_driving to RS path generator.
    if (!hybrid_a_star_params.enable_reverse_driving() && length < 0.0) {
      return res;
    }
  }
  constexpr double kMinPathLength = 0.8;
  // The first segment is connected to ha* path end, so if they have the same
  // direction, we don't need to consider the length of this segment.
  const int start_index =
      (start.forward() == (segs_lengths.front() > 0.0) ? 1 : 0);
  for (int i = start_index; i < segs_lengths.size(); ++i) {
    if (std::abs(segs_lengths[i]) < kMinPathLength) {
      return res;
    }
  }
  // If the ha* path end node has changed direction, rs path shouldn't change
  // direction again.
  if (start.prev_node() != nullptr &&
      start.forward() != start.prev_node()->forward() &&
      start.forward() != (rs_path->segs_lengths.front() > 0.0)) {
    return res;
  }

  const auto compute_motion = [&](double length, char type) -> Motion {
    const bool forward = (length > 0.0);
    const double sign = (forward ? 1.0 : -1.0);
    switch (type) {
      case 'L': {
        const double delta_theta = max_steer * length;
        double delta_theta_cos_sin[2];
        fast_math::CosAndSin<7>(delta_theta * 0.5, delta_theta_cos_sin);
        const double sin_theta = delta_theta_cos_sin[1];
        const double cos_theta = delta_theta_cos_sin[0];
        const double delta_x = 2.0 * sin_theta * cos_theta / max_steer;
        const double delta_y = 2.0 * sin_theta * sin_theta / max_steer;
        return {delta_x,   delta_y,          delta_theta,
                max_steer, std::abs(length), forward};
      }
      case 'R': {
        const double delta_theta = -max_steer * length;
        double delta_theta_cos_sin[2];
        fast_math::CosAndSin<7>(delta_theta * 0.5, delta_theta_cos_sin);
        const double sin_theta = delta_theta_cos_sin[1];
        const double cos_theta = delta_theta_cos_sin[0];
        const double delta_x = 2.0 * sin_theta * cos_theta / max_steer;
        const double delta_y = 2.0 * sin_theta * sin_theta / max_steer;
        return {-delta_x,   -delta_y,         delta_theta,
                -max_steer, std::abs(length), forward};
      }
      case 'S':
        return {sign * std::abs(length), 0.0,    0.0, 0.0,
                std::abs(length),        forward};
    }
    QLOG(FATAL);  // Unexpected.
    return {0.0, 0.0, 0.0, 0.0, 0.0, false};
  };

  QCHECK_EQ(rs_path->segs_lengths.size(), rs_path->segs_types.size());
  // Record the last node.
  std::shared_ptr<Node3d> last_node = std::make_shared<Node3d>(start);
  double end_s_of_last_segment = 0.0;
  for (int i = 0; i < rs_path->segs_lengths.size(); ++i) {
    const double length = rs_path->segs_lengths[i];
    const char type = rs_path->segs_types[i];
    Motion motion = compute_motion(
        std::copysign(hybrid_a_star_params.search_step(), length), type);
    double s = 0.0;
    constexpr double kEpsilon = 0.01;
    while (s + kEpsilon < std::abs(length)) {
      double s_increment = hybrid_a_star_params.search_step();
      if (s + s_increment >= std::abs(length)) {
        s_increment = std::abs(length) - s;
        motion = compute_motion(std::copysign(s_increment, length), type);
      }
      s += s_increment;
      const double next_x = last_node->x() +
                            motion.delta_x * last_node->cos_theta() -
                            motion.delta_y * last_node->sin_theta();
      const double next_y = last_node->y() +
                            motion.delta_x * last_node->sin_theta() +
                            motion.delta_y * last_node->cos_theta();
      const double next_theta =
          NormalizeAngle(last_node->theta() + motion.delta_theta);
      res.push_back(std::make_shared<Node3d>(next_x, next_y, next_theta));
      res.back()->set_forward(motion.forward);
      res.back()->set_steer(motion.steer);
      res.back()->set_s(s + end_s_of_last_segment);
      last_node = res.back();
    }
    if (!res.empty()) {
      end_s_of_last_segment = res.back()->s();
    }
  }
  if (res.empty()) return res;
  // Check if the last point is close to end.
  constexpr double kMaxXYError = 0.05;
  constexpr double kMaxThetaError = 0.01;
  const double xy_error =
      Hypot(res.back()->x() - end.x(), res.back()->y() - end.y());
  const double theta_error =
      std::abs(NormalizeAngle(res.back()->theta() - end.theta()));
  if (xy_error > kMaxXYError || theta_error > kMaxThetaError) {
    res.clear();
    VLOG(1) << "RS extension has a big error, xy error = " << xy_error
            << ", theta error = " << theta_error;
  }
  return res;
}

void MaybeUpdatePath(const HybridAStarParamsProto& hybrid_a_star_params,
                     const VehicleGeometryParamsProto& veh_geo_params,
                     const std::vector<SpecialBoundary>& boundaries,
                     FreespaceTaskProto::TaskType task_type,
                     const PathPoint& goal,
                     const std::shared_ptr<Node3d>& final_node,
                     const std::vector<std::shared_ptr<Node3d>>& rs_extension,
                     int sampled_paths_num, bool exchange_start_end,
                     double max_kappa,
                     std::optional<HybridAStarPath>* current_path) {
  HybridAStarPath path_candidate;
  path_candidate.nodes.reserve(rs_extension.size());
  // Add hybrid a star path.
  auto cur_node = final_node;
  while (cur_node) {
    path_candidate.nodes.push_back(cur_node);
    cur_node = cur_node->prev_node();
  }
  std::reverse(path_candidate.nodes.begin(), path_candidate.nodes.end());
  path_candidate.ha_star_nodes_num = path_candidate.nodes.size();
  const double hybrid_a_star_end_s = path_candidate.nodes.back()->s();
  // Add RS extension.
  auto prev_node = path_candidate.nodes.back();
  for (const auto& node : rs_extension) {
    node->set_s(node->s() + hybrid_a_star_end_s);
    node->set_traj_cost(GetTrajectoryCost(hybrid_a_star_params, veh_geo_params,
                                          boundaries, max_kappa, *prev_node,
                                          *node));
    node->set_prev_node(prev_node);
    path_candidate.nodes.push_back(node);
    prev_node = node;
  }
  // Check path validity.
  if (path_candidate.nodes.size() < 2) {
    return;
  }
  if (task_type == FreespaceTaskProto::THREE_POINT_TURN) {
    // A safety protection for uturn, we don't allow too much reverse driving.
    constexpr double kMaxReversePathLength = 8.0;  // m.
    double reverse_path_length = 0.0;
    for (int i = 1; i < path_candidate.nodes.size(); ++i) {
      if (!path_candidate.nodes[i]->forward()) {
        reverse_path_length +=
            path_candidate.nodes[i]->s() - path_candidate.nodes[i - 1]->s();
      } else {
        reverse_path_length = 0.0;
      }
      if (reverse_path_length > kMaxReversePathLength) {
        return;
      }
    }
  }
  // Compute cost.
  path_candidate.cost = path_candidate.nodes.back()->traj_cost();
  // ParallelCost for reverse perpendicular parking. This cost is to make path
  // parallel to parking spot when entering spot.
  constexpr double kEpsilon = 1e-6;
  constexpr double kParallelCostWeight = 10.0;
  constexpr double kParallelCostDist = 4.0;
  if (task_type == FreespaceTaskProto::PERPENDICULAR_PARKING) {
    auto current_node = path_candidate.nodes.back();
    double current_s = current_node->s();
    while (current_node->prev_node()) {
      auto prev_node = current_node->prev_node();
      // Compute cost every search_step because rs node s_step maybe smaller
      // than search_step.
      if (current_s - prev_node->s() >
          hybrid_a_star_params.search_step() - kEpsilon) {
        current_s = prev_node->s();
        // TODO(Zhuang): Maybe weights are different for different theta error.
        path_candidate.cost +=
            std::abs(NormalizeAngle(current_node->theta() - goal.theta())) *
            kParallelCostWeight;
      }
      current_node = prev_node;
      if (path_candidate.nodes.back()->s() - current_node->s() >
          kParallelCostDist) {
        break;
      }
    }
  }
  if (FLAGS_send_sampling_rs_extension_to_canvas) {
    SendHybridAStarPathToCanvas(
        path_candidate, exchange_start_end,
        absl::StrFormat("freespace/hybrid_a_star_sample/path_%03d",
                        sampled_paths_num));
  }
  // Check if need update.
  if (!current_path->has_value()) {
    *current_path = path_candidate;
    return;
  }
  if (path_candidate.cost < current_path->value().cost) {
    *current_path = path_candidate;
    return;
  }
}

bool CheckNarrowPerpendicularParkingSpot(
    const VehicleGeometryParamsProto& veh_geo_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const Node3d& node) {
  constexpr double kMinDistThreshold = 0.8;  // meters
  const auto nearby_named_segments = segments_kd_tree.GetNamedSegmentsInRadius(
      node.x(), node.y(), veh_geo_params.length());
  const auto normal_unit = Vec2d(-node.sin_theta(), node.cos_theta());

  const Vec2d av_pose(node.x(), node.y());
  const Vec2d left_shift_pose = av_pose + normal_unit * kMinDistThreshold * 0.5;
  const Vec2d right_shift_pose =
      av_pose - normal_unit * kMinDistThreshold * 0.5;
  const auto left_shift_vehicle_box = ComputeAvBoxWithBuffer(
      left_shift_pose, node.theta(), veh_geo_params, /*length_buffer=*/0.0,
      /*width_buffer=*/kMinDistThreshold * 0.5);
  const auto right_shift_vehicle_box = ComputeAvBoxWithBuffer(
      right_shift_pose, node.theta(), veh_geo_params, /*length_buffer=*/0.0,
      /*width_buffer=*/kMinDistThreshold * 0.5);
  bool left_collided = false, right_collided = false;
  for (const auto& named_segment : nearby_named_segments) {
    if (left_collided && right_collided) {
      break;
    }
    const auto iter = objects_map.find(named_segment.second);
    if (iter != objects_map.end()) {
      left_collided |= iter->second->contour.HasOverlap(left_shift_vehicle_box);
      right_collided |=
          iter->second->contour.HasOverlap(right_shift_vehicle_box);
    } else {
      const auto boundary_iter = boundaries_map.find(named_segment.second);
      QCHECK(boundary_iter != boundaries_map.end());
      left_collided |= left_shift_vehicle_box.HasOverlap(*named_segment.first);
      right_collided |=
          right_shift_vehicle_box.HasOverlap(*named_segment.first);
    }
  }
  return left_collided && right_collided;
}

struct Comparator {
  bool operator()(const std::shared_ptr<Node3d>& a,
                  const std::shared_ptr<Node3d>& b) const {
    return a->total_cost() > b->total_cost();
  }
};

}  // namespace

// NOLINTNEXTLINE(readability-function-size)
absl::StatusOr<std::vector<DirectionalPath>> FindPathWithKDTree(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    double max_kappa, FreespaceTaskProto::TaskType task_type,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const AABox2d& search_region,
    const std::vector<SpecialBoundary>& special_boundaries,
    const PathPoint& start, const PathPoint& end,
    PathFinderDebugProto* debug_info) {
  SCOPED_QTRACE("FindPathWithKDTree");
  // Check input.
  auto start_node = std::make_shared<Node3d>(start.x(), start.y(),
                                             NormalizeAngle(start.theta()));
  auto end_node =
      std::make_shared<Node3d>(end.x(), end.y(), NormalizeAngle(end.theta()));

  if (!CheckNodeValidityWithKDTree(
          path_finder_params, veh_geo_params, vehicle_model_params,
          search_region, segments_kd_tree, objects_map, boundaries_map,
          special_boundaries, *start_node)) {
    debug_info->set_ha_status(PathFinderDebugProto::START_INVALID);
    return absl::UnavailableError("Start pose invalid!");
  }
  if (!CheckNodeValidityWithKDTree(
          path_finder_params, veh_geo_params, vehicle_model_params,
          search_region, segments_kd_tree, objects_map, boundaries_map,
          special_boundaries, *end_node)) {
    debug_info->set_ha_status(PathFinderDebugProto::GOAL_INVALID);
    return absl::UnavailableError("End pose invalid!");
  }
  // Problem set up.
  // Store each node's handle in open list
  // https://fossies.org/linux/boost/libs/heap/examples/interface.cpp
  boost::heap::fibonacci_heap<std::shared_ptr<Node3d>,
                              boost::heap::compare<Comparator>>
      open_pq;
  absl::flat_hash_map<unsigned int, decltype(open_pq)::handle_type> open_map;
  absl::flat_hash_set<unsigned int> close_set;

  std::shared_ptr<Node3d> final_node = nullptr;
  std::vector<std::shared_ptr<Node3d>> rs_extension;
  std::optional<HybridAStarPath> result_candidate = std::nullopt;

  const auto& hybrid_a_star_params = path_finder_params.hybrid_a_star_params();
  // Construct motions.
  // Currently only three steers, the delta_theta produced by these steers
  // should be larger than theta_resolution of hybrid a star, if not, different
  // nodes may occupy the same 3D grid.
  std::vector<double> possible_steers;
  possible_steers.reserve(
      path_finder_params.hybrid_a_star_params().possible_steer_ratios_size());
  for (const double steer_ratio :
       path_finder_params.hybrid_a_star_params().possible_steer_ratios()) {
    possible_steers.push_back(steer_ratio * max_kappa);
  }
  const auto possible_motions =
      GenerateMotions(possible_steers, hybrid_a_star_params.search_step());

  const bool exchange_start_end =
      FLAGS_enable_parking_reverse_plan &&
      (task_type == FreespaceTaskProto::PARALLEL_PARKING ||
       (task_type == FreespaceTaskProto::PERPENDICULAR_PARKING &&
        CheckNarrowPerpendicularParkingSpot(veh_geo_params, segments_kd_tree,
                                            objects_map, boundaries_map,
                                            *end_node)) ||
       task_type == FreespaceTaskProto::CUSTOM_PARKING);
  if (exchange_start_end) {
    std::swap(start_node, end_node);
  }
  // Construct A star cost map.
  const auto a_star_map_start_time = absl::Now();
  AStarHeuristics a_star_heuristics(
      search_region, hybrid_a_star_params.a_star_resolution(),
      hybrid_a_star_params.a_star_vehicle_radius(), end_node->x(),
      end_node->y());
  if (!a_star_heuristics.GenerateCostMap(segments_kd_tree, objects_map,
                                         boundaries_map)) {
    debug_info->set_ha_status(PathFinderDebugProto::HA_COST_MAP_FAIL);
    return absl::InternalError("Generate cost map fail!");
  }
  VLOG(2) << "Construct a star heuristics map time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - a_star_map_start_time);

  start_node->SetGridsAndId(search_region, hybrid_a_star_params.xy_resolution(),
                            hybrid_a_star_params.theta_resolution());
  end_node->SetGridsAndId(search_region, hybrid_a_star_params.xy_resolution(),
                          hybrid_a_star_params.theta_resolution());
  open_map.emplace(start_node->id(), open_pq.emplace(start_node));
  int iterations = 0;
  int sampled_paths_num = 0;
  int explored_nodes = 0;
  double node_check_time = 0.0;
  double distance_calculate_time = 0.0;
  double rs_extension_time = 0.0;
  // Hybrid A star search.
  while (!open_pq.empty() && iterations < hybrid_a_star_params.max_iters()) {
    // Get the node with lowest cost.
    const auto current_node = open_pq.top();
    open_pq.pop();
    // Try RS extension.
    const auto rs_extension_start = absl::Now();
    constexpr int kTryEveryNIters = 3;
    // Currently, parking task must use RS extension.
    if (iterations % kTryEveryNIters == 0) {
      const auto rs_extension_nodes = GetReedsSheppExtension(
          hybrid_a_star_params, max_kappa, *current_node, *end_node);
      if (CheckRSExtensionValidityWithKDTree(
              path_finder_params, veh_geo_params, vehicle_model_params,
              search_region, segments_kd_tree, objects_map, boundaries_map,
              special_boundaries, rs_extension_nodes)) {
        rs_extension = rs_extension_nodes;
        final_node = current_node;
        MaybeUpdatePath(hybrid_a_star_params, veh_geo_params,
                        special_boundaries, task_type, end, final_node,
                        rs_extension, sampled_paths_num, exchange_start_end,
                        max_kappa, &result_candidate);
        // If we use sampling, ha* should not finish under this condition.
        if (!hybrid_a_star_params.use_sampling_rs() ||
            sampled_paths_num >= hybrid_a_star_params.sampling_rs_max_iters()) {
          break;
        }
        ++sampled_paths_num;
      }
    }
    rs_extension_time +=
        absl::ToDoubleMilliseconds(absl::Now() - rs_extension_start);
    // Check if reach the destination.
    if (Sqr(current_node->x() - end_node->x()) +
                Sqr(current_node->y() - end_node->y()) <
            Sqr(hybrid_a_star_params.goal_xy_tolerance()) &&
        std::abs(NormalizeAngle(current_node->theta() - end_node->theta())) <
            Sqr(hybrid_a_star_params.goal_theta_tolerance())) {
      final_node = current_node;
      MaybeUpdatePath(hybrid_a_star_params, veh_geo_params, special_boundaries,
                      task_type, end, final_node, /*rs_extension=*/{},
                      sampled_paths_num, exchange_start_end, max_kappa,
                      &result_candidate);
      // If we use sampling, ha* should not finish under this condition.
      if (!hybrid_a_star_params.use_sampling_rs()) {
        break;
      }
      ++sampled_paths_num;
    }
    // Mark the node as explored.
    close_set.emplace(current_node->id());
    // Explore next nodes.
    const auto next_nodes =
        GenerateNextNodes(possible_motions, possible_steers,
                          hybrid_a_star_params.enable_reverse_driving(),
                          *current_node, max_kappa);
    for (const auto& next_node : next_nodes) {
      // Check if out of boundary or has collision.
      const auto node_check_start = absl::Now();
      if (!CheckNodeValidityWithKDTree(
              path_finder_params, veh_geo_params, vehicle_model_params,
              search_region, segments_kd_tree, objects_map, boundaries_map,
              special_boundaries, *next_node)) {
        continue;
      }
      node_check_time +=
          absl::ToDoubleMilliseconds(absl::Now() - node_check_start);
      // Set node ID.
      next_node->SetGridsAndId(search_region,
                               hybrid_a_star_params.xy_resolution(),
                               hybrid_a_star_params.theta_resolution());
      // Check if already in close set.
      if (close_set.find(next_node->id()) != close_set.end()) {
        continue;
      }
      const auto next_node_iter = open_map.find(next_node->id());
      const bool in_open_map = (next_node_iter != open_map.end());
      // Distance to obstacle
      const auto distance_calculate_start = absl::Now();
      if (in_open_map) {
        next_node->set_dis_to_obs((*(next_node_iter->second))->dis_to_obs());
      } else {
        next_node->set_dis_to_obs(GetPoseDistanceToObstacle(
            veh_geo_params, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map,
            Vec2d(next_node->x(), next_node->y()), next_node->theta(),
            Vec2d(next_node->cos_theta(), next_node->sin_theta())));
      }
      distance_calculate_time +=
          absl::ToDoubleMilliseconds(absl::Now() - distance_calculate_start);
      // Current traj cost.
      const double traj_cost = GetTrajectoryCost(
          hybrid_a_star_params, veh_geo_params, special_boundaries, max_kappa,
          *current_node, *next_node);
      // Check if need to push to queue.
      if (!in_open_map ||
          (in_open_map &&
           traj_cost < (*(next_node_iter->second))->traj_cost())) {
        explored_nodes++;
        // A star cost.
        const double a_star_cost =
            hybrid_a_star_params.forward_gear_weight() *
            a_star_heuristics.GetHeuristicsCost(next_node->x(), next_node->y());
        // RS cost.
        const auto rs_path =
            GetShortestReedsSheppFromTable(*next_node, *end_node, max_kappa);
        double rs_cost = GetReedsSheppCost(hybrid_a_star_params, rs_path,
                                           next_node->forward());
        // Heuristic cost.
        const double heuristic_cost =
            a_star_cost * hybrid_a_star_params.a_star_heuristic_weight() +
            rs_cost * hybrid_a_star_params.rs_heuristic_weight();
        next_node->set_traj_cost(traj_cost);
        next_node->set_heuristic_cost(heuristic_cost);
        next_node->set_total_cost(traj_cost + heuristic_cost);
        next_node->set_s(current_node->s() +
                         hybrid_a_star_params.search_step());
        next_node->set_prev_node(current_node);
        if (in_open_map) {
          // Update the node if it's open.
          open_pq.update(next_node_iter->second, next_node);
        } else {
          // Push to queue and mark it as open.
          open_map.emplace(next_node->id(), open_pq.emplace(next_node));
        }
      }
    }
    iterations++;
  }

  VLOG(2) << "Node validity check time(ms): " << node_check_time;
  VLOG(2) << "Distance calculation time(ms): " << distance_calculate_time;
  VLOG(2) << "RS extension time(ms): " << rs_extension_time;

  if (!result_candidate.has_value()) {
    QLOG(ERROR) << "Hybrid A star search fail, iterations = " << iterations
                << ", explored_nodes = " << explored_nodes;
    debug_info->set_ha_iters(iterations);
    debug_info->set_ha_status(PathFinderDebugProto::SEARCH_FAIL);
    return absl::UnavailableError(
        "Hybrid A star search fail after max iterations or explored all "
        "possible nodes.");
  }
  VLOG(2) << "Final node cost: " << result_candidate->cost;

  if (FLAGS_send_hybrid_a_star_output_path_to_canvas) {
    SendHybridAStarPathToCanvas(*result_candidate, exchange_start_end,
                                "freespace/hybrid_a_star_output_path");
    vis::Canvas& canvas_path = vis::vantage::GetCanvasClient()->GetCanvas(
        "freespace/hybrid_a_star_octagon");
    for (const auto& node : result_candidate->nodes) {
      const auto box = node->GetVehicleBoundingBoxWithBuffer(
          veh_geo_params, /*lateral_buffer=*/0.0,
          /*longitudinal_buffer=*/0.0);
      VehicleOctagonModel octagon(
          box, vehicle_model_params.front_corner_side_length(),
          vehicle_model_params.rear_corner_side_length());
      for (const auto& line : octagon.line_segments()) {
        canvas_path.DrawLine(Vec3d(line.start()), Vec3d(line.end()),
                             vis::Color::kWhite);
      }
    }
  }

  // Extract result.
  auto stitched_path = result_candidate->nodes;
  constexpr int kMaxPathSize = 64;
  std::vector<std::vector<PathPoint>> paths(1);
  std::deque<bool> gears;
  // Start node is always forward, however its real direction is determined by
  // next node.
  stitched_path[0]->set_forward(stitched_path[1]->forward());
  for (int i = 0; i < stitched_path.size(); ++i) {
    PathPoint pt;
    pt.set_x(stitched_path[i]->x());
    pt.set_y(stitched_path[i]->y());
    pt.set_z(0.0);
    pt.set_theta(stitched_path[i]->forward()
                     ? stitched_path[i]->theta()
                     : NormalizeAngle(stitched_path[i]->theta() + M_PI));
    pt.set_kappa(stitched_path[i]->forward() ? stitched_path[i]->steer()
                                             : -stitched_path[i]->steer());
    pt.set_s(stitched_path[i]->s());
    paths.back().push_back(pt);
    // Switch gear.
    if (i > 0 && i + 1 < stitched_path.size() &&
        stitched_path[i]->forward() != stitched_path[i + 1]->forward()) {
      if (paths.size() >= kMaxPathSize) {
        debug_info->set_ha_status(PathFinderDebugProto::PATH_ABNORMAL);
        return absl::InternalError("Path gear switches for too many times!");
      }
      gears.push_back(stitched_path[i]->forward());
      paths.push_back({});
    }
  }
  gears.push_back(stitched_path.back()->forward());
  // Insert one more point when gear switches.
  for (int i = 1; i < paths.size(); ++i) {
    PathPoint pt;
    pt.set_x(paths[i - 1].back().x());
    pt.set_y(paths[i - 1].back().y());
    pt.set_z(0.0);
    pt.set_theta(NormalizeAngle(paths[i - 1].back().theta() + M_PI));
    pt.set_kappa(paths[i].front().kappa());
    pt.set_s(paths[i - 1].back().s());
    paths[i].insert(paths[i].begin(), pt);
  }
  // Reverse path if needed.
  if (exchange_start_end) {
    const double end_s = paths.back().back().s();
    std::reverse(paths.begin(), paths.end());
    for (auto& path : paths) {
      std::reverse(path.begin(), path.end());
      for (auto& pt : path) {
        pt.set_theta(NormalizeAngle(pt.theta() + M_PI));
        pt.set_kappa(-pt.kappa());
        pt.set_s(end_s - pt.s());
      }
    }
    std::reverse(gears.begin(), gears.end());
    for (auto& gear : gears) {
      gear = !gear;
    }
  }
  // Make sure the path s starts from zero.
  for (int i = 0; i < paths.size(); ++i) {
    const double start_s = paths[i][0].s();
    for (int j = 0; j < paths[i].size(); ++j) {
      paths[i][j].set_s(paths[i][j].s() - start_s);
    }
  }
  // Reset init point kappa.
  if (task_type == FreespaceTaskProto::THREE_POINT_TURN) {
    paths.front().front().set_kappa(paths.front()[1].kappa());
  }

  std::vector<DirectionalPath> res;
  res.reserve(paths.size());
  for (int i = 0; i < paths.size(); ++i) {
    res.push_back({DiscretizedPath(std::move(paths[i])), gears[i]});
  }
  VLOG(2) << "Hybrid A star search success, iterations = " << iterations
          << ", explored_nodes = " << explored_nodes << ", find "
          << sampled_paths_num << " result candidates.";
  debug_info->set_ha_iters(iterations);
  debug_info->set_ha_status(PathFinderDebugProto::SUCCESS);
  return res;
}

absl::StatusOr<std::vector<DirectionalPath>> FindPath(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    FreespaceTaskProto::TaskType task_type, const FreespaceMap& freespace_map,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const PathPoint& start, const PathPoint& end,
    PathFinderDebugProto* debug_info) {
  SCOPED_QTRACE("FindPath");
  QCHECK_NOTNULL(debug_info);

  const auto start_time = absl::Now();
  const auto& hybrid_a_star_params = path_finder_params.hybrid_a_star_params();
  // Grid index should be less than 1000 or the hash of grid id may have
  // problem.
  QCHECK_LT(
      freespace_map.region.length() / hybrid_a_star_params.xy_resolution(),
      1000);
  QCHECK_LT(freespace_map.region.width() / hybrid_a_star_params.xy_resolution(),
            1000);
  QCHECK_LT(M_PI * 2.0 / hybrid_a_star_params.theta_resolution(), 1000);

  std::vector<std::pair<std::string, FreespaceObject>> stationary_objects;
  for (const auto& traj_ptr : stalled_object_trajs) {
    std::string id = std::string(traj_ptr->object_id());
    const auto& object_proto = traj_ptr->planner_object().object_proto();
    FreespaceObject obj = {
        .contour = traj_ptr->contour(),
        .height = object_proto.max_z() - object_proto.ground_z()};
    stationary_objects.emplace_back(std::move(id), std::move(obj));
  }

  // Construct k-D tree.
  const auto kd_tree_start_time = absl::Now();
  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  int boundary_index = 0;
  for (const auto& boundary : freespace_map.boundaries) {
    QCHECK_GE(boundary.points.size(), 2);
    for (int i = 0; i + 1 < boundary.points.size(); ++i) {
      std::string id = "b" + std::to_string(boundary_index);
      const Segment2d seg(boundary.points[i], boundary.points[i + 1]);
      named_segments.push_back(std::make_pair(id, seg));
      boundaries_map.emplace(id, &boundary);
      boundary_index++;
    }
  }
  for (const auto& named_obj : stationary_objects) {
    named_segments.push_back(std::make_pair(
        named_obj.first, Segment2d(Vec2d(named_obj.second.contour.min_x(),
                                         named_obj.second.contour.min_y()),
                                   Vec2d(named_obj.second.contour.max_x(),
                                         named_obj.second.contour.max_y()))));
    objects_map.emplace(named_obj.first, &named_obj.second);
  }
  SegmentMatcherKdtree segments_kd_tree(named_segments);
  VLOG(2) << "Construct K-D tree time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - kd_tree_start_time);

  const double max_kappa =
      hybrid_a_star_params.kappa_slack_ratio() *
      ComputeCenterMaxCurvature(veh_geo_params, vehicle_drive_params);

  // Hybrid a start main func.
  auto res = FindPathWithKDTree(
      path_finder_params, veh_geo_params, vehicle_model_params, max_kappa,
      task_type, segments_kd_tree, objects_map, boundaries_map,
      freespace_map.region, freespace_map.special_boundaries, start, end,
      debug_info);

  VLOG(1) << "Total time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - start_time);

  return res;
}

}  // namespace planner
}  // namespace qcraft
