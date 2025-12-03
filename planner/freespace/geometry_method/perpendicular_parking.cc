#include "onboard/planner/freespace/geometry_method/perpendicular_parking.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"

#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/freespace/geometry_method/geometry_connection.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_util.h"

namespace qcraft {
namespace planner {

namespace {

std::vector<GeometryMethodPoint> ConstructTransitionPoses(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, const GeometryMethodPoint& goal) {
  // Convert goal to origin.
  const GeometryMethodPoint normalized_start = {
      .pos = (start.pos - goal.pos).Rotate(goal.tangent.x(), -goal.tangent.y()),
      .theta = NormalizeAngle(start.theta - goal.theta),
      .tangent = Vec2d::FastUnitFromAngle(start.theta - goal.theta)};
  const bool is_on_left = start.tangent.CrossProd(goal.pos - start.pos) > 0.0;

  const auto& params = path_finder_params.geometry_method_params();
  std::vector<double> x_candidates;
  x_candidates.reserve(params.transition_pose_x_candidates_size());
  for (const double x : params.transition_pose_x_candidates()) {
    x_candidates.push_back(x + veh_geo_params.front_edge_to_center());
  }
  std::vector<double> y_candidates;
  y_candidates.reserve(params.transition_pose_y_candidates_size());
  for (const double y : params.transition_pose_y_candidates()) {
    y_candidates.push_back(is_on_left ? -y : y);
  }
  std::vector<double> theta_candidates;
  theta_candidates.reserve(params.transition_pose_theta_rate_candidates_size());
  for (const double rate : params.transition_pose_theta_rate_candidates()) {
    theta_candidates.push_back(normalized_start.theta * rate);
  }

  std::vector<GeometryMethodPoint> transition_poses;
  for (const double x : x_candidates) {
    if (x > normalized_start.pos.x()) continue;
    for (const double y : y_candidates) {
      for (const double theta : theta_candidates) {
        // Convert back to previous coordinate.
        GeometryMethodPoint gear_change_pose = {
            .pos = Vec2d(x, y).Rotate(goal.tangent.x(), goal.tangent.y()) +
                   goal.pos,
            .theta = NormalizeAngle(theta + goal.theta),
            .tangent = Vec2d::FastUnitFromAngle(theta + goal.theta)};
        if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
                veh_geo_params, path_finder_params, vehicle_model_params,
                segments_kd_tree, objects_map, boundaries_map,
                virtual_boundaries, gear_change_pose)) {
          continue;
        }
        transition_poses.push_back(std::move(gear_change_pose));
      }
    }
  }
  return transition_poses;
}

bool FindPathWithAdjustments(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, const GeometryMethodPoint& goal,
    const std::vector<GeometryMethodPoint>& transition_poses,
    GeometryPathType forward_type, LineCirclePath* result) {
  const auto max_adjust_distance =
      [&veh_geo_params, &path_finder_params, &vehicle_model_params,
       &segments_kd_tree, &objects_map, &boundaries_map, &virtual_boundaries](
          const GeometryMethodPoint& start, GeometryPathType type,
          bool is_forward, double kappa) -> std::optional<double> {
    constexpr double kMaxForwardAdjustDistance = 3.5;  // m.
    constexpr double kResultSearchStep = 0.05;         // m.
    constexpr double kCoarseResultSearchStep = 0.3;    // m.
    std::optional<double> coarse_res = std::nullopt;
    for (double s =
             path_finder_params.geometry_method_params().min_drive_distance();
         s <= kMaxForwardAdjustDistance; s += kCoarseResultSearchStep) {
      const auto cur_pose =
          ExtendPathByConstantKappa(start, kappa, is_forward ? s : -s, type);
      if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
              veh_geo_params, path_finder_params, vehicle_model_params,
              segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
              cur_pose)) {
        break;
      }
      coarse_res = s;
    }
    if (!coarse_res.has_value()) return std::nullopt;
    double res = *coarse_res;
    for (double s = *coarse_res; s <= kCoarseResultSearchStep + *coarse_res;
         s += kResultSearchStep) {
      const auto cur_pose =
          ExtendPathByConstantKappa(start, kappa, is_forward ? s : -s, type);
      if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
              veh_geo_params, path_finder_params, vehicle_model_params,
              segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
              cur_pose)) {
        break;
      }
      res = s;
    }
    return res;
  };

  LineCirclePath result_candidate;
  std::optional<int> result_adjustment_times = std::nullopt;

  LineCirclePath adjust_path;
  for (const auto& gear_change_pose : transition_poses) {
    if (!CircleLineConection(start, gear_change_pose, max_kappa,
                             &adjust_path)) {
      continue;
    }
    if (!CheckPathValidityWithKDTreeAndVirtualBoundaries(
            veh_geo_params, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            adjust_path)) {
      continue;
    }
    // Drive to transition pose is the first ajustment.
    int iterations = 1;
    auto cur_start = gear_change_pose;
    while (true) {
      // If adjust for too many times or we already have better path, exit this
      // while loop.
      if (iterations > path_finder_params.geometry_method_params()
                           .perpendicular_parking_max_adjustments() ||
          (result_adjustment_times.has_value() &&
           iterations >= *result_adjustment_times)) {
        break;
      }
      // Get the max distance that AV can drive forward.
      const auto max_forward_distance = max_adjust_distance(
          cur_start, forward_type, /*is_forward=*/true, max_kappa);
      if (!max_forward_distance.has_value()) {
        // This means AV can't move any more, plan fail.
        break;
      }
      // Try drive forward and park in.
      constexpr double kForwardDriveStep = 0.3;  // m.
      for (double s = kForwardDriveStep; s < *max_forward_distance;
           s += kForwardDriveStep) {
        LineCirclePath path1 = {.start = cur_start,
                                .types = {forward_type},
                                .lengths = {s},
                                .kappas = {max_kappa},
                                .ends = {ExtendPathByConstantKappa(
                                    cur_start, max_kappa, s, forward_type)}};
        LineCirclePath path2;
        if (CircleLineConection(path1.ends.back(), goal, max_kappa, &path2) &&
            CheckPathValidityWithKDTreeAndVirtualBoundaries(
                veh_geo_params, path_finder_params, vehicle_model_params,
                segments_kd_tree, objects_map, boundaries_map,
                virtual_boundaries, path2)) {
          ConnectPaths({adjust_path, path1, path2}, result);
          result_candidate = *result;
          result_adjustment_times = iterations;
          break;
        }
      }
      if (result_adjustment_times.has_value() &&
          iterations >= *result_adjustment_times) {
        break;
      }
      // Drive forward util collision.
      LineCirclePath forward_path = {
          .start = cur_start,
          .types = {forward_type},
          .lengths = {*max_forward_distance},
          .kappas = {max_kappa},
          .ends = {ExtendPathByConstantKappa(
              cur_start, max_kappa, *max_forward_distance, forward_type)}};
      ConnectPaths({adjust_path, forward_path}, result);
      adjust_path = *result;
      cur_start = adjust_path.ends.back();
      // Drive backward util collision.
      const auto max_backward_dist =
          max_adjust_distance(cur_start, /*type=*/GeometryPathType::STRAIGHT,
                              /*is_forward=*/false, /*kappa=*/0.0);
      LineCirclePath backward_path = {.start = cur_start,
                                      .types = {GeometryPathType::STRAIGHT},
                                      .lengths = {-*max_backward_dist},
                                      .kappas = {0.0},
                                      .ends = {ExtendPathByConstantKappa(
                                          cur_start, 0.0, -*max_backward_dist,
                                          GeometryPathType::STRAIGHT)}};
      ConnectPaths({adjust_path, backward_path}, result);
      adjust_path = *result;
      cur_start = adjust_path.ends.back();

      ++iterations;
    }  // While loop.
  }    // For loop.
  if (result_adjustment_times.has_value()) {
    *result = result_candidate;
    return true;
  }
  return false;
}

bool FindPerpendicularParkingPathForSpotBehindStart(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const std::vector<GeometryMethodPoint>& transition_poses,
    const GeometryMethodPoint& start, const GeometryMethodPoint& goal,
    LineCirclePath* result) {
  const Vec2d normalized_start =
      (start.pos - goal.pos).Rotate(goal.tangent.x(), -goal.tangent.y());
  const bool is_on_left = start.tangent.CrossProd(goal.pos - start.pos) > 0.0;
  const GeometryPathType forward_type =
      is_on_left ? GeometryPathType::RIGHT : GeometryPathType::LEFT;
  // Check if spot is really behind start.
  if (is_on_left ^ (normalized_start.y() > 0.0)) return false;

  // Strategy 1: Directly park in.
  if (CircleLineConection(start, goal, max_kappa, result) &&
      CheckPathValidityWithKDTreeAndVirtualBoundaries(
          veh_geo_params, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          *result)) {
    return true;
  }
  if (LineCircleLineConection(start, goal, max_kappa, result) &&
      CheckPathValidityWithKDTreeAndVirtualBoundaries(
          veh_geo_params, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          *result)) {
    return true;
  }

  // Strategy 2 to strategy_level: Parking in with adjustments. For one more
  // adjustment, add one strategy_level.
  if (FindPathWithAdjustments(
          veh_geo_params, max_kappa, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          start, goal, transition_poses, forward_type, result)) {
    return true;
  }
  return false;
}

}  // namespace

absl::StatusOr<LineCirclePath> FindPerpendicularParkingPath(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, const GeometryMethodPoint& goal,
    bool use_fast_method) {
  if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
          veh_geo_params, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          start)) {
    return absl::InternalError("Start Invalid.");
  }
  if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
          veh_geo_params, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          goal)) {
    return absl::InternalError("Goal Invalid.");
  }
  // Construct transition pose, AV will
  // drive to transition pose firstly, so how to generate them is very
  // important.
  const auto transition_poses = ConstructTransitionPoses(
      veh_geo_params, path_finder_params, vehicle_model_params,
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      goal);
  // Assume parking spot is behind start.
  LineCirclePath result;
  if (FindPerpendicularParkingPathForSpotBehindStart(
          veh_geo_params, max_kappa, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          transition_poses, start, goal, &result)) {
    return result;
  }

  // Try driving forward and then use
  // FindPerpendicularParkingPathForSpotBehindStart().
  const bool is_on_left = start.tangent.CrossProd(goal.pos - start.pos) > 0.0;
  const GeometryPathType forward_type =
      is_on_left ? GeometryPathType::RIGHT : GeometryPathType::LEFT;
  const double signed_dist = (start.pos - goal.pos).CrossProd(goal.tangent);
  const double longitudinal_distance_to_goal =
      is_on_left ? signed_dist : -signed_dist;
  // If parking spot is far behind AV, we don't need to try the following
  // strategy.
  constexpr double kMinDistance = 4.0;  // m.
  if (longitudinal_distance_to_goal < -kMinDistance) {
    return absl::InternalError("Reverse Parking plan fail.");
  }

  // Forward path is Line-Circle, when AV's longitudinal distance to goal is
  // within kMinCirclePathStartDist, AV can execute circle path.
  const auto& params = path_finder_params.geometry_method_params();
  constexpr double kMinCirclePathStartDist = 4.0;  // m.
  std::vector<double> l_candidates;
  for (double l = 0.0; l <= params.forward_line_path_max_distance();
       l += params.forward_line_path_distance_step()) {
    if (l - longitudinal_distance_to_goal < kMinCirclePathStartDist &&
        l - longitudinal_distance_to_goal > -kMinCirclePathStartDist) {
      l_candidates.push_back(l);
    }
  }

  std::vector<double> theta_change_candidates;
  theta_change_candidates.reserve(
      params.forward_circle_path_theta_candidates_pi_size());
  for (const double theta : params.forward_circle_path_theta_candidates_pi()) {
    const double transition_theta =
        is_on_left ? goal.theta + theta * M_PI : goal.theta - theta * M_PI;
    theta_change_candidates.push_back(
        std::abs(NormalizeAngle(transition_theta - start.theta)));
  }

  std::vector<double> kappa_candidates;
  kappa_candidates.reserve(
      params.forward_circle_path_kappa_rate_candidates_size());
  for (const double rate : params.forward_circle_path_kappa_rate_candidates()) {
    kappa_candidates.push_back(rate * max_kappa);
  }

  std::vector<LineCirclePath> forward_paths;

  for (const double l : l_candidates) {
    for (const double theta : theta_change_candidates) {
      for (const double kappa : kappa_candidates) {
        const double r = theta / kappa;
        if (l == 0.0 && r == 0.0) continue;
        const auto end1 = ExtendPathByConstantKappa(start, 0.0, l,
                                                    GeometryPathType::STRAIGHT);
        const auto end2 =
            ExtendPathByConstantKappa(end1, kappa, r, forward_type);
        LineCirclePath path;
        if (l == 0.0) {
          path = {.start = start,
                  .types = {forward_type},
                  .lengths = {r},
                  .kappas = {kappa},
                  .ends = {end2}};
        } else if (r == 0.0) {
          path = {.start = start,
                  .types = {GeometryPathType::STRAIGHT},
                  .lengths = {l},
                  .kappas = {0.0},
                  .ends = {end2}};
        } else {
          path = {.start = start,
                  .types = {GeometryPathType::STRAIGHT, forward_type},
                  .lengths = {l, r},
                  .kappas = {0.0, kappa},
                  .ends = {end1, end2}};
        }
        // The end of the path should be before parking spot.
        const Vec2d normalized_end =
            (end2.pos - goal.pos).Rotate(goal.tangent.x(), -goal.tangent.y());
        if (is_on_left ^ (normalized_end.y() > 0.0)) {
          continue;
        }
        if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
                veh_geo_params, path_finder_params, vehicle_model_params,
                segments_kd_tree, objects_map, boundaries_map,
                virtual_boundaries, path.ends.back())) {
          continue;
        }
        if (!CheckPathValidityWithKDTreeAndVirtualBoundaries(
                veh_geo_params, path_finder_params, vehicle_model_params,
                segments_kd_tree, objects_map, boundaries_map,
                virtual_boundaries, path)) {
          continue;
        }
        forward_paths.push_back(std::move(path));
      }
    }
  }

  // Get the optimal path.
  const auto get_path_segments_num = [](const LineCirclePath& path) {
    if (path.lengths.empty()) {
      return std::numeric_limits<int>::infinity();
    }
    int res = 0;
    bool cur_dir = path.lengths.front() > 0.0;
    for (int i = 1; i < path.lengths.size(); ++i) {
      const bool next_dir = path.lengths[i] > 0.0;
      if (cur_dir != next_dir) {
        ++res;
      }
      cur_dir = next_dir;
    }
    return res;
  };

  std::optional<LineCirclePath> path_candidate;
  for (const auto& path : forward_paths) {
    LineCirclePath park_in_path;
    if (FindPerpendicularParkingPathForSpotBehindStart(
            veh_geo_params, max_kappa, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            transition_poses, path.ends.back(), goal, &park_in_path)) {
      ConnectPaths({path, park_in_path}, &result);
      if (use_fast_method) {
        return result;
      }
      if (!path_candidate.has_value() ||
          get_path_segments_num(result) <
              get_path_segments_num(*path_candidate)) {
        path_candidate = result;
      }
    }
  }
  if (path_candidate.has_value()) {
    return *path_candidate;
  } else {
    return absl::InternalError("All strategies fail.");
  }
}

}  // namespace planner
}  // namespace qcraft
