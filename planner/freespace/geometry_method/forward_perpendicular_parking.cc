#include "onboard/planner/freespace/geometry_method/forward_perpendicular_parking.h"

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"

#include "onboard/math/vec.h"
#include "onboard/planner/freespace/geometry_method/geometry_connection.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_util.h"

namespace qcraft {
namespace planner {
namespace {

std::vector<LineCirclePath> ConstructDriveOutPaths(
    bool spot_on_left, double max_kappa,
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const GeometryMethodPoint& goal) {
  const GeometryPathType drive_out_dir =
      spot_on_left ? GeometryPathType::LEFT : GeometryPathType::RIGHT;
  const double dist_step = path_finder_params.geometry_method_params()
                               .forward_perpendicular_drive_out_dist_step();

  std::vector<LineCirclePath> result;
  for (double l = 0.0; l < veh_geo_params.length(); l += dist_step) {
    const auto end1 =
        ExtendPathByConstantKappa(goal, 0.0, -l, GeometryPathType::STRAIGHT);
    for (double r = std::max(0.0, veh_geo_params.length() - l);
         r < veh_geo_params.length(); r += dist_step) {
      if (l == 0.0 && r == 0.0) continue;
      if (l + r > veh_geo_params.length() +
                      path_finder_params.geometry_method_params()
                          .forward_perpendicular_drive_out_max_dist_buffer()) {
        break;
      }
      const auto end2 =
          ExtendPathByConstantKappa(end1, max_kappa, -r, drive_out_dir);
      LineCirclePath path;
      if (l == 0.0) {
        path = {.start = goal,
                .types = {drive_out_dir},
                .lengths = {-r},
                .kappas = {max_kappa},
                .ends = {end2}};
      } else if (r == 0.0) {
        path = {.start = goal,
                .types = {GeometryPathType::STRAIGHT},
                .lengths = {-l},
                .kappas = {0.0},
                .ends = {end2}};
      } else {
        path = {.start = goal,
                .types = {GeometryPathType::STRAIGHT, drive_out_dir},
                .lengths = {-l, -r},
                .kappas = {0.0, max_kappa},
                .ends = {end1, end2}};
      }
      result.push_back(std::move(path));
    }
  }
  return result;
}

void MaybeUpdatePath(const LineCirclePath& current_path,
                     std::optional<LineCirclePath>* path_candidate,
                     int* path_candidate_gear_change) {
  if (!path_candidate->has_value()) {
    *path_candidate = current_path;
    *path_candidate_gear_change = ComputeGearChangeTimes(current_path);
    return;
  }
  const int cur_gear_change = ComputeGearChangeTimes(current_path);
  if (cur_gear_change < *path_candidate_gear_change) {
    *path_candidate = current_path;
    *path_candidate_gear_change = cur_gear_change;
    return;
  } else if (cur_gear_change == *path_candidate_gear_change) {
    // TODO(zhuang): Select the path far from objects.
    return;
  }
}

bool ConnectWithPoint(
    const GeometryMethodPoint& start, const GeometryMethodPoint& goal,
    double max_kappa, const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries, LineCirclePath* result) {
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
  return false;
}

}  // namespace

absl::StatusOr<LineCirclePath> FindForwardPerpendicularParkingPath(
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

  LineCirclePath result;
  std::optional<LineCirclePath> path_candidate = std::nullopt;
  int path_candidate_gear_change = 0;
  // For forward perpendicular parking, we exchange start and goal, so our
  // strategy is to find how to drive out of the parking spot and go to the
  // start pose.

  // To drive out, we firstly drive backward on straight path and then on curve
  // path util AV is out of parking spot.
  const bool is_on_left = start.tangent.CrossProd(goal.pos - start.pos) > 0.0;
  const GeometryPathType forward_dir =
      is_on_left ? GeometryPathType::RIGHT : GeometryPathType::LEFT;
  const auto drive_out_paths = ConstructDriveOutPaths(
      is_on_left, max_kappa, veh_geo_params, path_finder_params, goal);
  for (const auto& path1 : drive_out_paths) {
    if (!CheckPathValidityWithKDTreeAndVirtualBoundaries(
            veh_geo_params, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            path1)) {
      continue;
    }
    // Try to connect with start.
    LineCirclePath path2;
    if (ConnectWithPoint(path1.ends.back(), start, max_kappa, veh_geo_params,
                         path_finder_params, vehicle_model_params,
                         segments_kd_tree, objects_map, boundaries_map,
                         virtual_boundaries, &path2)) {
      ConnectPaths({path1, path2}, &result);
      ReversePath(&result);
      if (use_fast_method) {
        return result;
      } else {
        MaybeUpdatePath(result, &path_candidate, &path_candidate_gear_change);
      }
      continue;
    }

    // If fail, try to drive forward in curve for a short distance, and then
    // try to connect with start.
    for (double forward_s =
             path_finder_params.geometry_method_params().min_drive_distance();
         forward_s < path_finder_params.geometry_method_params()
                         .forward_perpendicular_adjust_max_dist();
         forward_s += path_finder_params.geometry_method_params()
                          .forward_perpendicular_adjust_dist_step()) {
      LineCirclePath forward_path = {
          .start = path1.ends.back(),
          .types = {forward_dir},
          .lengths = {forward_s},
          .kappas = {max_kappa},
          .ends = {ExtendPathByConstantKappa(path1.ends.back(), max_kappa,
                                             forward_s, forward_dir)}};
      if (!CheckPathValidityWithKDTreeAndVirtualBoundaries(
              veh_geo_params, path_finder_params, vehicle_model_params,
              segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
              forward_path)) {
        break;
      }
      LineCirclePath path3;
      if (ConnectWithPoint(forward_path.ends.back(), start, max_kappa,
                           veh_geo_params, path_finder_params,
                           vehicle_model_params, segments_kd_tree, objects_map,
                           boundaries_map, virtual_boundaries, &path3)) {
        ConnectPaths({path1, forward_path, path3}, &result);
        ReversePath(&result);
        if (use_fast_method) {
          return result;
        } else {
          MaybeUpdatePath(result, &path_candidate, &path_candidate_gear_change);
        }
      }
    }
    // If fails, we can drive forward for a short distance and then try the
    // previous strategies, but we ignore such solutions because we don't want
    // path to be too complex.
  }

  if (path_candidate.has_value()) {
    return *path_candidate;
  }
  return absl::InternalError("All strategies fail.");
}

}  // namespace planner
}  // namespace qcraft
