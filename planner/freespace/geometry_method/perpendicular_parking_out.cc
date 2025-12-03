#include "onboard/planner/freespace/geometry_method/perpendicular_parking_out.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kEps = 1e-3;

absl::StatusOr<LineCirclePath> GenerateStraightParkingOutPath(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, double park_out_distance,
    ParkingOutDirection parking_out_direction) {
  const double s = (parking_out_direction == PARKING_OUT_DIR_PERP_FWD)
                       ? park_out_distance
                       : -park_out_distance;
  const GeometryMethodPoint end = ExtendPathByConstantKappa(
      start, /*kappa=*/0.0, s, GeometryPathType::STRAIGHT);
  const LineCirclePath path = {.start = start,
                               .types = {GeometryPathType::STRAIGHT},
                               .lengths = {s},
                               .kappas = {0.0},
                               .ends = {end}};
  if (CheckPathValidityWithKDTreeAndVirtualBoundaries(
          veh_geo_params, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          path)) {
    return path;
  }
  return absl::InternalError("Straight parking out path generation failed.");
}

absl::StatusOr<LineCirclePath> ZigZagOnceToTargetLine(
    double max_kappa, const GeometryMethodPoint& start, double target_heading,
    double min_drive_distance, const Vec2d& ref_line_point, bool left) {
  constexpr double kMaxHeadingDiff = M_PI_2 + kEps;  // rad.
  const double local_theta = NormalizeAngle(start.theta - target_heading);
  if (std::abs(local_theta) > kMaxHeadingDiff) {
    return absl::InternalError("Heading diff too large.");
  }

  // By default, we drive reversely first.
  const double radius = 1.0 / max_kappa;
  // Convert refline point & target heading to origin.
  const Vec2d local_pos = (start.pos - ref_line_point).Rotate(-target_heading);
  const Vec2d local_reverse_center =
      local_pos + radius * Vec2d::FastUnitFromAngle(local_theta +
                                                    (left ? -M_PI_2 : M_PI_2));
  const double local_forward_center_y = left ? radius : -radius;
  const double local_forward_center_delta_x_sqr =
      4.0 * Sqr(radius) -
      Sqr(local_reverse_center.y() - local_forward_center_y);
  if (local_forward_center_delta_x_sqr < 0.0) {
    return absl::InternalError("No solution.");
  }
  const double local_forward_center_x =
      local_reverse_center.x() + std::sqrt(local_forward_center_delta_x_sqr);
  const double local_theta1 = NormalizeAngle(
      (left ? -M_PI_2 : M_PI_2) +
      fast_math::Atan2(local_forward_center_y - local_reverse_center.y(),
                       local_forward_center_x - local_reverse_center.x()));
  if (std::abs(local_theta1) > std::abs(local_theta)) {
    return absl::InternalError("Turning angle too large.");
  }
  const double dtheta1 = NormalizeAngle(local_theta1 - local_theta);
  const double dtheta2 = -local_theta1;
  const double s1 = -std::abs(dtheta1 * radius);
  const double s2 = std::abs(dtheta2 * radius);
  if (-s1 < min_drive_distance || s2 < min_drive_distance) {
    return absl::InternalError("Drive distance too small.");
  }
  const auto type1 = left ? GeometryPathType::RIGHT : GeometryPathType::LEFT;
  const auto type2 = left ? GeometryPathType::LEFT : GeometryPathType::RIGHT;
  const auto end1 = ExtendPathByConstantKappa(start, max_kappa, s1, type1);
  const auto end2 = ExtendPathByConstantKappa(end1, max_kappa, s2, type2);
  QCHECK_NEAR(NormalizeAngle(end2.theta - target_heading), 0.0, kEps);
  const LineCirclePath res = {.start = start,
                              .types = {type1, type2},
                              .lengths = {s1, s2},
                              .kappas = {max_kappa, max_kappa},
                              .ends = {end1, end2}};
  return res;
}

absl::StatusOr<LineCirclePath> ZigZagToTargetHeadingWithReferenceLine(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, double target_heading,
    const Vec2d& ref_line_point, bool forward, bool left) {
  constexpr double kMaxTurningAngle = M_PI;                           // rad.
  const double max_expected_distance = kMaxTurningAngle * max_kappa;  // m.
  int zigzag_num = 0;
  double kappa = left ^ !forward ? max_kappa : -max_kappa;
  bool current_forward = forward;
  const double radius = 1.0 / max_kappa;
  GeometryMethodPoint cur_pos = start;
  LineCirclePath res;
  res.start = start;
  const double min_drive_distance =
      path_finder_params.geometry_method_params().min_drive_distance();
  while (zigzag_num <= path_finder_params.geometry_method_params()
                           .perpendicular_parking_out_max_adjustments()) {
    if (!current_forward) {
      // Drive to reference line.
      const auto path_to_line =
          ZigZagOnceToTargetLine(max_kappa, cur_pos, target_heading,
                                 min_drive_distance, ref_line_point, left);
      if (path_to_line.ok() &&
          CheckPathValidityWithKDTreeAndVirtualBoundaries(
              veh_geo_params, path_finder_params, vehicle_model_params,
              segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
              *path_to_line)) {
        ConnectPaths({res, *path_to_line}, &res);
        return res;
      }
    }

    // Drive until reaching max distance without collision or target heading.
    const auto first_overlap_dist =
        GetFirstOverlapDistanceWithKDTreeAndVirtualBoundaries(
            veh_geo_params, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            cur_pos, kappa, current_forward, max_expected_distance);
    const double max_dtheta = NormalizeAngle(target_heading - cur_pos.theta);
    const double max_s = std::abs(max_dtheta * radius);
    const bool reach_target_heading =
        !first_overlap_dist.has_value() || *first_overlap_dist >= max_s;
    if (!reach_target_heading && *first_overlap_dist < min_drive_distance) {
      return absl::InternalError("Ego vehicle stuck.");
    }
    const double abs_s =
        reach_target_heading ? max_s : *first_overlap_dist - kEps;
    const double s = current_forward ? abs_s : -abs_s;
    const auto type =
        kappa > 0.0 ? GeometryPathType::LEFT : GeometryPathType::RIGHT;
    cur_pos = ExtendPathByConstantKappa(cur_pos, max_kappa, s, type);
    res.types.push_back(type);
    res.kappas.push_back(max_kappa);
    res.lengths.push_back(s);
    res.ends.push_back(cur_pos);
    if (reach_target_heading) return res;
    current_forward = !current_forward;
    kappa = -kappa;
    zigzag_num++;
  }
  return absl::InternalError("Too many gear changes.");
}

absl::StatusOr<LineCirclePath> ZigZagToTargetBox(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, const Box2d& target_box, bool forward,
    bool left) {
  ASSIGN_OR_RETURN(
      auto zigzag_path,
      ZigZagToTargetHeadingWithReferenceLine(
          veh_geo_params, max_kappa, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          start, target_box.heading(), target_box.center(), forward, left));

  // Drive into target box.
  const auto& end = zigzag_path.ends.back();
  const Vec2d pos_diff = target_box.center() - end.pos;
  const double abs_lat_diff = std::abs(pos_diff.CrossProd(end.tangent));
  if (abs_lat_diff > target_box.half_width()) {
    return absl::InternalError("Failed to reach target box.");
  }

  const double lon_diff = pos_diff.Dot(end.tangent);
  if (lon_diff > target_box.half_length()) {
    const double s = lon_diff - target_box.half_length();
    auto final_end = ExtendPathByConstantKappa(end, /*kappa=*/0.0, s,
                                               GeometryPathType::STRAIGHT);
    const LineCirclePath straight_path = {.start = end,
                                          .types = {GeometryPathType::STRAIGHT},
                                          .lengths = {s},
                                          .kappas = {0.0},
                                          .ends = {std::move(final_end)}};
    if (!CheckPathValidityWithKDTreeAndVirtualBoundaries(
            veh_geo_params, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            straight_path)) {
      return absl::InternalError(
          "Failed to reach target box without collision.");
    }
    ConnectPaths({zigzag_path, straight_path}, &zigzag_path);
  }
  return zigzag_path;
}

}  // namespace

absl::StatusOr<LineCirclePath> FindPerpendicularStraightParkingOutPath(
    const VehicleGeometryParamsProto& veh_geo_params, double /*max_kappa*/,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, double park_out_distance,
    ParkingOutDirection parking_out_direction) {
  QCHECK(parking_out_direction == PARKING_OUT_DIR_PERP_FWD ||
         parking_out_direction == PARKING_OUT_DIR_PERP_BACK);
  QCHECK_GT(park_out_distance, 0.0);
  if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
          veh_geo_params, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          start)) {
    return absl::InternalError("Start Invalid.");
  }

  // Straight parking out.
  return GenerateStraightParkingOutPath(
      veh_geo_params, path_finder_params, vehicle_model_params,
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      park_out_distance, parking_out_direction);
}

absl::StatusOr<LineCirclePath> FindPerpendicularTurningParkingOutPath(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, const Box2d& target_box,
    ParkingOutDirection parking_out_direction, bool use_fast_method) {
  QCHECK(parking_out_direction == PARKING_OUT_DIR_PERP_LEFT_FWD ||
         parking_out_direction == PARKING_OUT_DIR_PERP_RIGHT_FWD ||
         parking_out_direction == PARKING_OUT_DIR_PERP_LEFT_BACK ||
         parking_out_direction == PARKING_OUT_DIR_PERP_RIGHT_BACK);
  if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
          veh_geo_params, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          start)) {
    return absl::InternalError("Start Invalid.");
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
  const bool forward = parking_out_direction == PARKING_OUT_DIR_PERP_LEFT_FWD ||
                       parking_out_direction == PARKING_OUT_DIR_PERP_RIGHT_FWD;
  const bool left = parking_out_direction == PARKING_OUT_DIR_PERP_LEFT_FWD ||
                    parking_out_direction == PARKING_OUT_DIR_PERP_LEFT_BACK;
  QCHECK_EQ(left, start.tangent.CrossProd(
                      Vec2d::FastUnitFromAngle(target_box.heading())) > 0.0);
  constexpr double kForwardAdjustDistanceFactor = 1.0;
  constexpr double kBackwardAdjustDistanceFactor = 1.5;
  double max_adjust_distance =
      forward ? veh_geo_params.length() * kForwardAdjustDistanceFactor
              : veh_geo_params.length() * kBackwardAdjustDistanceFactor;
  const auto first_overlap_distance =
      GetFirstOverlapDistanceWithKDTreeAndVirtualBoundaries(
          veh_geo_params, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          start, /*kappa=*/0.0, forward, max_adjust_distance);
  if (first_overlap_distance.has_value() &&
      *first_overlap_distance < max_adjust_distance) {
    max_adjust_distance = *first_overlap_distance;
  }
  constexpr double kPathStep = 0.1;  // m.
  for (double s =
           path_finder_params.geometry_method_params().min_drive_distance();
       s <= max_adjust_distance; s += kPathStep) {
    const auto cur_pose = ExtendPathByConstantKappa(
        start, /*kappa=*/0.0, forward ? s : -s, GeometryPathType::STRAIGHT);
    ASSIGN_OR_CONTINUE(
        const auto zigzag_path,
        ZigZagToTargetBox(veh_geo_params, max_kappa, path_finder_params,
                          vehicle_model_params, segments_kd_tree, objects_map,
                          boundaries_map, virtual_boundaries, cur_pose,
                          target_box, forward, left));
    const LineCirclePath adjust_path = {.start = start,
                                        .types = {GeometryPathType::STRAIGHT},
                                        .lengths = {forward ? s : -s},
                                        .kappas = {0.0},
                                        .ends = {cur_pose}};
    LineCirclePath result;
    ConnectPaths({adjust_path, zigzag_path}, &result);
    if (use_fast_method) {
      return result;
    }
    if (!path_candidate.has_value() ||
        get_path_segments_num(result) <
            get_path_segments_num(*path_candidate)) {
      path_candidate = std::move(result);
    }
  }

  if (path_candidate.has_value()) {
    return *path_candidate;
  } else {
    return absl::InternalError("Parking out path generation failed.");
  }
}

}  // namespace planner
}  // namespace qcraft
