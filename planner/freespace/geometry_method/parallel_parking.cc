#include "onboard/planner/freespace/geometry_method/parallel_parking.h"

#include <algorithm>
#include <cmath>
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

bool FindDriveOutPathFromParkingSpot(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    GeometryPathType drive_dir,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, const GeometryMethodPoint& end,
    LineCirclePath* result) {
  constexpr double kDriveOutMinDistance = 1.0;   // m.
  constexpr double kKDriveOutMaxDistance = 5.0;  // m.
  constexpr double kKDriveOutTrialStep = 0.2;    // m.
  for (double s = kDriveOutMinDistance; s < kKDriveOutMaxDistance;
       s += kKDriveOutTrialStep) {
    LineCirclePath path1 = {
        .start = start,
        .types = {drive_dir},
        .lengths = {s},
        .kappas = {max_kappa},
        .ends = {ExtendPathByConstantKappa(start, max_kappa, s, drive_dir)}};
    if (s == kDriveOutMinDistance &&
        !CheckPathValidityWithKDTreeAndVirtualBoundaries(
            veh_geo_params, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            path1)) {
      break;
    }
    // This can prevent unnecessary check because KDriveOutTrialStep is
    // small.
    if (s > kDriveOutMinDistance &&
        !CheckPoseValidityWithKDTreeAndVirtualBoundaries(
            veh_geo_params, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            path1.ends[0])) {
      break;
    }
    // Check if can connect to start.
    LineCirclePath path2;
    if (CircleLineConection(path1.ends[0], end, max_kappa, &path2) &&
        CheckPathValidityWithKDTreeAndVirtualBoundaries(
            veh_geo_params, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            path2)) {
      ConnectPaths({path1, path2}, result);
      return true;
    }
  }
  return false;
}

bool FindParallelParkingPathWithAdjustments(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    GeometryPathType backward_adjustment_dir,
    GeometryPathType forward_adjustment_dir,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, const GeometryMethodPoint& goal,
    LineCirclePath* result) {
  const auto max_backward_adjust_dist =
      [&backward_adjustment_dir, &max_kappa, &veh_geo_params,
       &path_finder_params, &vehicle_model_params, &segments_kd_tree,
       &objects_map, &boundaries_map, &virtual_boundaries](
          const GeometryMethodPoint& start) -> std::optional<double> {
    constexpr double kMaxBackwardAdjustDistance = 1.5;  // m.
    constexpr double kBackwardAdjustDistStep = 0.05;    // m.
    std::optional<double> res = std::nullopt;
    for (double s =
             path_finder_params.geometry_method_params().min_drive_distance();
         s < kMaxBackwardAdjustDistance; s += kBackwardAdjustDistStep) {
      const auto cur_pose = ExtendPathByConstantKappa(start, max_kappa, -s,
                                                      backward_adjustment_dir);
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

  const auto max_forward_adjust_dist =
      [&forward_adjustment_dir, &max_kappa, &veh_geo_params,
       &path_finder_params, &vehicle_model_params, &segments_kd_tree,
       &objects_map, &boundaries_map, &virtual_boundaries](
          const GeometryMethodPoint& start) -> std::optional<double> {
    constexpr double kMaxForwardAdjustDistance = 2.0;  // m.
    constexpr double kForwardAdjustDistStep = 0.05;    // m.
    std::optional<double> res = std::nullopt;
    for (double s =
             path_finder_params.geometry_method_params().min_drive_distance();
         s < kMaxForwardAdjustDistance; s += kForwardAdjustDistStep) {
      const auto cur_pose = ExtendPathByConstantKappa(start, max_kappa, s,
                                                      forward_adjustment_dir);
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

  // Drive backward util collision, and then change steer drive forward, these
  // two actions is a loop. We can repeat such loop util av can drive out of
  // spot or reach max iterations.
  int iterations = 0;
  LineCirclePath adjust_path;
  auto cur_start = goal;
  while (true) {
    // Drive backward util collision.
    const auto max_backward_drive_s = max_backward_adjust_dist(cur_start);
    // This means we can't drive backward anymore, at the same time we can't
    // drive out in previous loop or strategy, so plan failed.
    if (!max_backward_drive_s.has_value()) {
      return false;
    }
    LineCirclePath backward_path = {
        .start = cur_start,
        .types = {backward_adjustment_dir},
        .lengths = {-(*max_backward_drive_s)},
        .kappas = {max_kappa},
        .ends = {ExtendPathByConstantKappa(cur_start, max_kappa,
                                           -*max_backward_drive_s,
                                           backward_adjustment_dir)}};
    if (iterations == 0) {
      adjust_path = std::move(backward_path);
    } else {
      ConnectPaths({adjust_path, backward_path}, result);
      adjust_path = *result;
    }
    cur_start = adjust_path.ends.back();
    // Try to drive out.
    LineCirclePath drive_out_path;
    if (FindDriveOutPathFromParkingSpot(
            veh_geo_params, max_kappa, forward_adjustment_dir,
            path_finder_params, vehicle_model_params, segments_kd_tree,
            objects_map, boundaries_map, virtual_boundaries, cur_start, start,
            &drive_out_path)) {
      ConnectPaths({adjust_path, drive_out_path}, result);
      ReversePath(result);
      return true;
    }
    // We don't allow adjusting for too many times.
    if (iterations >= path_finder_params.geometry_method_params()
                          .parallel_parking_max_adjustments()) {
      break;
    }
    // Dirve forward util collision.
    const auto max_forward_drive_s = max_forward_adjust_dist(cur_start);
    if (!max_forward_drive_s.has_value()) {
      return false;
    }
    LineCirclePath forward_path = {
        .start = cur_start,
        .types = {forward_adjustment_dir},
        .lengths = {*max_forward_drive_s},
        .kappas = {max_kappa},
        .ends = {ExtendPathByConstantKappa(cur_start, max_kappa,
                                           *max_forward_drive_s,
                                           forward_adjustment_dir)}};
    ConnectPaths({adjust_path, forward_path}, result);
    adjust_path = *result;
    cur_start = adjust_path.ends.back();
    ++iterations;
  }
  return false;
}

bool FindParallelParkingPathForSpotBehindStart(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, const GeometryMethodPoint& goal,
    int strategy_level, LineCirclePath* result) {
  // Check parking spot is on left or right.
  const bool is_on_left = start.tangent.CrossProd(goal.pos - start.pos) > 0.0;
  const GeometryPathType forward_dir =
      is_on_left ? GeometryPathType::RIGHT : GeometryPathType::LEFT;
  const GeometryPathType backward_dir =
      is_on_left ? GeometryPathType::LEFT : GeometryPathType::RIGHT;

  // For parallel parking, we exchange start and goal, so our strategy is to
  // find how to drive out of the parking spot and go to the start pose.

  // Strategy 1: Directly drive out.
  if (FindDriveOutPathFromParkingSpot(
          veh_geo_params, max_kappa, forward_dir, path_finder_params,
          vehicle_model_params, segments_kd_tree, objects_map, boundaries_map,
          virtual_boundaries, goal, start, result)) {
    ReversePath(result);
    return true;
  }
  if (strategy_level <= 1) return false;

  constexpr double kMaxBackwardDist = 2.0;     // m.
  constexpr double kBackwardDriveStep = 0.05;  // m.
  // Strategy 2: Drive backward in spot, then drive out.
  {
    for (double s =
             path_finder_params.geometry_method_params().min_drive_distance();
         s < kMaxBackwardDist; s += kBackwardDriveStep) {
      LineCirclePath path1 = {.start = goal,
                              .types = {GeometryPathType::STRAIGHT},
                              .lengths = {-s},
                              .kappas = {0.0},
                              .ends = {ExtendPathByConstantKappa(
                                  goal, 0.0, -s, GeometryPathType::STRAIGHT)}};
      if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
              veh_geo_params, path_finder_params, vehicle_model_params,
              segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
              path1.ends[0])) {
        break;
      }
      // Try to drive out.
      LineCirclePath drive_out_path;
      if (FindDriveOutPathFromParkingSpot(
              veh_geo_params, max_kappa, forward_dir, path_finder_params,
              vehicle_model_params, segments_kd_tree, objects_map,
              boundaries_map, virtual_boundaries, path1.ends[0], start,
              &drive_out_path)) {
        ConnectPaths({path1, drive_out_path}, result);
        ReversePath(result);
        return true;
      }
    }
  }

  // Strategy 2.5: Drive backward and turn direction in the spot, then try to
  // drive out.
  {
    // Drive backward util collision.
    for (double s =
             path_finder_params.geometry_method_params().min_drive_distance();
         s < kMaxBackwardDist; s += kBackwardDriveStep) {
      LineCirclePath path1 = {.start = goal,
                              .types = {backward_dir},
                              .lengths = {-s},
                              .kappas = {max_kappa},
                              .ends = {ExtendPathByConstantKappa(
                                  goal, max_kappa, -s, backward_dir)}};
      if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
              veh_geo_params, path_finder_params, vehicle_model_params,
              segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
              path1.ends[0])) {
        break;
      }
      // Try to drive out.
      LineCirclePath drive_out_path;
      if (FindDriveOutPathFromParkingSpot(
              veh_geo_params, max_kappa, forward_dir, path_finder_params,
              vehicle_model_params, segments_kd_tree, objects_map,
              boundaries_map, virtual_boundaries, path1.ends[0], start,
              &drive_out_path)) {
        ConnectPaths({path1, drive_out_path}, result);
        ReversePath(result);
        return true;
      }
    }
  }
  if (strategy_level <= 2) return false;

  // Strategy 3: Adjust vehicle pos and heading in the spot, then drive out. For
  // every adjustment loop, vehicle will drive util collision.
  if (FindParallelParkingPathWithAdjustments(
          veh_geo_params, max_kappa, backward_dir, forward_dir,
          path_finder_params, vehicle_model_params, segments_kd_tree,
          objects_map, boundaries_map, virtual_boundaries, start, goal,
          result)) {
    return true;
  }
  if (strategy_level <= 3) return false;

  // All strategies fail.
  return false;
}
}  // namespace

absl::StatusOr<LineCirclePath> FindParallelParkingPath(
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
  // Convert goal pose to origin.
  const Vec2d pos = start.pos - goal.pos;
  const double delta_theta = NormalizeAngle(start.theta - goal.theta);
  const Vec2d normalized_start =
      pos.Rotate(goal.tangent.x(), -goal.tangent.y());

  // For start pose out of this region, we don't plan.
  constexpr double kMaxThetaDeviationWhenPlanStart = M_PI_4;      // rad.
  constexpr double kMaxDistanceBehindSpotWhenPlanStart = 10.0;    // m.
  constexpr double kMaxDistanceBeforeSpotWhenPlanStart = 30.0;    // m.
  constexpr double kMaxLateralDistanceToSpotWhenPlanStart = 7.0;  // m.
  if (std::abs(delta_theta) > kMaxThetaDeviationWhenPlanStart) {
    return absl::InternalError("Theta error too large.");
  }
  if (normalized_start.x() > kMaxDistanceBeforeSpotWhenPlanStart ||
      normalized_start.x() < -kMaxDistanceBehindSpotWhenPlanStart) {
    return absl::InternalError("Longitudinal distance too large.");
  }
  if (std::abs(normalized_start.y()) > kMaxLateralDistanceToSpotWhenPlanStart) {
    return absl::InternalError("Lateral distance too large.");
  }

  // For parking spot that is not behind start, we need first drive forward for
  // a short distance.
  constexpr double kEpsilon = 1e-6;
  // If start pos is less than kDriveForwardStartPosition before the parking
  // spot, we firstly need to drive forward. If start is more than
  // kDriveForwardEndPosition before the spot, we don't need to dirve forward.
  // If between two values, we firstly try parking without driving forward, and
  // then drive forward if previous trial failed.
  constexpr double kDriveForwardStartPosition = 3.5;  // m.
  constexpr double kDriveForwardEndPosition = 6.5;    // m.
  // Distance of start pos before the parking spot. If we need to drive forward,
  // we must firstly drive to this region and then try
  // FindParallelParkingPathForSpotBehindStart().
  constexpr double kMinDistanceBeforeSpot = 5.5;  // m.
  constexpr double kMaxDistanceBeforeSpot = 8.5;  // m.
  constexpr double kForwardDrivingStep = 1.0;     // m.

  if (normalized_start.x() < kDriveForwardStartPosition) {
    LineCirclePath forward_path;
    LineCirclePath parking_path;
    // We firstly try path with fewer gear change, this will cause unnecessary
    // computation but now we ignore it because the code is cleaner when we do
    // like this. In parking spot finding mode (use_fast_method = true), we
    // don't need to consider this.
    const int strategy_level_start = use_fast_method ? 3 : 1;
    for (int strategy_level = strategy_level_start; strategy_level <= 3;
         ++strategy_level) {
      for (double s = kMinDistanceBeforeSpot - normalized_start.x();
           s < kMaxDistanceBeforeSpot + kEpsilon - normalized_start.x();
           s += kForwardDrivingStep) {
        if (s <
            path_finder_params.geometry_method_params().min_drive_distance()) {
          continue;
        }
        // TODO(Zhuang): Maybe we can try polynomial path in the future.
        forward_path = {.start = start,
                        .types = {GeometryPathType::STRAIGHT},
                        .lengths = {s},
                        .kappas = {0.0},
                        .ends = {ExtendPathByConstantKappa(
                            start, 0.0, s, GeometryPathType::STRAIGHT)}};
        if (!CheckPathValidityWithKDTreeAndVirtualBoundaries(
                veh_geo_params, path_finder_params, vehicle_model_params,
                segments_kd_tree, objects_map, boundaries_map,
                virtual_boundaries, forward_path)) {
          break;
        }
        if (FindParallelParkingPathForSpotBehindStart(
                veh_geo_params, max_kappa, path_finder_params,
                vehicle_model_params, segments_kd_tree, objects_map,
                boundaries_map, virtual_boundaries, forward_path.ends[0], goal,
                strategy_level, &parking_path)) {
          ConnectPaths({forward_path, parking_path}, &result);
          return result;
        }
      }
    }
  } else if (normalized_start.x() >= kDriveForwardStartPosition &&
             normalized_start.x() < kDriveForwardEndPosition) {
    // We firstly try parking backward, if fail, we drive forward for a short
    // distance and continue to parking in.
    if (FindParallelParkingPathForSpotBehindStart(
            veh_geo_params, max_kappa, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            start, goal,
            /*strategy_level=*/2, &result)) {
      return result;
    }
    LineCirclePath forward_path;
    LineCirclePath parking_path;
    const int strategy_level_start = use_fast_method ? 3 : 1;
    for (int strategy_level = strategy_level_start; strategy_level <= 3;
         ++strategy_level) {
      // Here we use a smaller step because the adjustment space is narrow.
      for (double s = kDriveForwardEndPosition - normalized_start.x();
           s < kMaxDistanceBeforeSpot + kEpsilon - normalized_start.x();
           s += 0.5 * kForwardDrivingStep) {
        if (s <
            path_finder_params.geometry_method_params().min_drive_distance()) {
          continue;
        }
        forward_path = {.start = start,
                        .types = {GeometryPathType::STRAIGHT},
                        .lengths = {s},
                        .kappas = {0.0},
                        .ends = {ExtendPathByConstantKappa(
                            start, 0.0, s, GeometryPathType::STRAIGHT)}};
        if (!CheckPathValidityWithKDTreeAndVirtualBoundaries(
                veh_geo_params, path_finder_params, vehicle_model_params,
                segments_kd_tree, objects_map, boundaries_map,
                virtual_boundaries, forward_path)) {
          break;
        }
        if (FindParallelParkingPathForSpotBehindStart(
                veh_geo_params, max_kappa, path_finder_params,
                vehicle_model_params, segments_kd_tree, objects_map,
                boundaries_map, virtual_boundaries, forward_path.ends[0], goal,
                strategy_level, &parking_path)) {
          ConnectPaths({forward_path, parking_path}, &result);
          LineCirclePath path_candidate_without_driving_forward;
          // If path strategy level is 3, we need to see if we can park without
          // driving forward because we only try this with level 2 before.
          if (strategy_level == 3 &&
              FindParallelParkingPathForSpotBehindStart(
                  veh_geo_params, max_kappa, path_finder_params,
                  vehicle_model_params, segments_kd_tree, objects_map,
                  boundaries_map, virtual_boundaries, start, goal,
                  /*strategy_level=*/3,
                  &path_candidate_without_driving_forward)) {
            return path_candidate_without_driving_forward;
          } else {
            return result;
          }
        }
      }
    }
  } else {
    if (FindParallelParkingPathForSpotBehindStart(
            veh_geo_params, max_kappa, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            start, goal,
            /*strategy_level=*/3, &result)) {
      return result;
    }
  }
  return absl::InternalError("Plan fail.");
}

absl::StatusOr<LineCirclePath> FindParallelParkingReplanPath(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    FreespaceReplanReasonProto::ReplanReason replan_reason,
    const GeometryMethodPoint& start, const GeometryMethodPoint& goal) {
  if (replan_reason !=
      FreespaceReplanReasonProto::PARALLEL_PARKING_FINAL_PATH) {
    return absl::InternalError("Invalid replan reason.");
  }

  const double delta_theta = NormalizeAngle(start.theta - goal.theta);
  const Vec2d delta_pos =
      (start.pos - goal.pos).Rotate(goal.tangent.x(), -goal.tangent.y());
  constexpr double kReplanMaxThetaError = M_PI * 0.1;  // rad.
  constexpr double kReplanMaxLongitudinalError = 2.0;  // m.
  constexpr double kReplanMaxLateralError = 1.0;       // m.
  if (std::abs(delta_theta) > kReplanMaxThetaError) {
    return absl::InternalError("Replan theta error too big.");
  }
  if (std::abs(delta_pos.x()) > kReplanMaxLongitudinalError) {
    return absl::InternalError("Replan distance error too big.");
  }
  if (std::abs(delta_pos.y()) > kReplanMaxLateralError) {
    return absl::InternalError("Replan distance error too big.");
  }

  // Check if we need circle path.
  constexpr double kThetaErrorThreshold = 1.0e-6;
  if (std::abs(delta_theta) < kThetaErrorThreshold) {
    // We only need to drive forward or backward in this situation.
    constexpr double kMaxLateralError = 0.5;  // m.
    if (std::abs(delta_pos.y()) > kMaxLateralError) {
      return absl::InternalError("Replan start pose not proper.");
    }
    const LineCirclePath line_path = {
        .start = start,
        .types = {GeometryPathType::STRAIGHT},
        .lengths = {delta_pos.x()},
        .kappas = {0.0},
        .ends = {ExtendPathByConstantKappa(start, 0.0, delta_pos.x(),
                                           GeometryPathType::STRAIGHT)}};
    // We don't check path because start pose maybe invalid due to reset.
    if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
            veh_geo_params, path_finder_params, vehicle_model_params,
            segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
            line_path.ends.back())) {
      return absl::InternalError("Replan target pose invalid.");
    }
    return line_path;
  }

  // Circle path firstly.
  const bool forward = (delta_pos.x() < 0.0);
  const GeometryPathType dir = (forward ^ (delta_theta > 0.0))
                                   ? GeometryPathType::LEFT
                                   : GeometryPathType::RIGHT;
  const double distance =
      std::max(path_finder_params.geometry_method_params().min_drive_distance(),
               std::abs(delta_theta) / max_kappa);
  const double kappa = std::abs(delta_theta) / distance;
  const LineCirclePath circle_path = {
      .start = start,
      .types = {dir},
      .lengths = {forward ? distance : -distance},
      .kappas = {kappa},
      .ends = {ExtendPathByConstantKappa(start, kappa,
                                         forward ? distance : -distance, dir)}};
  if (!CheckPoseValidityWithKDTreeAndVirtualBoundaries(
          veh_geo_params, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          circle_path.ends.back())) {
    return absl::InternalError("Replan target pose invalid.");
  }

  // Line path later.
  const double longitudinal_distance =
      (goal.pos - circle_path.ends.back().pos).Dot(goal.tangent);
  if (std::abs(longitudinal_distance) >=
      path_finder_params.geometry_method_params().min_drive_distance()) {
    const LineCirclePath line_path = {
        .start = circle_path.ends.back(),
        .types = {GeometryPathType::STRAIGHT},
        .lengths = {longitudinal_distance},
        .kappas = {0.0},
        .ends = {ExtendPathByConstantKappa(circle_path.ends.back(), 0.0,
                                           longitudinal_distance,
                                           GeometryPathType::STRAIGHT)}};
    LineCirclePath result;
    ConnectPaths({circle_path, line_path}, &result);
    return result;
  }
  return circle_path;
}

}  // namespace planner
}  // namespace qcraft
