#include "onboard/planner/assist/tja_internal.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/polynomial.h"
#include "onboard/math/polynomial_fitter.h"
#include "onboard/math/util.h"
#include "onboard/planner/assist/planner_odc_generator.h"
#include "onboard/planner/assist/vision_lane_path_filter.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {
constexpr double kSampleInterval = 2.0;           // m.
constexpr double kLonConsiderTime = 3.0;          // s.
constexpr double kMinLonConsiderDistance = 60.0;  // m.
constexpr double kMaxPotentialLeaderDist = 50.0;  // m.
constexpr double kMaxPotentialLatDist = 0.5;      // m.
constexpr double kMaxLatConsiderDist = 1.6;       // m.
constexpr double kMaxLatConsiderLcDist = 2.0;     // m.
constexpr double kMinHistoryPointDist = 4.0;      // m.
constexpr double kMaxEnableTjaSpeed = 16.67;      // m/s
constexpr double kMaxHeadingDiff = M_PI * 8.0 / 180.0;
constexpr double kMaxHeadingDiffInTja = M_PI * 15.0 / 180.0;
constexpr double kMaxCenterLineAngleDiffStepForObs = M_PI * 0.1 / 180.0;
constexpr double kMaxCenterLineAngleDiffStepForLane = M_PI * 10.0 / 180.0;
constexpr double kLastTrajWeight = 10.0;
constexpr double kNearestLaneWeight = 20.0;
constexpr double kLatOffsetWeight = 3.0;
constexpr double kConsiderPredictionTime = 6.0;  // s.
constexpr int kMaxSamplePoints = 50;
constexpr int kBehindSamplePoints = 10;
constexpr int kMinHistoryPoints = 5;
constexpr int kPolyFitterDegree = 1;
constexpr int kMinCenterLineSize = 10;
constexpr int kVirtualLaneId = 1;
constexpr int kVirtualLeftBoundaryId = 2;
constexpr int kVirtualRightBoundaryId = 3;
constexpr double kCheckExtraLength = 2.0;       // m.
constexpr double kTargetLanePathLength = 30.0;  // m.
constexpr double kNearAngleErrorThreshold = M_PI_4;
constexpr double kLateralApproachRate = 0.1;    // m.
constexpr double kDistanceEpsilon = 0.1;        // m.
constexpr double kLongiProjectThreshold = 1.0;  // m.
constexpr double kLatProjectThreshold = 0.5;    // m.
constexpr double kDrivePassageStepS = 1.0;      // m.
constexpr int kExitCounterThreshold = 5;
constexpr int kFillPlannerCenterLineSize = 100;
constexpr double kValidLanePathLateralThreshold = 5.0;  // m.

std::vector<Vec2d> GetObstacleTrajectoryPoints(
    const FrenetFrame& frenet_frame, const SpacetimeObjectTrajectory& traj) {
  // Use prediction to extend the points.
  std::vector<Vec2d> obs_pos_points;
  obs_pos_points.reserve(traj.states().size());
  for (const auto& traj_state : traj.states()) {
    if (traj_state.traj_point->t() > kConsiderPredictionTime) break;
    if (obs_pos_points.size() == 0 ||
        obs_pos_points.front().DistanceTo(traj_state.contour.CircleCenter()) >
            kMinHistoryPointDist) {
      obs_pos_points.insert(obs_pos_points.begin(),
                            traj_state.contour.CircleCenter());
    }
  }

  // Convert the point to frenet frame.
  std::vector<Vec2d> obs_frenet_pos_points;
  obs_frenet_pos_points.reserve(obs_pos_points.size());
  for (int i = 0; i < obs_pos_points.size(); i++) {
    const auto frenet = frenet_frame.XYToSL(obs_pos_points.at(i));
    if (std::fabs(frenet.l) > kMaxLatConsiderLcDist) {
      obs_frenet_pos_points.clear();
      break;
    }
    obs_frenet_pos_points.emplace_back(frenet.s, frenet.l);
  }

  return obs_frenet_pos_points;
}

mapping::LaneBoundaryProto::Type MapStationBoundaryTypeToOnlineBoundaryType(
    StationBoundaryType type) {
  switch (type) {
    case StationBoundaryType::UNKNOWN_TYPE:
      return mapping::LaneBoundaryProto::UNKNOWN_TYPE;
    case StationBoundaryType::BROKEN_WHITE:
      return mapping::LaneBoundaryProto::BROKEN_WHITE;
    case StationBoundaryType::SOLID_WHITE:
      return mapping::LaneBoundaryProto::SOLID_WHITE;
    case StationBoundaryType::BROKEN_YELLOW:
      return mapping::LaneBoundaryProto::BROKEN_YELLOW;
    case StationBoundaryType::SOLID_YELLOW:
      return mapping::LaneBoundaryProto::SOLID_YELLOW;
    case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      return mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW;
    case StationBoundaryType::CURB:
    case StationBoundaryType::VIRTUAL_CURB:
    case StationBoundaryType::PREDICTION_VIRTUAL_CURB:
      return mapping::LaneBoundaryProto::CURB;
    case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
      return mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE;
    case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
      return mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE;
  }
}

bool CheckOnlineSemanticMap(const PoseProto& pose,
                            const PlannerSemanticMapManager& psmm,
                            const TjaState& tja_state) {
  // Enter tja a little earlier before exit lcc.
  double lcc_min_required_length =
      CalculateLccMinRequiredLaneLength(pose) + kCheckExtraLength;
  // Avoid short lane path in intersection.
  lcc_min_required_length =
      tja_state.planner_use_tja_map
          ? std::max(kTargetLanePathLength, lcc_min_required_length)
          : lcc_min_required_length;
  // Check is there is enough length.
  ASSIGN_OR_RETURN(
      const auto lane_paths,
      FindNearLanePathsFromEgoPose(
          psmm, Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()), pose.yaw(),
          lcc_min_required_length,
          /*heading_penalty_weight=*/0.0,
          /*distance_threshold=*/10.0, kNearAngleErrorThreshold),
      false);

  for (const auto& lane_path : lane_paths) {
    if (lane_path.length() < lcc_min_required_length - kDistanceEpsilon) {
      continue;
    }
    const auto lane_smooth_points = SampleLanePathPoints(psmm, lane_path);
    ASSIGN_OR_CONTINUE(
        const auto lane_frenet_frame,
        BuildBruteForceFrenetFrame(lane_smooth_points,
                                   /*down_sample_raw_points=*/true));
    const auto ego_pos_sl = lane_frenet_frame.XYToSL(Vec2dFromPoseProto(pose));
    if (tja_state.planner_use_tja_map) {
      // Check ego init pos when last state is tja.
      if (std::fabs(ego_pos_sl.s) > kLongiProjectThreshold ||
          std::fabs(ego_pos_sl.l) > kLatProjectThreshold) {
        continue;
      }
    } else {
      // Check if there is a valid lane path arond ego pose.
      if (std::fabs(ego_pos_sl.l) > kValidLanePathLateralThreshold) {
        continue;
      }
    }
    return true;
  }
  return false;
}

void BuildBoundaryFromLanePath(const PlannerSemanticMapManager& psmm,
                               const mapping::LanePath& lane_path,
                               TjaState* tja_state) {
  const auto dp_or = BuildDrivePassageFromLanePath(
      psmm, lane_path, kDrivePassageStepS,
      /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
      /*required_planning_horizon=*/0.0, /*required_backward_len=*/0.0,
      std::nullopt);
  if (dp_or.ok()) {
    tja_state->left_boundary.reserve(kMaxSamplePoints);
    tja_state->right_boundary.reserve(kMaxSamplePoints);
    const auto fill_boundary = [tja_state, &dp_or](
                                   const FrenetCoordinate& center_frenet,
                                   OptionalBoundary boundary) {
      if (boundary.has_value() && boundary->IsSolid(center_frenet.l)) {
        auto boundary_point_or =
            dp_or->QueryPointXYAtSL(center_frenet.s, boundary->lat_offset);
        if (boundary_point_or.ok()) {
          if (boundary->lat_offset > center_frenet.l) {
            tja_state->left_boundary.push_back(
                BoundaryPoint{.type = boundary->type,
                              .point = std::move(boundary_point_or).value()});
          } else {
            tja_state->right_boundary.push_back(
                BoundaryPoint{.type = boundary->type,
                              .point = std::move(boundary_point_or).value()});
          }
        }
      }
    };
    for (const auto& center_point : tja_state->center_line) {
      ASSIGN_OR_CONTINUE(const auto center_frenet,
                         dp_or->QueryUnboundedFrenetCoordinateAt(center_point));
      const auto boundary_response =
          dp_or->QueryEnclosingLaneBoundariesAtS(center_frenet.s);
      fill_boundary(center_frenet, boundary_response.left);
      fill_boundary(center_frenet, boundary_response.right);
    }
  }
}

absl::StatusOr<mapping::LanePath> FindNearestTargetLanePath(
    const PoseProto& pose, const std::vector<Vec2d>& last_center_line,
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map) {
  // Preview the find best suitable lane path.
  constexpr double kPreviewTime = 2.0;      // .s
  constexpr double kPreviewTimeStep = 0.2;  // .s
  auto accel = pose.accel_body().x();
  auto speed = pose.vel_body().x();

  PathPoint curr_path_point;
  curr_path_point.set_x(pose.pos_smooth().x());
  curr_path_point.set_y(pose.pos_smooth().y());
  curr_path_point.set_s(0.0);
  curr_path_point.set_theta(pose.yaw());
  curr_path_point.set_lambda(0.0);

  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  if (std::fabs(speed) < kLowSpeedThreshold) {
    curr_path_point.set_kappa(0.0);
  } else {
    curr_path_point.set_kappa(pose.ar_smooth().z() / speed);
  }

  for (double t = 0.0; t <= kPreviewTime; t += kPreviewTimeStep) {
    speed += accel * kPreviewTimeStep;
    curr_path_point =
        GetPathPointAlongCircle(curr_path_point, speed * kPreviewTimeStep);
  }

  const auto prev_frenet_frame_or =
      BuildKdTreeFrenetFrame(last_center_line,
                             /*down_sample_raw_points=*/true);
  if (!prev_frenet_frame_or.ok()) {
    // Find init lane path from ego pose.
    return FindInitialLanePathFromEgoPose(psmm, pose, /*thread_pool=*/nullptr);
  }
  return ProjectLaneFrenetFrameCurrentOnlineMap(
      psmm, online_map, prev_frenet_frame_or.value(),
      Vec2d(curr_path_point.x(), curr_path_point.y()), " tja center line",
      speed, /*check_preview_length=*/0.0, /*thread_pool=*/nullptr);
}

std::vector<Vec2d> ConnectFromEgoToTargetLanePath(
    const PoseProto& pose, const mapping::LanePath& lane_path,
    const PlannerSemanticMapManager& psmm) {
  const auto lane_smooth_points = SampleLanePathPoints(psmm, lane_path);
  ASSIGN_OR_RETURN(const auto lane_frenet_frame,
                   BuildKdTreeFrenetFrame(lane_smooth_points,
                                          /*down_sample_raw_points=*/true),
                   std::vector<Vec2d>());
  const auto ego_pos_sl = lane_frenet_frame.XYToSL(Vec2dFromPoseProto(pose));
  std::vector<Vec2d> approach_points;
  approach_points.reserve(kMaxSamplePoints);
  bool is_left = ego_pos_sl.l > 0.0;
  auto approach_point = ego_pos_sl;
  for (int i = 0; i < kMaxSamplePoints; ++i) {
    if (is_left) {
      approach_point.l = std::max(
          0.0, approach_point.l - kSampleInterval * kLateralApproachRate);
    } else {
      approach_point.l = std::min(
          0.0, approach_point.l + kSampleInterval * kLateralApproachRate);
    }
    approach_points.push_back(lane_frenet_frame.SLToXY(approach_point));
    approach_point.s += kSampleInterval;
  }

  return approach_points;
}

absl::StatusOr<Polynomial> BuildPolynomialInFrenetFrame(
    const std::vector<Vec2d>& points, const FrenetFrame& frenet_frame,
    int poly_degree) {
  std::vector<Vec2d> frenet_points;
  frenet_points.reserve(points.size());
  for (const auto& point : points) {
    const auto point_frenet = frenet_frame.XYToSL(point);
    frenet_points.emplace_back(point_frenet.s, point_frenet.l);
  }
  return FitPolynomialToData(poly_degree, frenet_points);
}

absl::StatusOr<KdTreeFrenetFrame> BuildFrenetFrameFromEgoPose(
    const PoseProto& pose) {
  std::vector<Vec2d> ego_heading_points;
  ego_heading_points.reserve(kMaxSamplePoints);
  auto ego_virtual_pos = Vec2dFromPoseProto(pose);

  const double heading = pose.yaw();
  const auto shift_vec = Vec2d::UnitFromAngle(heading) * kSampleInterval;
  for (int i = 0; i < kMaxSamplePoints; ++i) {
    ego_heading_points.push_back(ego_virtual_pos);
    ego_virtual_pos = ego_virtual_pos + shift_vec;
  }
  return BuildKdTreeFrenetFrame(ego_heading_points,
                                /*down_sample_raw_points=*/true);
}

absl::Status UpdateTjaLaneData(
    const PoseProto& pose, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& veh_geo_params,
    const mapping::OnlineSemanticMapProto& online_map,
    const PlannerSemanticMapManager& psmm, TjaState* tja_state) {
  SCOPED_QTRACE("UpdateTjaLaneData");
  // Deduce lane from other vehicle prediction trajectories.
  // 1. construct ego coordinate converter.
  const auto last_center_line = tja_state->center_line.empty()
                                    ? std::move(tja_state->planner_center_line)
                                    : std::move(tja_state->center_line);
  tja_state->center_line.clear();
  tja_state->planner_center_line.clear();
  tja_state->target_obs_ids.clear();
  tja_state->left_boundary.clear();
  tja_state->right_boundary.clear();
  tja_state->potential_obs_ids.clear();
  tja_state->exit_lane_ids.clear();
  tja_state->update_id = tja_state->update_id + 1;
  const double half_length = veh_geo_params.length() * 0.5;
  ASSIGN_OR_RETURN(const auto frenet_frame, BuildFrenetFrameFromEgoPose(pose),
                   _ << "Failed to build frenet frame from ego pose.");

  // 2.1 fit polynomial line for different obstacles.
  std::vector<std::vector<double>> betas;
  std::vector<double> weights;
  std::vector<std::string> potential_obs_ids;
  absl::flat_hash_set<std::string> checked_set;
  const double longi_consider_dist =
      pose.speed() * kLonConsiderTime + kMinLonConsiderDistance;
  const double max_heading_diff =
      tja_state->planner_use_tja_map ? kMaxHeadingDiffInTja : kMaxHeadingDiff;
  for (const auto& traj : st_traj_mgr.trajectories()) {
    // Filter out valid obstacles.
    if (checked_set.contains(traj.object_id())) continue;
    checked_set.emplace(traj.object_id());
    if (traj.object_type() != ObjectType::OT_VEHICLE &&
        traj.object_type() != ObjectType::OT_LARGE_VEHICLE) {
      continue;
    }
    ASSIGN_OR_CONTINUE(const auto aabbox,
                       frenet_frame.QueryFrenetBoxAtContour(traj.contour()));
    const auto min_lat_offset =
        aabbox.l_min * aabbox.l_max > 0
            ? std::min(std::fabs(aabbox.l_min), std::fabs(aabbox.l_max))
            : 0.0;
    const auto center_frenet = aabbox.center();
    if (std::fabs(center_frenet.s) > longi_consider_dist ||
        min_lat_offset > kMaxLatConsiderDist || aabbox.s_min < half_length) {
      continue;
    }
    const auto frenet_tan = frenet_frame.InterpolateTangentByS(center_frenet.s);
    if (std::abs(NormalizeAngle(frenet_tan.FastAngle() - traj.pose().theta())) >
        max_heading_diff) {
      continue;
    }
    if (std::fabs(center_frenet.s) < kMaxPotentialLeaderDist &&
        min_lat_offset < kMaxPotentialLatDist) {
      tja_state->potential_obs_ids.push_back(std::string(traj.object_id()));
    }

    // Get the prediction points of current trajectory.
    const auto obs_frenet_pos = GetObstacleTrajectoryPoints(frenet_frame, traj);
    if (obs_frenet_pos.size() < kMinHistoryPoints) {
      continue;
    }
    ASSIGN_OR_CONTINUE(const auto poly,
                       FitPolynomialToData(kPolyFitterDegree, obs_frenet_pos));
    tja_state->target_obs_ids.push_back(std::string(traj.object_id()));
    betas.push_back(poly.coeffs());
    betas.back().front() = 0.0;

    const double weight =
        std::max(0.1, kLatOffsetWeight + 1.0 -
                          std::abs(center_frenet.s / longi_consider_dist) -
                          kLatOffsetWeight *
                              std::abs(min_lat_offset / kMaxLatConsiderDist));
    weights.push_back(weight);
  }

  // 2.2 Find nearest lane path from psmm.
  const auto nearest_lane_path_or =
      FindNearestTargetLanePath(pose, last_center_line, psmm, online_map);
  if (pose.speed() > kMaxEnableTjaSpeed) {
    return absl::InternalError("Exceed speed limit for tja func.");
  }
  if (tja_state->potential_obs_ids.empty() &&
      tja_state->target_obs_ids.empty()) {
    if (!tja_state->planner_use_tja_map ||
        (tja_state->planner_use_tja_map && !nearest_lane_path_or.ok())) {
      return absl::InternalError("No potential leader for tja func.");
    }
  }

  // 3. Compute the weighted polynomial
  if (nearest_lane_path_or.ok()) {
    // Find nearest lane path to approach.
    // Ignore obs traj when there is nearest lane.
    tja_state->target_obs_ids.clear();
    tja_state->potential_obs_ids.clear();
    tja_state->exit_lane_ids.reserve(nearest_lane_path_or->lane_ids_size());
    for (int i = 0; i < nearest_lane_path_or->lane_ids_size(); ++i) {
      tja_state->exit_lane_ids.push_back(nearest_lane_path_or->lane_id(i));
    }
    const auto approach_path =
        ConnectFromEgoToTargetLanePath(pose, *nearest_lane_path_or, psmm);
    auto poly_or = BuildPolynomialInFrenetFrame(approach_path, frenet_frame,
                                                kPolyFitterDegree);
    if (poly_or.ok()) {
      betas.clear();
      weights.clear();
      betas.push_back(poly_or->coeffs());
      weights.push_back(kNearestLaneWeight);
    }
  }
  if (last_center_line.size() > kPolyFitterDegree + 1) {
    // Use last betas to stablize the weighted polynomial.
    auto poly_or = BuildPolynomialInFrenetFrame(last_center_line, frenet_frame,
                                                kPolyFitterDegree);
    if (poly_or.ok()) {
      betas.push_back(poly_or->coeffs());
      betas.back().front() = 0.0;
      weights.push_back(kLastTrajWeight);
    }
  }

  if (betas.empty()) {
    return absl::InternalError("No potential polynomial for tja func.");
  }

  std::vector<double> weighted_beta(betas.front().size(), 0.0);
  double total_weight = 0.0;
  for (int i = 0; i < betas.size(); i++) {
    for (int j = 0; j < weighted_beta.size(); j++) {
      weighted_beta[j] = weighted_beta[j] + betas[i][j] * weights[i];
    }
    total_weight += weights[i];
  }

  if (total_weight < 1e-6) {
    return absl::InternalError("Error total weight for tja func.");
  }

  for (int i = 0; i < weighted_beta.size(); i++) {
    weighted_beta[i] = weighted_beta[i] / total_weight;
  }

  const Polynomial weighted_polynomial(weighted_beta);
  const auto max_angle_vec =
      nearest_lane_path_or.ok()
          ? Vec2d::UnitFromAngle(kMaxCenterLineAngleDiffStepForLane)
          : Vec2d::UnitFromAngle(kMaxCenterLineAngleDiffStepForObs);
  const double max_lat_offset_rate =
      max_angle_vec.y() / std::max(max_angle_vec.x(), 0.001);
  // 4. Generate the weighted polynoimal.
  tja_state->center_line.reserve(kMaxSamplePoints);
  for (int i = 0; i < kMaxSamplePoints; ++i) {
    const double x = (i - kBehindSamplePoints) * kSampleInterval;
    double y = std::clamp(weighted_polynomial.Evaluate(x),
                          -x * max_lat_offset_rate, x * max_lat_offset_rate);
    tja_state->center_line.push_back(
        frenet_frame.SLToXY(FrenetCoordinate{.s = x, .l = y}));
  }

  // 5. Generate the left and right boundary.
  if (nearest_lane_path_or.ok()) {
    BuildBoundaryFromLanePath(psmm, *nearest_lane_path_or, tja_state);
  }

  return absl::OkStatus();
}

absl::StatusOr<std::shared_ptr<const mapping::OnlineSemanticMapProto>>
ConstructTjaOnlineSemanticMap(const PoseProto& pose,
                              const mapping::OnlineSemanticMapProto& origin_map,
                              const TjaState& tja_state) {
  SCOPED_QTRACE("ConstructTjaOnlineSemanticMap");
  // 1. Check input.
  if (tja_state.center_line.size() < kMinCenterLineSize) {
    return absl::NotFoundError("No valid tja center line.");
  }
  // 2. Get online map from lane and boundary
  const auto ego_pos = Vec2dFromPoseProto(pose);
  mapping::OnlineSemanticMapProto online_map_proto;
  online_map_proto.set_update_id(tja_state.update_id);
  online_map_proto.mutable_ego_pos()->set_x(ego_pos.x());
  online_map_proto.mutable_ego_pos()->set_y(ego_pos.y());
  online_map_proto.set_ego_yaw(pose.yaw());

  auto* online_lane = online_map_proto.add_lanes();
  online_lane->set_id(kVirtualLaneId);
  online_lane->mutable_smooth_points()->Reserve(tja_state.center_line.size());
  for (const auto& lane_point : tja_state.center_line) {
    auto* online_lane_pt = online_lane->add_smooth_points();
    online_lane_pt->set_x(lane_point.x());
    online_lane_pt->set_y(lane_point.y());
  }

  if (!tja_state.left_boundary.empty()) {
    online_lane->set_left_boundary_id(kVirtualLeftBoundaryId);
    auto* left_boundary = online_map_proto.add_boundaries();
    left_boundary->set_id(kVirtualLeftBoundaryId);
    left_boundary->set_right_lane_id(kVirtualLaneId);
    left_boundary->mutable_points()->Reserve(tja_state.left_boundary.size());
    for (const auto& boundary : tja_state.left_boundary) {
      auto* online_lane_pt = left_boundary->add_points();
      online_lane_pt->mutable_smooth_point()->set_x(boundary.point.x());
      online_lane_pt->mutable_smooth_point()->set_y(boundary.point.y());
      online_lane_pt->set_type(
          MapStationBoundaryTypeToOnlineBoundaryType(boundary.type));
    }
  }

  if (!tja_state.right_boundary.empty()) {
    online_lane->set_right_boundary_id(kVirtualRightBoundaryId);
    auto* right_boundary = online_map_proto.add_boundaries();
    right_boundary->set_id(kVirtualRightBoundaryId);
    right_boundary->set_left_lane_id(kVirtualLaneId);
    right_boundary->mutable_points()->Reserve(tja_state.right_boundary.size());
    for (const auto& boundary : tja_state.right_boundary) {
      auto* online_lane_pt = right_boundary->add_points();
      online_lane_pt->mutable_smooth_point()->set_x(boundary.point.x());
      online_lane_pt->mutable_smooth_point()->set_y(boundary.point.y());
      online_lane_pt->set_type(
          MapStationBoundaryTypeToOnlineBoundaryType(boundary.type));
    }
  }

  // Fill other fields from origin map.
  online_map_proto.set_timestamp_s(origin_map.timestamp_s());
  online_map_proto.mutable_polylines()->CopyFrom(origin_map.polylines());
  online_map_proto.mutable_polygons()->CopyFrom(origin_map.polygons());
  online_map_proto.mutable_localization_transform()->CopyFrom(
      origin_map.localization_transform());

  return std::make_shared<const mapping::OnlineSemanticMapProto>(
      std::move(online_map_proto));
}

}  // namespace

absl::StatusOr<std::shared_ptr<const mapping::OnlineSemanticMapProto>>
ActivateOnlineSemanticMap(const PoseProto& pose,
                          const SpacetimeTrajectoryManager& st_traj_mgr,
                          const VehicleGeometryParamsProto& veh_geo_params,
                          const PlannerSemanticMapManager& psmm,
                          const mapping::OnlineSemanticMapProto& origin_map,
                          TjaState* tja_state) {
  SCOPED_QTRACE("ActivateOnlineSemanticMap");
  // 1. update tja lane data.
  RETURN_IF_ERROR(UpdateTjaLaneData(pose, st_traj_mgr, veh_geo_params,
                                    origin_map, psmm, tja_state));

  // 2. construct online semantic map.
  return ConstructTjaOnlineSemanticMap(pose, origin_map, *tja_state);
}

bool ShouldUseTjaOnlineSemanticMap(const PoseProto& pose,
                                   const PlannerSemanticMapManager& psmm,
                                   TjaState* tja_state) {
  const bool online_semantic_map_check =
      CheckOnlineSemanticMap(pose, psmm, *tja_state);
  bool should_use_tja_semantic_map = false;
  if (!online_semantic_map_check) {
    should_use_tja_semantic_map = true;
    tja_state->exit_counter = kExitCounterThreshold;
  } else if (!tja_state->planner_use_tja_map) {
    tja_state->exit_counter = 0;
  } else {
    tja_state->exit_counter = std::max(0, tja_state->exit_counter - 1);
    should_use_tja_semantic_map = tja_state->exit_counter != 0;
  }

  return should_use_tja_semantic_map;
}

void FillPlannerCenterLine(
    const google::protobuf::RepeatedPtrField<Vec2dProto>& input_center_line,
    std::vector<Vec2d>* center_line) {
  center_line->clear();
  center_line->reserve(
      std::min(kFillPlannerCenterLineSize, input_center_line.size()));
  for (const auto& point : input_center_line) {
    center_line->emplace_back(point.x(), point.y());
    if (center_line->size() >= kFillPlannerCenterLineSize) break;
  }
}

}  // namespace qcraft::planner
