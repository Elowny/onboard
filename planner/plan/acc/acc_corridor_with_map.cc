
#include "onboard/planner/plan/acc/acc_corridor_with_map.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/control/steering_converter.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/cubic_spline.h"
#include "onboard/math/fitter_def.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/polynomial_fitter.h"
#include "onboard/math/polynomialxd.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"
#include "onboard/planner/plan/acc/acc_defs.h"
#include "onboard/planner/plan/acc/acc_flags.h"
#include "onboard/planner/plan/acc/acc_target_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/pole_placement_util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace planner {

namespace {

const PiecewiseLinearFunction<double, double> kMapCorridorPathBufferTimePLF(
    std::vector<double>{0.0, 3.0, 5.0, 6.0},
    std::vector<double>{0.2, 0.2, 0.5, 1.0});

const PiecewiseLinearFunction<double, double> kMapCorridorOuterBoundaryTimePLF(
    std::vector<double>{0.0, 0.25, 0.5, 1.0, 2.0},
    std::vector<double>{0.0, 0.1, 0.2, 1.0, 4.0});

constexpr double kSoftBoundaryEpsilon = 0.1;  // m.
constexpr double kHardBoundaryEpsilon = 2.0;  // m.

// TODO(changqing): calculate target lateral offset based on lateral velocity
// and heading.
constexpr double kMinCorridorLength =
    1.0;  // At least 1 meter length for map based corridor.

// For generating pole placement reference path.
constexpr double kNominalDtForPath = 0.2;  // s.

const PiecewiseLinearFunction<double, double> kVelocityKphCredibleLengthPLF(
    std::vector<double>{0.0, 65.0, 70.0, 95.0, 100.0, 145.0, 150.0, 195.0,
                        200.0},
    std::vector<double>{50.0, 100.0, 120.0, 140.0, 180.0, 180.0, 200.0, 200.0,
                        200.0});

struct PotentialLane {
  mapping::ElementId id;
  mapping::LanePoint lane_pt;
  Vec2d lane_pt_pos;
  mapping::LanePath lane_path;
  DiscretizedPath smoothed_path;
  double distance = 0.0;  // Signed distance. (AV's right: positive).
  double cost = 0.0;

  bool IsDifferentFrom(const PotentialLane& o) const {
    constexpr double kSameLaneDistanceThreshold = 1.0;
    const auto o_start_sl = smoothed_path.XYToSL(o.lane_pt_pos);
    const auto t_start_sl = o.smoothed_path.XYToSL(lane_pt_pos);
    if (std::fabs(o_start_sl.l) > kSameLaneDistanceThreshold ||
        std::fabs(t_start_sl.l) > kSameLaneDistanceThreshold) {
      return true;
    }
    // Compare end points (sl coordinates)
    constexpr double kEndPointLateralDistanceThreshold = 2.0;
    const Vec2d o_end_pt_pos =
        Vec2d(o.smoothed_path.back().x(), o.smoothed_path.back().y());
    const Vec2d t_end_pt_pos =
        Vec2d(smoothed_path.back().x(), smoothed_path.back().y());

    const auto o_end_sl = smoothed_path.XYToSL(o_end_pt_pos);
    const auto t_end_sl = o.smoothed_path.XYToSL(t_end_pt_pos);
    if (std::fabs<double>(o_end_sl.l) > kEndPointLateralDistanceThreshold ||
        std::fabs<double>(t_end_sl.l) > kEndPointLateralDistanceThreshold) {
      return true;
    }
    return false;
  }

  std::string DebugString() const {
    return absl::StrFormat(
        "potential lane: %d, lane_path length: %.5f, lanes: %s. Distance %.5f, "
        "cost: %.5f.",
        id, lane_path.length(), absl::StrJoin(lane_path.lane_ids(), ","),
        distance, cost);
  }
};

void SendUniCycleStatesToCanvas(absl::Span<UniCycleState> states,
                                const std::string& channel,
                                const vis::Color& color) {
  std::vector<Vec2d> pos;
  pos.reserve(states.size());
  for (const auto& state : states) {
    pos.push_back(Vec2d(state.x, state.y));
  }
  SendPointsToCanvas(absl::MakeSpan(pos), channel, color);
}

ApolloTrajectoryPointProto ConvertToTrajPointProto(const PoseProto& pose) {
  ApolloTrajectoryPointProto traj_point;
  traj_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  traj_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  traj_point.mutable_path_point()->set_z(pose.pos_smooth().z());
  traj_point.mutable_path_point()->set_theta(pose.yaw());
  traj_point.mutable_path_point()->set_kappa(pose.curvature());
  traj_point.mutable_path_point()->set_lambda(0.0);
  traj_point.mutable_path_point()->set_s(0.0);
  traj_point.set_v(pose.vel_body().x());
  traj_point.set_a(
      std::hypot(pose.accel_smooth().x(), pose.accel_smooth().y()));
  traj_point.set_j(0.0);
  traj_point.set_relative_time(0.0);

  return traj_point;
}

}  // namespace

namespace internal {

// For polynomial fitter.
constexpr int kPolynomialDegree = 5;

absl::Status FindClosestLanePointToSmoothPointOnLane(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    const mapping::ElementId& lane_id, std::optional<double> max_allow_distance,
    mapping::LanePoint* lane_point, Vec2d* pos, Vec2d* tangent,
    double* signed_distance) {
  const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(lane_id);
  if (lane_info_ptr == nullptr) {
    return absl::NotFoundError(
        absl::StrFormat("Can not find lane info for lane_id: %d", lane_id));
  }
  const auto& lane_info = *lane_info_ptr;
  const auto& lane_pts = lane_info.points_smooth;
  const auto& cumulative_lengths = lane_info.cumulative_lengths;
  double min_d_cumulative_length = 0.0;
  Vec2d min_d_closest_point;
  double min_d_sqr = std::numeric_limits<double>::infinity();
  Vec2d min_d_tangent;
  for (int j = 0; j < lane_pts.size() - 1; ++j) {
    const auto& p0 = lane_pts[j];
    const auto& p1 = lane_pts[j + 1];
    const Vec2d segment = p1 - p0;
    const double l_sqr = segment.squaredNorm();
    const Vec2d query_segment = query_point - p0;
    const double projection = query_segment.dot(segment) / l_sqr;
    double d_sqr;
    double cumulative_length;
    Vec2d closest_point;
    if (projection < 0.0) {
      d_sqr = p0.DistanceSquareTo(query_point);
      closest_point = p0;
      cumulative_length = cumulative_lengths[j];
    } else if (projection > 1.0) {
      d_sqr = p1.DistanceSquareTo(query_point);
      closest_point = p1;
      cumulative_length = cumulative_lengths[j + 1];
    } else {
      d_sqr = Sqr(query_segment.CrossProd(segment.normalized()));
      closest_point = Lerp(p0, p1, projection);
      cumulative_length =
          Lerp(cumulative_lengths[j], cumulative_lengths[j + 1], projection);
    }

    if (min_d_sqr > d_sqr) {
      min_d_sqr = d_sqr;
      min_d_closest_point = closest_point;
      min_d_cumulative_length = cumulative_length;
      min_d_tangent = segment.normalized();
    }
  }
  if (min_d_sqr == std::numeric_limits<double>::infinity()) {
    return absl::NotFoundError(
        absl::StrFormat("Can not find closest lane point from %s to %d.",
                        query_point.DebugString(), lane_id));
  }
  const double fraction = min_d_cumulative_length / lane_info.length();

  // Calculate signed distance (left positive, right negative).
  const double distance = min_d_closest_point.DistanceTo(query_point);
  if (max_allow_distance.has_value() && distance > *max_allow_distance) {
    return absl::NotFoundError(
        "Lane projection point has distance greater than max allowed "
        "distance to query point.");
  }
  const Vec2d lane_pt_to_query_pt = query_point - min_d_closest_point;

  if (lane_point != nullptr) {
    *lane_point = mapping::LanePoint(lane_id, fraction);
  }
  if (pos != nullptr) {
    *pos = min_d_closest_point;
  }
  if (tangent != nullptr) {
    *tangent = min_d_tangent;
  }
  if (signed_distance != nullptr) {
    *signed_distance =
        std::copysign(distance, min_d_tangent.CrossProd(lane_pt_to_query_pt));
  }
  return absl::OkStatus();
}

double EstimateMinDrivePassageDivergeLength(const DrivePassage& dp_base,
                                            const DrivePassage& dp_target,
                                            const Vec2d& start_point) {
  FUNC_QTRACE();
  StationIndex base_idx = dp_base.FindNearestStationIndex(start_point);
  StationIndex target_idx = dp_target.FindNearestStationIndex(start_point);
  double accum_s = 0.0;
  Vec2d ref_point = start_point;
  while (base_idx <= dp_base.last_real_station_index() &&
         target_idx <= dp_target.last_real_station_index()) {
    const auto& base_station = dp_base.station(base_idx);
    const auto& target_station = dp_target.station(target_idx);
    const double dp_distance =
        base_station.xy().DistanceTo(target_station.xy());
    const auto curr_ref_point = (base_station.xy() + target_station.xy()) * 0.5;
    accum_s += curr_ref_point.DistanceTo(ref_point);
    if (dp_distance > kDefaultHalfLaneWidth) {
      constexpr double kDrivePassageDivergeLengthBuffer = 5.0;
      return accum_s + kDrivePassageDivergeLengthBuffer;
    }
    ref_point = curr_ref_point;
    base_idx++;
    target_idx++;
  }
  return accum_s;
}

double ComputeDrivePassageCost(const DrivePassage& dp,
                               absl::Span<const UniCycleState> probing_traj) {
  constexpr double kSigmoidXAxisFactor = 2.0;  // Sigmoid(1) ~ 0.73.
  constexpr double kSigmoidXAxisTranslation = -0.5;
  constexpr double kInvDefaultHalfLaneWidth = 1.0 / kDefaultHalfLaneWidth;
  constexpr double kEpsilon = 1e-6;
  constexpr double kProjectionFailureCost = 100.0;
  double cost = 0.0;
  int num = 0;  // Valid probing trajectory projection.
  for (int i = 0; i < probing_traj.size(); ++i) {
    const auto& state = probing_traj[i];
    const auto sl_or =
        dp.QueryUnboundedFrenetCoordinateAt(Vec2d(state.x, state.y));
    if (sl_or.ok() && sl_or->s <= (dp.end_s() + kEpsilon) &&
        sl_or->s >= (dp.front_s() - kEpsilon)) {
      // Point should be longitudinally bounded.
      const double l_regularized =
          std::fabs(sl_or->l) * kInvDefaultHalfLaneWidth;
      num++;
      cost += Sigmoid(kSigmoidXAxisFactor *
                      (l_regularized + kSigmoidXAxisTranslation));
    }
  }
  const double avg_cost = cost / (num + kEpsilon);
  VLOG(1) << absl::StrFormat(
      "Compute drive passage cost for lane path: [%s], valid/all num: %d/%d, "
      "avg_cost: %.5f.",
      absl::StrJoin(dp.lane_path().lane_ids(), ","), num, probing_traj.size(),
      avg_cost);
  return num == 0 ? kProjectionFailureCost : avg_cost;
}

std::vector<UniCycleState> GenerateProbingTrajs(
    const PoseProto& pose, const VehicleGeometryParamsProto& vehicle_geom,
    double ds, double desire_length) {
  constexpr double kProbingTrajTimeHorizon = 2.0;  // s.
  constexpr double kProbingTrajMinLength = 10.0;   // m.
  constexpr int kMinProbingTrajStateNum = 5;
  constexpr double kLowSpeedThreshold = Kph2Mps(5);  // 5 kmh.
  desire_length = ds * kMinProbingTrajStateNum + desire_length;
  double length = std::max<double>(kProbingTrajMinLength,
                                   pose.speed() * kProbingTrajTimeHorizon);
  length = std::max<double>(length, desire_length);
  if (pose.speed() < kLowSpeedThreshold) {
    length = kProbingTrajMinLength;
  }
  const int traj_size = FloorToInt(length / ds) + 1;
  std::vector<UniCycleState> traj;
  traj.reserve(traj_size);
  std::vector<UniCycleState> front_center_traj;
  front_center_traj.reserve(traj_size);

  UniCycleState curr_state{
      .x = pose.pos_smooth().x(),
      .y = pose.pos_smooth().y(),
      .heading = pose.yaw(),
      .kappa = pose.curvature(),
  };
  double s = 0.0;
  for (int i = 0; i < traj_size; ++i) {
    traj.push_back(curr_state);
    const double kappa_decay = 1.0 - s / length;
    curr_state = SimulateUniCycleState(curr_state, ds, kappa_decay);
    const auto curr_front_center = ComputeAvFrontCenter(
        Vec2d(curr_state.x, curr_state.y), curr_state.heading, vehicle_geom);
    auto front_curr_state = curr_state;
    front_curr_state.x = curr_front_center.x();
    front_curr_state.y = curr_front_center.y();
    front_center_traj.push_back(front_curr_state);
    s += ds;
  }
  if (FLAGS_planner_acc_corridor_draw_target_lane_choice_debug) {
    SendUniCycleStatesToCanvas(absl::MakeSpan(front_center_traj),
                               "with_map/lane_choice/front_center_probing_traj",
                               vis::Color::kDarkRed);
  }
  return front_center_traj;
}

void GetLaneBoundaryPointsAndWidth(const PlannerSemanticMapManager& psmm,
                                   const mapping::ElementId& lane_id,
                                   const Vec2d& pos, const Vec2d& tangent,
                                   Vec2d* right, Vec2d* left, double* width) {
  const auto left_width = psmm.GetLeftLaneWidth(pos, lane_id);
  const auto right_width = psmm.GetRightLaneWidth(pos, lane_id);
  const auto left_offset = left_width.value_or(kDefaultHalfLaneWidth);
  const auto right_offset = -right_width.value_or(kDefaultHalfLaneWidth);
  *right = LatPoint(pos, tangent, right_offset);
  *left = LatPoint(pos, tangent, left_offset);
  *width = right->DistanceTo(*left);
}

Box2d BuildLaneBoxApproxByProjection(const PlannerSemanticMapManager& psmm,
                                     const mapping::ElementId& start_lane_id,
                                     const mapping::ElementId& end_lane_id,
                                     const Vec2d& start_pos,
                                     const Vec2d& end_pos,
                                     const Vec2d& start_tangent,
                                     const Vec2d& end_tangent) {
  Vec2d start_left, start_right, end_left, end_right;
  double start_width, end_width;
  GetLaneBoundaryPointsAndWidth(psmm, start_lane_id, start_pos, start_tangent,
                                &start_right, &start_left, &start_width);
  GetLaneBoundaryPointsAndWidth(psmm, end_lane_id, end_pos, end_tangent,
                                &end_right, &end_left, &end_width);
  const auto start_center = 0.5 * (start_left + start_right);
  const auto end_center = 0.5 * (end_left + end_right);

  Box2d raw_approx_box(Segment2d(start_center, end_center),
                       0.5 * (start_width + end_width));
  // Maybe need expansion?
  return raw_approx_box;
}

std::optional<Box2d> BuildLaneBoxApprox(const PlannerSemanticMapManager& psmm,
                                        const Vec2d& start_pt,
                                        const Vec2d& end_pt,
                                        mapping::ElementId lane_id) {
  mapping::LanePoint start_lp, end_lp;
  Vec2d start_tangent, end_tangent;
  Vec2d start_pos, end_pos;
  const auto start_proj = FindClosestLanePointToSmoothPointOnLane(
      psmm, start_pt, lane_id, std::nullopt, &start_lp, &start_pos,
      &start_tangent, /*signed_distance=*/nullptr);
  const auto end_proj = FindClosestLanePointToSmoothPointOnLane(
      psmm, end_pt, lane_id, std::nullopt, &end_lp, &end_pos, &end_tangent,
      /*signed_distance=*/nullptr);
  if (!start_proj.ok() || !end_proj.ok()) return std::nullopt;
  return BuildLaneBoxApproxByProjection(psmm, start_lp.lane_id(),
                                        end_lp.lane_id(), start_pos, end_pos,
                                        start_tangent, end_tangent);
}

bool CanFindClosestSlowObjectOnPathInRegionBox(
    absl::Span<const SpacetimeObjectTrajectory> trajectories,
    const Box2d& region_box, const QtfmEnhancedKdTreeFrenetFrame& path_ff,
    double av_max_s, double low_speed_threshold, Vec2d* obj_closest_pt) {
  constexpr double kMaxLaneObjectHeadingDiff = M_PI / 4.0;
  Vec2d obj_pt;
  double obj_min_s = std::numeric_limits<double>::infinity();
  bool obj_found = false;
  for (const auto& st_traj : trajectories) {
    const auto& obj_box = st_traj.bounding_box();
    const auto& obj_speed = st_traj.pose().v();
    const auto& box_center = obj_box.center();
    const auto box_path_heading_diff =
        std::fabs(NormalizeAngle(obj_box.heading() - region_box.heading()));
    if (region_box.IsPointIn(box_center) && obj_speed < low_speed_threshold &&
        box_path_heading_diff < kMaxLaneObjectHeadingDiff) {
      // Object center should be within lane box and at low speed and heading
      // diff with current lane box shouldn't exceed 45 deg.
      for (const auto& corner : obj_box.GetCornersCounterClockwise()) {
        const auto corner_sl = path_ff.XYToSL(corner);
        if (corner_sl.s < obj_min_s && corner_sl.s > av_max_s) {
          obj_min_s = corner_sl.s;
          obj_pt = corner;
          obj_found = true;
        }
      }
    }
  }
  if (obj_found && obj_closest_pt != nullptr) {
    *obj_closest_pt = obj_pt;
  }
  return obj_found;
}

// Cut-in should consider path boundaries and overlap rate of lanes.
std::set<std::string> FindSlowCutinObjectsInRegionBox(
    absl::Span<const SpacetimeObjectTrajectory> trajectories,
    const Box2d& region_box, const PathSlBoundary& path_sl,
    const QtfmEnhancedKdTreeFrenetFrame& path_ff, double low_speed_threshold) {
  std::set<std::string> object_ids;
  for (const auto& st_traj : trajectories) {
    if (st_traj.object_type() != ObjectType::OT_VEHICLE &&
        st_traj.object_type() != ObjectType::OT_LARGE_VEHICLE) {
      // Only consider vehicles.
      continue;
    }
    const auto& obj_box = st_traj.bounding_box();
    const auto& obj_speed = st_traj.pose().v();
    if (obj_speed < low_speed_threshold && region_box.HasOverlap(obj_box)) {
      // Continue to check object type, OnPathType, lane overlap rate
      // (slightly over lane boundary).
      const auto obj_fbox_or = path_ff.QueryFrenetBoxAt(obj_box);
      if (!obj_fbox_or.ok()) continue;

      // Convert fbox on frenet frame coordinate to path sl coord. (start_s
      // offset)
      auto obj_fbox_pathsl = *obj_fbox_or;
      obj_fbox_pathsl.s_max += path_sl.start_s();
      obj_fbox_pathsl.s_min += path_sl.start_s();

      const auto on_path_type = ObjectFrenetBoxOnPathType(
          path_sl, obj_fbox_pathsl, /*lateral_distance_buffer=*/0.0);
      if (on_path_type != OnPathType::OPT_TARGET_BOUND) continue;
      const auto lane_overlap_or = ComputeBoxLaneInvasionPercentage(
          path_sl, path_ff, obj_box, /*use_obj_width=*/false);
      if (!lane_overlap_or.ok()) continue;
      constexpr double kCrowdedSceneSideObjectMaxLaneOverlap = 0.15;
      if (*lane_overlap_or < kCrowdedSceneSideObjectMaxLaneOverlap) {
        // On target bound && lane overlap < 15% wrt. lane width.
        object_ids.insert(std::string(st_traj.object_id()));
      }
    }
  }
  return object_ids;
}

std::pair<const mapping::LanePath*, mapping::ElementId>
FindClosestLanePathAtPositionOrNull(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const mapping::LanePath> lane_paths, const Vec2d& query_pos) {
  double min_dist = std::numeric_limits<double>::infinity();
  const mapping::LanePath* closest_lane_path = nullptr;
  mapping::ElementId closest_lane_id;
  for (const auto& lane_path : lane_paths) {
    for (const auto& lane_id : lane_path.lane_ids()) {
      double signed_distance = std::numeric_limits<double>::infinity();
      const auto lane_projection_status =
          FindClosestLanePointToSmoothPointOnLane(
              psmm, query_pos, lane_id, std::nullopt,
              /*lane_point=*/nullptr,
              /*pos=*/nullptr, /*tangent=*/nullptr, &signed_distance);
      if (!lane_projection_status.ok()) continue;
      if (std::fabs(signed_distance) < min_dist) {
        min_dist = std::fabs(signed_distance);
        closest_lane_path = &lane_path;
        closest_lane_id = lane_id;
      }
    }
  }
  return std::make_pair(closest_lane_path, closest_lane_id);
}

// According to the position relation between AV current pose and drive
// passage. Postpone converging to lane center if AV is moving away from
// center otherwise track lane center offset directly.
absl::StatusOr<PiecewiseLinearFunction<double, double>>
ComputeTimeLateralOffsetPlfFromPose(const DrivePassage& drive_passage,
                                    const PoseProto& pose) {
  constexpr double kAVLowSpeedThreshold = 10.0 / 3.6;  // 10kmh.
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  const auto path_heading_or = drive_passage.QueryTangentAt(pose_pos);
  if (!path_heading_or.ok()) {
    return absl::InternalError(
        absl::StrFormat("Cannot find heading on path at av position: %s",
                        path_heading_or.status().message()));
  }
  const auto pose_sl_or = drive_passage.QueryFrenetCoordinateAt(pose_pos);
  if (!pose_sl_or.ok()) {
    return absl::InternalError(
        "Acc corridor with map: cannot project pose point on drive "
        "passage.");
  }
  const auto av_velocity = Vec2d(pose.vel_smooth().x(), pose.vel_smooth().y());
  const auto av_lateral_v = path_heading_or->CrossProd(av_velocity);
  constexpr double kMotionPreviewTime = 1.0;
  constexpr double kMinPreviewDistance = 5.0;
  auto motion_preview_distance =
      std::max<double>(pose.speed() * kMotionPreviewTime, kMinPreviewDistance);
  // Do not preview beyond drive passage (longitudinally). It will result in
  // incorrect lateral distance computation.
  motion_preview_distance = std::min<double>(
      drive_passage.end_s() - pose_sl_or->s, motion_preview_distance);
  PathPoint pose_pt;
  pose_pt.set_x(pose_pos.x());
  pose_pt.set_y(pose_pos.y());
  pose_pt.set_theta(pose.yaw());
  pose_pt.set_s(0.0);
  pose_pt.set_kappa(pose.speed() < kAVLowSpeedThreshold ? 0.0
                                                        : pose.curvature());
  pose_pt.set_lambda(0.0);
  // Forward preview 1.0 seconds from current pose.
  const auto preview_pt_from_pose =
      GetPathPointAlongCircle(pose_pt, motion_preview_distance);
  const auto preview_pt_pos =
      Vec2d(preview_pt_from_pose.x(), preview_pt_from_pose.y());
  const auto preview_sl_or =
      drive_passage.QueryFrenetCoordinateAt(preview_pt_pos);
  if (!preview_sl_or.ok()) {
    return absl::InternalError(
        "Acc corridor with map: cannot project preview point on drive "
        "passage.");
  }
  const bool near_or_converge =
      IsAVConvergeOrNearCenter(av_lateral_v, pose_sl_or->l);
  if (!near_or_converge) {
    constexpr double kOffCenterStartConvergeTime = 1.0;
    constexpr double kOffCenterToPreviewOffsetTime = 3.0;
    std::vector<double> ts{
        0.0, kOffCenterToPreviewOffsetTime,
        kOffCenterToPreviewOffsetTime + kOffCenterStartConvergeTime};
    const double preview_l =
        preview_sl_or->l * pose_sl_or->l < 0.0 ? 0.0 : preview_sl_or->l;
    std::vector<double> l_offsets{preview_l, preview_l, 0.0};
    return PiecewiseLinearFunction<double, double>(ts, l_offsets);
  }
  return PiecewiseLinearFunction<double, double>(
      {0.0, kAccTrajectoryTimeHorizon}, {0.0, 0.0});
}

double ComputeCurvatureCost(absl::Span<const PathPoint> pts,
                            double ref_curvature, double look_forward_length) {
  constexpr double kCurvatureRegularizer = 0.01;  // 1/m.
  constexpr double kInvCurvatureRegularizer = 1.0 / kCurvatureRegularizer;
  constexpr double kEpsilon = 1e-6;
  double accum_cost = 0.0;
  int count = 0;
  const double inv_look_forward_length = 1.0 / look_forward_length;
  for (const auto& pt : pts) {
    if (pt.s() >= look_forward_length) break;
    count++;
    double weight = pt.s() * inv_look_forward_length;
    double cost =
        (ref_curvature - pt.kappa()) * kInvCurvatureRegularizer * weight;
    accum_cost += Sqr(cost);
  }
  return accum_cost / (static_cast<double>(count) + kEpsilon);
}

std::optional<DiscretizedPath> SampleSmoothedPathOnLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double start_s, double sample_step_s) {
  constexpr int kPathSmoothingMinSamplePointNum = kPolynomialDegree + 1;
  constexpr double kLanePathSampleStepS = 2.0;

  const double length = lane_path.length() - start_s;
  if (length < sample_step_s) return std::nullopt;
  const int num_lane_step =
      std::max(static_cast<int>(length / kLanePathSampleStepS),
               kPathSmoothingMinSamplePointNum);
  const double lane_sample_step_s = length / num_lane_step;

  std::vector<double> accum_s;
  std::vector<Vec2d> xys;
  accum_s.reserve(num_lane_step);
  xys.reserve(num_lane_step);

  for (int i = 0; i < num_lane_step; ++i) {
    const double query_s_offset =
        std::min<double>(length, i * lane_sample_step_s);
    Vec2d xy = ArclengthToPos(psmm, lane_path, query_s_offset + start_s);
    xys.push_back(xy);
    accum_s.push_back(query_s_offset);
  }

  return ResampleSmoothedDiscretizedPathOnXY(xys, accum_s, sample_step_s);
}

std::set<mapping::ElementId> ExpandLaneIdsBySemantic(
    absl::Span<const mapping::LaneInfo* const> lanes) {
  std::set<mapping::ElementId> all_lanes;
  // Expand lane to build by semantics.
  for (const auto* lane : lanes) {
    if (lane == nullptr) continue;
    const auto& curr_lane_proto = *lane->proto;
    if (lane->IsVirtual() || (curr_lane_proto.has_is_in_intersection() &&
                              curr_lane_proto.is_in_intersection())) {
      continue;
    }
    all_lanes.insert(lane->id);
    // Left lanes.
    const auto& left_lanes = curr_lane_proto.lane_neighbors_on_left();
    for (const auto& left_lane : left_lanes) {
      if (!left_lane.has_other_id()) continue;
      all_lanes.insert(mapping::ElementId(left_lane.other_id()));
    }
    // Right lanes.
    const auto& right_lanes = curr_lane_proto.lane_neighbors_on_right();
    for (const auto& right_lane : right_lanes) {
      if (!right_lane.has_other_id()) continue;
      all_lanes.insert(mapping::ElementId(right_lane.other_id()));
    }
  }
  return all_lanes;
}

absl::StatusOr<std::unique_ptr<DrivePassage>>
DetermineTargetDrivePassageFromDrivingMap(
    const PoseProto& pose, const PlannerSemanticMapManager& psmm,
    const DrivingMapTopo& dm, const VehicleGeometryParamsProto& vehicle_geom,
    const VehicleDriveParamsProto& vehicle_drive,
    const std::optional<double>& steering_percentage, double dp_step_s,
    double credible_length) {
  FUNC_QTRACE();

  // compute current steer angle if available.
  std::optional<double> steer_angle = std::nullopt;
  if (steering_percentage.has_value()) {
    control::SteeringConverter steering_converter(vehicle_geom, vehicle_drive);
    steer_angle = steering_converter.SteerPctToSteerAngle(*steering_percentage);
    constexpr double kLaneKeepManueverSteerAngleDeadZone = 5.0 / 180.0 * M_PI;
    if (std::fabs<double>(*steer_angle) <=
        kLaneKeepManueverSteerAngleDeadZone) {
      steer_angle = 0.0;
    }
  }

  const auto pose_pos = Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y());
  std::vector<std::pair<mapping::ElementId, double>> id_dist;
  id_dist.reserve(dm.starting_lane_ids().size());
  std::vector<std::string> filter_reason;
  // Filter starting lanes laterally too far.
  for (const auto& start_id : dm.starting_lane_ids()) {
    const auto* dm_lane_ptr = dm.GetLaneById(start_id);
    if (dm_lane_ptr == nullptr) continue;
    const auto* start_lane_info = psmm.FindLaneInfoOrNull(start_id);
    if (start_lane_info == nullptr) continue;
    const auto pose_sl_on_lane = start_lane_info->SmoothXYToSL(pose_pos);
    if (pose_sl_on_lane.s < 0.0) {
      // Start point in front of pose or end point behind pose.
      filter_reason.push_back(absl::StrFormat(
          "%d: pose_sl_on_lane.s %.3f, lane length: %.3f", start_id,
          pose_sl_on_lane.s, start_lane_info->length()));
      continue;
    }
    id_dist.push_back({start_id, pose_sl_on_lane.l});
  }

  // Lane from left to right.
  std::sort(id_dist.begin(), id_dist.end(),
            [](const std::pair<mapping::ElementId, double>& lane_1,
               const std::pair<mapping::ElementId, double>& lane_2) {
              return lane_1.second < lane_2.second;
            });

  // Find candidate lane keeping lane id in a lateral range +/- 3.0m.
  std::vector<std::pair<mapping::ElementId, double>> candidate_lk_lane;
  constexpr double kLaneKeepingLaneLateralDistance = 3.0;
  for (const auto& [id, dist] : id_dist) {
    if (std::fabs(dist) < kLaneKeepingLaneLateralDistance) {
      filter_reason.push_back(absl::StrFormat("%d: dist: %.3f.", id, dist));
      candidate_lk_lane.push_back({id, std::fabs(dist)});
    }
  }
  if (candidate_lk_lane.empty()) {
    return absl::NotFoundError(absl::StrFormat(
        "Can not find any candidate lk lane. filtered lane id: %s",
        absl::StrJoin(filter_reason, ",")));
  }
  const auto min_dist_lk =
      std::min_element(candidate_lk_lane.begin(), candidate_lk_lane.end(),
                       [](const std::pair<mapping::ElementId, double>& lane_1,
                          const std::pair<mapping::ElementId, double>& lane_2) {
                         return lane_1.second < lane_2.second;
                       });
  int lk_idx = 0;
  while (lk_idx < id_dist.size()) {
    if (id_dist[lk_idx].first == min_dist_lk->first) {
      break;
    }
    lk_idx++;
  }
  std::set<mapping::ElementId> final_start_ids;
  // Current lk lane path.
  final_start_ids.insert(id_dist[lk_idx].first);

  // If we have steer angle is large enough (driver has lane-changing
  // intention), take corresponding left/right lane paths into consideration.
  // If we are at low speed, consider both left and right since the vehicle
  // might be have large heading diff from lane paths.

  constexpr double kEgoLowSpeedThreshold = Kph2Mps(10.0);
  if (!steer_angle.has_value() ||
      (steer_angle.has_value() && *steer_angle > 0.0) ||
      pose.vel_body().x() < kEgoLowSpeedThreshold) {
    const int left_idx = std::max<int>(lk_idx - 1, 0);
    final_start_ids.insert(id_dist[left_idx].first);
  }
  if (!steer_angle.has_value() ||
      (steer_angle.has_value() && *steer_angle < 0.0) ||
      pose.vel_body().x() < kEgoLowSpeedThreshold) {
    const int right_idx =
        std::min<int>(lk_idx + 1, static_cast<int>(id_dist.size()) - 1);
    final_start_ids.insert(id_dist[right_idx].first);
  }

  // Collect all lane paths with desired credible length.
  std::vector<mapping::LanePath> all_lane_paths;
  for (const auto& start_id : final_start_ids) {
    auto lane_paths =
        BuildAllLanePathsFromStartIdInDrivingMapTopoWithDesireLength(
            dm, psmm, start_id, credible_length, kMinCorridorLength);
    std::move(lane_paths.begin(), lane_paths.end(),
              std::back_inserter(all_lane_paths));
  }
  if (FLAGS_planner_acc_corridor_draw_target_lane_choice_debug) {
    DrawLanePathsToCanvas(psmm, absl::MakeSpan(all_lane_paths),
                          "with_map/lane_choice/driving_map_lps",
                          vis::Color::kSalmonPink);
  }
  if (all_lane_paths.empty()) {
    return absl::NotFoundError(
        "Can not build any lane paths with enough length.");
  }

  // Build drive passages for all lane paths and compute min probing length.
  std::vector<std::pair<std::unique_ptr<DrivePassage>, double>> dp_costs;
  for (const auto& lp : all_lane_paths) {
    auto dp_or = BuildDrivePassageFromLanePath(
        psmm, lp, dp_step_s, /*avoid_loop=*/true, kDrivePassageKeepBehindLength,
        /*required_planning_horizon=*/0.0, /*required_backward_len=*/0.0,
        std::nullopt);
    if (dp_or.ok()) {
      dp_costs.push_back(
          std::make_pair(std::make_unique<DrivePassage>(std::move(*dp_or)),
                         std::numeric_limits<double>::max()));
    }
  }
  double min_probing_length = 0.0;
  for (int i = 0; i < dp_costs.size(); ++i) {
    for (int j = i + 1; j < dp_costs.size(); ++j) {
      min_probing_length = std::max<double>(
          min_probing_length,
          internal::EstimateMinDrivePassageDivergeLength(
              *dp_costs[i].first, *dp_costs[j].first, pose_pos));
    }
  }
  const auto probing_traj = internal::GenerateProbingTrajs(
      pose, vehicle_geom, dp_step_s, min_probing_length);
  for (auto& pair : dp_costs) {
    pair.second = internal::ComputeDrivePassageCost(*pair.first, probing_traj);
    VLOG(1) << absl::StrFormat(
        "drive passage build from lane path[%s] cost: %.5f.",
        pair.first->lane_path().DebugString(), pair.second);
  }
  auto min_iter = std::min_element(
      dp_costs.begin(), dp_costs.end(),
      [](const std::pair<std::unique_ptr<DrivePassage>, double>& dp1,
         const std::pair<std::unique_ptr<DrivePassage>, double>& dp2) {
        return dp1.second < dp2.second;
      });

  constexpr double kDrivePassageCostThreshold = 0.8;
  if (min_iter->second > kDrivePassageCostThreshold) {
    return absl::NotFoundError(
        "Chosen DrivePassage has cost greater than max allowed cost.");
  }
  return std::move(min_iter->first);
}

}  // namespace internal

absl::StatusOr<AccCorridor> BuildAccCorridorFromClosestLanePath(
    const PoseProto& pose, const ApolloTrajectoryPointProto& plan_start_point,
    const PlannerSemanticMapManager& psmm, const DrivingMapTopo& dm,
    const std::optional<double>& steering_percentage,
    const VehicleGeometryParamsProto& vehicle_geom,
    const VehicleDriveParamsProto& vehicle_drive, double corridor_step_s,
    double total_preview_time) {
  FUNC_QTRACE();
  // 1. Get left and right lane centers. Build possible drive passages on left,
  // current, right lane. Use probing trajs to calculate cost and choose final
  // DrivePassage for corridor construction.
  const double credible_length =
      kVelocityKphCredibleLengthPLF(Mps2Kph(plan_start_point.v()));
  std::unique_ptr<DrivePassage> dp_ptr = nullptr;
  constexpr double kDrivePassageStepS = 1.0;
  // Build dp from dm first.
  auto dp_or = internal::DetermineTargetDrivePassageFromDrivingMap(
      pose, psmm, dm, vehicle_geom, vehicle_drive, steering_percentage,
      kDrivePassageStepS, credible_length);
  if (dp_or.ok()) {
    dp_ptr = std::move(*dp_or);
  } else {
    return absl::NotFoundError(
        absl::StrCat("Determine target drive passage from driving map fails.",
                     dp_or.status().message()));
  }
  // When building corridor, use plan start point to avoid discontinuity
  // between frames.
  const Vec2d av_pos(plan_start_point.path_point().x(),
                     plan_start_point.path_point().y());
  const auto& drive_passage = *dp_ptr;
  const auto av_frenet_or = drive_passage.QueryFrenetCoordinateAt(av_pos);
  if (!av_frenet_or.ok()) {
    return absl ::NotFoundError(absl::StrFormat(
        "Cannot project plan start point position onto target "
        "drive passage by lane path: %s, error message: %s",
        absl::StrJoin(drive_passage.lane_path().lane_ids(), ","),
        av_frenet_or.status().message()));
  }
  const auto& av_frenet_on_dp = av_frenet_or.value();

  // 2. Develop a smooth reference path converging to target lane.
  const prediction::BicycleModelState plan_start_state{
      .x = plan_start_point.path_point().x(),
      .y = plan_start_point.path_point().y(),
      .v = plan_start_point.v(),
      .heading = plan_start_point.path_point().theta(),
      .acc = plan_start_point.a(),
      .front_wheel_angle = 0.0,  // Use VehicleControlStatusProto later.
  };

  ASSIGN_OR_RETURN(
      const auto t_offset_plf,
      internal::ComputeTimeLateralOffsetPlfFromPose(drive_passage, pose));

  const double av_length = vehicle_geom.length();
  constexpr double kAccCorridorNominalHorizon = 20.0;  // s.
  const double max_steer =
      vehicle_drive.max_steer_angle() / vehicle_drive.steer_ratio();
  const double& wheelbase = vehicle_geom.wheel_base();

  ASSIGN_OR_RETURN(
      const auto ref_path,
      prediction::DevelopPolePlacementPathWithSmoothConvergence(
          plan_start_state, drive_passage, t_offset_plf, kNominalDtForPath,
          kAccCorridorNominalHorizon, av_length, max_steer, wheelbase));
  if (FLAGS_planner_acc_corridor_draw_target_lane_ref_path) {
    SendDiscretizedPathToCanvas(ref_path, "poleplacement_path",
                                vis::Color::kLightBlue);
  }

  // 3. Build PathSlBoundary as well as use reference path as ref path center.
  const double length =
      std::min(credible_length, drive_passage.end_s() - av_frenet_on_dp.s);
  if (length < kMinCorridorLength) {
    return absl::NotFoundError("Insufficient corridor length");
  }

  auto map_sl_boundary_or =
      BuildPathBoundaryFromDrivePassage(psmm, drive_passage);
  if (!map_sl_boundary_or.ok()) {
    return absl::NotFoundError(
        "Acc corridor with map failed to compute map sl boundary.");
  }
  const auto& map_sl_boundary = *map_sl_boundary_or;

  auto path_sl_map_center_or = BuildPathSlBoundaryOnReferencePath(
      drive_passage, map_sl_boundary, ref_path, av_pos,
      plan_start_point.path_point().theta(), plan_start_point.v(),
      av_frenet_on_dp, length, corridor_step_s, vehicle_geom);
  if (!path_sl_map_center_or.ok()) {
    return absl::InternalError(
        absl::StrFormat("Acc corridor with map failed to generate new path sl "
                        "boundary around reference path: %s",
                        path_sl_map_center_or.status().message()));
  }
  auto path_boundary = std::move(path_sl_map_center_or->first);
  const auto& map_center_xy = path_sl_map_center_or->second;
  const auto& ref_center_xy = path_boundary.reference_center_xy_vector();

  auto frenet_frame_or =
      BuildQtfmEnhancedKdTreeFrenetFrame(ref_center_xy,
                                         /*down_sample_raw_points=*/true);
  if (!frenet_frame_or.ok()) {
    return absl::NotFoundError("Frenet frame construction failed");
  }
  const double extend_length =
      std::max(plan_start_point.v() *
                   (total_preview_time + kCorridorExtendLengthTimeHorizon),
               credible_length);
  auto path = ComputeExtendedSmoothPath(
      ref_center_xy, path_boundary.s_vector(),
      (ref_center_xy.back() - ref_center_xy[ref_center_xy.size() - 2]).Unit(),
      corridor_step_s, kPathSampleInterval, extend_length);

  auto kappa_s = ComputeSmoothedCurvatureProfile(map_center_xy);

  return AccCorridor{.boundary = std::move(path_boundary),
                     .frenet_frame = std::move(frenet_frame_or).value(),
                     .path = std::move(path),
                     .credible_length = credible_length,
                     .kappa_s = std::move(kappa_s)};
}

std::set<std::string> CrowdedSceneObjects(
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr, const PoseProto& pose,
    const VehicleGeometryParamsProto& vehicle_geom, double look_forward_length,
    double look_backward_length) {
  constexpr int kMaxQuerySideBoxesNum = 2;  // curr lane, left and right.
  const Vec2d pose_pos = Vec3d(pose.pos_smooth()).xy();
  const ApolloTrajectoryPointProto pose_pt = ConvertToTrajPointProto(pose);
  const auto forward_lanes =
      FindForwardLanePathsAtPoint(pose_pos, pose.yaw(), pose.speed(),
                                  pose.curvature(), psmm, look_forward_length);
  const auto closest_lane_path_id_pair =
      internal::FindClosestLanePathAtPositionOrNull(
          psmm, absl::MakeSpan(forward_lanes), pose_pos);
  const auto* closest_lane_path = closest_lane_path_id_pair.first;
  if (closest_lane_path == nullptr) return {};
  // Build short dp!
  constexpr double kDrivePassageStepS = 1.0;
  const auto dp_or = BuildDrivePassageFromLanePath(
      psmm, *closest_lane_path, /*step_s=*/kDrivePassageStepS,
      /*avoid_loop=*/true, look_backward_length,
      /*required_planning_horizon=*/0.0, /*required_backward_len=*/0.0,
      std::nullopt);
  if (!dp_or.ok()) return {};
  const auto path_sl_or = BuildPathBoundaryFromDrivePassage(psmm, *dp_or);
  if (!path_sl_or.ok()) return {};
  const auto& path_sl = *path_sl_or;
  const auto path_ff_or = BuildQtfmEnhancedKdTreeFrenetFrame(
      path_sl.reference_center_xy_vector(), /*down_sample_raw_points=*/false);
  if (!path_ff_or.ok()) return {};

  // 0. Build current lane query box. Do not include regions behind av.
  const auto av_pose_box =
      ComputeAvBox(pose_pos, pose_pt.path_point().theta(), vehicle_geom);
  double av_min_s = std::numeric_limits<double>::infinity();
  double av_max_s = std::numeric_limits<double>::lowest();
  Vec2d min_s_corner = pose_pos;  // Use pose by default.
  Vec2d max_s_corner = pose_pos;
  for (const auto& corner : av_pose_box.GetCornersCounterClockwise()) {
    const auto corner_sl = path_ff_or->XYToSL(corner);
    if (corner_sl.s < av_min_s) {
      av_min_s = corner_sl.s;
      min_s_corner = corner;
    }
    if (corner_sl.s > av_max_s) {
      av_max_s = corner_sl.s;
      max_s_corner = corner;
    }
  }

  const auto last_idx = dp_or->last_real_station_index();
  const auto& last_station = dp_or->station(last_idx);
  const auto& current_lane_id = closest_lane_path_id_pair.second;
  auto curr_lane_box_opt = internal::BuildLaneBoxApprox(
      psmm, min_s_corner, last_station.xy(), current_lane_id);
  if (!curr_lane_box_opt.has_value()) return {};
  // Shrink search box to avoid false positive overlap.
  constexpr double kCurrentLaneApproxWidthExtendRatio = 0.8;
  curr_lane_box_opt->LateralExtendByRatio(kCurrentLaneApproxWidthExtendRatio);
  constexpr double kObjectSlowMovingSpeed = Kph2Mps(10.0);  // Below 10.0 km/h.
  Vec2d front_slow_obj_pt;
  if (!internal::CanFindClosestSlowObjectOnPathInRegionBox(
          st_traj_mgr.trajectories(), *curr_lane_box_opt, *path_ff_or, av_max_s,
          kObjectSlowMovingSpeed, &front_slow_obj_pt)) {
    return {};
  }
  // 1. If front slow obj found, use the closest point of this object to
  // generate new query boxes.
  std::vector<Box2d> side_boxes;
  side_boxes.reserve(kMaxQuerySideBoxesNum);
  auto side_box_raw_opt = internal::BuildLaneBoxApprox(
      psmm, min_s_corner, front_slow_obj_pt, current_lane_id);
  if (!side_box_raw_opt.has_value()) return {};
  const auto& side_box_raw = *side_box_raw_opt;
  const auto forward_dir = side_box_raw.tangent().Unit();
  // Right box.
  auto right_box = side_box_raw.Transform(forward_dir.Rotate(-M_PI / 2.0) *
                                          side_box_raw.width());
  // Left box.
  auto left_box = side_box_raw.Transform(forward_dir.Rotate(M_PI / 2.0) *
                                         side_box_raw.width());
  side_boxes.push_back(std::move(right_box));
  side_boxes.push_back(std::move(left_box));

  // Iterate through spacetime trajectories to see whether objects are moving
  // slowly in at least one of the side boxes. Calculate average speed of
  // these objects.
  std::set<std::string> object_ids;
  for (const auto& side_box : side_boxes) {
    auto objs_in_side_box = internal::FindSlowCutinObjectsInRegionBox(
        st_traj_mgr.trajectories(), side_box, path_sl, *path_ff_or,
        kObjectSlowMovingSpeed);
    object_ids.merge(objs_in_side_box);
  }
  return object_ids;
}

absl::StatusOr<std::pair<PathSlBoundary, std::vector<Vec2d>>>
BuildPathSlBoundaryOnReferencePath(
    const DrivePassage& drive_passage, const PathSlBoundary& map_sl_boundary,
    const DiscretizedPath& ref_path, const Vec2d& av_pos, double heading,
    double speed, const FrenetCoordinate& av_frenet_on_dp, double length,
    double corridor_step_s, const VehicleGeometryParamsProto& vehicle_geom) {
  FUNC_QTRACE();
  const int n = static_cast<int>(length / corridor_step_s) + 1;
  std::vector<double> s_vec, ref_center_l_vec, right_l_vec, left_l_vec,
      target_right_l_vec, target_left_l_vec;
  s_vec.reserve(n);
  ref_center_l_vec.reserve(n);
  right_l_vec.reserve(n);
  left_l_vec.reserve(n);
  target_right_l_vec.reserve(n);
  target_left_l_vec.reserve(n);
  std::vector<Vec2d> inner_right_xy, inner_left_xy, outer_right_xy,
      outer_left_xy, ref_center_xy;
  inner_right_xy.reserve(n);
  inner_left_xy.reserve(n);
  outer_right_xy.reserve(n);
  outer_left_xy.reserve(n);
  ref_center_xy.reserve(n);

  std::vector<Vec2d> map_center_xy;
  map_center_xy.reserve(n);

  const double av_width = vehicle_geom.width();
  const double half_width = 0.5 * av_width;
  const auto av_front_center =
      ComputeAvFrontCenter(av_pos, heading, vehicle_geom);
  const auto av_frenet_on_ref_path = ref_path.XYToSL(av_pos);
  const auto av_front_frenet_on_ref_path = ref_path.XYToSL(av_front_center);
  double cur_s = av_frenet_on_dp.s;
  double cur_s_on_ref_path = av_frenet_on_ref_path.s;

  for (int i = 0; i < n && (cur_s - av_frenet_on_dp.s) < length; ++i) {
    map_center_xy.push_back(map_sl_boundary.QueryReferenceCenterXY(cur_s));
    // Always sample on reference path.
    cur_s_on_ref_path = av_frenet_on_ref_path.s + i * corridor_step_s;
    if (cur_s_on_ref_path > ref_path.length()) {
      break;
    }
    const auto cur_av_path_point = ref_path.Evaluate(cur_s_on_ref_path);

    // AV pos is on generated reference path.
    Vec2d av_pos(cur_av_path_point.x(), cur_av_path_point.y());
    const auto cur_sl_on_dp_or =
        drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(av_pos);
    if (!cur_sl_on_dp_or.ok()) {
      return absl::OutOfRangeError(absl::StrFormat(
          "Cannot do laterally-unbounded projection from reference path "
          "position %s onto target dp [%s]: %s.",
          av_pos.DebugString(),
          absl::StrJoin(drive_passage.lane_path().lane_ids(), ","),
          cur_sl_on_dp_or.status().message()));
    }
    const auto& cur_sl_on_dp = *cur_sl_on_dp_or;
    cur_s = cur_sl_on_dp.s;
    const auto curb_offset_or = drive_passage.QueryCurbOffsetAtS(cur_s);
    if (!curb_offset_or.ok()) {
      return absl::OutOfRangeError(absl::StrFormat(
          "Query curb offset fails at %s (s = %.5f), for lane "
          "path: %s, message: %s.",
          av_pos.DebugString(), cur_s, drive_passage.lane_path().DebugString(),
          curb_offset_or.status().message()));
    }
    // Check whether lateral offset of reference path is within a certain
    // lateral distance buffer around curb. (Half av width)
    const double av_dp_l = cur_sl_on_dp.l;
    const auto out_of_curb_dist = av_dp_l >= 0.0
                                      ? av_dp_l - curb_offset_or->second
                                      : curb_offset_or->first - av_dp_l;
    constexpr double kLateralOutCurbDistAvWidthFactor = 0.25;
    if (out_of_curb_dist >
        vehicle_geom.width() * kLateralOutCurbDistAvWidthFactor) {
      return absl::OutOfRangeError(absl::StrFormat(
          "Reference path position: %s (s: %.5f, l: %.5f) is out of curb by "
          "distance: %.5f on dp: [%s].",
          av_pos.DebugString(), cur_sl_on_dp.s, cur_sl_on_dp.l,
          out_of_curb_dist, drive_passage.lane_path().DebugString()));
    }

    const auto tangent_or = drive_passage.QueryTangentAtS(cur_s);
    if (!tangent_or.ok()) {
      return tangent_or.status();
    }
    const auto& tangent = *tangent_or;

    // Relative to drive passage.
    auto [target_right_dp_l, target_left_dp_l] =
        map_sl_boundary.QueryTargetBoundaryL(cur_s);
    if (target_left_dp_l - target_right_dp_l > kAccMaxLaneWidth) {
      const auto ref_center_l = map_sl_boundary.QueryReferenceCenterL(cur_s);
      target_left_dp_l = ref_center_l + 0.5 * kAccMaxLaneWidth;
      target_right_dp_l = ref_center_l - 0.5 * kAccMaxLaneWidth;
    }

    // Compute buffer around AV's path
    constexpr double kNominalMinSpeed = 5.0;
    const double nominal_t = cur_s / std::max(kNominalMinSpeed, speed);
    const double path_buffer = kMapCorridorPathBufferTimePLF(nominal_t);
    target_left_dp_l = std::min(half_width + path_buffer, target_left_dp_l);
    target_right_dp_l = std::max(-half_width - path_buffer, target_right_dp_l);

    // Absolute left lateral offset wrt. DrivePassage.
    const double av_left_side = av_dp_l + half_width + kSoftBoundaryEpsilon;
    const double abs_target_left_l =
        cur_s_on_ref_path > av_front_frenet_on_ref_path.s
            ? std::max(av_left_side, target_left_dp_l)
            : av_left_side;
    // Use lateral distance relative to av pos.
    auto target_left_l = abs_target_left_l - av_dp_l;
    Vec2d target_left_xy = LatPoint(av_pos, tangent, target_left_l);

    // Absolute right lateral offset wrt. DrivePassage.
    const double av_right_side = av_dp_l - half_width - kSoftBoundaryEpsilon;
    const double abs_target_right_l =
        cur_s_on_ref_path > av_front_frenet_on_ref_path.s
            ? std::min(av_right_side, target_right_dp_l)
            : av_right_side;
    // Use lateral distance relative to av pos.
    auto target_right_l = abs_target_right_l - av_dp_l;
    Vec2d target_right_xy = LatPoint(av_pos, tangent, target_right_l);

    // Relative to drive passage.
    const auto [right_dp_l, left_dp_l] = map_sl_boundary.QueryBoundaryL(cur_s);
    double abs_left_l = left_dp_l + kHardBoundaryEpsilon;
    double abs_right_l = right_dp_l - kHardBoundaryEpsilon;
    // Clamp outer boundaries by curbs. Outer boundaries may exceed solid
    // yellow boundaries. But other vehicles might cut-in across solid lines.
    const auto curbs_or = drive_passage.QueryCurbBoundariesAtS(cur_s);
    if (curbs_or.ok()) {
      abs_left_l = std::min<double>(abs_left_l, curbs_or->second.lat_offset);
      abs_right_l = std::max<double>(abs_right_l, curbs_or->first.lat_offset);
    }
    const double outer_buffer = kMapCorridorOuterBoundaryTimePLF(nominal_t);

    // Make sure outer boundaries are beyond inner target boundaries.
    abs_left_l = std::min(av_dp_l + half_width + outer_buffer, abs_left_l);
    abs_left_l = std::max(abs_left_l, abs_target_left_l);
    const auto left_l = abs_left_l - av_dp_l;

    abs_right_l = std::max(abs_right_l, av_dp_l - half_width - outer_buffer);
    abs_right_l = std::min(abs_right_l, abs_target_right_l);
    const auto right_l = abs_right_l - av_dp_l;

    Vec2d left_xy = LatPoint(av_pos, tangent, left_l);
    Vec2d right_xy = LatPoint(av_pos, tangent, right_l);

    ref_center_l_vec.push_back(av_dp_l);
    right_l_vec.push_back(right_l);
    left_l_vec.push_back(left_l);
    target_right_l_vec.push_back(target_right_l);
    target_left_l_vec.push_back(target_left_l);
    inner_right_xy.push_back(std::move(target_right_xy));
    inner_left_xy.push_back(std::move(target_left_xy));
    outer_right_xy.push_back(std::move(right_xy));
    outer_left_xy.push_back(std::move(left_xy));
    ref_center_xy.push_back(std::move(av_pos));
    if (i == 0) {
      s_vec.push_back(0.0);
    } else {
      s_vec.push_back(s_vec.back() +
                      ref_center_xy.back().DistanceTo(ref_center_xy[i - 1]));
    }
  }

  constexpr int kMinPathSlBoundaryPointNum = 2;
  if (s_vec.size() < kMinPathSlBoundaryPointNum) {
    return absl::InternalError(
        "Sampled less than 2 points when building path sl boundary around "
        "reference path.");
  }

  auto path_boundary = PathSlBoundary(
      std::move(s_vec), std::move(ref_center_l_vec), std::move(right_l_vec),
      std::move(left_l_vec), std::move(target_right_l_vec),
      std::move(target_left_l_vec), std::move(ref_center_xy),
      std::move(outer_right_xy), std::move(outer_left_xy),
      std::move(inner_right_xy), std::move(inner_left_xy));

  return std::make_pair(std::move(path_boundary), map_center_xy);
}

std::optional<DiscretizedPath> ResampleSmoothedDiscretizedPathOnXY(
    absl::Span<const Vec2d> xy, const std::vector<double>& accum_s,
    double sample_step_s) {
  QCHECK_GE(xy.size(), internal::kPolynomialDegree + 1);
  QCHECK_EQ(xy.size(), accum_s.size());

  const int num_pt = xy.size();
  std::vector<double> vec_x, vec_y;
  std::vector<Vec2d> vec_xs, vec_ys;
  vec_x.reserve(num_pt);
  vec_y.reserve(num_pt);
  vec_xs.reserve(num_pt);
  vec_ys.reserve(num_pt);
  std::vector<double> x_weights(num_pt, 1.0);
  std::vector<double> y_weights(num_pt, 1.0);

  for (int i = 0; i < num_pt; ++i) {
    vec_x.push_back(xy[i].x());
    vec_y.push_back(xy[i].y());
    vec_xs.push_back(Vec2d(accum_s[i], xy[i].x()));
    vec_ys.push_back(Vec2d(accum_s[i], xy[i].y()));
  }

  const CubicSpline x_s(accum_s, vec_x);
  const CubicSpline y_s(accum_s, vec_y);
  const auto x_polynomial_or =
      FitPolynomialXdToData<internal::kPolynomialDegree>(vec_xs, x_weights,
                                                         LS_SOLVER::kQr);
  const auto y_polynomial_or =
      FitPolynomialXdToData<internal::kPolynomialDegree>(vec_ys, y_weights,
                                                         LS_SOLVER::kQr);
  const bool compute_kappa = x_polynomial_or.ok() && y_polynomial_or.ok();

  DiscretizedPath path;
  const double length = accum_s.back();
  const int resampled_num_pts = static_cast<int>(length / sample_step_s) + 1;
  path.reserve(resampled_num_pts);
  double cur_s = 0.0;
  for (int i = 0; i < resampled_num_pts && cur_s < length; ++i) {
    auto pt = EvaluatePathPointOnCubicSplines(x_s, y_s, cur_s);
    if (compute_kappa) {
      const double dx = x_polynomial_or->EvaluateDerivative</*order=*/1>(cur_s);
      const double d2x =
          x_polynomial_or->EvaluateDerivative</*order=*/2>(cur_s);
      const double dy = y_polynomial_or->EvaluateDerivative</*order=*/1>(cur_s);
      const double d2y =
          y_polynomial_or->EvaluateDerivative</*order=*/2>(cur_s);
      pt.set_kappa(ComputeCurvature(dx, dy, d2x, d2y));
    }
    path.push_back(std::move(pt));
    cur_s += sample_step_s;
  }

  if (path.size() <= 1) return std::nullopt;
  return path;
}

std::vector<mapping::LanePath> FindForwardLanePathsAtPoint(
    const Vec2d& pos, double heading, double speed, double curvature,
    const PlannerSemanticMapManager& psmm, double desire_forward_length) {
  constexpr double kLanePathSampleStepS = 2.0;                    // m.
  constexpr double kMaxSearchRadiusForNearestLane = 5.0;          // m.
  constexpr double kFindParallelLaneMaxHeadingDiff = M_PI / 4.0;  // 45 deg.
  const auto lanes = psmm.GetLanesInfoWithHeadingAtLevel(
      psmm.GetLevel(), pos, heading, kMaxSearchRadiusForNearestLane,
      kFindParallelLaneMaxHeadingDiff);
  constexpr double kMinLookForwardLength = 10.0;   // m.
  constexpr double kLookForwardTimeHorizon = 2.0;  // s.
  double look_forward_length =
      std::max(kMinLookForwardLength, speed * kLookForwardTimeHorizon);
  const auto all_lanes =
      internal::ExpandLaneIdsBySemantic(absl::MakeSpan(lanes));
  // Calculate potential lanes infomation.
  std::vector<PotentialLane> potential_lanes;
  potential_lanes.reserve(all_lanes.size());
  constexpr double kMaxLCLanePathDistance = 5.0;  // m.
  for (const auto& lane_id : all_lanes) {
    PotentialLane potential_lane;
    potential_lane.id = lane_id;
    const auto nearest_lane_point_proj_status =
        internal::FindClosestLanePointToSmoothPointOnLane(
            psmm, pos, lane_id, kMaxLCLanePathDistance, &potential_lane.lane_pt,
            &potential_lane.lane_pt_pos,
            /*tangent=*/nullptr, &potential_lane.distance);
    if (!nearest_lane_point_proj_status.ok()) continue;
    const auto raw_lane_path =
        mapping::LanePath(psmm.semantic_map_manager(), potential_lane.lane_pt);
    auto lane_path_or = ForwardExtendLanePathWithMinimumHeadingDiff(
        psmm, raw_lane_path, desire_forward_length, /*allow_virtual=*/false);
    if (!lane_path_or.ok()) continue;
    potential_lane.lane_path = std::move(lane_path_or).value();
    auto smoothed_path = internal::SampleSmoothedPathOnLanePath(
        psmm, potential_lane.lane_path, /*start_s=*/0.0, kLanePathSampleStepS);
    if (!smoothed_path.has_value()) continue;
    potential_lane.smoothed_path = std::move(*smoothed_path);
    look_forward_length =
        std::min(look_forward_length,
                 internal::ComputePathLengthFromPosition(
                     potential_lane.smoothed_path, potential_lane.lane_pt_pos));
    potential_lane.cost = internal::ComputeCurvatureCost(
        potential_lane.smoothed_path, curvature, look_forward_length);
    potential_lanes.push_back(std::move(potential_lane));
  }

  if (potential_lanes.empty()) return {};
  // From left to right.
  std::sort(potential_lanes.begin(), potential_lanes.end(),
            [](const PotentialLane& l1, const PotentialLane& l2) {
              return l1.distance <= l2.distance;
            });

  // Filter potential lanes, compare distance between start point and end
  // point to separate lanes for consideration.
  std::vector<std::vector<PotentialLane>> grouped_lanes;
  grouped_lanes.resize(potential_lanes.size());
  int group_idx = 0;
  for (auto& p : potential_lanes) {
    if (grouped_lanes[group_idx].empty()) {
      grouped_lanes[group_idx].push_back(std::move(p));
      continue;
    }
    const PotentialLane* reference_lane = &grouped_lanes[group_idx].front();
    if (reference_lane->IsDifferentFrom(p)) {
      group_idx++;
    }
    grouped_lanes[group_idx].push_back(std::move(p));
  }
  grouped_lanes.resize(group_idx + 1);
  if (grouped_lanes[0].empty()) return {};

  if (FLAGS_planner_acc_corridor_draw_target_lane_choice_debug) {
    for (int i = 0; i < grouped_lanes.size(); ++i) {
      std::vector<mapping::LanePath> groups;
      groups.reserve(grouped_lanes[i].size());
      for (const auto& l : grouped_lanes[i]) {
        groups.push_back(l.lane_path);
      }
      DrawLanePathsToCanvas(psmm, absl::MakeSpan(groups),
                            absl::StrFormat("with_map/lane_choice/group_%d", i),
                            vis::Color::kCyan);
    }
  }

  std::vector<mapping::LanePath> forward_lanes;
  forward_lanes.reserve(grouped_lanes.size());
  for (auto& group : grouped_lanes) {
    if (group.empty()) continue;
    auto min_cost_lane =
        std::min_element(group.begin(), group.end(),
                         [](const PotentialLane& l1, const PotentialLane& l2) {
                           return l1.cost < l2.cost;
                         });
    forward_lanes.push_back(std::move(min_cost_lane->lane_path));
  }

  if (FLAGS_planner_acc_corridor_draw_target_lane_choice_debug) {
    DrawLanePathsToCanvas(psmm, absl::MakeSpan(forward_lanes),
                          "with_map/lane_choice/possible_lanes",
                          vis::Color::kMagenta);
  }

  return forward_lanes;
}

}  // namespace planner
}  // namespace qcraft
