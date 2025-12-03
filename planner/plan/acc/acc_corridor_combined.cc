#include "onboard/planner/plan/acc/acc_corridor_combined.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/map_path_generator.h"
#include "onboard/planner/common/path_generator_util.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"
#include "onboard/planner/plan/acc/acc_defs.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace internal {

using mapping::LaneInfo;

constexpr double kInnerBoundaryHalfWidth = 1.2;  // m.
constexpr double kOuterBoundaryHalfWidth = 1.5;  // m.
constexpr double kMaxHeadingDiff = M_PI_4;       // rad.

std::vector<const LaneInfo*> FilterLanes(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const LaneInfo*> start_lane_infos,
    const QtfmEnhancedKdTreeFrenetFrame& frenet_frame, double av_heading) {
  std::vector<const LaneInfo*> lane_info_filtered;
  lane_info_filtered.reserve(start_lane_infos.size());

  absl::flat_hash_map<mapping::ElementId, bool> lane_attr_map;
  for (const auto* lane_info_ptr : start_lane_infos) {
    const auto& lane_info = *lane_info_ptr;
    // Filter lanes in intersection.
    if (lane_info.IsVirtual() || (lane_info.proto->has_is_in_intersection() &&
                                  lane_info.proto->is_in_intersection())) {
      continue;
    }
    const auto& points_smooth = lane_info.points_smooth;
    // Filter lanes behinds corridor start point.
    const auto lane_start_point_sl = frenet_frame.XYToSL(points_smooth.front());
    if (lane_start_point_sl.s < 0.0) {
      continue;
    }
    // Filter short lanes.
    constexpr int kMinPointsInSingleLane = 5;
    if (points_smooth.size() < kMinPointsInSingleLane) {
      continue;
    }
    // Filter lanes with big heading diff.
    const double lane_start_heading =
        (points_smooth[1] - points_smooth.front()).FastAngle();
    if (std::fabs(NormalizeAngle(av_heading - lane_start_heading)) >
        kMaxHeadingDiff) {
      continue;
    }
    // Filter opposite direction lanes.
    if (IsOppositeLane(psmm, &lane_info, &lane_attr_map)) {
      continue;
    }
    lane_info_filtered.push_back(&lane_info);
  }

  return lane_info_filtered;
}

std::optional<double> ComputeLaneCost(
    const QtfmEnhancedKdTreeFrenetFrame& frenet_frame,
    absl::Span<const Vec2d> points, double credible_length) {
  double cost = 0.0;
  double accumulate_weight = 0.0;
  int num = 0;
  for (const auto& pt : points) {
    const auto pt_sl = frenet_frame.XYToSL(pt);
    // Ignore points which are beyond credible length.
    if (pt_sl.s > credible_length) {
      continue;
    }
    const double weight = (1.0 - pt_sl.s / credible_length);
    cost += weight * std::fabs(pt_sl.l);
    accumulate_weight += weight;
    ++num;
  }
  if (num < 2) return std::nullopt;
  cost /= accumulate_weight;
  return cost;
}

absl::StatusOr<mapping::ElementId> FindMostPossibleLane(
    const PlannerSemanticMapManager& psmm,
    const QtfmEnhancedKdTreeFrenetFrame& frenet_frame, const DrivingMapTopo& dm,
    const DiscretizedPath& path, double credible_length) {
  std::vector<const LaneInfo*> start_lane_infos;
  start_lane_infos.reserve(dm.starting_lane_ids().size());
  for (const auto& id : dm.starting_lane_ids()) {
    const auto* lane_info = psmm.FindLaneInfoOrNull(id);
    if (lane_info == nullptr) continue;
    start_lane_infos.push_back(lane_info);
  }
  const auto lane_info_filtered =
      FilterLanes(psmm, absl::MakeSpan(start_lane_infos), frenet_frame,
                  path.front().theta());
  if (lane_info_filtered.empty()) {
    return absl::NotFoundError("No available lanes.");
  }

  // Find the minimal weighted deviation cost lane.
  std::vector<std::pair<mapping::ElementId, double>> lane_id_with_cost_vec;
  lane_id_with_cost_vec.reserve(lane_info_filtered.size());
  for (const auto& lane_info : lane_info_filtered) {
    const auto cost_opt =
        ComputeLaneCost(frenet_frame, absl::MakeSpan(lane_info->points_smooth),
                        credible_length);
    if (!cost_opt.has_value()) continue;
    lane_id_with_cost_vec.push_back(std::make_pair(lane_info->id, *cost_opt));
  }

  if (lane_id_with_cost_vec.empty()) {
    return absl::NotFoundError("No candidate lanes with deviation cost.");
  }

  VLOG(2) << "raw lane size: " << psmm.lane_info().size() << '\n'
          << "filtered lane size: " << lane_info_filtered.size() << '\n'
          << "lane with cost size: " << lane_id_with_cost_vec.size();
  for (const auto& lane_id : lane_id_with_cost_vec) {
    VLOG(2) << "lane id: " << lane_id.first << " ,cost: " << lane_id.second;
  }

  const auto min_iter = std::min_element(
      lane_id_with_cost_vec.begin(), lane_id_with_cost_vec.end(),
      [](const std::pair<mapping::ElementId, double>& c1,
         const std::pair<mapping::ElementId, double>& c2) {
        return c1.second < c2.second;
      });

  VLOG(2) << "min lane id: " << min_iter->first;
  return min_iter->first;
}

absl::StatusOr<PathPoint> FindConnectingPathPoint(
    const PlannerSemanticMapManager& psmm, const DiscretizedPath& estimate_path,
    double credible_length, mapping::ElementId target_lane_id, double av_v) {
  constexpr double kMaxSearchRadius = 5.0;         // m.
  constexpr double kEstimatePathSampleStep = 1.0;  // m.
  // We don't expect connecting point is too near av real axle, because
  // estimated path is more credible in short time.
  constexpr double kCrediblePreviewTime = 1.5;         // s.
  constexpr double kMinCrediblePreviewDistance = 8.0;  // m.
  const double connecting_pt_beyond_av_ra_min_distance =
      std::max(av_v * kCrediblePreviewTime, kMinCrediblePreviewDistance);
  for (double cur_s = connecting_pt_beyond_av_ra_min_distance;
       cur_s < credible_length; cur_s += kEstimatePathSampleStep) {
    const auto path_point = estimate_path.Evaluate(cur_s);
    const auto* lane = psmm.GetNearestLaneInfoWithHeadingAtLevel(
        psmm.GetLevel(), ToVec2d(path_point), path_point.theta(),
        kMaxSearchRadius, kMaxHeadingDiff);
    if (lane != nullptr && lane->id == target_lane_id) {
      const auto& cumulative_length = lane->cumulative_lengths;
      const auto sl_on_lane = lane->SmoothXYToSL(ToVec2d(path_point));
      if (sl_on_lane.s > cumulative_length.front() &&
          sl_on_lane.s < cumulative_length.back()) {
        // Sampled path point should have valid longitudinal projection on
        // lane.
        return path_point;
      }
    }
  }

  return absl::NotFoundError(absl::StrFormat(
      "Cannot find connecting point on estimate path to lane %d.",
      target_lane_id));
}

absl::StatusOr<DrivePassage> BuildDrivePassageAndCheckValid(
    const PlannerSemanticMapManager& psmm, mapping::ElementId target_lane_id,
    const PathPoint& connecting_pt, double credible_length) {
  const double desired_length_on_map = credible_length - connecting_pt.s();
  VLOG(2) << "desired_length_on_map = " << desired_length_on_map;
  constexpr double kMinLengthOnMap = 3.0;  // m.
  if (desired_length_on_map < kMinLengthOnMap) {
    return absl::NotFoundError(
        "Desired length on map is too short, which may make corridor bend.");
  }
  constexpr double kDpExtendLengthFactor = 1.5;
  const auto dp_opt = BuildDrivePassageFromLaneId(
      psmm, target_lane_id, connecting_pt,
      kDpExtendLengthFactor * desired_length_on_map);
  if (!dp_opt.has_value()) {
    return absl::NotFoundError("Preview drive passage build failed.");
  }
  const auto& dp = *dp_opt;
  const Vec2d connect_pt_pos = Vec2d(connecting_pt.x(), connecting_pt.y());
  const auto connect_pt_sl_or = dp.QueryFrenetCoordinateAt(connect_pt_pos);
  if (!connect_pt_sl_or.ok()) {
    return absl::InternalError(absl::StrFormat(
        "Cannot project connecting point onto build drive passage. status "
        "message: %s. Drive passage is built based on lane path: %s.",
        connect_pt_sl_or.status().message(), dp.lane_path().DebugString()));
  }
  const double left_dp_length = dp.end_s() - connect_pt_sl_or->s;
  if (left_dp_length < desired_length_on_map) {
    return absl::NotFoundError(
        absl::StrFormat("Preview drive passage too short. Length left on drive "
                        "passage: %3.f, desired length: %.3f.",
                        left_dp_length, desired_length_on_map));
  }

  return dp;
}

absl::StatusOr<std::vector<ReferenceCenterPoint>>
CombineReferenceCenterFromEstimatedPathAndDp(
    const DiscretizedPath& estimate_path, const DrivePassage& dp,
    const PathPoint& connecting_point, double credible_length,
    double corridor_step_s) {
  constexpr int kMinEstimatePathSampleNum = 2;  // Until connecting_point.s().
  constexpr int kMinMapPathSampleNum = 5;
  constexpr double kMinSampleStepS = 0.1;
  double estimate_path_step_s = std::min<double>(
      corridor_step_s, connecting_point.s() / (kMinEstimatePathSampleNum - 1));
  estimate_path_step_s = std::max(estimate_path_step_s, kMinSampleStepS);
  const auto connecting_point_pos = ToVec2d(connecting_point);
  const auto connect_pt_sl_or =
      dp.QueryFrenetCoordinateAt(connecting_point_pos);
  if (!connect_pt_sl_or.ok()) {
    return absl::NotFoundError(absl::StrFormat(
        "Cannot project connecting point %s onto drive passage "
        "build from lane path: %s. Message: %s.",
        connecting_point_pos.DebugString(), dp.lane_path().DebugString(),
        connect_pt_sl_or.status().message()));
  }

  const auto left_dp_length = credible_length - connect_pt_sl_or->s;
  double map_path_step_s = std::min<double>(
      corridor_step_s, left_dp_length / (kMinMapPathSampleNum - 1));
  map_path_step_s = std::max(map_path_step_s, kMinSampleStepS);
  const int n_estimate =
      static_cast<int>(connecting_point.s() / estimate_path_step_s) + 1;
  const int n_map = static_cast<int>(left_dp_length / map_path_step_s) + 1;
  const int n = n_estimate + n_map;

  std::vector<ReferenceCenterPoint> reference_center_vec;
  reference_center_vec.reserve(n);

  std::vector<Vec2d> no_map;
  no_map.reserve(n_estimate);
  std::vector<Vec2d> map;
  map.reserve(n_map);

  for (int i = 0; i < n_estimate; ++i) {
    const double s_on_estimate = estimate_path_step_s * i;
    const auto pt = ToVec2d(estimate_path.Evaluate(s_on_estimate));
    if (no_map.empty()) {
      reference_center_vec.emplace_back(
          ReferenceCenterPoint{.x = pt.x(), .y = pt.y(), .s = 0.0});
    } else {
      reference_center_vec.emplace_back(ReferenceCenterPoint{
          .x = pt.x(),
          .y = pt.y(),
          .s = reference_center_vec.back().s + pt.DistanceTo(no_map.back()),
      });
    }
    no_map.push_back(pt);
  }

  // If connecting point current lateral offset is large, compute converging
  // points to be smoothed.
  constexpr int kConvergePointNum = 3;
  const PiecewiseLinearFunction<double, double> s_l(
      {connect_pt_sl_or->s,
       connect_pt_sl_or->s + kConvergePointNum * map_path_step_s},
      {connect_pt_sl_or->l, 0.0});
  double s_on_dp = connect_pt_sl_or->s;
  for (int i = 0; i < n_map; ++i) {
    if (i == 0) {
      s_on_dp = connect_pt_sl_or->s + 0.5 * map_path_step_s;
    } else {
      s_on_dp = connect_pt_sl_or->s + i * map_path_step_s;
    }
    const auto pt_xy_or = dp.QueryPointXYAtSL(s_on_dp, s_l(s_on_dp));
    if (!pt_xy_or.ok()) {
      break;
    }
    if (map.empty()) {
      reference_center_vec.emplace_back(ReferenceCenterPoint{
          .x = pt_xy_or->x(),
          .y = pt_xy_or->y(),
          .s = reference_center_vec.back().s +
               pt_xy_or->DistanceTo(no_map.back()),
      });
    } else {
      reference_center_vec.emplace_back(ReferenceCenterPoint{
          .x = pt_xy_or->x(),
          .y = pt_xy_or->y(),
          .s = reference_center_vec.back().s + pt_xy_or->DistanceTo(map.back()),
      });
    }
    map.push_back(*pt_xy_or);
  }
  if (map.size() < kMinMapPathSampleNum) {
    return absl::NotFoundError("Can not sample enough points on map.");
  }

  VLOG(2) << "raw n: " << n << '\n'
          << "new path point size: " << reference_center_vec.size() << '\n'
          << "credible length: " << credible_length << '\n'
          << "path point back.s(): " << reference_center_vec.back().s;

  return reference_center_vec;
}

PathSlBoundary BuildPathSlBoundaryWithSmoothedPath(
    const DiscretizedPath& smoothed_path) {
  const int n = smoothed_path.size();
  std::vector<double> s_vec, ref_center_l, right_l, left_l, target_right_l,
      target_left_l;
  s_vec.reserve(n);
  ref_center_l.reserve(n);
  right_l.reserve(n);
  left_l.reserve(n);
  target_right_l.reserve(n);
  target_left_l.reserve(n);

  std::vector<Vec2d> inner_right_xy, inner_left_xy, outer_right_xy,
      outer_left_xy, ref_center_xy;
  inner_left_xy.reserve(n);
  inner_right_xy.reserve(n);
  outer_right_xy.reserve(n);
  outer_left_xy.reserve(n);
  ref_center_xy.reserve(n);

  for (int i = 0; i < n; ++i) {
    const auto& pt = smoothed_path[i];
    s_vec.push_back(pt.s());
    ref_center_l.push_back(0.0);
    right_l.push_back(-kOuterBoundaryHalfWidth);
    left_l.push_back(kOuterBoundaryHalfWidth);
    target_right_l.push_back(-kInnerBoundaryHalfWidth);
    target_left_l.push_back(kInnerBoundaryHalfWidth);

    const auto tangent = Vec2d::FastUnitFromAngle(pt.theta());
    const auto pt_xy = ToVec2d(pt);
    inner_left_xy.emplace_back(
        LatPoint(pt_xy, tangent, kInnerBoundaryHalfWidth));
    inner_right_xy.emplace_back(
        LatPoint(pt_xy, tangent, -kInnerBoundaryHalfWidth));
    outer_left_xy.emplace_back(
        LatPoint(pt_xy, tangent, kOuterBoundaryHalfWidth));
    outer_right_xy.emplace_back(
        LatPoint(pt_xy, tangent, -kOuterBoundaryHalfWidth));
    ref_center_xy.push_back(pt_xy);
  }

  return PathSlBoundary(s_vec, std::move(ref_center_l), std::move(right_l),
                        std::move(left_l), std::move(target_right_l),
                        std::move(target_left_l), ref_center_xy,
                        std::move(outer_right_xy), std::move(outer_left_xy),
                        std::move(inner_right_xy), std::move(inner_left_xy));
}

}  // namespace internal

absl::StatusOr<AccCorridor> BuildPossiblePreviewLaneKeepAccCorridor(
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DiscretizedPath& estimate_path,
    const QtfmEnhancedKdTreeFrenetFrame& estimate_ff, const DrivingMapTopo& dm,
    double credible_length, double corridor_step_s, double total_preview_time) {
  FUNC_QTRACE();

  // 1. Find the most possible lane.
  ASSIGN_OR_RETURN(const mapping::ElementId min_lane_id,
                   internal::FindMostPossibleLane(
                       psmm, estimate_ff, dm, estimate_path, credible_length));

  // 2. Find the farthest estimated path point within credible length which can
  // match the most possible lane.
  const auto nearest_connecting_pt_or = internal::FindConnectingPathPoint(
      psmm, estimate_path, credible_length, min_lane_id, plan_start_point.v());
  if (!nearest_connecting_pt_or.ok()) {
    return nearest_connecting_pt_or.status();
  }
  const auto& nearest_connecting_pt = *nearest_connecting_pt_or;

  // 3. Build drive passage based on map from the connecting point.
  const auto dp_or = internal::BuildDrivePassageAndCheckValid(
      psmm, min_lane_id, nearest_connecting_pt, credible_length);
  if (!dp_or.ok()) {
    return dp_or.status();
  }
  const auto& dp = *dp_or;

  // 4. Combine reference center points from path boundary reference center
  // points before connecting point and drive passage center points after
  // connecting point.
  const auto combined_reference_center_or =
      internal::CombineReferenceCenterFromEstimatedPathAndDp(
          estimate_path, dp, nearest_connecting_pt, credible_length,
          corridor_step_s);
  if (!combined_reference_center_or.ok()) {
    return combined_reference_center_or.status();
  }
  const auto& combined_reference_center = *combined_reference_center_or;

  // 5. Smooth path points.
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> ss_vec;
  const int size = combined_reference_center.size();
  x_vec.reserve(size);
  y_vec.reserve(size);
  ss_vec.reserve(size);
  for (int i = 0; i < size; ++i) {
    x_vec.push_back(combined_reference_center[i].x);
    y_vec.push_back(combined_reference_center[i].y);
    ss_vec.push_back(combined_reference_center[i].s);
  }

  constexpr double kMaxLateralAccLimit = 8.0;  // m/s^2.
  const auto smoothed_path_or =
      GenerateSmoothPath(plan_start_point.v(), corridor_step_s, credible_length,
                         kMaxLateralAccLimit, &x_vec, &y_vec, &ss_vec);
  if (!smoothed_path_or.ok()) {
    VLOG(2) << absl::StrCat(
        "Fail to smooth discretized path, detailed reason: ",
        smoothed_path_or.status().message());
    return absl::NotFoundError(
        absl::StrCat("Fail to smooth discretized path, detailed reason: ",
                     smoothed_path_or.status().message()));
  }
  const auto& smoothed_path = *smoothed_path_or;

  // 6. Build path boundary with smoothed path points.
  auto path_boundary_sl =
      internal::BuildPathSlBoundaryWithSmoothedPath(smoothed_path);

  // 7. Build frenet frame.
  const auto& ref_center_xy = path_boundary_sl.reference_center_xy_vector();
  const auto& s_vec = path_boundary_sl.s_vector();
  auto frenet_frame_or =
      BuildQtfmEnhancedKdTreeFrenetFrame(ref_center_xy,
                                         /*down_sample_raw_points=*/true);
  if (!frenet_frame_or.ok()) {
    return frenet_frame_or.status();
  }

  // 8. Generate extended path.
  const double extend_length =
      std::max(plan_start_point.v() *
                   (total_preview_time + kCorridorExtendLengthTimeHorizon),
               credible_length);
  auto extended_path = ComputeExtendedSmoothPath(
      absl::MakeSpan(ref_center_xy), absl::MakeSpan(s_vec),
      (ref_center_xy.back() - ref_center_xy[ref_center_xy.size() - 2]).Unit(),
      corridor_step_s, kPathSampleInterval, extend_length);

  // Kappa profile is computed basing on road curvature to avoid human driving
  // impact.
  auto kappa_s = ComputeSmoothedCurvatureProfile(ref_center_xy);

  return AccCorridor{.boundary = std::move(path_boundary_sl),
                     .frenet_frame = std::move(frenet_frame_or).value(),
                     .path = std::move(extended_path),
                     .credible_length = credible_length,
                     .kappa_s = std::move(kappa_s)};
}

}  // namespace planner
}  // namespace qcraft
