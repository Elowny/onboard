#include "onboard/planner/assist/vision_lane_path_filter.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <stddef.h>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/range1d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/lane_path_preprocess_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/online_semantic_map_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

struct LaneProjectionResult {
  mapping::LanePath lane_path;
  double ego_lat_offset = 0.0;
  double avg_angle_diff = 0.0;
  double length = 0.0;
  double avg_lat_offset = 0.0;
  double front_avg_lat_offset = 0.0;
  double virtual_angle_diff = 0.0;
  bool cross_bound = false;
};

std::vector<Vec2d> PreviewEgoPathUsingPose(const PoseProto& pose,
                                           double preview_time,
                                           double time_step) {
  PathPoint curr_path_point;
  curr_path_point.set_x(pose.pos_smooth().x());
  curr_path_point.set_y(pose.pos_smooth().y());
  curr_path_point.set_s(0.0);
  curr_path_point.set_theta(pose.yaw());
  curr_path_point.set_lambda(0.0);

  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  const double speed = pose.vel_body().x();
  if (std::fabs(speed) < kLowSpeedThreshold) {
    curr_path_point.set_kappa(0.0);
  } else {
    curr_path_point.set_kappa(pose.ar_smooth().z() / speed);
  }

  const int n = CeilToInt(preview_time / time_step) + 1;
  const double ds = speed * time_step;
  std::vector<Vec2d> path_xy_vec;
  path_xy_vec.reserve(n);

  for (double t = 0.0; t <= preview_time; t += time_step) {
    path_xy_vec.emplace_back(Vec2dFromPathPoint(curr_path_point));

    curr_path_point = GetPathPointAlongCircle(curr_path_point, ds);
  }

  return path_xy_vec;
}

mapping::LanePath SelectBestStartLane(
    absl::Span<const LaneProjectionResult> candidate_lanes) {
  if (candidate_lanes.size() == 1) {
    return candidate_lanes.front().lane_path;
  }

  constexpr double kMaxLengthThres = 100.0;  // m.
  constexpr double kLengthWeight = 0.02;
  constexpr double kAngleWeight = 100.0;
  constexpr double kEgoLatOffsetWeight = 1.0;
  constexpr double kCrossBoundCost = 0.5;
  constexpr double kPrevLanePathCost = 0.1;
  constexpr double kMaxIgnorableLatOffset = 0.3;
  constexpr double kVirtualAngleWeight = 2.0;

  const PiecewiseLinearFunction<double> avg_offset_cost_plf({0.5, 1.0, 1.5},
                                                            {0.0, 0.1, 0.4});

  const double min_avg_offset =
      std::min_element(candidate_lanes.begin(), candidate_lanes.end(),
                       [](const auto& lhs, const auto& rhs) {
                         return lhs.avg_lat_offset < rhs.avg_lat_offset;
                       })
          ->avg_lat_offset;

  std::vector<double> cost_vec;
  cost_vec.reserve(candidate_lanes.size());
  for (int i = 0; i < candidate_lanes.size(); ++i) {
    const auto& proj_res = candidate_lanes[i];

    const double length_cost =
        std::max(kLengthWeight * (kMaxLengthThres - proj_res.length), 0.0);
    const double angle_cost =
        std::min(kAngleWeight * proj_res.avg_angle_diff, 1.0);
    const double ego_offset_cost =
        kEgoLatOffsetWeight *
        std::max(proj_res.ego_lat_offset - kMaxIgnorableLatOffset, 0.0);
    const double cross_bound_cost =
        proj_res.cross_bound ? kCrossBoundCost : 0.0;
    const double prev_lp_cost = proj_res.avg_lat_offset > kMaxIgnorableLatOffset
                                    ? kPrevLanePathCost
                                    : 0.0;
    const double avg_offset_cost =
        avg_offset_cost_plf(proj_res.front_avg_lat_offset - min_avg_offset);
    const double virtual_angle_cost =
        std::min(proj_res.virtual_angle_diff * kVirtualAngleWeight, 1.0);

    const double total_cost = length_cost + angle_cost + ego_offset_cost +
                              cross_bound_cost + prev_lp_cost +
                              avg_offset_cost + virtual_angle_cost;
    cost_vec.push_back(total_cost);

    VLOG(1) << absl::StrFormat(
        "\n%s:\n\tavg lat offset %f, FRONT avg lat offset %f, length %f, angle "
        "%f, ego "
        "lat offset %f, cross bound %f, prev lp %f, avg offset %f, virtual "
        "angle %f, total %f.",
        proj_res.lane_path.DebugString(), proj_res.avg_lat_offset,
        proj_res.front_avg_lat_offset, length_cost, angle_cost, ego_offset_cost,
        cross_bound_cost, prev_lp_cost, avg_offset_cost, virtual_angle_cost,
        total_cost);
  }

  const int best_lane_idx = std::distance(
      cost_vec.begin(), std::min_element(cost_vec.begin(), cost_vec.end()));

  return candidate_lanes[best_lane_idx].lane_path;
}

bool HasLanePathPointsCrossedBoundary(const PlannerSemanticMapManager& psmm,
                                      const FrenetFrame& lane_ff,
                                      const mapping::LanePath& lane_path) {
  Range1d<int> intersection_range;
  for (const auto id : lane_path.lane_ids()) {
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, id);
    if (!lane_info.is_in_intersection) continue;

    const auto& raw_points = lane_info.points_smooth;
    const auto& s_knots = lane_ff.s_knots();

    const double start_s = lane_ff.XYToSL(raw_points.front()).s;
    const int start_idx = std::distance(
        s_knots.begin(),
        std::lower_bound(s_knots.begin(), s_knots.end(), start_s));

    const double end_s = lane_ff.XYToSL(raw_points.back()).s;
    const int end_idx =
        std::distance(s_knots.begin(),
                      std::lower_bound(s_knots.begin(), s_knots.end(), end_s)) +
        1;

    intersection_range = Range1d<int>(start_idx, end_idx);
    break;
  }

  return HasSmoothPointsCrossedBoundary(
      psmm, lane_ff.points(), intersection_range, /*solid_only=*/false);
}

double CalcVirtualConnectionAngleDiff(const PlannerSemanticMapManager& psmm,
                                      const mapping::LanePath& lane_path) {
  constexpr double kLaneCheckLength = 20.0;  // m.
  double angle_diff = 0.0;

  for (int i = 1; i < lane_path.size(); ++i) {
    const auto prev_id = lane_path.lane_id(i - 1);
    const auto curr_id = lane_path.lane_id(i);

    SMM_ASSIGN_LANE_OR_CONTINUE(prev_lane_info, psmm, prev_id);
    SMM_ASSIGN_LANE_OR_CONTINUE(curr_lane_info, psmm, curr_id);

    if (prev_lane_info.IsVirtual() == curr_lane_info.IsVirtual()) {
      continue;
    }

    const double prev_start_frac =
        std::max(0.0, 1.0 - kLaneCheckLength / prev_lane_info.length());
    const double curr_end_frac =
        std::min(1.0, kLaneCheckLength / curr_lane_info.length());
    const auto prev_lane_pt =
        prev_lane_info.LerpPointFromFraction(prev_start_frac);
    const auto curr_lane_pt =
        curr_lane_info.LerpPointFromFraction(curr_end_frac);
    const auto center_pt =
        curr_lane_info.LerpPointFromFraction(/*fraction=*/0.0);

    angle_diff +=
        std::fabs(NormalizeAngle((curr_lane_pt - center_pt).FastAngle() -
                                 (center_pt - prev_lane_pt).FastAngle()));
  }

  return angle_diff;
}

void CalcSmoothedCandidateLanePathInfo(const PlannerSemanticMapManager& psmm,
                                       absl::Span<const Vec2d> ego_path,
                                       double sample_step,
                                       LaneProjectionResult* lane_proj) {
  const auto& lane_path = lane_proj->lane_path;
  const auto smoothed_points = StraightenedLanePathPoints(
      psmm, lane_path, /*start_s=*/0.0, lane_path.length(), sample_step);

  double avg_heading_diff = 0.0, accum_s = 0.0;
  int i = 1;
  for (; i + 1 < smoothed_points.size(); ++i) {
    const auto& curr_xy = smoothed_points[i];
    const auto& prev_xy = smoothed_points[i - 1];
    const auto& next_xy = smoothed_points[i + 1];

    const double angle_diff = NormalizeAngle((next_xy - curr_xy).FastAngle() -
                                             (curr_xy - prev_xy).FastAngle());
    avg_heading_diff += std::fabs(angle_diff);
    accum_s += curr_xy.DistanceTo(prev_xy);
  }

  accum_s += smoothed_points[i - 1].DistanceTo(smoothed_points[i]);
  lane_proj->avg_angle_diff = avg_heading_diff / accum_s;

  ASSIGN_OR_VOID_RETURN(const auto ff,
                        BuildBruteForceFrenetFrame(
                            smoothed_points, /*down_sample_raw_points=*/false));

  lane_proj->cross_bound =
      HasLanePathPointsCrossedBoundary(psmm, ff, lane_path);

  double accum_lat_offset = 0.0;
  for (const auto& xy : ego_path) {
    accum_lat_offset += std::fabs(ff.XYToSL(xy).l);
  }
  lane_proj->ego_lat_offset = accum_lat_offset / ego_path.size();
}

std::vector<LaneProjectionResult> ProjectStartLaneToPrevLanePath(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& curr_online_map,
    const FrenetFrame& prev_frenet_frame, const Vec2d& ego_pos, double ego_v,
    double check_preview_length, ThreadPool* thread_pool) {
  FUNC_QTRACE();

  constexpr double kProjEpsilon = 1.0;           // m.
  constexpr double kMaxBackProjEpsilon = 8.0;    // m.
  constexpr double kValidOnlineLaneLen = 100.0;  // m.

  const auto starting_lane_ids = ComputeStartLanesByPos(
      psmm, curr_online_map, ego_pos, kProjEpsilon, kMaxBackProjEpsilon);

  std::vector<mapping::LanePath> lane_path_vec;
  for (const auto lane_id : starting_lane_ids) {
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, lane_id);
    const auto ego_pos_sl = lane_info.SmoothXYToSL(ego_pos);
    const double start_fraction =
        std::clamp(ego_pos_sl.s / lane_info.length(), 0.0, 1.0);

    mapping::LanePath lane_path(psmm.semantic_map_manager(), {lane_id},
                                start_fraction, /*end_fraction=*/1.0);

    auto new_lane_path_vec =
        CollectAllLanePathFromStartLane(psmm, lane_path, kValidOnlineLaneLen);
    lane_path_vec.insert(lane_path_vec.end(),
                         std::make_move_iterator(new_lane_path_vec.begin()),
                         std::make_move_iterator(new_lane_path_vec.end()));
  }

  constexpr double kLatOffsetDecayFactor = 0.00625;
  constexpr double kSampleLaneStep = 1.0;            // m.
  constexpr double kMaxEndExtrapolationDist = 10.0;  // m.
  constexpr double kMinFrontLen = 5.0;               // m.

  const PiecewiseLinearFunction<double> preview_len_plf(
      {5.0, 8.0, 10.0, 13.0}, {20.0, 40.0, 60.0, 100.0});

  const double front_check_len = preview_len_plf(ego_v);

  std::vector<LaneProjectionResult> candidate_lanes;
  candidate_lanes.reserve(candidate_lanes.size());
  for (auto& lane_path : lane_path_vec) {
    ASSIGN_OR_CONTINUE(const auto points,
                       SampleLanePathByStep(psmm, lane_path, kSampleLaneStep));

    if (points.size() < 3) continue;

    const double start_check_s = std::max(
        std::min(check_preview_length, lane_path.length() - kMinFrontLen), 0.0);
    const double end_check_s =
        std::min(lane_path.length(), start_check_s + front_check_len);
    if (end_check_s - start_check_s < kMinFrontLen) continue;

    const auto start_proj = prev_frenet_frame.XYToSL(points.front());
    auto proj = start_proj;
    double avg_lat_offset = 0.0, front_lat_offset = 0.0;
    double accum_weight = 0.0, front_accum_weight = 0.0;
    for (const auto& xy : points) {
      proj = prev_frenet_frame.XYToSL(xy);
      const auto relative_s = proj.s - start_proj.s;
      const auto s_weight =
          std::clamp(1.0 - relative_s * kLatOffsetDecayFactor, 0.0, 1.0);
      const double abs_l = std::fabs(proj.l);

      accum_weight += s_weight;
      avg_lat_offset += abs_l * s_weight;

      if (relative_s >= start_check_s && relative_s <= end_check_s) {
        front_accum_weight += s_weight;
        front_lat_offset += abs_l * s_weight;
      }

      if (proj.s - prev_frenet_frame.end_s() > kMaxEndExtrapolationDist) {
        break;
      }
    }

    avg_lat_offset /= accum_weight;
    front_lat_offset /= front_accum_weight;

    if (front_lat_offset < kDefaultHalfLaneWidth &&
        start_proj.s < prev_frenet_frame.end_s() &&
        proj.s > prev_frenet_frame.start_s()) {
      const double lp_length = lane_path.length();
      const double virtual_angle_diff =
          CalcVirtualConnectionAngleDiff(psmm, lane_path);
      candidate_lanes.push_back({.lane_path = std::move(lane_path),
                                 .length = lp_length,
                                 .avg_lat_offset = avg_lat_offset,
                                 .front_avg_lat_offset = front_lat_offset,
                                 .virtual_angle_diff = virtual_angle_diff});
    }
  }

  if (candidate_lanes.size() > 1) {
    ParallelFor(0, candidate_lanes.size(), thread_pool, [&](int i) {
      CalcSmoothedCandidateLanePathInfo(psmm, {ego_pos}, kSampleLaneStep,
                                        &candidate_lanes[i]);
    });
  }

  return candidate_lanes;
}

std::vector<LaneProjectionResult> FindCandidateLanePathsFromEgo(
    const PlannerSemanticMapManager& psmm, const PoseProto& pose,
    ThreadPool* thread_pool) {
  FUNC_QTRACE();

  constexpr double kMaxBackProjEpsilon = 8.0;  // m.
  constexpr double kValidLaneLen = 100.0;      // m.
  constexpr double kSampleLaneStep = 1.0;      // m.
  constexpr double kMinFrontLen = 5.0;         // m.
  constexpr double kMaxLatOffset = 2.0;        // m.

  const auto ego_pos = Vec2dFromPoseProto(pose);
  const auto close_points =
      FindCloseLanePointsToSmoothPointWithHeadingBoundAmongLanesAtLevel(
          psmm.GetLevel(), psmm, ego_pos, pose.yaw(),
          /*heading_penalty_weight=*/0.0, /*spatial_distance_threshold=*/5.0,
          /*angle_error_threshold=*/M_PI_4);

  absl::flat_hash_set<mapping::ElementId> lane_ids;
  std::vector<LaneProjectionResult> candidate_lanes;
  for (const auto& lp : close_points) {
    if (lane_ids.contains(lp.lane_id())) continue;
    lane_ids.emplace(lp.lane_id());

    mapping::LanePath lane_path(psmm.semantic_map_manager(), {lp.lane_id()},
                                lp.fraction(), /*end_fraction=*/1.0);
    lane_path = lane_path.BeforeArclength(kValidLaneLen);

    auto lane_path_vec =
        CollectAllLanePathFromStartLane(psmm, lane_path, kValidLaneLen);

    for (auto& lane_path : lane_path_vec) {
      ASSIGN_OR_CONTINUE(
          const auto points,
          SampleLanePathByStep(psmm, lane_path, kSampleLaneStep));

      if (points.size() < 3) continue;

      ASSIGN_OR_CONTINUE(
          const auto curr_frenet_frame,
          BuildBruteForceFrenetFrame(points, /*down_sample_raw_points=*/false));

      const auto ego_pos_sl = curr_frenet_frame.XYToSL(ego_pos);
      if (ego_pos_sl.s + kMinFrontLen > curr_frenet_frame.end_s() ||
          ego_pos_sl.s < curr_frenet_frame.start_s() - kMaxBackProjEpsilon ||
          std::fabs(ego_pos_sl.l) > kMaxLatOffset) {
        continue;
      }

      const double lp_length = lane_path.length();
      const double virtual_angle_diff =
          CalcVirtualConnectionAngleDiff(psmm, lane_path);
      candidate_lanes.push_back({.lane_path = std::move(lane_path),
                                 .length = lp_length - ego_pos_sl.s,
                                 .virtual_angle_diff = virtual_angle_diff});
    }
  }

  if (candidate_lanes.size() > 1) {
    constexpr double kPreviewT = 0.5;  // s.
    constexpr double kStepT = 0.1;     // s.
    const auto preview_ego_path =
        PreviewEgoPathUsingPose(pose, kPreviewT, kStepT);
    ParallelFor(0, candidate_lanes.size(), thread_pool, [&](int i) {
      CalcSmoothedCandidateLanePathInfo(psmm, preview_ego_path, kSampleLaneStep,
                                        &candidate_lanes[i]);
    });
  }

  return candidate_lanes;
}

}  // namespace

absl::StatusOr<mapping::LanePath> ProjectLaneFrenetFrameCurrentOnlineMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& curr_online_map,
    const KdTreeFrenetFrame& prev_frenet_frame, const Vec2d& ego_pos,
    const std::string& prev_lane_string, double ego_v,
    double check_preview_length, ThreadPool* thread_pool) {
  VLOG(1) << "Selecting lane path near prev lane path:";
  const auto candidate_lanes = ProjectStartLaneToPrevLanePath(
      psmm, curr_online_map, prev_frenet_frame, ego_pos, ego_v,
      check_preview_length, thread_pool);

  if (candidate_lanes.empty()) {
    return absl::InternalError(absl::StrFormat(
        "Cannot find any start lane near prev lane path %s", prev_lane_string));
  }

  const auto selected_lp = SelectBestStartLane(candidate_lanes);

  return ForwardExtendLanePathWithMinimumHeadingDiff(
      psmm,
      BackwardExtendLanePath(psmm, selected_lp, kDrivePassageKeepBehindLength),
      kAlccReferenceLineRequiredLength - selected_lp.length(),
      /*allow_virtual=*/true);
}

absl::StatusOr<mapping::LanePath> ProjectLanePathToCurrentOnlineMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& curr_online_map,
    const PlannerSemanticMapManager& prev_psmm,
    const mapping::LanePath& prev_lane_path, const Vec2d& ego_pos, double ego_v,
    double check_preview_length, ThreadPool* thread_pool) {
  FUNC_QTRACE();

  if (prev_lane_path.IsEmpty()) {
    return absl::InvalidArgumentError(
        "ProjectLanePathToCurrentOnlineMap: Empty prev lane path.");
  }

  // Build frenet frame from prev lane path.
  const auto prev_lane_smooth_points =
      SampleLanePathPoints(prev_psmm, prev_lane_path);
  ASSIGN_OR_RETURN(const auto prev_frenet_frame,
                   BuildKdTreeFrenetFrame(prev_lane_smooth_points,
                                          /*down_sample_raw_points=*/true),
                   _.SetPrepend() << "ProjectLanePathToCurrentOnlineMap: ");

  return ProjectLaneFrenetFrameCurrentOnlineMap(
      psmm, curr_online_map, prev_frenet_frame, ego_pos,
      prev_lane_path.DebugString(), ego_v, check_preview_length, thread_pool);
}

absl::StatusOr<mapping::LanePath> FindInitialLanePathFromEgoPose(
    const PlannerSemanticMapManager& psmm, const PoseProto& pose,
    ThreadPool* thread_pool) {
  FUNC_QTRACE();

  const auto candidate_lanes =
      FindCandidateLanePathsFromEgo(psmm, pose, thread_pool);

  if (candidate_lanes.empty()) {
    LOG(INFO) << absl::StrFormat(
        "Cannot find any lane path near ego pose [%f, %f]",
        pose.pos_smooth().x(), pose.pos_smooth().y());
    return absl::InternalError(
        absl::StrFormat("Cannot find any lane path near ego pose [%f, %f]",
                        pose.pos_smooth().x(), pose.pos_smooth().y()));
  }

  VLOG(1) << "Selecting initial lane path:";
  const auto selected_lp = SelectBestStartLane(candidate_lanes);

  return ForwardExtendLanePathWithMinimumHeadingDiff(
      psmm, selected_lp,
      kAlccReferenceLineRequiredLength - selected_lp.length(),
      /*allow_virtual=*/true);
}

}  // namespace qcraft::planner
