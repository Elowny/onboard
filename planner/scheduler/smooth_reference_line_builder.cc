#include "onboard/planner/scheduler/smooth_reference_line_builder.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/reference_line_qp_smoother.pb.h"
#include "onboard/planner/scheduler/reference_line_qp_smoother.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
// Reference line smoother weight.
// TODO(huaiyuan): use planner param.
constexpr double kKappaWeight = 1000.0;
constexpr double kBaseKappaGain = 0.8;
constexpr double kLengthWeight = 20.0;
constexpr double kBaseLengthGain = 0.2;

absl::StatusOr<mapping::LanePath> GenerateLanePath(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids) {
  mapping::LanePath lane_path(psmm.semantic_map_manager(), lane_ids,
                              /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  // Check connectivity and validity.
  for (int i = 0; i < lane_path.size(); ++i) {
    SMM_ASSIGN_LANE_OR_ERROR_ISSUE(lane_info, psmm, lane_ids[i]);
    if (i + 2 >= lane_path.size()) break;
    const auto& outgoing_lanes = lane_info.outgoing_lanes();
    if (std::find_if(outgoing_lanes.begin(), outgoing_lanes.end(),
                     [&lane_ids, i](mapping::ElementId id) {
                       return id == lane_ids[i + 1];
                     }) == outgoing_lanes.end()) {
      return absl::NotFoundError(
          absl::StrFormat("LanePath validation: lane %d does not have lane "
                          "%d as an outgoing lane",
                          lane_ids[i], lane_ids[i + 1]));
    }
  }

  return lane_path;
}

}  // namespace

absl::StatusOr<SmoothedReferenceCenterResult> SmoothLanePathByLaneIds(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width) {
  ASSIGN_OR_RETURN(const auto lane_path, GenerateLanePath(psmm, lane_ids));

  constexpr double kLanePathSampleStep = 1.0;  // m.
  ASSIGN_OR_RETURN(const auto drive_passage,
                   BuildDrivePassageFromLanePath(
                       psmm, lane_path, kLanePathSampleStep,
                       /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                       /*required_planning_horizon=*/0.0,
                       /*required_backward_len=*/0.0,
                       /*override_speed_limit_mps=*/std::nullopt,
                       FrenetFrameType::kBruteFroce),
                   _ << "SmoothLanePathByLaneIds failed.");

  ASSIGN_OR_RETURN(const auto path_boundary,
                   BuildPathBoundaryFromDrivePassage(psmm, drive_passage),
                   _ << "SmoothLanePathByLaneIds failed.");

  return SmoothLanePathBoundedByPathBoundary(psmm, drive_passage, path_boundary,
                                             lane_ids, half_av_width);
}

absl::StatusOr<SmoothedReferenceLineResultMap>
BuildSmoothedResultMapFromRouteSections(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    double half_av_width, SmoothedReferenceLineResultMap results) {
  const double local_horizon = route_sections.planning_horizon(psmm);
  ASSIGN_OR_RETURN(
      const auto clamped_route_sections,
      ClampRouteSectionsBeforeArcLength(psmm, route_sections, local_horizon));

  ASSIGN_OR_RETURN(const auto lane_ids_vec,
                   FindLanesToSmoothFromRoute(psmm, clamped_route_sections));

  absl::flat_hash_set<mapping::ElementId> new_lane_id_set;

  for (const auto& lane_ids : lane_ids_vec) {
    new_lane_id_set.insert(lane_ids.begin(), lane_ids.end());
    if (!results.Contains(lane_ids)) {
      auto smoothed_result =
          SmoothLanePathByLaneIds(psmm, lane_ids, half_av_width);
      if (smoothed_result.ok()) {
        results.AddResult(lane_ids, std::move(smoothed_result).value());
      }
    }
  }

  std::vector<std::vector<mapping::ElementId>> lane_ids_to_delete;
  for (const auto& pair_it : results.smoothed_result_map()) {
    bool should_delete = true;
    for (const auto id : pair_it.first) {
      if (new_lane_id_set.find(id) != new_lane_id_set.end()) {
        should_delete = false;
        break;
      }
    }

    if (should_delete) {
      lane_ids_to_delete.push_back(pair_it.first);
    }
  }
  for (const auto& lane_ids : lane_ids_to_delete) {
    results.DeleteResult(lane_ids);
  }

  return results;
}

absl::StatusOr<std::vector<std::vector<mapping::ElementId>>>
FindLanesToSmoothFromRoute(const PlannerSemanticMapManager& psmm,
                           const RouteSections& route_sections) {
  std::vector<std::vector<mapping::ElementId>> final_result;
  std::vector<std::vector<mapping::ElementId>> lane_ids_to_extend_vec;
  std::vector<mapping::ElementId> prev_lane_id_vec;

  SMM_ASSIGN_SECTION_OR_ERROR_ISSUE(first_section_info, psmm,
                                    route_sections.front().id);

  for (const auto lane_id : first_section_info.lane_ids) {
    prev_lane_id_vec.push_back(lane_id);
    if (IsTurningLane(psmm, lane_id)) {
      lane_ids_to_extend_vec.push_back({lane_id});
    }
  }

  for (int i = 1; i < route_sections.size(); ++i) {
    const auto& route_section_seg = route_sections.route_section_segment(i);
    SMM_ASSIGN_SECTION_OR_BREAK(section_info, psmm, route_section_seg.id);
    std::vector<mapping::ElementId> new_lane_id_vec;
    std::vector<std::vector<mapping::ElementId>> new_lane_ids_to_extend_vec;

    for (const auto lane_id : section_info.lane_ids) {
      for (const auto& prev_lane_id : prev_lane_id_vec) {
        SMM_ASSIGN_LANE_OR_ERROR_ISSUE(prev_lane_info, psmm, prev_lane_id);
        if (IsOutgoingLane(psmm, prev_lane_info, lane_id) &&
            !IsTurningLane(psmm, prev_lane_id) &&
            IsTurningLane(psmm, lane_id)) {
          new_lane_ids_to_extend_vec.push_back({lane_id});
        }
      }
      new_lane_id_vec.push_back(lane_id);
    }

    for (const auto& lane_id_vec : lane_ids_to_extend_vec) {
      SMM_ASSIGN_LANE_OR_ERROR_ISSUE(prev_lane_info, psmm, lane_id_vec.back());
      bool has_outgoing_to_smooth = false;

      for (const auto lane_id : new_lane_id_vec) {
        if (IsOutgoingLane(psmm, prev_lane_info, lane_id) &&
            IsTurningLane(psmm, lane_id)) {
          has_outgoing_to_smooth = true;
          std::vector<mapping::ElementId> lane_ids = lane_id_vec;
          lane_ids.push_back(lane_id);
          new_lane_ids_to_extend_vec.emplace_back(std::move(lane_ids));
        }
      }

      if (!has_outgoing_to_smooth) {
        final_result.push_back(lane_id_vec);
      }
    }

    prev_lane_id_vec = std::move(new_lane_id_vec);
    lane_ids_to_extend_vec = std::move(new_lane_ids_to_extend_vec);
  }

  for (auto& lane_ids : lane_ids_to_extend_vec) {
    final_result.emplace_back(std::move(lane_ids));
  }

  if (final_result.empty()) {
    return absl::NotFoundError(
        "No lane found to be smoothed on route sections: " +
        route_sections.DebugString());
  }

  return final_result;
}

absl::StatusOr<SmoothedReferenceCenterResult>
SmoothLanePathBoundedByPathBoundary(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const PathSlBoundary& boundary,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width) {
  constexpr double kSmootherStepHint = 1.0;
  constexpr int kPre = ReferenceLineQpSmoother::kNumPreOptimizationPoints;
  constexpr int kPost = ReferenceLineQpSmoother::kNumPostOptimizationPoints;
  absl::flat_hash_map<mapping::ElementId,
                      PiecewiseLinearFunction<double, double>>
      smoothed_lane_path_map;

  ASSIGN_OR_RETURN(const auto lane_path, GenerateLanePath(psmm, lane_ids));

  // ==== Sampling ====
  const double length = lane_path.length();
  QCHECK_GE(length, 0);
  const int n = static_cast<int>(length / kSmootherStepHint + 1.0) + 1;
  if (n < std::max(kPre + kPost + 1, 2)) {
    // Too short, no need to smooth.
    return absl::NotFoundError("Lane path is too short.");
  }
  const int opt_n = n - kPre - kPost;
  const double step = length / (n - 1);
  const auto smooth_start_point = ArclengthToPos(psmm, lane_path, 0.0);

  ASSIGN_OR_RETURN(const auto start_point_on_dp,
                   drive_passage.QueryFrenetCoordinateAt(smooth_start_point),
                   _ << "Unable to project start point on drive passage.");

  std::vector<double> sample_s;
  std::vector<mapping::LanePoint> lane_points;
  sample_s.reserve(n);
  lane_points.reserve(n);
  for (int i = 0; i < n; ++i) {
    sample_s.push_back(i * step + start_point_on_dp.s);
    // O(log(num of lanes in lane path)) aciton, cached beforehand.
    lane_points.push_back(lane_path.ArclengthToLanePoint(i * step));
  }

  // ===== Build problem input ====
  ReferenceLineQpSmoother::Input input;
  input.base_poses.reserve(n);
  input.base_tangents.reserve(n);
  input.l_lower_bound.reserve(opt_n);
  input.l_upper_bound.reserve(opt_n);
  input.pre_fixed_lateral_offsets.fill(0.0);
  input.post_fixed_lateral_offsets.fill(0.0);

  std::optional<std::pair<Vec2d, Vec2d>> pre_pose_tangent;
  for (int i = 0; i < n; ++i) {
    const mapping::LanePoint& lane_point = lane_points[i];
    const Vec2d base_pose = ComputeLanePointPos(psmm, lane_point);
    const Vec2d base_tangent = ComputeLanePointTangent(psmm, lane_point);

    input.base_poses.push_back(base_pose);
    input.base_tangents.push_back(base_tangent);

    if (i >= kPre && i < n - kPost) {
      // Limited by boundary.
      auto [l_min, l_max] = boundary.QueryTargetBoundaryL(sample_s[i]);
      l_min += half_av_width;
      l_max -= half_av_width;

      // Limited by lane center radius.
      if (pre_pose_tangent) {
        const double d_theta = NormalizeAngle(
            base_tangent.FastAngle() - pre_pose_tangent->second.FastAngle());
        if (std::abs(d_theta) * kMaxLateralOffset >= step) {
          const double turn_radius = step / d_theta;
          if (turn_radius < 0.0) {
            l_min = std::max(l_min, turn_radius);
          } else {
            l_max = std::min(l_max, turn_radius);
          }
        }
      }

      constexpr double kEpsilon = 0.1;  // m.
      if (l_max - l_min < kEpsilon) {
        return absl::NotFoundError(
            "Implausible lower and upper bound for smoother.");
      }

      input.l_lower_bound.push_back(l_min);
      input.l_upper_bound.push_back(l_max);
    }
    pre_pose_tangent = std::make_pair(base_pose, base_tangent);
  }

  // ====Try smooth the lane path====
  planner::ReferenceLineQpSmootherParamProto param;
  param.set_lateral_offset_weight(0.0);
  param.set_delta_weight(0.0);
  param.set_kappa_weight(kKappaWeight);
  param.set_lambda_weight(0.0);
  param.set_base_kappa_gain(kBaseKappaGain);
  param.set_length_weight(kLengthWeight);
  param.set_base_length_gain(kBaseLengthGain);

  const ReferenceLineQpSmoother smoother(&param, std::move(input));
  const std::optional<std::vector<double>> result = smoother.Solve();
  if (!result) {
    const auto debug_string =
        absl::StrCat("Reference line qp smoother failed on lane path<",
                     lane_path.DebugString(), ">");
    return absl::InternalError(debug_string);
  }
  QCHECK_EQ(result->size(), opt_n);

  std::vector<double> smoothed_l;
  smoothed_l.reserve(n);
  for (int i = 0; i < n; ++i) {
    if (i < kPre || i >= n - kPost) {
      smoothed_l.push_back(0.0);
    } else {
      smoothed_l.push_back((*result)[i - kPre]);
    }
  }

  // ==== Append the result ====
  int sample_index = 0;
  for (int i = 0; i < lane_path.size(); ++i) {
    const double start_s = lane_path.start_s(i) + start_point_on_dp.s;
    const double end_s = lane_path.end_s(i) + start_point_on_dp.s;
    QCHECK_GT(end_s, start_s);

    while (sample_index < n && sample_s[sample_index] <= start_s) {
      ++sample_index;
    }
    const int last_sample_leq_start_s = std::max(sample_index - 1, 0);
    while (sample_index < n && sample_s[sample_index] < end_s) {
      ++sample_index;
    }
    const int first_sample_geq_end_s =
        std::max(sample_index, last_sample_leq_start_s + 1);

    if (smoothed_lane_path_map.contains(lane_path.lane_id(i))) {
      // Will NOT override "first loop" smoothed result if same lane get
      // smoothed twice.
      continue;
    }

    const int num_sub_sample =
        first_sample_geq_end_s - last_sample_leq_start_s + 1;
    std::vector<double> lane_fraction;
    std::vector<double> smoothed_offset;
    lane_fraction.reserve(num_sub_sample);
    smoothed_offset.reserve(num_sub_sample);

    for (int j = last_sample_leq_start_s; j <= first_sample_geq_end_s; ++j) {
      lane_fraction.push_back((sample_s[j] - start_s) / (end_s - start_s));
      smoothed_offset.push_back(smoothed_l[j]);
    }

    smoothed_lane_path_map[lane_path.lane_id(i)] = PiecewiseLinearFunction(
        std::move(lane_fraction), std::move(smoothed_offset));
  }

  return SmoothedReferenceCenterResult{.lane_id_to_smoothed_lateral_offset =
                                           std::move(smoothed_lane_path_map)};
}

}  // namespace qcraft::planner
