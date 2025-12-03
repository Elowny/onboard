#include "onboard/planner/router/drive_passage_builder.h"

// IWYU pragma: no_include <stddef.h>  // for size_t

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/circle.h"
#include "onboard/math/circle_fitter.h"
#include "onboard/math/fitter_def.h"  // for LS_SOLVER
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/lane_path_preprocess_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner {

namespace {
constexpr double kVehicleWidth = 2.02;
constexpr double kDrivePassageCutOffAngleDiff = 1.1 * M_PI;
constexpr double kDrivePassageMaxForwardExtendLength = 10.0;  // m.
constexpr double kCurbDefaultHeight = 10.0;                   // m.

constexpr double kFarStationHeadwayTimeThreshold = 5.0;  // s.
constexpr double kFarStationHorizonRatio = 1.0 - 0.3;
constexpr double kFarRouteStationStep = 2.0 * kRouteStationUnitStep;
constexpr double kEpsilon = 1e-5;

// Returns true if type1 is strictly lower than type2.
bool LowerType(StationBoundaryType type1, StationBoundaryType type2) {
  switch (type1) {
    case StationBoundaryType::UNKNOWN_TYPE:
      return true;
    case StationBoundaryType::BROKEN_WHITE:
      return type2 == StationBoundaryType::SOLID_WHITE ||
             type2 == StationBoundaryType::BROKEN_YELLOW ||
             type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_WHITE:
    case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
    case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
      return type2 == StationBoundaryType::BROKEN_YELLOW ||
             type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::BROKEN_YELLOW:
      return type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_YELLOW:
      return type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      return type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::CURB:
    case StationBoundaryType::VIRTUAL_CURB:
    case StationBoundaryType::PREDICTION_VIRTUAL_CURB:
      return false;
  }
}

std::pair<double, double> ComputeLateralLimits(
    const StationCenter& prev_center, const StationCenter& current_center) {
  const double d_theta = NormalizeAngle(current_center.tangent.FastAngle() -
                                        prev_center.tangent.FastAngle());
  const double ds = current_center.accum_s - prev_center.accum_s;
  if (std::abs(d_theta) * kMaxLateralOffset < ds)
    return {-kMaxLateralOffset, kMaxLateralOffset};

  constexpr double kVehicleMinTurnRadius = 5.05;  // m.
  const double turn_radius =
      std::max(kVehicleMinTurnRadius, ds / std::abs(d_theta));

  if (d_theta < 0.0) {
    // Turn right
    return {-turn_radius, kMaxLateralOffset};
  }
  // Turn left
  return {-kMaxLateralOffset, turn_radius};
}

std::vector<StationBoundary> CollectStationBoundaries(
    const PlannerSemanticMapManager& psmm, const StationCenter& center,
    double max_right_offset, double max_left_offset, double max_lateral_offset,
    bool add_prediction_virtual_boundary) {
  const Segment2d normal_line(center.lat_point(max_right_offset),
                              center.lat_point(max_left_offset));

  const auto candidate_boundaries = psmm.GetLaneBoundariesInfoAtLevel(
      psmm.GetLevel(), center.xy, max_lateral_offset);
  std::vector<StationBoundary> station_boundaries;
  bool has_left_curb = false;
  bool has_right_curb = false;
  for (const auto& boundary : candidate_boundaries) {
    // Virtual lane, only keep curbs.
    if ((center.is_virtual || center.is_merging) &&
        boundary->type != mapping::LaneBoundaryProto::CURB) {
      continue;
    }
    double boundary_height = 0.0;
    // Only curb has height.
    if (boundary->type == mapping::LaneBoundaryProto::CURB) {
      boundary_height = boundary->proto->has_height()
                            ? boundary->proto->height()
                            : kCurbDefaultHeight;
    }
    // Compute cross point of boundary and normal line
    Vec2d closest_intersection;
    double min_sqr_dis = std::numeric_limits<double>::max();
    int closest_index;
    for (int k = 0; k + 1 < boundary->points_smooth.size(); ++k) {
      const Vec2d& p0 = boundary->points_smooth[k];
      const Vec2d& p1 = boundary->points_smooth[k + 1];
      const Segment2d boundary_segment(p0, p1);

      Vec2d intersection;
      if (!normal_line.GetIntersect(boundary_segment, &intersection)) continue;
      const double sqr_dis = center.xy.DistanceSquareTo(intersection);
      if (sqr_dis < min_sqr_dis) {
        closest_index = k;
        min_sqr_dis = sqr_dis;
        closest_intersection = intersection;
      }
    }
    if (min_sqr_dis < std::numeric_limits<double>::max()) {
      station_boundaries.push_back(
          {MapBoundaryTypeToStationBoundaryType(
               GetBoundaryType(*boundary, closest_index)),
           center.lat_offset(closest_intersection), boundary_height});
      if (station_boundaries.back().type == StationBoundaryType::CURB) {
        station_boundaries.back().lat_offset > 0 ? has_left_curb = true
                                                 : has_right_curb = true;
      }
    }
  }
  if (!has_left_curb)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_CURB, max_left_offset});
  if (!has_right_curb)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_CURB, max_right_offset});
  if (add_prediction_virtual_boundary) {
    constexpr double kEpsilonOffsetDiff = 0.1;
    station_boundaries.push_back({StationBoundaryType::PREDICTION_VIRTUAL_CURB,
                                  max_left_offset - kEpsilonOffsetDiff});
    station_boundaries.push_back({StationBoundaryType::PREDICTION_VIRTUAL_CURB,
                                  max_right_offset + kEpsilonOffsetDiff});
  }

  return station_boundaries;
}

void PostProcessStationBoundaries(
    std::vector<StationBoundary>* mutable_boundaries) {
  std::stable_sort(mutable_boundaries->begin(), mutable_boundaries->end(),
                   [](const StationBoundary& lhs, const StationBoundary& rhs) {
                     return lhs.lat_offset < rhs.lat_offset;
                   });
  std::vector<StationBoundary> remaining_boundaries;

  auto current_top_type = StationBoundaryType::UNKNOWN_TYPE;
  for (auto it = mutable_boundaries->rbegin(); it != mutable_boundaries->rend();
       ++it) {
    // From center to right.
    if (it->lat_offset > 0.0 || LowerType(it->type, current_top_type)) {
      continue;
    }
    remaining_boundaries.push_back(*it);
    if (it->type == StationBoundaryType::CURB ||
        it->type == StationBoundaryType::VIRTUAL_CURB) {
      break;
    }
    current_top_type = it->type;
  }
  std::reverse(remaining_boundaries.begin(), remaining_boundaries.end());

  current_top_type = StationBoundaryType::UNKNOWN_TYPE;
  for (const auto& bound : *mutable_boundaries) {
    // From center to left.
    if (bound.lat_offset < 0.0 || LowerType(bound.type, current_top_type)) {
      continue;
    }

    remaining_boundaries.push_back(bound);
    if (bound.type == StationBoundaryType::CURB ||
        bound.type == StationBoundaryType::VIRTUAL_CURB) {
      break;
    }
    current_top_type = bound.type;
  }

  // We can only borrow one lane at most if crossed yellow lane boundary.
  const double borrow_road_width = kDefaultLaneWidth + kVehicleWidth * 0.5;
  double left_solid_yellow_line_offset = std::numeric_limits<double>::max();
  double right_solid_yellow_line_offset = std::numeric_limits<double>::lowest();
  for (const auto& bound : remaining_boundaries) {
    if (bound.type == StationBoundaryType::SOLID_YELLOW ||
        bound.type == StationBoundaryType::SOLID_DOUBLE_YELLOW) {
      bound.lat_offset < 0.0 ? right_solid_yellow_line_offset = bound.lat_offset
                             : left_solid_yellow_line_offset = bound.lat_offset;
    }
  }
  if (left_solid_yellow_line_offset + borrow_road_width <
      remaining_boundaries.back().lat_offset) {
    remaining_boundaries.back().type = StationBoundaryType::VIRTUAL_CURB;
    remaining_boundaries.back().lat_offset =
        left_solid_yellow_line_offset + borrow_road_width;
  }
  if (right_solid_yellow_line_offset - borrow_road_width >
      remaining_boundaries.front().lat_offset) {
    remaining_boundaries.front().type = StationBoundaryType::VIRTUAL_CURB;
    remaining_boundaries.front().lat_offset =
        right_solid_yellow_line_offset - borrow_road_width;
  }

  *mutable_boundaries = std::move(remaining_boundaries);
}

struct DrivePassageData {
  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
};

std::vector<StationCenter>
SampleSmoothedLanePathCenterwithPlannerSemanticMapMgr(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double start_s, double end_s, double step, bool avoid_loop,
    std::optional<double> override_speed_limit_mps) {
  const auto straightened_points =
      StraightenedLanePathPoints(psmm, lane_path, start_s, end_s, step);

  std::vector<StationCenter> centers;
  const int n = straightened_points.size();
  centers.reserve(n);

  double prev_station_angle = 0.0;
  double max_accumulated_angle_diff = 0.0;
  double min_accumulated_angle_diff = 0.0;
  double accum_s = start_s;

  const auto start_lane_point = lane_path.ArclengthToLanePoint(start_s);
  double prev_speed_limit = psmm.QueryLaneSpeedLimitByFraction(
      start_lane_point.lane_id(), start_lane_point.fraction());
  bool is_override_speed_limit = override_speed_limit_mps.has_value();
  for (int i = 0; i < n; ++i) {
    const auto& xy = straightened_points[i];
    if (i > 0) {
      accum_s += xy.DistanceTo(centers.back().xy);
    }

    const auto sample_lane_point = lane_path.ArclengthToLanePoint(accum_s);

    // Ensure we do not create a loop in drive passage. Planner does not support
    // looped passage.
    if (i == 0) {
      prev_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
    } else {
      const auto cur_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
      const double angle_diff =
          NormalizeAngle(cur_station_angle - prev_station_angle);
      max_accumulated_angle_diff =
          std::max(max_accumulated_angle_diff + angle_diff, angle_diff);
      min_accumulated_angle_diff =
          std::min(min_accumulated_angle_diff + angle_diff, angle_diff);
      prev_station_angle = cur_station_angle;
    }
    if ((max_accumulated_angle_diff >= kDrivePassageCutOffAngleDiff ||
         min_accumulated_angle_diff <= -kDrivePassageCutOffAngleDiff) &&
        avoid_loop) {
      break;
    }

    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm,
                                      sample_lane_point.lane_id());

    const double current_speed_limit = psmm.QueryLaneSpeedLimitByFraction(
        sample_lane_point.lane_id(), sample_lane_point.fraction());
    is_override_speed_limit &=
        std::fabs(prev_speed_limit - current_speed_limit) < kEpsilon;
    prev_speed_limit = current_speed_limit;

    // Station center:
    StationCenter center{
        .lane_id = sample_lane_point.lane_id(),
        .fraction = sample_lane_point.fraction(),
        .xy = xy,
        .tangent = (i == 0 ? straightened_points[1] - straightened_points[0]
                           : xy - straightened_points[i - 1])
                       .normalized(),
        .accum_s = accum_s,
        .speed_limit = is_override_speed_limit ? *override_speed_limit_mps
                                               : current_speed_limit,
        .is_virtual = lane_info.IsVirtual(),
        .is_merging = lane_info.proto->is_merging(),
        .is_in_intersection = lane_info.is_in_intersection,
        .direction = lane_info.direction};
    centers.emplace_back(std::move(center));
  }

  return centers;
}

std::vector<StationCenter> SampleLanePathCentersWithPlannerSemanticMapMgr(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double start_s, double end_s, double step, bool avoid_loop,
    std::optional<double> override_speed_limit_mps) {
  std::vector<StationCenter> centers;
  const int n = CeilToInt((end_s - start_s) / step);
  centers.reserve(n);

  int station_idx = 0;
  double prev_station_angle = 0.0;
  double max_accumulated_angle_diff = 0.0;
  double min_accumulated_angle_diff = 0.0;

  const auto start_lane_point = lane_path.ArclengthToLanePoint(start_s);
  double prev_speed_limit = psmm.QueryLaneSpeedLimitByFraction(
      start_lane_point.lane_id(), start_lane_point.fraction());
  bool is_override_speed_limit = override_speed_limit_mps.has_value();

  for (double sample_s = start_s; sample_s <= end_s; sample_s += step) {
    const auto sample_lane_point = lane_path.ArclengthToLanePoint(sample_s);

    // Ensure we do not create a loop in drive passage. Planner does not support
    // looped passage.
    if (station_idx == 0) {
      prev_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
    } else {
      const auto cur_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
      const double angle_diff =
          NormalizeAngle(cur_station_angle - prev_station_angle);
      max_accumulated_angle_diff =
          std::max(max_accumulated_angle_diff + angle_diff, angle_diff);
      min_accumulated_angle_diff =
          std::min(min_accumulated_angle_diff + angle_diff, angle_diff);
      prev_station_angle = cur_station_angle;
    }
    if ((max_accumulated_angle_diff >= kDrivePassageCutOffAngleDiff ||
         min_accumulated_angle_diff <= -kDrivePassageCutOffAngleDiff) &&
        avoid_loop) {
      break;
    }
    station_idx++;

    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm,
                                      sample_lane_point.lane_id());

    const double current_speed_limit = psmm.QueryLaneSpeedLimitByFraction(
        sample_lane_point.lane_id(), sample_lane_point.fraction());
    is_override_speed_limit &=
        std::fabs(prev_speed_limit - current_speed_limit) < kEpsilon;
    prev_speed_limit = current_speed_limit;

    // Station center:
    StationCenter center{
        .lane_id = sample_lane_point.lane_id(),
        .fraction = sample_lane_point.fraction(),
        .xy = ComputeLanePointPos(psmm, sample_lane_point),
        .tangent = ComputeLanePointTangent(psmm, sample_lane_point),
        .accum_s = sample_s,
        .speed_limit = is_override_speed_limit ? *override_speed_limit_mps
                                               : current_speed_limit,
        .is_virtual = lane_info.IsVirtual(),
        .is_merging = lane_info.IsMerging(),
        .is_in_intersection = lane_info.is_in_intersection,
        .direction = lane_info.direction};
    centers.emplace_back(std::move(center));
  }
  return centers;
}

std::vector<std::vector<StationBoundary>>
SampleLanePathBoundariesWithPlannerSemanticMapMgr(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const StationCenter> centers, double max_lateral_boundary,
    bool add_prediction_virtual_boundary) {
  std::vector<std::vector<StationBoundary>> stations_boundaries;
  for (int i = 0; i < centers.size(); ++i) {
    const auto& center = centers[i];
    // Station boundaries:
    double right_lat_offset = -kMaxLateralOffset;
    double left_lat_offset = kMaxLateralOffset;
    if (i > 0) {
      const auto offsets = ComputeLateralLimits(centers[i - 1], center);
      right_lat_offset = offsets.first;
      left_lat_offset = offsets.second;
    }
    auto current_station_boundaries = CollectStationBoundaries(
        psmm, center, right_lat_offset, left_lat_offset, max_lateral_boundary,
        add_prediction_virtual_boundary);
    PostProcessStationBoundaries(&current_station_boundaries);
    stations_boundaries.emplace_back(std::move(current_station_boundaries));
  }
  return stations_boundaries;
}

DrivePassageData SampleLanePathWithPlannerSemanticMapMgr(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double start_s, double end_s, double step, bool avoid_loop,
    double max_lateral_boundary, bool add_prediction_virtual_boundary,
    std::optional<double> override_speed_limit_mps, bool smooth_lane_path) {
  auto centers = smooth_lane_path
                     ? SampleSmoothedLanePathCenterwithPlannerSemanticMapMgr(
                           psmm, lane_path, start_s, end_s, step, avoid_loop,
                           override_speed_limit_mps)
                     : SampleLanePathCentersWithPlannerSemanticMapMgr(
                           psmm, lane_path, start_s, end_s, step, avoid_loop,
                           override_speed_limit_mps);
  auto stations_boundaries = SampleLanePathBoundariesWithPlannerSemanticMapMgr(
      psmm, centers, max_lateral_boundary, add_prediction_virtual_boundary);
  return DrivePassageData{
      .centers = std::move(centers),
      .stations_boundaries = std::move(stations_boundaries)};
}

void ExtendDrivePassageDataByCircleFit(double fit_length, double step_s,
                                       DrivePassageData* dp_data_ptr) {
  constexpr int kFitDataMaxSize = 30;
  constexpr int kFitEveryNPt = 1;

  auto& centers = dp_data_ptr->centers;
  auto& stations_boundaries = dp_data_ptr->stations_boundaries;

  std::vector<Vec2d> fit_data;
  std::vector<double> weights;
  fit_data.reserve(kFitDataMaxSize);
  weights.reserve(kFitDataMaxSize);
  for (int i = centers.size() - 1; i >= 0; i -= kFitEveryNPt) {
    fit_data.push_back(centers[i].xy);
    weights.push_back(Sqr(kFitDataMaxSize - fit_data.size()));
    if (fit_data.size() >= kFitDataMaxSize) break;
  }
  double mse = 0.0;
  const auto circle_or =
      FitCircleToData(fit_data, weights, /*solver=*/LS_SOLVER::kSvd, &mse);
  if (!circle_or.ok()) {
    QLOG(INFO) << "Drive passage forward fitting discarded: "
               << circle_or.status().message();
    return;
  }
  constexpr double kMaxFittingMSError = 1e-3;
  if (mse > kMaxFittingMSError) {
    QLOG(INFO) << "Drive passage forward fitting discarded: fitting error "
               << mse << " is too large.";
    return;
  }
  const auto& circle = *circle_or;
  constexpr double kMinFittedRadius = 60.0;
  constexpr double kMaxFittedRadius = 2000.0;
  if (circle.radius() < kMinFittedRadius ||
      circle.radius() > kMaxFittedRadius) {
    QLOG(INFO) << "Drive passage forward fitting discarded: fitted radius "
               << circle.radius() << " out of reasonable range ("
               << kMinFittedRadius << ", " << kMaxFittedRadius << ").";
    return;
  }

  double last_accum_s = centers.back().accum_s;
  const double d_theta = std::copysign(
      step_s / circle.radius(),
      AngleDifference(circle.EvaluateTheta(centers[centers.size() - 2].xy),
                      circle.EvaluateTheta(centers.back().xy)));
  while (last_accum_s < fit_length) {
    auto center = centers.back();
    center.lane_id = mapping::kInvalidElementId;
    center.fraction = 1.0;
    center.is_virtual = true;

    const Vec2d new_pos =
        circle.EvaluateXY(circle.EvaluateTheta(center.xy) + d_theta);
    center.tangent = (new_pos - center.xy).normalized();
    last_accum_s += center.xy.DistanceTo(new_pos);
    center.xy = new_pos;
    center.accum_s = last_accum_s;

    centers.emplace_back(std::move(center));
    stations_boundaries.push_back(
        {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
         {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
  }
}

std::vector<std::vector<StationBoundary>> CreateBoundariesFromCache(
    absl::Span<const StationCenter> centers,
    const LaneBoundaryCache& lane_boundary_cache, double default_lane_width) {
  std::vector<std::vector<StationBoundary>> stations_boundaries;

  for (int i = 0; i < centers.size(); ++i) {
    const auto& center = centers[i];
    // Station boundaries:
    double right_lat_offset = -kMaxLateralOffset;
    double left_lat_offset = kMaxLateralOffset;
    if (i > 0) {
      const auto offsets = ComputeLateralLimits(centers[i - 1], center);
      right_lat_offset = offsets.first;
      left_lat_offset = offsets.second;
    }
    std::vector<StationBoundary> current_station_boundaries;
    current_station_boundaries.push_back(StationBoundary{
        .type = StationBoundaryType::PREDICTION_VIRTUAL_CURB,
        .lat_offset = right_lat_offset,
    });
    current_station_boundaries.push_back(StationBoundary{
        .type = StationBoundaryType::PREDICTION_VIRTUAL_CURB,
        .lat_offset = left_lat_offset,
    });
    const auto* plf = FindOrNull(lane_boundary_cache, center.lane_id);
    if (plf == nullptr) {
      current_station_boundaries.push_back(StationBoundary{
          .type = StationBoundaryType::BROKEN_WHITE,
          .lat_offset = -0.5 * default_lane_width,
      });
      current_station_boundaries.push_back(StationBoundary{
          .type = StationBoundaryType::BROKEN_WHITE,
          .lat_offset = 0.5 * default_lane_width,
      });
    } else {
      const auto bound = (*plf)(center.fraction);
      current_station_boundaries.push_back(StationBoundary{
          .type = bound.right_type,
          .lat_offset = bound.right_bound,
      });
      current_station_boundaries.push_back(StationBoundary{
          .type = bound.left_type,
          .lat_offset = bound.left_bound,
      });
    }
    PostProcessStationBoundaries(&current_station_boundaries);
    stations_boundaries.emplace_back(std::move(current_station_boundaries));
  }
  return stations_boundaries;
}

bool IsUpdatedHdMapBoundaryType(StationBoundaryType type) {
  switch (type) {
    case StationBoundaryType::BROKEN_WHITE:
    case StationBoundaryType::SOLID_WHITE:
    case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
    case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
    case StationBoundaryType::BROKEN_YELLOW:
    case StationBoundaryType::SOLID_YELLOW:
    case StationBoundaryType::SOLID_DOUBLE_YELLOW:
    case StationBoundaryType::UNKNOWN_TYPE:
      return true;
    case StationBoundaryType::CURB:
    case StationBoundaryType::VIRTUAL_CURB:
    case StationBoundaryType::PREDICTION_VIRTUAL_CURB:
      return false;
  }
}

bool IsValidVisionMapBoundaryType(StationBoundaryType type) {
  switch (type) {
    case StationBoundaryType::BROKEN_WHITE:
    case StationBoundaryType::SOLID_WHITE:
    case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
    case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
    case StationBoundaryType::BROKEN_YELLOW:
    case StationBoundaryType::SOLID_YELLOW:
    case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      return true;
    case StationBoundaryType::UNKNOWN_TYPE:
    case StationBoundaryType::CURB:
    case StationBoundaryType::VIRTUAL_CURB:
    case StationBoundaryType::PREDICTION_VIRTUAL_CURB:
      return false;
  }
}

std::optional<int> FindNearestBoundaryIndexByOffset(
    const std::vector<StationBoundary>& boundaries, double offset) {
  if (boundaries.empty()) return std::nullopt;
  const auto it = std::upper_bound(
      boundaries.begin(), boundaries.end(), offset,
      [](double val, const auto& it) { return val < it.lat_offset; });
  if (it == boundaries.begin()) return 0;
  if (it == boundaries.end()) return boundaries.size() - 1;
  const auto prev_it = std::prev(it);
  return (it->lat_offset - offset) > (offset - prev_it->lat_offset)
             ? std::distance(boundaries.begin(), prev_it)
             : std::distance(boundaries.begin(), it);
}

void UpdateBoundariesTypeByVisionMap(
    const std::vector<StationBoundary>& vision_map_boundaries,
    std::vector<StationBoundary>* mutable_boundaries) {
  constexpr double kMatchDistanceThreshold = 0.2;  // m
  for (auto& bound : *mutable_boundaries) {
    if (!IsUpdatedHdMapBoundaryType(bound.type)) {
      continue;
    }

    const auto nearest_index = FindNearestBoundaryIndexByOffset(
        vision_map_boundaries, bound.lat_offset);
    if (!nearest_index.has_value()) return;
    const auto& nearest_vision_bound = vision_map_boundaries[*nearest_index];
    if (!IsValidVisionMapBoundaryType(nearest_vision_bound.type)) {
      continue;
    }

    if (std::fabs(nearest_vision_bound.lat_offset - bound.lat_offset) <
        kMatchDistanceThreshold) {
      bound.type = nearest_vision_bound.type;
    }
  }
}
}  // namespace
absl::StatusOr<DrivePassage> BuildDrivePassageForPrediction(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step_s, bool avoid_loop, double backward_extend_len,
    double max_lateral_boundary, FrenetFrameType type) {
  SCOPED_QTRACE("BuildDrivePassageForPrediction");
  double start_s;
  mapping::LanePath extended_lane_path;
  extended_lane_path =
      BackwardExtendLanePath(psmm, lane_path, backward_extend_len);
  start_s = extended_lane_path.FirstOccurrenceOfLanePointToArclength(
                lane_path.front()) *
            (backward_extend_len > 0.0);

  auto dp_data = SampleLanePathWithPlannerSemanticMapMgr(
      psmm, extended_lane_path, 0.0, extended_lane_path.length(), step_s,
      avoid_loop, max_lateral_boundary,
      /*add_prediction_virtual_boundary=*/true,
      /*override_speed_limit_mps=*/std::nullopt,
      /*smooth_lane_path=*/false);
  if (dp_data.centers.size() < 2) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Too few stations in BuildDrivePassageForPrediction: Lane path: ",
        lane_path.DebugString(),
        "\nExtended lane path: ", extended_lane_path.DebugString()));
  }

  // Create station vector.
  StationVector<Station> stations;
  stations.reserve(dp_data.centers.size());
  for (int i = 0; i < dp_data.centers.size(); ++i) {
    dp_data.centers[i].accum_s -= start_s * (backward_extend_len > 0.0);
    stations.emplace_back(std::move(dp_data.centers[i]),
                          std::move(dp_data.stations_boundaries[i]));
  }

  return DrivePassage(std::move(stations), lane_path,
                      std::move(extended_lane_path),
                      /*lane_path_start_s=*/start_s,
                      /*reach_destination=*/false, type);
}

absl::StatusOr<DrivePassage>
BuildDrivePassageForPredictionWithLaneBoundaryCache(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    const LaneBoundaryCache& lane_boundary_cache, double step_s,
    bool avoid_loop, double backward_extend_len, FrenetFrameType type) {
  SCOPED_QTRACE("BuildDrivePassageForPredictionWithLaneBoundaryCache");
  double start_s;
  mapping::LanePath extended_lane_path;
  extended_lane_path =
      BackwardExtendLanePath(psmm, lane_path, backward_extend_len);
  start_s = extended_lane_path.FirstOccurrenceOfLanePointToArclength(
                lane_path.front()) *
            (backward_extend_len > 0.0);

  auto centers = SampleLanePathCentersWithPlannerSemanticMapMgr(
      psmm, extended_lane_path, start_s, extended_lane_path.length(), step_s,
      avoid_loop,
      /*override_speed_limit_mps=*/std::nullopt);
  if (centers.size() < 2) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Too few stations in BuildDrivePassageForPrediction: Lane path: ",
        lane_path.DebugString(),
        "\nExtended lane path: ", extended_lane_path.DebugString()));
  }
  auto stations_boundaries = CreateBoundariesFromCache(
      centers, lane_boundary_cache, kDefaultLaneWidth);
  // Create station vector.
  StationVector<Station> stations;
  stations.reserve(centers.size());
  for (int i = 0; i < centers.size(); ++i) {
    centers[i].accum_s -= start_s * (backward_extend_len > 0.0);
    stations.emplace_back(std::move(centers[i]),
                          std::move(stations_boundaries[i]));
  }

  return DrivePassage(std::move(stations), lane_path,
                      std::move(extended_lane_path),
                      /*lane_path_start_s=*/start_s,
                      /*reach_destination=*/false, type);
}

absl::StatusOr<DrivePassage> BuildDrivePassageFromLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step_s, bool avoid_loop, double backward_extend_len,
    double required_planning_horizon, double required_backward_len,
    std::optional<double> override_speed_limit_mps, FrenetFrameType type,
    bool smooth_lane_path) {
  SCOPED_QTRACE("BuildDrivePassageFromLanePath");
  double start_s;
  mapping::LanePath extended_lane_path;
  extended_lane_path =
      BackwardExtendLanePath(psmm, lane_path, backward_extend_len);
  start_s = extended_lane_path.FirstOccurrenceOfLanePointToArclength(
                lane_path.front()) *
            (backward_extend_len > 0.0);

  auto dp_data = SampleLanePathWithPlannerSemanticMapMgr(
      psmm, extended_lane_path, 0.0, extended_lane_path.length(), step_s,
      avoid_loop, kMaxLateralOffset, /*add_prediction_virtual_boundary=*/false,
      override_speed_limit_mps, smooth_lane_path);

  const double forward_extend_len =
      required_planning_horizon - lane_path.length();
  const auto forward_extend_lane_path =
      ForwardExtendLanePath(psmm, lane_path, forward_extend_len);
  if (forward_extend_len > 0.0 &&
      forward_extend_lane_path.back() == lane_path.back()) {
    SCOPED_QTRACE("BuildDrivePassageFromLanePath_1");
    // If no forward extension is made (meaning the lane path goes beyond loaded
    // map), fake the rest part of drive passage to the required length.
    const int n = CeilToInt(required_planning_horizon / step_s) + 1;
    dp_data.centers.reserve(n);
    dp_data.stations_boundaries.reserve(n);

    double last_accum_s = dp_data.centers.back().accum_s;
    constexpr double kMaxFittedLength = 100.0;         // m.
    constexpr double kMinLaneLengthForFitting = 40.0;  // m.
    const double fit_length =
        std::min(required_planning_horizon, kMaxFittedLength);
    if (last_accum_s < fit_length && last_accum_s >= kMinLaneLengthForFitting) {
      ExtendDrivePassageDataByCircleFit(fit_length, step_s, &dp_data);
    }

    last_accum_s = dp_data.centers.back().accum_s;
    while ((last_accum_s += step_s) <= required_planning_horizon) {
      auto center = dp_data.centers.back();
      center.lane_id = mapping::kInvalidElementId;
      center.fraction = 1.0;
      center.xy += center.tangent * step_s;
      center.accum_s = last_accum_s;
      center.is_virtual = true;

      dp_data.centers.emplace_back(std::move(center));
      dp_data.stations_boundaries.push_back(
          {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
           {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
    }
  }

  const double backward_fake_len = required_backward_len - start_s;
  std::vector<StationCenter> backward_centers;
  std::vector<std::vector<StationBoundary>> backward_boundaries;
  if (backward_fake_len > 0.0) {
    const int n = CeilToInt(backward_fake_len / step_s) + 1;
    backward_centers.reserve(n);
    backward_boundaries.reserve(n);

    double back_accum_s = 0.0;
    auto center = dp_data.centers.front();
    center.lane_id = mapping::kInvalidElementId;
    center.fraction = 1.0;
    center.is_virtual = true;
    while ((back_accum_s += step_s) <= backward_fake_len) {
      center.xy -= center.tangent * step_s;
      center.accum_s -= step_s;

      backward_centers.emplace_back(center);
      backward_boundaries.push_back(
          {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
           {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
    }
  }

  if (dp_data.centers.size() < 2) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Too few stations in BuildDrivePassageFromLanePath: Lane path: ",
        lane_path.DebugString(),
        "\nExtended lane path: ", extended_lane_path.DebugString()));
  }

  // Create station vector.
  StationVector<Station> stations;
  stations.reserve(dp_data.centers.size() + backward_centers.size());
  for (int i = backward_centers.size() - 1; i >= 0; --i) {
    backward_centers[i].accum_s -= start_s * (backward_extend_len > 0.0);
    stations.emplace_back(std::move(backward_centers[i]),
                          std::move(backward_boundaries[i]));
  }
  for (int i = 0; i < dp_data.centers.size(); ++i) {
    dp_data.centers[i].accum_s -= start_s * (backward_extend_len > 0.0);
    stations.emplace_back(std::move(dp_data.centers[i]),
                          std::move(dp_data.stations_boundaries[i]));
  }

  return DrivePassage(std::move(stations), lane_path,
                      std::move(extended_lane_path),
                      /*lane_path_start_s=*/0.0,
                      /*reach_destination=*/false, type);
}

absl::StatusOr<DrivePassage> BuildDrivePassage(
    const PlannerSemanticMapManager& psmm,
    const std::shared_ptr<PlannerSemanticMapManager>& vision_map_ptr,
    const mapping::LanePath& lane_path_from_pose,
    const mapping::LanePath& backward_extended_lane_path,
    const mapping::LanePoint& anchor_point, double planning_horizon,
    const mapping::LanePoint& destination, bool all_lanes_virtual,
    std::optional<double> override_speed_limit_mps, FrenetFrameType type) {
  SCOPED_QTRACE("BuildDrivePassage");

  auto lane_path_in_horizon =
      lane_path_from_pose.BeforeArclength(planning_horizon);

  // Forward and backward extend lane path. Forward for projection of objects
  // that are slightly beyond the current lane path's end, and backward for plan
  // start point projection of the next frame.
  const double forward_extend_len =
      std::min(planning_horizon - lane_path_from_pose.length(),
               kDrivePassageMaxForwardExtendLength);
  auto ref_lane_path = ForwardExtendLanePath(
      psmm,
      backward_extended_lane_path.BeforeFirstOccurrenceOfLanePoint(
          lane_path_in_horizon.back()),
      forward_extend_len);

  const double ref_ego_s = ref_lane_path.FirstOccurrenceOfLanePointToArclength(
      lane_path_from_pose.front());
  const double ref_anchor_s =
      ref_lane_path.ContainsLanePoint(anchor_point)
          ? ref_lane_path.FirstOccurrenceOfLanePointToArclength(anchor_point)
          : ref_ego_s;
  const double ref_neutral_s =
      ref_anchor_s +
      RoundToInt((ref_ego_s - ref_anchor_s) / kRouteStationUnitStep) *
          kRouteStationUnitStep;
  const double start_station_accum_s =
      -FloorToInt(ref_neutral_s / kRouteStationUnitStep) *
      kRouteStationUnitStep;
  const double start_station_ref_s = ref_neutral_s + start_station_accum_s;

  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
  const int n =
      CeilToInt((planning_horizon + ref_neutral_s) / kRouteStationUnitStep) + 1;
  centers.reserve(n);
  stations_boundaries.reserve(n);

  int station_idx = 0;
  double prev_station_angle = 0.0;
  double max_accumulated_angle_diff = 0.0;
  double min_accumulated_angle_diff = 0.0;

  double sample_s = start_station_ref_s;
  const double loaded_length = ref_lane_path.length();
  bool angle_diff_cutoff = false;

  const double base_speed_limit = psmm.QueryLaneSpeedLimitByFraction(
      lane_path_from_pose.front().lane_id(),
      lane_path_from_pose.front().fraction());
  bool is_override_speed_limit = override_speed_limit_mps.has_value();

  double far_station_thres =
      kFarStationHorizonRatio * planning_horizon + ref_ego_s;
  if (is_override_speed_limit) {
    far_station_thres =
        std::min(far_station_thres, kFarStationHeadwayTimeThreshold *
                                        override_speed_limit_mps.value());
  }
  while (sample_s <= loaded_length) {
    const auto sample_lane_point = ref_lane_path.ArclengthToLanePoint(sample_s);

    // Ensure we do not create a loop in drive passage. Planner does not support
    // looped passage.
    if (station_idx == 0) {
      prev_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
    } else {
      const auto cur_station_angle =
          ComputeLanePointTangent(psmm, sample_lane_point).FastAngle();
      const double angle_diff =
          NormalizeAngle(cur_station_angle - prev_station_angle);
      max_accumulated_angle_diff =
          std::max(max_accumulated_angle_diff + angle_diff, angle_diff);
      min_accumulated_angle_diff =
          std::min(min_accumulated_angle_diff + angle_diff, angle_diff);
      prev_station_angle = cur_station_angle;
    }
    if (max_accumulated_angle_diff >= kDrivePassageCutOffAngleDiff ||
        min_accumulated_angle_diff <= -kDrivePassageCutOffAngleDiff) {
      angle_diff_cutoff = true;
      break;
    }
    station_idx++;

    SMM_ASSIGN_LANE_OR_BREAK_ISSUE(lane_info, psmm,
                                   sample_lane_point.lane_id());

    const double current_speed_limit = psmm.QueryLaneSpeedLimitByFraction(
        sample_lane_point.lane_id(), sample_lane_point.fraction());
    const bool sample_s_is_front_ego = sample_s >= ref_ego_s;
    if (sample_s_is_front_ego) {
      is_override_speed_limit &=
          std::fabs(base_speed_limit - current_speed_limit) < kEpsilon;
    }

    // Station center:
    StationCenter center{
        .lane_id = sample_lane_point.lane_id(),
        .fraction = sample_lane_point.fraction(),
        .xy = ComputeLanePointPos(psmm, sample_lane_point),
        .tangent = ComputeLanePointTangent(psmm, sample_lane_point),
        .accum_s = sample_s - ref_neutral_s,
        .speed_limit = (sample_s_is_front_ego && is_override_speed_limit)
                           ? *override_speed_limit_mps
                           : current_speed_limit,
        .is_virtual = (all_lanes_virtual || lane_info.IsVirtual()),
        .is_merging = lane_info.IsMerging(),
        .is_in_intersection = lane_info.is_in_intersection,
        .direction = lane_info.direction};

    // Station boundaries:
    double right_lat_offset = -kMaxLateralOffset;
    double left_lat_offset = kMaxLateralOffset;
    if (!centers.empty()) {
      const auto offsets = ComputeLateralLimits(centers.back(), center);
      right_lat_offset = offsets.first;
      left_lat_offset = offsets.second;
    }
    auto current_station_boundaries = CollectStationBoundaries(
        psmm, center, right_lat_offset, left_lat_offset, kMaxLateralOffset,
        /*add_prediction_virtual_boundary=*/false);
    PostProcessStationBoundaries(&current_station_boundaries);

    // Update HD boundaries type by vision map boundaries type.
    if (vision_map_ptr != nullptr) {
      auto vision_map_boundaries =
          CollectStationBoundaries(*vision_map_ptr, center, right_lat_offset,
                                   left_lat_offset, kMaxLateralOffset,
                                   /*add_prediction_virtual_boundary=*/false);
      PostProcessStationBoundaries(&vision_map_boundaries);
      UpdateBoundariesTypeByVisionMap(vision_map_boundaries,
                                      &current_station_boundaries);
    }
    centers.push_back(std::move(center));
    stations_boundaries.push_back(std::move(current_station_boundaries));

    sample_s += sample_s >= far_station_thres ? kFarRouteStationStep
                                              : kRouteStationUnitStep;
  }

  // Keep lane speed consistency behind ego.
  const int ego_idx =
      CeilToInt((ref_ego_s - start_station_ref_s) / kRouteStationUnitStep);
  if (override_speed_limit_mps.has_value()) {
    for (int i = std::min(ego_idx, static_cast<int>(centers.size())) - 1;
         i >= 0; --i) {
      if (std::fabs(centers[i].speed_limit - base_speed_limit) < kEpsilon) {
        centers[i].speed_limit = *override_speed_limit_mps;
      } else {
        break;
      }
    }
  }

  if (!angle_diff_cutoff &&
      lane_path_in_horizon.back() == ref_lane_path.back()) {
    // If no forward extension is made (meaning the lane path goes beyond loaded
    // map), fake the rest part of drive passage to the required length.
    double last_accum_s = centers.back().accum_s;
    while ((last_accum_s += kFarRouteStationStep) <= planning_horizon) {
      auto center = centers.back();
      center.lane_id = mapping::kInvalidElementId;
      center.fraction = 1.0;
      center.xy += center.tangent * kFarRouteStationStep;
      center.accum_s = last_accum_s;
      center.is_virtual = true;

      centers.push_back(std::move(center));
      stations_boundaries.push_back(
          {{StationBoundaryType::VIRTUAL_CURB, -kMaxLateralOffset},
           {StationBoundaryType::VIRTUAL_CURB, kMaxLateralOffset}});
    }
  }
  if (centers.size() < 2) {
    return absl::FailedPreconditionError(
        absl::StrCat("Too few stations in BuildDrivePassage: Lane path: ",
                     lane_path_from_pose.DebugString(),
                     "\nExtended lane path: ", ref_lane_path.DebugString()));
  }

  // Create station vector.
  StationVector<Station> stations;
  stations.reserve(centers.size());
  for (int i = 0; i < centers.size(); ++i) {
    stations.emplace_back(std::move(centers[i]),
                          std::move(stations_boundaries[i]));
  }

  const bool reach_destination =
      lane_path_in_horizon.ContainsLanePoint(destination);
  return DrivePassage(std::move(stations), std::move(lane_path_in_horizon),
                      std::move(ref_lane_path),
                      /*lane_path_start_s=*/ref_ego_s - ref_neutral_s,
                      reach_destination, type);
}

}  // namespace qcraft::planner
