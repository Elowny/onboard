#include "onboard/planner/assist/planner_odc_generator.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

constexpr double kCheckAvBoxPreviewTime = 0.8;  // s.
constexpr double kDrivePassageStepS = 1.0;      // m.
constexpr double kEpsilon = 1e-5;
// kMinCheckLaneLength is used to build drivepassage, make sure project av box
// to dp is ok.
constexpr double kMinProjectLaneLength = 10.0;
// kCheckLaneLength is used to calc average curvature radius and average
// lane width.
constexpr double kCheckLaneLength = 20.0;            // m.
constexpr double kMinOdcCheckLanePathLength = 10.0;  // m.

absl::StatusOr<RouteRelatedOdc> CalculateRouteRelatedOdc(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections_from_start) {
  SCOPED_QTRACE("CalculateRouteRelatedOdc");

  std::optional<double> distance_to_toll = std::nullopt;
  std::optional<double> distance_to_traffic_light = std::nullopt;
  const int n = route_sections_from_start.size();
  double accumulate_s = 0.0;
  for (int i = 0; i < n; ++i) {
    const auto& section_segment =
        route_sections_from_start.route_section_segment(i);
    const auto* section_info_ptr =
        psmm.FindSectionInfoOrNull(section_segment.id);
    if (section_info_ptr == nullptr) continue;

    for (const auto lane_id : section_info_ptr->lane_ids) {
      const auto* lane_proto_ptr = psmm.FindLaneByIdOrNull(lane_id);
      if (lane_proto_ptr == nullptr) continue;

      if (lane_proto_ptr->endpoint_toll() == true &&
          !distance_to_toll.has_value()) {
        distance_to_toll = accumulate_s;
      }

      const bool is_traffic_light_control_lane =
          !lane_proto_ptr->multi_traffic_light_control_points().empty() ||
          !lane_proto_ptr->startpoint_associated_traffic_lights().empty();

      if (is_traffic_light_control_lane &&
          !distance_to_traffic_light.has_value()) {
        distance_to_traffic_light = accumulate_s;
      }

      if (distance_to_toll.has_value() &&
          distance_to_traffic_light.has_value()) {
        return RouteRelatedOdc{distance_to_toll, distance_to_traffic_light};
      }
    }

    accumulate_s +=
        (section_segment.end_fraction - section_segment.start_fraction) *
        section_info_ptr->average_length;
  }
  return RouteRelatedOdc{distance_to_toll, distance_to_traffic_light};
}

double CalculateAverageCurvatureRadiusOfLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  constexpr double kSampleStep = 5.0;  // m.
  double sample_step = kSampleStep;
  // Make sure at least 3 points to calculate curvature.
  constexpr int kMinPointsNum = 3;
  if (lane_path.length() < (kMinPointsNum - 1) * sample_step) {
    sample_step = (lane_path.length() / (kMinPointsNum - 1));
  }
  const int size = FloorToInt(lane_path.length() / sample_step) + 1;

  const auto calc_sample_point = [&psmm, &lane_path](double s) -> Vec2d {
    return ArclengthToPos(psmm, lane_path, s);
  };

  double sum_abs_curvature = 0.0;
  Vec2d prev_point = calc_sample_point(sample_step);
  double prev_theta = (prev_point - calc_sample_point(/*s=*/0.0)).Angle();
  int i = 2;
  for (; i < size; ++i) {
    const double s = i * sample_step;
    if (s > kCheckLaneLength) break;

    const Vec2d cur_point = calc_sample_point(s);
    const double cur_theta = (cur_point - prev_point).Angle();
    const double abs_curvature = std::fabs(
        AngleDifference(prev_theta, cur_theta) / (sample_step + kEpsilon));
    sum_abs_curvature += abs_curvature;
    prev_theta = cur_theta;
    prev_point = cur_point;
  }

  // NOTE(jiayu): i is greater or equal than 3.
  const double average_abs_curvature = sum_abs_curvature / (i - 2);
  return average_abs_curvature > kEpsilon ? 1.0 / average_abs_curvature
                                          : 1.0 / kEpsilon;
}

bool IsValidBoundaryType(StationBoundaryType type) {
  switch (type) {
    case StationBoundaryType::SOLID_WHITE:
    case StationBoundaryType::SOLID_YELLOW:
    case StationBoundaryType::SOLID_DOUBLE_YELLOW:
    case StationBoundaryType::CURB:
    case StationBoundaryType::BROKEN_WHITE:
    case StationBoundaryType::BROKEN_YELLOW:
    case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
    case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
      return true;
    case StationBoundaryType::UNKNOWN_TYPE:
    case StationBoundaryType::VIRTUAL_CURB:
    case StationBoundaryType::PREDICTION_VIRTUAL_CURB:
      return false;
  }
}

bool IsValidBothBoundaryType(const BoundaryQueryResponse& boundaries) {
  return boundaries.right.has_value() && boundaries.left.has_value() &&
         IsValidBoundaryType(boundaries.right->type) &&
         IsValidBoundaryType(boundaries.left->type);
}

absl::StatusOr<bool> IsValidAvBothSideBoundary(
    const DrivePassage& drive_passage, const PoseProto& pose,
    const VehicleGeometryParamsProto& vehicle_geom) {
  const Box2d ego_box =
      ComputeAvBox(Vec2dFromPoseProto(pose), pose.yaw(), vehicle_geom);

  ASSIGN_OR_RETURN(const auto frenet_box,
                   drive_passage.QueryFrenetBoxAt(ego_box));
  const double ego_s = frenet_box.center_s();
  constexpr double kCheckBoundaryValidationStep = 2.0;  // m.
  for (double s = ego_s; s < ego_s + kCheckLaneLength;
       s += kCheckBoundaryValidationStep) {
    const auto boundaries = drive_passage.QueryEnclosingLaneBoundariesAtS(s);
    if (!IsValidBothBoundaryType(boundaries)) return false;
  }

  return true;
}

double CalculateLaneWidth(const BoundaryQueryResponse& boundaries) {
  return boundaries.left->lat_offset - boundaries.right->lat_offset;
}

absl::StatusOr<double> CalculateAverageLaneWidth(
    const DrivePassage& drive_passage) {
  int valid_boundaries_count = 0;
  double accumulate_lane_width = 0.0;

  for (const auto& station : drive_passage.stations()) {
    if (station.accumulated_s() > kCheckLaneLength) break;

    ASSIGN_OR_BREAK(const auto boundaries,
                    station.QueryEnclosingLaneBoundariesAt(/*signed_lat=*/0.0));
    if (IsValidBothBoundaryType(boundaries)) {
      ++valid_boundaries_count;
      accumulate_lane_width += station.is_virtual()
                                   ? kDefaultLaneWidth
                                   : CalculateLaneWidth(boundaries);
    }
  }

  if (valid_boundaries_count == 0) {
    // NOTE(jiayu): Never return error.
    return absl::InternalError(
        "[ODC]: No valid boundaries to calculate lane width.");
  }
  return accumulate_lane_width / valid_boundaries_count;
}

bool IsBoxCrossBoundary(const DrivePassage& drive_passage,
                        const Box2d& ego_box) {
  ASSIGN_OR_RETURN(const auto frenet_box,
                   drive_passage.QueryFrenetBoxAt(ego_box), false);

  double right_l = -std::numeric_limits<double>::infinity();
  double left_l = std::numeric_limits<double>::infinity();
  const auto& station =
      drive_passage.FindNearestStationAtS(frenet_box.center().s);
  for (const auto& bound : station.boundaries()) {
    if (bound.lat_offset < -kMinHalfLaneWidth && bound.lat_offset > right_l) {
      right_l = bound.lat_offset;
    } else if (kMinHalfLaneWidth < bound.lat_offset &&
               bound.lat_offset < left_l) {
      left_l = bound.lat_offset;
    }
  }

  return frenet_box.l_max > left_l || frenet_box.l_min < right_l;
}

void CheckAvCrossedBoundary(const DrivePassage& drive_passage,
                            const PoseProto& pose,
                            const VehicleGeometryParamsProto& vehicle_geom) {
  // Check current av box.
  constexpr double kEgoBoxBuffer = 0.2;
  const Box2d curr_box_with_buffer =
      ComputeAvBoxWithBuffer(Vec2dFromPoseProto(pose), pose.yaw(), vehicle_geom,
                             -kEgoBoxBuffer, -kEgoBoxBuffer);
  if (IsBoxCrossBoundary(drive_passage, curr_box_with_buffer)) {
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_WARNING, QIssueType::QIT_BUSINESS,
                      QIssueSubType::QIST_PLANNER_VEHICLE_CROSS_BOUNDARY,
                      "ODC check failed: ", "Current ego box out of boundary.");
  }

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
    accel = 0.0;
  } else {
    curr_path_point.set_kappa(pose.ar_smooth().z() / speed);
  }

  // Check av box with preview.
  for (double t = 0.0; t <= kCheckAvBoxPreviewTime; t += kTrajectoryTimeStep) {
    const Box2d ego_box = ComputeAvBox(Vec2dFromPathPoint(curr_path_point),
                                       curr_path_point.theta(), vehicle_geom);

    if (IsBoxCrossBoundary(drive_passage, ego_box)) {
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_WARNING, QIssueType::QIT_BUSINESS,
          QIssueSubType::QIST_PLANNER_VEHICLE_CROSS_BOUNDARY_WITH_PREVIEW,
          "ODC check failed: ",
          absl::StrCat("Ego box out of boundary. Preview time ", t));
      return;
    }

    speed += accel * kTrajectoryTimeStep;
    curr_path_point =
        GetPathPointAlongCircle(curr_path_point, speed * kTrajectoryTimeStep);
  }
}

bool IsAvOnEmergencyLane(const PlannerSemanticMapManager& psmm,
                         const DrivePassage& drive_passage, const Vec2d& pos) {
  const auto nearest_lane_id = drive_passage.FindNearestStation(pos).lane_id();
  const auto* nearest_lane_proto = psmm.FindLaneByIdOrNull(nearest_lane_id);
  if (nearest_lane_proto == nullptr ||
      nearest_lane_proto->has_type() == false) {
    return false;
  }
  return nearest_lane_proto->type() == mapping::LaneProto::EMERGENCY;
}

double CalcUniformAccelerationDistance(double v, double a, double t) {
  return v * t + 0.5 * a * t * t;
}

absl::StatusOr<OdcGeneratorOutput> GenerateOnRoadOdc(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    const PoseProto& pose, const VehicleGeometryParamsProto& vehicle_geom,
    bool is_lane_changing, bool is_acc_engage_only) {
  OdcGeneratorOutput output;
  output.is_acc_engage_only = is_acc_engage_only;
  output.current_lane_length = lane_path.length();
  output.lane_path_lost =
      output.current_lane_length < CalculateLccMinRequiredLaneLength(pose);
  // Calculate min curvature radius.
  output.current_lane_average_curvature_radius =
      CalculateAverageCurvatureRadiusOfLanePath(psmm, lane_path);

  const double proj_length =
      CalcUniformAccelerationDistance(
          pose.vel_body().x(),
          std::max<double>(pose.accel_body().x(), kEpsilon),
          kCheckAvBoxPreviewTime) +
      kMinProjectLaneLength;
  const auto proj_lane_path = lane_path.BeforeArclength(proj_length);
  ASSIGN_OR_RETURN(
      const auto drive_passage,
      BuildDrivePassageFromLanePath(
          psmm, proj_lane_path, kDrivePassageStepS,
          /*avoid_loop=*/true, kDrivePassageKeepBehindLength,
          /*required_planning_horizon=*/0.0, kDrivePassageKeepBehindLength,
          /*override_speed_limit_mps=*/std::nullopt,
          FrenetFrameType::kBruteFroce));

  // Check AV position to lane.
  if (!is_lane_changing) {
    CheckAvCrossedBoundary(drive_passage, pose, vehicle_geom);
  }
  const auto tangent = drive_passage.QueryTangentAt(Vec2dFromPoseProto(pose));
  if (tangent.ok()) {
    constexpr double kMaxAllowedHeading = 45.0 / 180.0 * M_PI;  // radius.
    const auto heading_diff =
        std::fabs(AngleDifference(tangent->FastAngle(), pose.yaw()));
    if (heading_diff >= kMaxAllowedHeading) {
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_WARNING, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_PLANNER_LARGE_HEADING_DIFF_TO_LANE,
                        "ODC check failed: ", "Heading diff is too large.");
    }
  }

  // Check both side boundary validation.
  if (output.lane_path_lost == true) {
    // Force reset when lane path lost.
    output.is_valid_both_side_boundary = false;
  } else {
    ASSIGN_OR_RETURN(
        output.is_valid_both_side_boundary,
        IsValidAvBothSideBoundary(drive_passage, pose, vehicle_geom));
  }

  if (output.is_valid_both_side_boundary == true) {
    // Calculate average lane width.
    ASSIGN_OR_RETURN(output.current_lane_width,
                     CalculateAverageLaneWidth(drive_passage));
  }

  // Check av on emergency lane.
  output.is_av_in_emergency_lane =
      IsAvOnEmergencyLane(psmm, drive_passage, Vec2dFromPoseProto(pose));

  return output;
}

}  // namespace

double CalculateLccMinRequiredLaneLength(const PoseProto& pose) {
  constexpr double kMinRequiredLaneLength = 15.0;  // .m
  constexpr double kMaxRequiredLaneLength = 40.0;  // .m
  constexpr double kMinRequiredTime = 2.0;         // .s
  const double speed = std::max(pose.vel_body().x(), 0.0);
  const double acc = pose.accel_body().x();

  const double consider_time =
      acc < 0.0 ? std::min(-speed / acc, kMinRequiredTime) : kMinRequiredTime;

  const double min_required_lane_length =
      speed * consider_time + 0.5 * acc * consider_time * consider_time;

  return std::clamp(min_required_lane_length, kMinRequiredLaneLength,
                    kMaxRequiredLaneLength);
}

absl::StatusOr<OdcGeneratorOutput> GeneratePlannerOdcByPose(
    const OdcGeneratorInput& input) {
  SCOPED_QTRACE("GenerateOdcByPose");

  const auto* psmm = input.psmm;
  const auto& pose = *input.pose;
  const auto& vehicle_geom = *input.vehicle_geom;

  OdcGeneratorOutput output;

  if (input.is_acc_engage_only) {
    output.is_acc_engage_only = true;
  }

  if (psmm == nullptr) return output;

  constexpr double kMaxOdcCheckLanePathLength = 100.0;  // m.
  auto lane_path_or =
      FindNearestLanePathFromEgoPose(pose, *psmm, kMaxOdcCheckLanePathLength);
  if (!lane_path_or.ok() ||
      lane_path_or->length() < kMinOdcCheckLanePathLength) {
    output.lane_path_lost = true;
    return output;
  }

  // Check lateral distance to lane path.
  constexpr double kSampleLaneStep = 1.0;  // m.
  ASSIGN_OR_RETURN(const auto points,
                   SampleLanePathByStep(*psmm, *lane_path_or, kSampleLaneStep));
  ASSIGN_OR_RETURN(
      const auto ff,
      BuildBruteForceFrenetFrame(points, /*down_sample_raw_points=*/false));

  const auto ego_pos_sl = ff.XYToSL(Vec2dFromPoseProto(pose));
  if (std::fabs(ego_pos_sl.l) > kDefaultHalfLaneWidth) {
    output.lane_path_lost = true;
    return output;
  }

  // Generate on road odc.
  const auto lane_path = std::move(lane_path_or).value();
  ASSIGN_OR_RETURN(
      output,
      GenerateOnRoadOdc(*psmm, lane_path, pose, vehicle_geom,
                        input.is_lane_changing, input.is_acc_engage_only),
      _ << "GeneratePlannerOdcByPose failed:\npose: " << pose.DebugString()
        << "\ntarget lane path: " << lane_path.DebugString());

  return output;
}

absl::StatusOr<OdcGeneratorOutput> GeneratePlannerOdcByLanePath(
    const OdcGeneratorInput& input) {
  SCOPED_QTRACE("GeneratePlannerOdcByLanePath");

  const auto& psmm = *input.psmm;
  const auto& pose = *input.pose;
  const auto& vehicle_geom = *input.vehicle_geom;
  const auto& target_lane_path = *input.target_lane_path;
  if (target_lane_path.IsEmpty()) {
    return absl::InvalidArgumentError("[ODC]: Lane path is empty.");
  }

  // Align lane path to pos.
  ASSIGN_OR_RETURN(
      const auto aligned_lane_path,
      ClampLanePathFromPos(psmm, target_lane_path, Vec2dFromPoseProto(pose)));

  OdcGeneratorOutput output;
  // Generate on road odc.
  ASSIGN_OR_RETURN(
      output,
      GenerateOnRoadOdc(psmm, aligned_lane_path, pose, vehicle_geom,
                        input.is_lane_changing, input.is_acc_engage_only),
      _ << "GeneratePlannerOdcByLanePath failed:\npose: " << pose.DebugString()
        << "\ntarget lane path: " << target_lane_path.DebugString());

  return output;
}

absl::StatusOr<RouteRelatedOdc> GeneratePlannerOdcByRoute(
    const PlannerSemanticMapManager* psmm,
    const RouteManagerOutput* route_mgr_output) {
  if (psmm == nullptr || route_mgr_output == nullptr) {
    return absl::FailedPreconditionError("[ODC]: Psmm or route is empty.");
  }

  if (!HasValidRouteResults(*route_mgr_output)) return RouteRelatedOdc();

  return CalculateRouteRelatedOdc(
      *psmm, route_mgr_output->route_sections_from_current);
}
}  // namespace qcraft::planner
