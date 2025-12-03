#include "onboard/planner/util/hmi_content_util.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kBoundaryResampleStep = 10;

bool IsYellowLaneBoundary(StationBoundaryType type) {
  switch (type) {
    case StationBoundaryType::BROKEN_YELLOW:
    case StationBoundaryType::SOLID_YELLOW:
    case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      return true;
    case StationBoundaryType::VIRTUAL_CURB:
    case StationBoundaryType::PREDICTION_VIRTUAL_CURB:
    case StationBoundaryType::CURB:
    case StationBoundaryType::SOLID_WHITE:
    case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
    case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
    case StationBoundaryType::UNKNOWN_TYPE:
    case StationBoundaryType::BROKEN_WHITE:
      return false;
  }
}

bool CheckTrajectoryCrossingLeftYellowLaneBoundary(
    const DrivePassage& drive_passage,
    const std::vector<ApolloTrajectoryPointProto>& traj_points) {
  for (int i = 0; i < traj_points.size(); ++i) {
    const auto path_point_xy = ToVec2d(traj_points[i].path_point());
    ASSIGN_OR_CONTINUE(const auto path_point_sl,
                       drive_passage.QueryFrenetCoordinateAt(path_point_xy));
    const auto [_, left_boundary] =
        drive_passage.QueryEnclosingLaneBoundariesAtS(path_point_sl.s);
    if (left_boundary.has_value() &&
        IsYellowLaneBoundary(left_boundary->type) &&
        path_point_sl.l > left_boundary->lat_offset) {
      return true;
    }
  }
  return false;
}

HmiContentProto::MergeAlarmType CalcMergeAlarmByLaneFraction(
    double start_fraction) {
  constexpr double kMergeDangerFractionThreshold = 0.5;
  constexpr double kMergeTakeoverFractionThreshold = 0.7;
  if (start_fraction > kMergeTakeoverFractionThreshold) {
    return HmiContentProto::MERGE_ALARM_TAKEOVER;
  }
  if (start_fraction > kMergeDangerFractionThreshold) {
    return HmiContentProto::MERGE_ALARM_DANGER;
  }
  return HmiContentProto::MERGE_ALARM_NONE;
}

absl::StatusOr<HmiContentProto::MergeAlarmType> CalcMergeAlarmByLaneWidth(
    const PlannerSemanticMapManager& psmm, const mapping::LaneInfo& lane_info,
    double fraction, double av_half_width) {
  if (lane_info.lane_boundaries_on_left.empty() ||
      lane_info.lane_boundaries_on_right.empty()) {
    return absl::NotFoundError("Cannot found left or right boundaries.");
  }

  const auto xy = lane_info.LerpPointFromFraction(fraction);
  const auto tangent = lane_info.GetTangent(fraction);
  const auto calc_lateral_point = [&xy,
                                   &tangent](double signed_offset) -> Vec2d {
    return xy + tangent.Perp() * signed_offset;
  };
  constexpr double kMaxHalfMergeLaneWidth = 5.0;  // m
  const Segment2d normal_line(calc_lateral_point(-kMaxHalfMergeLaneWidth),
                              calc_lateral_point(kMaxHalfMergeLaneWidth));

  const auto calc_lateral_offset = [&xy,
                                    &tangent](const Vec2d& point) -> double {
    return tangent.CrossProd(point - xy);
  };
  const auto calc_boundary_offset =
      [&psmm, &calc_lateral_offset,
       &normal_line](const std::vector<mapping::LaneBoundaryIncidenceInfo>&
                         lane_boundaries) -> std::optional<double> {
    std::optional<double> offset = std::nullopt;
    for (const auto& boundary : lane_boundaries) {
      const auto* boundary_info =
          psmm.FindLaneBoundaryByIdOrNull(boundary.lane_boundary_id);
      if (boundary_info == nullptr) continue;
      const auto& points_smooth = boundary_info->points_smooth;
      for (int k = 0; k + 1 < points_smooth.size(); ++k) {
        const Vec2d& p0 = points_smooth[k];
        const Vec2d& p1 = points_smooth[k + 1];
        const Segment2d boundary_segment(p0, p1);
        Vec2d intersection;
        if (normal_line.GetIntersect(boundary_segment, &intersection)) {
          offset = calc_lateral_offset(intersection);
          break;
        }
      }
      if (offset.has_value()) break;
    }
    return offset;
  };

  std::optional<double> left_offset =
      calc_boundary_offset(lane_info.lane_boundaries_on_left);
  std::optional<double> right_offset =
      calc_boundary_offset(lane_info.lane_boundaries_on_right);

  if (!left_offset.has_value() || !right_offset.has_value()) {
    return absl::NotFoundError("Cannot calc lane width.");
  }

  if (*left_offset > av_half_width) return HmiContentProto::MERGE_ALARM_NONE;
  const double lane_width = *left_offset - *right_offset;
  constexpr double kMergeDangerLaneWidthThreshold = 2.5;    // m.
  constexpr double kMergeTakeoverLaneWidthThreshold = 2.2;  // m.
  if (lane_width < kMergeTakeoverLaneWidthThreshold) {
    return HmiContentProto::MERGE_ALARM_TAKEOVER;
  }
  if (lane_width < kMergeDangerLaneWidthThreshold) {
    return HmiContentProto::MERGE_ALARM_DANGER;
  }
  return HmiContentProto::MERGE_ALARM_NONE;
}
}  // namespace

HmiContentProto ReportHmiContent(const HmiContentInput& input) {
  SCOPED_QTRACE("ReportHmiContent");
  const auto& psmm = *QCHECK_NOTNULL(input.psmm);
  const auto& lane_path = *QCHECK_NOTNULL(input.lane_path);

  HmiContentProto hmi_proto;
  if (lane_path.IsEmpty() || input.drive_passage == nullptr ||
      input.traj_points == nullptr) {
    return hmi_proto;
  }

  const auto& current_lane_id = lane_path.front().lane_id();
  SMM_ASSIGN_LANE_OR_RETURN(current_lane_info, psmm, current_lane_id,
                            hmi_proto);
  const auto* current_lane_proto = current_lane_info.proto;
  if (current_lane_proto == nullptr) return hmi_proto;

  if (current_lane_proto->has_direction()) {
    // Writing lane path direction info to hmi turning message.
    const auto& direction = current_lane_proto->direction();
    int turning = 0;
    switch (direction) {
      case mapping::LaneProto::LEFT_TURN:
        turning = 1;
        break;
      case mapping::LaneProto::RIGHT_TURN:
        turning = -1;
        break;
      case mapping::LaneProto::UTURN:
        turning = 1;
        break;
      case mapping::LaneProto::STRAIGHT:
        break;
    }
    hmi_proto.set_turning(turning);
  }

  // Adding unprotected left turning driving state to hmi content.
  if (current_lane_proto->has_left_turn_protect() &&
      current_lane_proto->left_turn_protect() ==
          mapping::LaneProto::UNPROTECTED) {
    hmi_proto.add_driving_states(HmiContentProto::DS_UNPROTECTED_LEFT_TURNING);
  }

  // Adding borrow pposite lane driving state to hmi content.
  if (input.borrow_lane && IsLeftMostLane(psmm, current_lane_id) &&
      CheckTrajectoryCrossingLeftYellowLaneBoundary(*input.drive_passage,
                                                    *input.traj_points)) {
    hmi_proto.add_driving_states(HmiContentProto::DS_BORROW_OPPOSITE_LANE);
  }

  // Adding overtaking driving state to hmi conntent.
  if (input.lane_change_type == LaneChangeType::OVERTAKE_CHANGE) {
    hmi_proto.add_driving_states(HmiContentProto::DS_OVERTAKING);
  }

  if (input.alerted_front_vehicle != nullptr) {
    auto& alerted_front_vehicle = *hmi_proto.add_highlight_objects();
    alerted_front_vehicle.set_object_id(*input.alerted_front_vehicle);
    alerted_front_vehicle.set_type(
        HmiContentProto::HighlightObjectInfo::HLO_ACC_FOLLOWING_TARGET);
  }

  hmi_proto.set_collision_warning_request(input.collision_warning_request);

  if (input.request_help_lane_change_by_route == true) {
    hmi_proto.add_driving_states(
        HmiContentProto::DS_REQUEST_HELP_LANE_CHANGE_BY_ROUTE);
  }

  if (input.distance_to_traffic_light_stop_line.has_value()) {
    hmi_proto.set_distance_to_traffic_light_stop_line(
        *input.distance_to_traffic_light_stop_line);
  }

  if (input.unsafe_object_ids != nullptr) {
    for (const auto& object_id : *input.unsafe_object_ids) {
      auto& unsafe_object = *hmi_proto.add_highlight_objects();
      unsafe_object.set_object_id(object_id);
      unsafe_object.set_type(
          HmiContentProto::HighlightObjectInfo::HLO_LANE_CHANGE_UNSAFE);
    }
  }

  if (input.plc_target_lane_path != nullptr &&
      !input.plc_target_lane_path->IsEmpty()) {
    input.plc_target_lane_path->ToProto(
        hmi_proto.mutable_plc_target_lane_path());
  }

  if (input.distance_to_roadblock.has_value()) {
    hmi_proto.set_distance_to_roadblock(*input.distance_to_roadblock);
  }

  if (input.nudge_object_info != nullptr) {
    auto& nudge_object = *hmi_proto.add_highlight_objects();
    nudge_object.set_object_id(input.nudge_object_info->id);
    if (input.nudge_object_info->type == NudgeOjbectInfo::Type::kNormal) {
      nudge_object.set_type(
          HmiContentProto::HighlightObjectInfo::HLO_NUDGE_OBJECT);
    } else {
      nudge_object.set_type(
          HmiContentProto::HighlightObjectInfo::HLO_NUDGE_LARGE_VEHICLE);
    }
    nudge_object.set_dist_to_object(
        input.nudge_object_info->arc_dist_to_object);
  }

  if (input.all_trajectories_blocked.has_value()) {
    hmi_proto.set_all_trajectories_blocked(*input.all_trajectories_blocked);
  }

  hmi_proto.set_lc_left(input.lc_left);
  hmi_proto.set_lane_change_type(input.lane_change_type);
  hmi_proto.set_lane_change_general_type(input.lane_change_general_type);
  hmi_proto.set_lane_change_stage(input.lane_change_stage);
  hmi_proto.set_lane_change_for_obstacle_fail(
      input.lane_change_for_obstacle_fail);

  if (current_lane_info.IsMerging() &&
      current_lane_proto->type() == mapping::LaneProto::ENTRANCE) {
    const double start_fraction = lane_path.start_fraction();
    const auto merge_alarm_or = CalcMergeAlarmByLaneWidth(
        psmm, current_lane_info, start_fraction, input.av_half_width);
    if (merge_alarm_or.ok()) {
      hmi_proto.set_merge_alarm_type(*merge_alarm_or);
    } else {
      hmi_proto.set_merge_alarm_type(
          CalcMergeAlarmByLaneFraction(start_fraction));
    }
  }
  return hmi_proto;
}

HmiPathBoundaryProto ReportBoundaryPointsToHmiContent(
    const std::vector<Vec2d>& points, bool is_left,
    HmiPathBoundaryProto::BoundaryRenderStyle style) {
  HmiPathBoundaryProto path_boundary;
  if (points.empty()) return path_boundary;

  if (is_left) {
    path_boundary.set_left_render_style(style);
  } else {
    path_boundary.set_right_render_style(style);
  }

  const int size = points.size();
  const int resample_size = size / kBoundaryResampleStep + 2;
  auto* boundary_ptr = is_left ? path_boundary.mutable_left_boundary()
                               : path_boundary.mutable_right_boundary();
  boundary_ptr->Reserve(resample_size);
  for (int i = 0; i < size; ++i) {
    // Collect sample points and last point.
    if (!(i % kBoundaryResampleStep == 0 || i == size - 1)) continue;
    Vec2dToProto(points[i], boundary_ptr->Add());
  }
  return path_boundary;
}

HmiPathBoundaryProto ReportPathBoundaryToHmiContent(
    const PathSlBoundary* sl_boundary,
    HmiPathBoundaryProto::BoundaryRenderStyle left_style,
    HmiPathBoundaryProto::BoundaryRenderStyle right_style) {
  HmiPathBoundaryProto path_boundary;
  if (sl_boundary == nullptr || sl_boundary->IsEmpty()) return path_boundary;

  path_boundary.set_left_render_style(left_style);
  path_boundary.set_right_render_style(right_style);

  const int size = sl_boundary->size();
  const int resample_size = size / kBoundaryResampleStep + 2;
  path_boundary.mutable_left_boundary()->Reserve(resample_size);
  path_boundary.mutable_right_boundary()->Reserve(resample_size);

  constexpr double kMaxBoundaryRenderDist = 50.0;  // m.
  for (int i = 0; i < size; ++i) {
    // Collect sample points and last point.
    if (!(i % kBoundaryResampleStep == 0 || i == size - 1)) continue;
    Vec2dToProto(sl_boundary->left_xy_vector()[i],
                 path_boundary.add_left_boundary());
    Vec2dToProto(sl_boundary->right_xy_vector()[i],
                 path_boundary.add_right_boundary());
    if (sl_boundary->s_vector()[i] > kMaxBoundaryRenderDist) break;
  }
  return path_boundary;
}

}  // namespace planner
}  // namespace qcraft
