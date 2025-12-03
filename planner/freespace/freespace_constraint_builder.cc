#include "onboard/planner/freespace/freespace_constraint_builder.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

bool IsMapRegionContainsAv(const VehicleGeometryParamsProto& /*vehicle_geom*/,
                           const PoseProto& ego_pose,
                           const AABox2d& map_aabox) {
  if (ego_pose.pos_smooth().x() < map_aabox.min_x() ||
      ego_pose.pos_smooth().x() > map_aabox.max_x() ||
      ego_pose.pos_smooth().y() < map_aabox.min_y() ||
      ego_pose.pos_smooth().y() > map_aabox.max_y()) {
    return false;
  }
  return true;
}

absl::StatusOr<ConstraintProto::PathStopLineProto> AddEndOfLocalPathConstraint(
    const VehicleGeometryParamsProto& veh_geo_params,
    const DirectionalPath& path) {
  if (path.path.empty()) {
    return absl::InternalError("Local path empty.");
  }
  constexpr double kEndOfPathExtensionDistance = 0.1;  // m.
  const auto stop_point_pose =
      GetPathPointAlongCircle(path.path.back(), kEndOfPathExtensionDistance);
  const auto unit = Vec2d::FastUnitFromAngle(stop_point_pose.theta());
  const auto perp = unit.Perp();
  const Vec2d pos(stop_point_pose.x(), stop_point_pose.y());
  const Vec2d center =
      pos + unit * (path.forward ? veh_geo_params.front_edge_to_center()
                                 : veh_geo_params.back_edge_to_center());

  constexpr double kHalfPlaneHalfWidth = 3.0;  // m.
  const HalfPlane halfplane(center - perp * kHalfPlaneHalfWidth,
                            center + perp * kHalfPlaneHalfWidth);

  ConstraintProto::PathStopLineProto stop_line;
  // For path stop line, s means the path s for rac.
  stop_line.set_s(stop_point_pose.s());
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id("end_of_local_path");
  stop_line.mutable_source()->mutable_end_of_local_path()->set_reason(
      "End of local path.");
  return stop_line;
}

absl::StatusOr<ConstraintProto::PathStopLineProto> AddForceStopConstraint(
    const VehicleGeometryParamsProto& veh_geo_params,
    const DirectionalPath& path, const PoseProto& ego_pose,
    FreespacePlannerStateProto* state) {
  if (path.path.empty()) {
    return absl::InternalError("Local path empty.");
  }
  const double abs_ego_vel_x = std::abs(ego_pose.vel_body().x());
  PathPoint stop_point_pose;
  constexpr double kStationaryVelThres = 0.05;
  if (abs_ego_vel_x < kStationaryVelThres) {
    stop_point_pose = path.path.front();
    state->clear_prev_force_stop_point();
  } else {
    if (state->has_prev_force_stop_point()) {
      const auto& point = state->prev_force_stop_point();
      const auto stop_point_sl = path.path.XYToSL(Vec2d(point.x(), point.y()));
      stop_point_pose = path.path.Evaluate(stop_point_sl.s);
    } else {
      constexpr double kDecelLatency = 0.5;    // s.
      constexpr double kForceStopDecel = 0.3;  // m/s2.
      const double stop_point_s =
          abs_ego_vel_x * kDecelLatency +
          0.5 * abs_ego_vel_x * abs_ego_vel_x / kForceStopDecel;
      stop_point_pose = path.path.Evaluate(stop_point_s);

      *state->mutable_prev_force_stop_point() = stop_point_pose;
    }
  }
  const auto unit = Vec2d::FastUnitFromAngle(stop_point_pose.theta());
  const auto perp = unit.Perp();
  const Vec2d pos(stop_point_pose.x(), stop_point_pose.y());
  const Vec2d center =
      pos + unit * (path.forward ? veh_geo_params.front_edge_to_center()
                                 : veh_geo_params.back_edge_to_center());

  constexpr double kHalfPlaneHalfWidth = 3.0;  // m.
  const HalfPlane halfplane(center - perp * kHalfPlaneHalfWidth,
                            center + perp * kHalfPlaneHalfWidth);

  ConstraintProto::PathStopLineProto stop_line;
  // For path stop line, s means the path s for rac.
  stop_line.set_s(stop_point_pose.s());
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id("force_stop");
  stop_line.mutable_source()->mutable_end_of_local_path()->set_reason(
      "Force stop.");
  return stop_line;
}

bool IsLaneBoundaryNearParkingSpot(
    const mapping::ParkingSpotInfo* parking_spot_info,
    const std::vector<Vec2d>& boundary) {
  constexpr double kNearSegmentsDist = 0.3;
  if (parking_spot_info == nullptr) return false;
  const Polygon2d& spot = parking_spot_info->polygon();
  for (int i = 0; i + 1 < boundary.size(); ++i) {
    Segment2d segment(boundary[i], boundary[i + 1]);
    if (spot.min_x() > segment.max_x() + kNearSegmentsDist ||
        spot.min_y() > segment.max_y() + kNearSegmentsDist ||
        spot.max_x() < segment.min_x() - kNearSegmentsDist ||
        spot.max_y() < segment.min_y() - kNearSegmentsDist) {
      continue;
    }
    if (spot.DistanceTo(segment) < kNearSegmentsDist) return true;
  }
  return false;
}

}  // namespace

absl::StatusOr<FreespaceMap> ConstructFreespaceMap(
    FreespaceTaskProto::TaskType task_type, double freespace_region_half_width,
    const VehicleGeometryParamsProto& vehicle_geom,
    const PlannerSemanticMapManager* psmm, const PoseProto& ego_pose,
    const mapping::ParkingSpotInfo* parking_spot_info, const PathPoint& goal) {
  SCOPED_QTRACE("ConstructParkingFreespaceMap");
  FreespaceMap freespace_map;
  using mapping::ParkingSpotInfo;
  // Construct freespace region.
  const auto goal_pos = ToVec2d(goal);
  freespace_map.region = AABox2d(freespace_region_half_width,
                                 freespace_region_half_width, goal_pos);
  if (!IsMapRegionContainsAv(vehicle_geom, ego_pose, freespace_map.region)) {
    // TODO(yumeng): Consider extend region.
    const double dist_to_goal = goal_pos.DistanceTo(
        Vec2d(ego_pose.pos_smooth().x(), ego_pose.pos_smooth().y()));
    return absl::InternalError(
        absl::StrFormat("Too Far away from goal: %f, %f, distance: %f",
                        goal_pos.x(), goal_pos.y(), dist_to_goal));
  }

  // Add map boundaries.
  std::vector<const mapping::LaneBoundaryInfo*> boundaries_info{};
  if (psmm != nullptr) {
    boundaries_info = psmm->GetLaneBoundariesInfoAtLevel(
        psmm->GetLevel(), goal_pos, freespace_region_half_width * M_SQRT2);
  }
  // Assumes a parking spot provides at most three boundaries.
  freespace_map.boundaries.reserve(boundaries_info.size() + 3);
  const Box2d region_box(
      freespace_map.region.half_length() + vehicle_geom.length(),
      freespace_map.region.half_width() + vehicle_geom.length(),
      freespace_map.region.center(), /*heading=*/0.0);
  int boundary_index = 1;
  // Add parking spot boundaries.
  if (parking_spot_info != nullptr) {
    for (const auto side : {ParkingSpotInfo::LEFT, ParkingSpotInfo::REAR,
                            ParkingSpotInfo::RIGHT, ParkingSpotInfo::FRONT}) {
      if (parking_spot_info->IsSideEnterable(side)) continue;
      Segment2d segment = parking_spot_info->GetEdge(side);
      // Ignore part of spot line of parallel parking.
      if (task_type == FreespaceTaskProto::PARALLEL_PARKING &&
          (side == ParkingSpotInfo::FRONT || side == ParkingSpotInfo::REAR)) {
        // Get enter side.
        Segment2d enter_side_segment;
        if (parking_spot_info->IsSideEnterable(ParkingSpotInfo::LEFT)) {
          enter_side_segment =
              parking_spot_info->GetEdge(ParkingSpotInfo::LEFT);
        } else {
          enter_side_segment =
              parking_spot_info->GetEdge(ParkingSpotInfo::RIGHT);
        }
        // Get new spot line.
        constexpr double kEpsilon = 1e-6;
        constexpr double kIgnoreSpotLineRatio = 0.35;
        auto start = segment.start();
        auto end = segment.end();
        double sign = 1.0;
        if (enter_side_segment.DistanceTo(start) > kEpsilon) {
          std::swap(start, end);
          sign = -1.0;
        }
        const auto new_start = start + kIgnoreSpotLineRatio * sign *
                                           segment.length() *
                                           segment.unit_direction();
        segment = Segment2d(new_start, end);
      }
      // We treat slot edges as soft constraints, push it to special
      // boundaries.
      freespace_map.special_boundaries.push_back(
          {.id = "parking_spot" + std::to_string(boundary_index),
           .type = FreespaceMapProto::SOFT_PARKING_SPOT_LINE,
           .points = {segment.start(), segment.end()}});
      ++boundary_index;
    }
  }

  const auto add_map_boundary = [&freespace_map, &parking_spot_info](
                                    const std::vector<Vec2d>& boundary,
                                    const FreespaceMapProto::BoundaryType& type,
                                    const mapping::ElementId id,
                                    double height = 0.0) {
    freespace_map.boundaries.push_back(
        {.id = std::to_string(id),
         .type = type,
         .points = boundary,
         .near_parking_spot =
             IsLaneBoundaryNearParkingSpot(parking_spot_info, boundary),
         .height = height});
  };

  const auto add_map_special_boundary =
      [&freespace_map](const std::vector<Vec2d>& boundary,
                       const FreespaceMapProto::SpecialBoundaryType& type,
                       const mapping::ElementId id) {
        freespace_map.special_boundaries.push_back(
            {.id = std::to_string(id), .type = type, .points = boundary});
      };

  const auto is_boundary_in_region =
      [&region_box](const std::vector<Vec2d>& boundary) {
        for (int i = 0; i + 1 < boundary.size(); ++i) {
          if (region_box.HasOverlap(Segment2d(boundary[i], boundary[i + 1])))
            return true;
        }
        return false;
      };

  // Add map boundaries.
  for (const mapping::LaneBoundaryInfo* lane_boundary_info : boundaries_info) {
    if (!is_boundary_in_region(lane_boundary_info->points_smooth)) {
      continue;
    }
    switch (lane_boundary_info->type) {
      case mapping::LaneBoundaryProto::CURB: {
        constexpr double kCurbDefualtHeight = 10.0;  // m.
        const double height = (lane_boundary_info->proto->has_height()
                                   ? lane_boundary_info->proto->height()
                                   : kCurbDefualtHeight);
        add_map_boundary(lane_boundary_info->points_smooth,
                         FreespaceMapProto::CURB, lane_boundary_info->id,
                         height);
      } break;
      case mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW:
      case mapping::LaneBoundaryProto::SOLID_YELLOW:
        if (task_type == FreespaceTaskProto::THREE_POINT_TURN ||
            task_type == FreespaceTaskProto::DRIVING_TO_LANE) {
          add_map_boundary(lane_boundary_info->points_smooth,
                           FreespaceMapProto::YELLOW_SOLID_LANE,
                           lane_boundary_info->id);
        }
        break;
      case mapping::LaneBoundaryProto::BROKEN_DOUBLE_YELLOW:
      case mapping::LaneBoundaryProto::BROKEN_YELLOW:
        if (task_type == FreespaceTaskProto::DRIVING_TO_LANE) {
          add_map_boundary(lane_boundary_info->points_smooth,
                           FreespaceMapProto::YELLOW_DASHED_LANE,
                           lane_boundary_info->id);
        }
        if (task_type == FreespaceTaskProto::THREE_POINT_TURN ||
            task_type == FreespaceTaskProto::DRIVING_TO_LANE) {
          add_map_special_boundary(lane_boundary_info->points_smooth,
                                   FreespaceMapProto::CROSSABLE_LANE_LINE,
                                   lane_boundary_info->id);
        }
        break;
      case mapping::LaneBoundaryProto::SOLID_DOUBLE_WHITE:
      case mapping::LaneBoundaryProto::SOLID_WHITE:
        if (task_type == FreespaceTaskProto::DRIVING_TO_LANE) {
          add_map_boundary(lane_boundary_info->points_smooth,
                           FreespaceMapProto::WHITE_SOLID_LANE,
                           lane_boundary_info->id);
        } else if (task_type == FreespaceTaskProto::THREE_POINT_TURN) {
          add_map_special_boundary(lane_boundary_info->points_smooth,
                                   FreespaceMapProto::CROSSABLE_LANE_LINE,
                                   lane_boundary_info->id);
        }
        break;
      case mapping::LaneBoundaryProto::BROKEN_DOUBLE_WHITE:
      case mapping::LaneBoundaryProto::BROKEN_WHITE:
      // TODO(luzou, renjie): for double white, with one of the sides being
      // broken, we treat it as passable. This can be the case where we are
      // doing U-Turn
      case mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE:
      case mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE:
        if (task_type == FreespaceTaskProto::THREE_POINT_TURN ||
            task_type == FreespaceTaskProto::DRIVING_TO_LANE) {
          add_map_special_boundary(lane_boundary_info->points_smooth,
                                   FreespaceMapProto::CROSSABLE_LANE_LINE,
                                   lane_boundary_info->id);
        }
        break;
      case mapping::LaneBoundaryProto::UNKNOWN_TYPE:
        add_map_boundary(lane_boundary_info->points_smooth,
                         FreespaceMapProto::OTHER, lane_boundary_info->id);
        break;
      case mapping::LaneBoundaryProto::VIRTUAL:
        break;
    }
  }
  return freespace_map;
}

void AddUTurnBoundary(const PlannerSemanticMapManager& psmm,
                      const mapping::LanePath* lane_path,
                      const VehicleGeometryParamsProto& /*vehicle_geom*/,
                      FreespaceMap* freespace_map) {
  // Judge left U-turn or right U-turn, currently no right U-turn in China.
  const Vec2d start_point = ComputeLanePointPos(psmm, lane_path->front());
  const Vec2d start_dir = ComputeLanePointTangent(psmm, lane_path->front());
  const Vec2d end_point = ComputeLanePointPos(psmm, lane_path->back());

  const bool left_turn = (start_dir.CrossProd(end_point - start_point) > 0.0);

  for (const auto& lane_id : lane_path->lane_ids()) {
    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(uturn_lane_info, psmm, lane_id);
    if (uturn_lane_info.direction != mapping::LaneProto::UTURN) continue;

    Vec2d start_point;
    Vec2d start_tangent;
    double shift_distance;

    constexpr double kDefaultLaneWidth = 3.75;       // m.
    constexpr double kSpecialBoundaryLength = 40.0;  // m.
    if (uturn_lane_info.incoming_lanes().size() == 1) {
      const auto* incoming_lane_info =
          psmm.FindLaneInfoOrNull(uturn_lane_info.incoming_lanes().front());
      QCHECK_NOTNULL(incoming_lane_info);
      const auto& income_lane_info = *incoming_lane_info;
      start_point = income_lane_info.points_smooth.back();
      income_lane_info.GetTangent(/*fraction=*/1.0, &start_tangent);
      constexpr double kDistanceToLaneEnd = 1.0;  // m.
      const auto point_at_lane_end = income_lane_info.LerpPointFromFraction(
          1.0 - kDistanceToLaneEnd / income_lane_info.length());
      double left_width = 0.5 * kDefaultLaneWidth;
      double right_width = 0.5 * kDefaultLaneWidth;
      const auto left_width_opt =
          psmm.GetLeftLaneWidth(point_at_lane_end, income_lane_info.id);
      const auto right_width_opt =
          psmm.GetRightLaneWidth(point_at_lane_end, income_lane_info.id);
      if (left_width_opt.has_value()) {
        left_width = std::min(*left_width_opt, left_width);
      }
      if (right_width_opt.has_value()) {
        right_width = std::min(*right_width_opt, right_width);
      }
      shift_distance = left_turn ? -right_width : left_width;
    } else {
      start_point = uturn_lane_info.points_smooth.front();
      uturn_lane_info.GetTangent(/*fraction=*/0.0, &start_tangent);
      shift_distance =
          left_turn ? -0.5 * kDefaultLaneWidth : 0.5 * kDefaultLaneWidth;
    }

    // Construct boundary.
    const auto start = start_point + shift_distance * start_tangent.Perp();
    freespace_map->special_boundaries.push_back(
        {.id = "uturn_reverse_stopper",
         .type = FreespaceMapProto::GEAR_REVERSE_STOPPER,
         .points = {start - start_tangent * kSpecialBoundaryLength,
                    start + start_tangent * kSpecialBoundaryLength}});
    break;
  }
  return;
}

absl::StatusOr<ConstraintManager> BuildFreespacePlannerConstraint(
    const VehicleGeometryParamsProto& veh_geo_params,
    const DirectionalPath& path, const PoseProto& ego_pose, bool force_stop,
    FreespacePlannerStateProto* state) {
  ConstraintManager constraint_manager;
  ASSIGN_OR_RETURN(auto end_of_path_constraint,
                   AddEndOfLocalPathConstraint(veh_geo_params, path));
  constraint_manager.AddPathStopLine(std::move(end_of_path_constraint));

  if (force_stop) {
    ASSIGN_OR_RETURN(
        auto force_stop_constraint,
        AddForceStopConstraint(veh_geo_params, path, ego_pose, state));
    constraint_manager.AddPathStopLine(std::move(force_stop_constraint));
  } else {
    state->clear_prev_force_stop_point();
  }

  return constraint_manager;
}

}  // namespace planner
}  // namespace qcraft
