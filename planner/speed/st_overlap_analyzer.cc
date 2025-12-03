#include "onboard/planner/speed/st_overlap_analyzer.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/circle2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kLaneInteractionPreviewDistance = 50.0;  // m.

struct LaneInteraction {
  double fraction;
  mapping::LanePoint other_lane_point;
  StOverlapMetaProto::OverlapPriority priority;
  mapping::LaneInteractionProto::GeometricConfiguration geo_config;
  mapping::LaneProto::Type other_lane_type;

  LaneInteraction(
      double frac, const mapping::LanePoint& other_lane_pt,
      StOverlapMetaProto::OverlapPriority prio,
      mapping::LaneInteractionProto::GeometricConfiguration geo_conf,
      mapping::LaneProto::Type other_lane_tp)
      : fraction(frac),
        other_lane_point(other_lane_pt),
        priority(prio),
        geo_config(geo_conf),
        other_lane_type(other_lane_tp) {}

  static StOverlapMetaProto::OverlapPriority ReactionRuleToPriority(
      mapping::LaneInteractionProto::ReactionRule reaction_rule) {
    switch (reaction_rule) {
      case mapping::LaneInteractionProto::YIELD:
      case mapping::LaneInteractionProto::YIELD_ON_RED:
      case mapping::LaneInteractionProto::YIELD_ON_GREEN_CIRCLE:
      case mapping::LaneInteractionProto::YIELD_MERGE:
      case mapping::LaneInteractionProto::STOP: {
        return StOverlapMetaProto::LOW;
      }
      case mapping::LaneInteractionProto::PROCEED_MERGE:
      case mapping::LaneInteractionProto::PROCEED: {
        return StOverlapMetaProto::HIGH;
      }
      case mapping::LaneInteractionProto::FFA:
      case mapping::LaneInteractionProto::BOTH_STOP: {
        return StOverlapMetaProto::EQUAL;
      }
    }
  }
};

using LaneInteractions = std::vector<LaneInteraction>;
using LaneInteractionMap =
    absl::flat_hash_map<mapping::ElementId, LaneInteractions>;

// Only run overlap analyzer for this object type of st boundaries.
bool RunStOverlapAnalzyerByStBoundaryObjectType(
    StBoundaryProto::ObjectType object_type) {
  switch (object_type) {
    case StBoundaryProto::VEHICLE:
    case StBoundaryProto::CYCLIST:
    case StBoundaryProto::PEDESTRIAN:
      return true;
    case StBoundaryProto::STATIC:
    case StBoundaryProto::UNKNOWN_OBJECT:
    case StBoundaryProto::IGNORABLE:
    case StBoundaryProto::VIRTUAL:
    case StBoundaryProto::IMPASSABLE_BOUNDARY:
    case StBoundaryProto::PATH_BOUNDARY:
      return false;
  }
}

// Only run overlap analyzer for this source type of st boundaries.
bool RunStOverlapnalyzerByStBoundarySourceType(
    StBoundarySourceTypeProto::Type source) {
  switch (source) {
    case StBoundarySourceTypeProto::ST_OBJECT:
      return true;
    case StBoundarySourceTypeProto::UNKNOWN:
    case StBoundarySourceTypeProto::VIRTUAL:
    case StBoundarySourceTypeProto::IMPASSABLE_BOUNDARY:
    case StBoundarySourceTypeProto::PATH_BOUNDARY:
      return false;
  }
}

struct OverlapSourcePriority {
  StOverlapMetaProto::OverlapSource source = StOverlapMetaProto::UNKNOWN_SOURCE;
  StOverlapMetaProto::OverlapPriority priority =
      StOverlapMetaProto::UNKNOWN_PRIORITY;
  std::string priority_reason;
  std::optional<double> time_to_lc_complete;
  std::optional<bool> is_making_u_turn;
  std::optional<bool> is_merging_straight_lane;
  std::optional<bool> is_crossing_straight_lane;
  std::optional<mapping::LaneProto::Direction> obj_lane_direction;
};

void SearchIncomingLanes(const PlannerSemanticMapManager& psmm,
                         const mapping::LaneInfo* lane_info, double current_s,
                         double max_s,
                         std::vector<mapping::ElementId>* lane_ids) {
  if (nullptr == lane_info) return;
  current_s += lane_info->length();
  lane_ids->push_back(lane_info->id);
  if (current_s > max_s) return;
  for (const auto incoming_lane_id : lane_info->incoming_lanes()) {
    const auto* incoming_lane_info = psmm.FindLaneInfoOrNull(incoming_lane_id);
    SearchIncomingLanes(psmm, incoming_lane_info, current_s, max_s, lane_ids);
  }
}

bool MatchOverlapWithLaneInteraction(StBoundaryProto::ObjectType object_type,
                                     const Vec2d& current_obj_pos,
                                     const Vec2d& first_overlap_obj_pos,
                                     double first_overlap_obj_heading,
                                     const LaneInteraction& lane_interaction,
                                     const PlannerSemanticMapManager& psmm) {
  // TODO(renjie): Consider the object's current position.
  QCHECK(object_type == StBoundaryProto::VEHICLE ||
         object_type == StBoundaryProto::CYCLIST);

  auto* lane_info_ptr =
      psmm.FindLaneInfoOrNull(lane_interaction.other_lane_point.lane_id());
  if (nullptr == lane_info_ptr) return false;

  const auto is_vehicle_drivable_lane_type =
      [](mapping::LaneProto::Type lane_type) {
        return lane_type != mapping::LaneProto::BICYCLE_ONLY &&
               lane_type != mapping::LaneProto::WALKING_STREET;
      };
  const auto is_cyclist_drivable_lane_type =
      [](mapping::LaneProto::Type lane_type,
         mapping::LaneProto::Direction lane_direction) {
        // Cyclists may enter a straight motorway.
        return lane_type == mapping::LaneProto::BICYCLE_ONLY ||
               lane_type == mapping::LaneProto::MIXED_WITH_CYCLIST ||
               lane_direction == mapping::LaneProto::STRAIGHT;
      };

  // We only match drivable other lanes for corresponding object type.
  if ((object_type == StBoundaryProto::VEHICLE &&
       !is_vehicle_drivable_lane_type(lane_interaction.other_lane_type)) ||
      (object_type == StBoundaryProto::CYCLIST &&
       !is_cyclist_drivable_lane_type(lane_interaction.other_lane_type,
                                      lane_info_ptr->direction))) {
    return false;
  }

  VLOG(4) << "Match interaction with lane: ("
          << lane_interaction.other_lane_point.lane_id() << ", "
          << lane_interaction.other_lane_point.fraction() << "), priority: "
          << StOverlapMetaProto::OverlapPriority_Name(lane_interaction.priority)
          << ", geo config: "
          << mapping::LaneInteractionProto::GeometricConfiguration_Name(
                 lane_interaction.geo_config);

  double preview_s = (1.0 - lane_interaction.other_lane_point.fraction()) *
                     lane_info_ptr->length();
  std::vector<mapping::ElementId> lane_ids;
  SearchIncomingLanes(psmm, lane_info_ptr, preview_s,
                      kLaneInteractionPreviewDistance, &lane_ids);
  // Check if object current pos is laterally too far from a middle point on
  // target lane.
  double curr_fraction = 0.0;
  double curr_min_dist = 0.0;
  Vec2d curr_close_point;
  constexpr double kCurrPosMatchLaneInteractionDistThres = 4.0;  // m.
  bool cur_pos_matched = false;
  for (const auto lane_id : lane_ids) {
    if (!psmm.GetLaneProjection(current_obj_pos, lane_id, &curr_fraction,
                                &curr_close_point, &curr_min_dist,
                                /*segment=*/nullptr)) {
      continue;
    }
    if (curr_fraction > 0.0 && curr_fraction < 1.0 &&
        curr_min_dist > kCurrPosMatchLaneInteractionDistThres) {
      continue;
    }
    cur_pos_matched = true;
    break;
  }
  if (!cur_pos_matched) {
    return false;
  }

  double fraction = 0.0;
  double min_dist = 0.0;
  for (const auto lane_id : lane_ids) {
    if (!psmm.GetLaneProjection(first_overlap_obj_pos, lane_id, &fraction,
                                /*point=*/nullptr, &min_dist,
                                /*segment=*/nullptr)) {
      continue;
    }
    constexpr double kMatchLaneInteractionDistThres = 2.0;  // m.
    VLOG(4) << "Object pos: " << first_overlap_obj_pos.DebugString()
            << ", distance to other lane: " << min_dist
            << ", closest other lane fraction: " << fraction;
    if (min_dist < kMatchLaneInteractionDistThres) {
      // Check heading diff.
      const mapping::LanePoint closest_other_lane_point(lane_id, fraction);
      const double closest_other_lane_point_heading =
          ComputeLanePointLerpTheta(psmm, closest_other_lane_point);
      constexpr double kMatchLaneInteractionHeadingThres = M_PI / 6.0;
      const double heading_diff = std::abs(NormalizeAngle(
          first_overlap_obj_heading - closest_other_lane_point_heading));
      VLOG(4) << "Object heading diff with closest other lane point: "
              << heading_diff;
      if (heading_diff < kMatchLaneInteractionHeadingThres) {
        // Matched lane interaction.
        return true;
      }
    }
  }
  return false;
}

// If drive passage lane is the current lane the AV is driving on, use
// std::greater<double>() to determine if AV is out of the current lane; If
// drive passage lane is the target lane for lane change, use
// std::less<double>() to determine if AV invades the target lane.
bool IsOverlapAreaOutOfCurrentLane(
    const SpacetimeObjectTrajectory& st_traj, const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& vehicle_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes, int av_idx, int obj_idx,
    bool is_ego_current_lane) {
  FUNC_QTRACE();
  std::function<bool(double, double)> comp;
  if (is_ego_current_lane) {
    comp = std::greater<double>();
  } else {
    comp = std::less<double>();
  }
  const auto& obj_contour = st_traj.states()[obj_idx].contour;
  // NOTE: Buffer may be time-varying in the future.
  const double buffer = st_traj.required_lateral_gap();

  const PiecewiseLinearFunction<double> time_boundary_buffer_plf(
      {0.0, 5.0, 7.0, 10.0}, {0.0, 0.0, 0.2, 0.5});
  const double boundary_buffer =
      time_boundary_buffer_plf(st_traj.states()[obj_idx].traj_point->t());

  const auto& av_shape_ptr = av_shapes[av_idx];

  const auto is_polygon_in_lane = [&drive_passage, &comp](
                                      const Polygon2d& polygon,
                                      double boundary_buffer) -> bool {
    const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(polygon);
    if (frenet_box_or.ok()) {
      const auto s_min_boundary =
          drive_passage.QueryEnclosingLaneBoundariesAtS(frenet_box_or->s_min);
      const auto s_max_boundary =
          drive_passage.QueryEnclosingLaneBoundariesAtS(frenet_box_or->s_max);
      // Check if polygon is entirely in drive passage lane.
      return (s_min_boundary.left.has_value() &&
              s_max_boundary.left.has_value() &&
              s_min_boundary.right.has_value() &&
              s_max_boundary.right.has_value() &&
              comp(s_min_boundary.left->lat_offset + boundary_buffer,
                   frenet_box_or->l_max) &&
              comp(s_max_boundary.left->lat_offset + boundary_buffer,
                   frenet_box_or->l_max) &&
              comp(frenet_box_or->l_min,
                   s_min_boundary.right->lat_offset - boundary_buffer) &&
              comp(frenet_box_or->l_min,
                   s_max_boundary.right->lat_offset - boundary_buffer));
    }
    return true;
  };

  const auto [min_mirror_height, max_mirror_height] =
      ComputeMinMaxMirrorAverageHeight(vehicle_params);
  const bool consider_mirrors =
      IsConsiderMirrorObject(st_traj.planner_object().object_proto(),
                             min_mirror_height, max_mirror_height);
  if (consider_mirrors) {
    const auto is_circle_in_lane =
        [&drive_passage, &comp](const Circle2d& circle, double buffer) -> bool {
      const auto frenet_pt_or =
          drive_passage.QueryFrenetCoordinateAt(circle.center());
      if (frenet_pt_or.ok()) {
        const auto s_boundary =
            drive_passage.QueryEnclosingLaneBoundariesAtS(frenet_pt_or->s);
        const double buffered_radius = circle.radius() + buffer;
        // Check if circle is entirely in drive passage lane.
        return (s_boundary.right.has_value() && s_boundary.left.has_value() &&
                comp(frenet_pt_or->l - buffered_radius,
                     s_boundary.right->lat_offset) &&
                comp(s_boundary.left->lat_offset,
                     frenet_pt_or->l + buffered_radius));
      }
      return true;
    };
    const double mirror_buffer = buffer - boundary_buffer;
    if ((av_shape_ptr->LeftMirrorHasOverlapWithBuffer(obj_contour, buffer) &&
         !is_circle_in_lane(*av_shape_ptr->left_mirror(), mirror_buffer)) ||
        (av_shape_ptr->RightMirrorHasOverlapWithBuffer(obj_contour, buffer) &&
         !is_circle_in_lane(*av_shape_ptr->right_mirror(), mirror_buffer))) {
      return true;
    }
  }

  Polygon2d overlap_polygon;
  const Polygon2d av_polygon =
      Polygon2d(av_shape_ptr->GetCornersWithBufferCounterClockwise(
                    /*lat_buffer=*/buffer, /*lon_buffer=*/buffer),
                /*is_convex=*/true);
  if (obj_contour.ComputeOverlap(av_polygon, &overlap_polygon) &&
      !is_polygon_in_lane(overlap_polygon, boundary_buffer)) {
    return true;
  }

  return false;
}

bool AnalyzeOverlapWithLaneInteraction(StBoundaryProto::ObjectType object_type,
                                       const Vec2d& current_obj_pos,
                                       const Vec2d& first_overlap_obj_pos,
                                       double first_overlap_obj_heading,
                                       const LaneInteraction& lane_interaction,
                                       const PlannerSemanticMapManager& psmm,
                                       const mapping::ElementId lane_id,
                                       OverlapSourcePriority* res) {
  if (!MatchOverlapWithLaneInteraction(
          object_type, current_obj_pos, first_overlap_obj_pos,
          first_overlap_obj_heading, lane_interaction, psmm)) {
    return false;
  }
  const auto lane_info_ptr = psmm.FindLaneInfoOrNull(lane_id);
  const auto other_lane_info_ptr =
      psmm.FindLaneInfoOrNull(lane_interaction.other_lane_point.lane_id());
  if (nullptr == lane_info_ptr || nullptr == other_lane_info_ptr) {
    return false;
  }
  const auto& lane_info = *lane_info_ptr;
  const auto& other_lane_info = *other_lane_info_ptr;
  if (lane_interaction.geo_config == mapping::LaneInteractionProto::MERGE) {
    res->source = StOverlapMetaProto::LANE_MERGE;
    res->priority = lane_interaction.priority;
    res->priority_reason = absl::StrCat(
        StOverlapMetaProto::OverlapPriority_Name(lane_interaction.priority),
        " priority for AV lane ", lane_id.value(), " merging object lane ",
        lane_interaction.other_lane_point.lane_id());
    res->is_merging_straight_lane =
        (lane_info.direction == mapping::LaneProto::STRAIGHT &&
         lane_interaction.priority == StOverlapMetaProto::HIGH);
  } else {
    res->source = StOverlapMetaProto::LANE_CROSS;
    res->priority = lane_interaction.priority;
    res->priority_reason = absl::StrCat(
        StOverlapMetaProto::OverlapPriority_Name(lane_interaction.priority),
        " priority for AV lane ", lane_id.value(), " crossing object lane ",
        lane_interaction.other_lane_point.lane_id());
    // If AV is going straight, we can safely assume that any left-turn
    // prediction cutting in AV path is unprotected.
    res->is_crossing_straight_lane =
        (lane_info.direction == mapping::LaneProto::STRAIGHT &&
         lane_interaction.priority == StOverlapMetaProto::HIGH);
  }
  res->is_making_u_turn =
      (lane_info.direction == mapping::LaneProto::STRAIGHT &&
       other_lane_info.direction == mapping::LaneProto::UTURN);
  res->obj_lane_direction = other_lane_info.direction;
  return true;
}

bool AnalyzeOverlapWithLaneInteractions(
    StBoundaryProto::ObjectType object_type, const Vec2d& current_obj_pos,
    const Vec2d& first_overlap_obj_pos, double first_overlap_obj_heading,
    const LaneInteractions& lane_interactions,
    const PlannerSemanticMapManager& psmm, const mapping::ElementId lane_id,
    OverlapSourcePriority* res) {
  for (const auto& lane_interaction : lane_interactions) {
    if (AnalyzeOverlapWithLaneInteraction(
            object_type, current_obj_pos, first_overlap_obj_pos,
            first_overlap_obj_heading, lane_interaction, psmm, lane_id, res)) {
      // TODO(renjie): Consider to match the most likely interaction instead
      // of early exit.
      return true;
    }
  }
  return false;
}

bool AnalyzeOverlapSourcePriorityByMaxDeviationInfo(
    const DrivePassage& drive_passage,
    absl::Span<const OverlapInfo> overlap_infos,
    absl::Span<const PathPointSemantic> path_semantics,
    const SpacetimeObjectTrajectory& st_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const DiscretizedPath& path, OverlapSourcePriority* res) {
  struct DeviationInfo {
    int av_idx;
    int obj_idx;
    double deviation_distance;
    DeviationInfo(int av_idx, int obj_idx, double deviation_distance)
        : av_idx(av_idx),
          obj_idx(obj_idx),
          deviation_distance(deviation_distance) {}
  };
  std::optional<DeviationInfo> max_deviation_info = std::nullopt;
  for (const auto& overlap_info : overlap_infos) {
    const int av_idx =
        (overlap_info.av_start_idx + overlap_info.av_end_idx) / 2;
    if (av_idx >= path_semantics.size() ||
        path_semantics[av_idx].lane_path_id_history.size() > 1) {
      continue;
    }
    constexpr double kFractionFilterThres = 0.99;
    if (path_semantics[av_idx].closest_lane_point.fraction() >
        kFractionFilterThres) {
      const auto& nearest_station =
          drive_passage.FindNearestStation(ToVec2d(path[av_idx]));
      if (nearest_station.is_virtual()) continue;
    }
    if (path_semantics[av_idx].lane_info->IsVirtual()) {
      const auto& nearest_station =
          drive_passage.FindNearestStation(ToVec2d(path[av_idx]));
      if (nearest_station.lane_id() == path_semantics[av_idx].lane_info->id ||
          nearest_station.is_virtual()) {
        // Both current lane & target lane are virtual.
        continue;
      }
    }
    if (!max_deviation_info.has_value() ||
        path_semantics[av_idx].deviation_distance >
            max_deviation_info->deviation_distance) {
      max_deviation_info =
          DeviationInfo(av_idx, overlap_info.obj_idx,
                        path_semantics[av_idx].deviation_distance);
    }
  }

  if (max_deviation_info.has_value()) {
    const auto av_idx = max_deviation_info->av_idx;
    const auto obj_idx = max_deviation_info->obj_idx;
    const auto& nearest_station =
        drive_passage.FindNearestStation(ToVec2d(path[av_idx]));
    const auto current_lane_id = path_semantics[av_idx].lane_info->id;
    const bool is_ego_current_lane =
        nearest_station.lane_id() == current_lane_id;
    if (IsOverlapAreaOutOfCurrentLane(st_traj, drive_passage,
                                      vehicle_geometry_params, av_shapes,
                                      av_idx, obj_idx, is_ego_current_lane)) {
      res->source = StOverlapMetaProto::AV_CUTIN;
      res->priority = StOverlapMetaProto::LOW;
      res->priority_reason =
          "LOW priority for AV cutting in object (overlap method)";
      return true;
    }
  }
  return false;
}

absl::StatusOr<OverlapSourcePriority> AnalyzeOverlapSourceAndPriority(
    const StBoundaryRef& st_boundary,
    StOverlapMetaProto::OverlapPattern overlap_pattern,
    const SpacetimeObjectTrajectory& st_traj,
    const PlannerSemanticMapManager& psmm, const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const DrivePassage* drive_passage,
    const LaneInteractionMap& lane_interaction_map,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v,
    bool set_low_priority_for_all) {
  FUNC_QTRACE();
  VLOG(4) << "Analyze overlap source and priority for st-boundary "
          << st_boundary->id();

  OverlapSourcePriority res;
  // Only analyze source and priority for overlap of pattern ENTER, CROSS and
  // INTERFERE.
  if (overlap_pattern != StOverlapMetaProto::ENTER &&
      overlap_pattern != StOverlapMetaProto::CROSS &&
      overlap_pattern != StOverlapMetaProto::INTERFERE) {
    return res;
  }

  // Only analyze source and priority for overlap happening at a future path
  // point.
  if (st_boundary->bottom_left_point().s() <= 0.0) {
    return res;
  }

  // For an overlap of pattern ENTER, CROSS and INTERFERE, it should have a
  // positive min_t.
  if (st_boundary->min_t() <= 0.0) {
    QEVENT("renjie", "unexpected_nonnegative_min_t", [&](QEvent* qevent) {
      qevent->AddField("st_boundary_id", st_boundary->id())
          .AddField("pattern",
                    StOverlapMetaProto::OverlapPattern_Name(overlap_pattern))
          .AddField("min_t", st_boundary->min_t());
    });
  }

  // Low priority for all pedestrian overlaps.
  if (st_boundary->object_type() == StBoundaryProto::PEDESTRIAN) {
    res.source = StOverlapMetaProto::OTHER;
    res.priority = StOverlapMetaProto::LOW;
    res.priority_reason = absl::StrCat(
        "LOW priority for object type ",
        StBoundaryProto::ObjectType_Name(st_boundary->object_type()));
    return res;
  }

  const auto& overlap_infos = st_boundary->overlap_infos();
  // In this function, 'fo' denotes 'first_overlap'.
  const auto& fo_info = overlap_infos.front();

  // NOLINTNEXTLINE
  enum class LaneChangeSemantic { NONE = 0, LEFT = 1, RIGHT = 2 };
  LaneChangeSemantic fo_av_lane_change_semantic = LaneChangeSemantic::NONE;
  std::vector<mapping::ElementId> fo_av_lane_id_list;
  constexpr double kLaneChangeCheckLookAheadDist = 20.0;  // m.
  // Check whether AV is changing lane during the first overlap up to a
  // lookahead distance. This is to compensate for the fact the AV front wheels
  // would enter target lane before its rac point.
  std::optional<std::vector<int>> lc_lane_path_id_history = std::nullopt;
  const double check_lc_end_path_s =
      path[fo_info.av_end_idx].s() + kLaneChangeCheckLookAheadDist;
  for (int i = fo_info.av_start_idx;
       i < path_semantics.size() && path[i].s() <= check_lc_end_path_s; ++i) {
    const auto& lane_path_id_history = path_semantics[i].lane_path_id_history;
    if (fo_av_lane_change_semantic == LaneChangeSemantic::NONE &&
        lane_path_id_history.back() != 0) {
      QCHECK_GT(lane_path_id_history.size(), 1);
      QCHECK_NE(lane_path_id_history.back(),
                lane_path_id_history[lane_path_id_history.size() - 2]);
      if (lane_path_id_history.back() >
          lane_path_id_history[lane_path_id_history.size() - 2]) {
        fo_av_lane_change_semantic = LaneChangeSemantic::LEFT;
      } else {
        fo_av_lane_change_semantic = LaneChangeSemantic::RIGHT;
      }
      lc_lane_path_id_history = lane_path_id_history;
    }
    if (i <= fo_info.av_end_idx &&
        std::find(fo_av_lane_id_list.begin(), fo_av_lane_id_list.end(),
                  path_semantics[i].closest_lane_point.lane_id()) ==
            fo_av_lane_id_list.end()) {
      fo_av_lane_id_list.push_back(
          path_semantics[i].closest_lane_point.lane_id());
    }
  }

  if (fo_info.av_end_idx >= path_semantics.size()) {
    return absl::NotFoundError("Overlap index exceeds path semantics range!");
  }

  const auto* fo_obj_point = st_traj.states()[fo_info.obj_idx].traj_point;
  const auto& fo_obj_pos = fo_obj_point->pos();
  const int av_mean_idx = (fo_info.av_start_idx + fo_info.av_end_idx) / 2;
  if (fo_av_lane_change_semantic != LaneChangeSemantic::NONE) {
    // The overlap happens during AV lane change. Check if the object is on
    // the corresponding side of path to see if it is being cut in by AV.
    const auto& fo_av_path_point = path[av_mean_idx];
    const bool is_fo_obj_on_path_left =
        Vec2d::FastUnitFromAngle(fo_av_path_point.theta())
            .CrossProd(fo_obj_pos - ToVec2d(fo_av_path_point)) >= 0.0;
    if ((fo_av_lane_change_semantic == LaneChangeSemantic::LEFT &&
         is_fo_obj_on_path_left) ||
        (fo_av_lane_change_semantic == LaneChangeSemantic::RIGHT &&
         !is_fo_obj_on_path_left)) {
      res.source = StOverlapMetaProto::AV_CUTIN;
      res.priority = StOverlapMetaProto::LOW;
      res.priority_reason = "LOW priority for AV cutting in object";
      QCHECK(lc_lane_path_id_history.has_value());
      for (int i = 0; i < path_semantics.size(); ++i) {
        if (path_semantics[i].lane_path_id_history ==
            *lc_lane_path_id_history) {
          res.time_to_lc_complete = path[i].s() / (init_v + 1e-6);
          break;
        }
      }
      QCHECK(res.time_to_lc_complete.has_value());
      return res;
    }
  } else {
    // The overlap happens during AV lane keeping.
    const auto fo_obj_heading = fo_obj_point->theta();
    const auto current_obj_pos = st_traj.pose().pos();
    // Check if first overlap can be matched to a lane interaction.
    for (const auto lane_id : fo_av_lane_id_list) {
      const auto* lane_interactions = FindOrNull(lane_interaction_map, lane_id);
      if (lane_interactions == nullptr) continue;
      if (AnalyzeOverlapWithLaneInteractions(
              st_boundary->object_type(), current_obj_pos, fo_obj_pos,
              fo_obj_heading, *lane_interactions, psmm, lane_id, &res)) {
        return res;
      }
    }
    const auto& closest_lane_point =
        path_semantics[fo_info.av_end_idx].closest_lane_point;
    const auto* lane_info_ptr =
        psmm.FindLaneInfoOrNull(closest_lane_point.lane_id());
    if (nullptr != lane_info_ptr) {
      double preview_s =
          (1.0 - closest_lane_point.fraction()) * lane_info_ptr->length();
      const auto find_matched_lane_id_and_interactions =
          [&lane_interaction_map](
              const std::vector<mapping::ElementId>& lane_ids)
          -> std::optional<
              std::pair<mapping::ElementId, const LaneInteractions*>> {
        for (const auto lane_id : lane_ids) {
          const auto* lane_interactions =
              FindOrNull(lane_interaction_map, lane_id);
          if (nullptr != lane_interactions) {
            return std::make_pair(lane_id, lane_interactions);
          }
        }
        return std::nullopt;
      };
      while (preview_s < kLaneInteractionPreviewDistance &&
             nullptr != lane_info_ptr) {
        const mapping::LaneInfo* next_lane_info_ptr = nullptr;
        if (const auto lane_id_and_interactions =
                find_matched_lane_id_and_interactions(
                    lane_info_ptr->outgoing_lanes());
            lane_id_and_interactions.has_value()) {
          const auto [lane_id, lane_interactions] = *lane_id_and_interactions;
          if (AnalyzeOverlapWithLaneInteractions(
                  st_boundary->object_type(), current_obj_pos, fo_obj_pos,
                  fo_obj_heading, *lane_interactions, psmm, lane_id, &res)) {
            return res;
          }
          next_lane_info_ptr = psmm.FindLaneInfoOrNull(lane_id);
          preview_s += next_lane_info_ptr->length();
        }
        lane_info_ptr = next_lane_info_ptr;
      }
    }
  }

  // Compute the overlap area of AV & object when overlap for 1st time. Then
  // check whether the overlap area is out of current lane. If so, we think AV
  // is changing lane and cut-in this object.
  if (drive_passage != nullptr &&
      AnalyzeOverlapSourcePriorityByMaxDeviationInfo(
          *drive_passage, overlap_infos, path_semantics, st_traj,
          vehicle_geometry_params, av_shapes, path, &res)) {
    return res;
  }

  // Other cases (AV is changing lane but object is not cut in by AV, or AV is
  // keeping lane but the overlap is not matched to a lane interaction), we
  // consider the object is cutting AV.
  res.source = StOverlapMetaProto::OBJECT_CUTIN;
  if (set_low_priority_for_all) {
    res.priority = StOverlapMetaProto::LOW;
    res.priority_reason = "LOW priority for all objects";
    return res;
  }

  // Check if AV is going straight
  bool fo_av_going_straight = false;
  for (const auto lane_id : fo_av_lane_id_list) {
    const auto lane_info_ptr = psmm.FindLaneInfoOrNull(lane_id);
    if (nullptr == lane_info_ptr) continue;
    const auto& lane_info = *lane_info_ptr;
    if (lane_info.direction == mapping::LaneProto::STRAIGHT) {
      fo_av_going_straight = true;
      break;
    }
  }

  // Check if AV is going straight and cut in by a u-turn like object
  // trajectory.
  const double init_obj_heading = st_traj.states().front().traj_point->theta();
  const double end_obj_heading = st_traj.states().back().traj_point->theta();
  constexpr double kUTurnHeadingDiff = 0.75 * M_PI;  // 135 deg.
  res.is_making_u_turn =
      (fo_av_going_straight &&
       std::abs(NormalizeAngle(end_obj_heading - init_obj_heading)) >=
           kUTurnHeadingDiff);

  // If AV is crossed by other objects while going straight in lane, AV has a
  // higher priority to pass.
  const auto get_angle_diff = [&path, &st_traj](int av_path_idx,
                                                int obj_traj_idx) -> double {
    const auto& obj_traj_point = *st_traj.states()[obj_traj_idx].traj_point;
    const auto& av_path_point = path[av_path_idx];
    return NormalizeAngle(obj_traj_point.theta() - av_path_point.theta());
  };

  const double theta_diff = get_angle_diff(av_mean_idx, fo_info.obj_idx);

  constexpr double kCrossingAngleDiffThre = M_PI_4;  // 45 deg.
  res.is_crossing_straight_lane =
      (fo_av_going_straight &&
       st_boundary->object_type() == StBoundaryProto::VEHICLE &&
       std::abs(theta_diff) > kCrossingAngleDiffThre);

  if (st_boundary->object_type() == StBoundaryProto::CYCLIST &&
      (path_semantics[av_mean_idx].lane_semantic ==
           LaneSemantic::INTERSECTION_RIGHT_TURN ||
       path_semantics[av_mean_idx].lane_semantic ==
           LaneSemantic::INTERSECTION_LEFT_TURN ||
       path_semantics[av_mean_idx].lane_semantic ==
           LaneSemantic::INTERSECTION_UTURN)) {
    res.priority = StOverlapMetaProto::LOW;
    res.priority_reason =
        "LOW priority for AV being cut in by cyclist during turning";
    return res;
  }
  res.priority = StOverlapMetaProto::HIGH;
  res.priority_reason = "HIGH priority for AV being cut in by object";
  return res;
}  // namespace

bool IsDrivingParallel(const std::optional<double>& theta_diff) {
  if (!theta_diff.has_value()) return false;
  constexpr double kDrivingParallelHeadingThres = M_PI / 6.0;
  return std::abs(*theta_diff) < kDrivingParallelHeadingThres;
}

std::optional<double> CalculateThetaDiff(
    const SpacetimeObjectTrajectory& st_traj, const DrivePassage* drive_passage,
    const std::optional<KdTreeFrenetFrame>& target_frenet_frame,
    StOverlapMetaProto::OverlapSource overlap_source) {
  if (drive_passage == nullptr && !target_frenet_frame.has_value()) {
    // Both methods to calculate theta diff not available.
    return std::nullopt;
  }
  if (overlap_source != StOverlapMetaProto::LANE_MERGE &&
      overlap_source != StOverlapMetaProto::OBJECT_CUTIN) {
    return std::nullopt;
  }

  // If object is too far away from frenet frame, theta calculation is
  // inaccurate.
  constexpr double kMaxLLimit = 10.0;  // m.
  const Vec2d& obj_pos = st_traj.pose().pos();
  if (target_frenet_frame.has_value()) {
    const auto obj_frenet_coord = target_frenet_frame->XYToSL(obj_pos);
    if (std::abs(obj_frenet_coord.l) > kMaxLLimit) {
      return std::nullopt;
    }
    const auto obj_frenet_tangent =
        target_frenet_frame->InterpolateTangentByS(obj_frenet_coord.s);
    const double ref_theta = obj_frenet_tangent.FastAngle();
    const double theta_diff =
        NormalizeAngle(st_traj.pose().theta() - ref_theta);
    return theta_diff;
  } else {
    const auto obj_frenet_coord =
        drive_passage->QueryFrenetCoordinateAt(obj_pos);
    if (obj_frenet_coord.ok()) {
      if (std::abs(obj_frenet_coord->l) > kMaxLLimit) {
        return std::nullopt;
      }
      const auto obj_frenet_theta =
          drive_passage->QueryTangentAngleAtS(obj_frenet_coord->s);
      if (obj_frenet_theta.ok()) {
        const double ref_theta = *obj_frenet_theta;
        const double theta_diff =
            NormalizeAngle(st_traj.pose().theta() - ref_theta);
        return theta_diff;
      }
    }
  }
  return std::nullopt;
}

std::optional<std::pair<double, double>> CalculateProjectionDistance(
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& path,
    StOverlapMetaProto::OverlapSource overlap_source) {
  if (overlap_source != StOverlapMetaProto::LANE_MERGE &&
      overlap_source != StOverlapMetaProto::LANE_CROSS &&
      overlap_source != StOverlapMetaProto::OBJECT_CUTIN &&
      overlap_source != StOverlapMetaProto::AV_CUTIN) {
    return std::nullopt;
  }
  const auto& obj_contour = st_traj.contour();

  const Vec2d av_heading_dir = Vec2d::FastUnitFromAngle(path.front().theta());
  const Vec2d av_pos = Vec2d(path.front().x(), path.front().y());
  Vec2d front_most, back_most;
  obj_contour.ExtremePoints(av_heading_dir, &back_most, &front_most);

  const double front_dis = (front_most - av_pos).Dot(av_heading_dir);
  const double back_dis = (back_most - av_pos).Dot(av_heading_dir);
  return std::make_pair(front_dis, back_dis);
}

StOverlapMetaProto::ModificationType AnalyzeOverlapModificationType(
    const StBoundaryRef& st_boundary, const std::optional<double>& theta_diff,
    StOverlapMetaProto::OverlapSource overlap_source,
    StOverlapMetaProto::OverlapPriority overlap_priority) {
  if (overlap_priority == StOverlapMetaProto::UNKNOWN_PRIORITY) {
    // An overlap without priority must be non-interactive.
    return StOverlapMetaProto::NON_MODIFIABLE;
  }
  QCHECK(st_boundary->object_type() == StBoundaryProto::VEHICLE ||
         st_boundary->object_type() == StBoundaryProto::CYCLIST ||
         st_boundary->object_type() == StBoundaryProto::PEDESTRIAN);

  // For vehicles/cyclists.
  if (st_boundary->object_type() == StBoundaryProto::VEHICLE ||
      st_boundary->object_type() == StBoundaryProto::CYCLIST) {
    switch (overlap_source) {
      case StOverlapMetaProto::LANE_MERGE:
      case StOverlapMetaProto::LANE_CROSS:
      case StOverlapMetaProto::AV_CUTIN: {
        return StOverlapMetaProto::LON_MODIFIABLE;
      }
      case StOverlapMetaProto::OBJECT_CUTIN: {
        if (IsDrivingParallel(theta_diff)) {
          return StOverlapMetaProto::LON_LAT_MODIFIABLE;
        } else {
          return StOverlapMetaProto::LON_MODIFIABLE;
        }
      }
      case StOverlapMetaProto::UNKNOWN_SOURCE:
      case StOverlapMetaProto::OTHER: {
        return StOverlapMetaProto::NON_MODIFIABLE;
      }
    }
  }

  // For pedestrians.
  return StOverlapMetaProto::NON_MODIFIABLE;
}

absl::StatusOr<StOverlapMetaProto> AnalyzeStOverlap(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const PlannerSemanticMapManager& psmm, const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const LaneInteractionMap& lane_interaction_map,
    const DrivePassage* drive_passage,
    const std::optional<KdTreeFrenetFrame>& target_frenet_frame,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v,
    bool set_low_priority_for_all) {
  FUNC_QTRACE();
  StOverlapMetaProto overlap_meta;
  // Step 1: Analyze overlap pattern.
  overlap_meta.set_pattern(AnalyzeOverlapPattern(st_boundary, st_traj, path,
                                                 vehicle_geometry_params));
  // Step 2: Analyze overlap source and priority.
  ASSIGN_OR_RETURN(auto source_priority,
                   AnalyzeOverlapSourceAndPriority(
                       st_boundary, overlap_meta.pattern(), st_traj, psmm, path,
                       path_semantics, drive_passage, lane_interaction_map,
                       vehicle_geometry_params, av_shapes, init_v,
                       set_low_priority_for_all));
  overlap_meta.set_source(source_priority.source);
  overlap_meta.set_priority(source_priority.priority);
  overlap_meta.set_priority_reason(std::move(source_priority.priority_reason));
  if (source_priority.time_to_lc_complete.has_value()) {
    overlap_meta.set_time_to_lc_complete(*source_priority.time_to_lc_complete);
  }
  if (source_priority.is_making_u_turn.has_value()) {
    overlap_meta.set_is_making_u_turn(*source_priority.is_making_u_turn);
  }
  if (source_priority.is_merging_straight_lane.has_value()) {
    overlap_meta.set_is_merging_straight_lane(
        *source_priority.is_merging_straight_lane);
  }
  if (source_priority.is_crossing_straight_lane.has_value()) {
    overlap_meta.set_is_crossing_straight_lane(
        *source_priority.is_crossing_straight_lane);
  }
  if (source_priority.obj_lane_direction.has_value()) {
    overlap_meta.set_obj_lane_direction(*source_priority.obj_lane_direction);
  }
  const auto theta_diff = CalculateThetaDiff(
      st_traj, drive_passage, target_frenet_frame, overlap_meta.source());
  if (theta_diff.has_value()) {
    overlap_meta.set_theta_diff(*theta_diff);
  }
  const auto projection_dis_pair =
      CalculateProjectionDistance(st_traj, path, overlap_meta.source());
  if (projection_dis_pair.has_value()) {
    overlap_meta.set_front_most_projection_distance(projection_dis_pair->first);
    overlap_meta.set_rear_most_projection_distance(projection_dis_pair->second);
  }
  // Step 3: Analyze modification type.
  overlap_meta.set_modification_type(AnalyzeOverlapModificationType(
      st_boundary, theta_diff, overlap_meta.source(), overlap_meta.priority()));

  return overlap_meta;
}

}  // namespace

StOverlapMetaProto::OverlapPattern AnalyzeOverlapPattern(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const auto& overlap_infos = st_boundary->overlap_infos();
  QCHECK(!overlap_infos.empty());

  const auto& first_overlap_info = overlap_infos.front();
  const auto& last_overlap_info = overlap_infos.back();

  const int state_num = st_traj.states().size();
  const int path_num = path.size();

  // If first object contour before overlap is behind the first AV box on path,
  // we consider it to be on the path at the beginning; if first object contour
  // after overlap is ahead of the last AV box on path, we consider it to be
  // on the path at the end.
  enum class RelativeLonPosition {
    BEHIND = 0,     // NOLINT
    AHEAD = 1,      // NOLINT
    INTERSECT = 2,  // NOLINT
  };
  const auto compute_obj_relative_position_with_av_box =
      [&path, &st_traj, &vehicle_geometry_params](
          int av_path_idx, int obj_traj_idx) -> RelativeLonPosition {
    const auto& obj_contour = st_traj.states()[obj_traj_idx].contour;
    const auto& av_path_point = path[av_path_idx];
    const auto av_path_dir = Vec2d::FastUnitFromAngle(av_path_point.theta());
    Vec2d front, back;
    obj_contour.ExtremePoints(av_path_dir, &back, &front);
    const Vec2d av_path_pos = ToVec2d(av_path_point);
    if ((back - av_path_pos).Dot(av_path_dir) >
        vehicle_geometry_params.front_edge_to_center() +
            st_traj.required_lateral_gap()) {
      return RelativeLonPosition::AHEAD;
    } else if ((front - av_path_pos).Dot(av_path_dir) <
               -vehicle_geometry_params.back_edge_to_center() -
                   st_traj.required_lateral_gap()) {
      return RelativeLonPosition::BEHIND;
    } else {
      return RelativeLonPosition::INTERSECT;
    }
  };

  if (first_overlap_info.obj_idx == 0 ||
      compute_obj_relative_position_with_av_box(
          0, first_overlap_info.obj_idx - 1) == RelativeLonPosition::BEHIND) {
    // If first object contour out of path is in front of the last AV box, we
    // also consider it to be of type STAY.
    if (last_overlap_info.obj_idx == state_num - 1 ||
        compute_obj_relative_position_with_av_box(
            path_num - 1, last_overlap_info.obj_idx + 1) ==
            RelativeLonPosition::AHEAD) {
      return StOverlapMetaProto::STAY;
    } else {
      return StOverlapMetaProto::LEAVE;
    }
  }

  // If first object contour out of path is in front of the last AV box, we
  // also consider it to be of type ENTER.
  if (last_overlap_info.obj_idx == state_num - 1 ||
      compute_obj_relative_position_with_av_box(
          path_num - 1, last_overlap_info.obj_idx + 1) ==
          RelativeLonPosition::AHEAD) {
    return StOverlapMetaProto::ENTER;
  } else {
    // True for left side, false for right side.
    const auto get_side_on_path = [&path, &st_traj](int av_path_idx,
                                                    int obj_traj_idx) -> bool {
      const auto& av_path_point = path[av_path_idx];
      const auto& obj_traj_point = *st_traj.states()[obj_traj_idx].traj_point;
      const Vec2d ref = obj_traj_point.pos() - ToVec2d(av_path_point);
      const bool is_on_path_left =
          Vec2d::FastUnitFromAngle(av_path_point.theta()).CrossProd(ref) >= 0.0;
      return is_on_path_left;
    };

    const bool first_overlap_side = get_side_on_path(
        (first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) / 2,
        first_overlap_info.obj_idx);
    const bool last_overlap_side = get_side_on_path(
        (last_overlap_info.av_start_idx + last_overlap_info.av_end_idx) / 2,
        last_overlap_info.obj_idx);
    if (first_overlap_side == last_overlap_side) {
      return StOverlapMetaProto::INTERFERE;
    } else {
      return StOverlapMetaProto::CROSS;
    }
  }
}

bool IsAnalyzableStBoundary(const StBoundaryRef& st_boundary) {
  if (!RunStOverlapnalyzerByStBoundarySourceType(st_boundary->source_type())) {
    return false;
  }
  if (!RunStOverlapAnalzyerByStBoundaryObjectType(st_boundary->object_type())) {
    return false;
  }
  if (st_boundary->is_stationary()) return false;
  return true;
}

void AnalyzeStOverlaps(
    const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage* drive_passage,
    const KdTreeFrenetFrame* built_target_frenet_frame,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedFinderParamsProto::StOverlapAnalyzerParamsProto&
        st_overlap_analyzer_params,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v,
    std::vector<StBoundaryRef>* st_boundaries) {
  FUNC_QTRACE();
  const bool set_low_priority_for_all =
      st_overlap_analyzer_params.set_low_priority_for_all_objects();

  std::optional<KdTreeFrenetFrame> target_frenet_frame = std::nullopt;
  // If we have drive passage, extend backward to build longer frenet frame.
  // Otherwise we should have a pre-built target frenet frame for analysis.
  if (drive_passage != nullptr) {
    constexpr double kBackwardExtendLength = 30.0;  // m.
    const auto target_lane_path_extend = BackwardExtendLanePath(
        psmm, drive_passage->extend_lane_path(), kBackwardExtendLength);
    auto target_frenet_frame_or = BuildKdTreeFrenetFrame(
        SampleLanePathPoints(psmm, target_lane_path_extend),
        /*down_sample_raw_points=*/true);
    if (target_frenet_frame_or.ok()) {
      target_frenet_frame = std::move(*target_frenet_frame_or);
    }
  }
  if (!target_frenet_frame.has_value() &&
      built_target_frenet_frame != nullptr) {
    target_frenet_frame = *built_target_frenet_frame;
  }

  // Generate lane interaction map.
  LaneInteractionMap path_lane_interaction_map;
  for (const auto& path_semantic : path_semantics) {
    const auto& current_closest_lane_point =
        path_semantics[0].closest_lane_point;
    const auto& closest_lane_point = path_semantic.closest_lane_point;
    if (path_lane_interaction_map.contains(closest_lane_point.lane_id())) {
      continue;
    }
    const auto lane_proto_ptr =
        psmm.FindLaneByIdOrNull(closest_lane_point.lane_id());
    if (nullptr == lane_proto_ptr) continue;
    const auto& lane_proto = *lane_proto_ptr;
    path_lane_interaction_map[closest_lane_point.lane_id()] = {};
    if ((psmm.GetDataSource() != OnlineMapProto_DataSource_NAVINFO_HDMAP &&
         psmm.GetDataSource() != OnlineMapProto_DataSource_QCRAFT_VISIONMAP) ||
        !lane_proto.interactions().empty()) {
      for (const auto& interaction : lane_proto.interactions()) {
        const auto* other_lane_proto_ptr = psmm.FindLaneByIdOrNull(
            mapping::ElementId(interaction.other_lane_id()));
        if (nullptr == other_lane_proto_ptr) continue;
        // Only add lane interactions beyond current AV position.
        if (closest_lane_point.lane_id() ==
                current_closest_lane_point.lane_id() &&
            interaction.this_lane_fraction() <
                current_closest_lane_point.fraction()) {
          continue;
        }
        path_lane_interaction_map[closest_lane_point.lane_id()].emplace_back(
            interaction.this_lane_fraction(),
            mapping::LanePoint(mapping::ElementId(interaction.other_lane_id()),
                               interaction.other_lane_fraction()),
            set_low_priority_for_all ? StOverlapMetaProto::LOW
                                     : LaneInteraction::ReactionRuleToPriority(
                                           interaction.reaction_rule()),
            interaction.geometric_configuration(),
            other_lane_proto_ptr->type());
      }
    } else {
      // For third-party HD map & vision map, there is only is_merging attribute
      // instead of lane interactions info.
      const auto determine_merging_priority = [](bool is_ego_lane_merging,
                                                 bool is_other_lane_merging)
          -> StOverlapMetaProto::OverlapPriority {
        // Merging lane has lower priority.
        if (is_ego_lane_merging && !is_other_lane_merging) {
          return StOverlapMetaProto::LOW;
        } else if (!is_ego_lane_merging && is_other_lane_merging) {
          return StOverlapMetaProto::HIGH;
        }
        return StOverlapMetaProto::EQUAL;
      };
      for (const auto& outgoing_lane : lane_proto.outgoing_lanes()) {
        const auto* outgoing_lane_proto_ptr =
            psmm.FindLaneByIdOrNull(mapping::ElementId(outgoing_lane));
        if (nullptr == outgoing_lane_proto_ptr) continue;
        for (const auto& incoming_lane :
             outgoing_lane_proto_ptr->incoming_lanes()) {
          const mapping::ElementId incoming_lane_id =
              mapping::ElementId(incoming_lane);
          if (closest_lane_point.lane_id() == incoming_lane_id) continue;
          const auto* incoming_lane_proto_ptr =
              psmm.FindLaneByIdOrNull(incoming_lane_id);
          if (nullptr == incoming_lane_proto_ptr) continue;
          path_lane_interaction_map[closest_lane_point.lane_id()].emplace_back(
              /*fraction=*/1.0,
              mapping::LanePoint(incoming_lane_id,
                                 /*fraction=*/1.0),
              set_low_priority_for_all
                  ? StOverlapMetaProto::LOW
                  : determine_merging_priority(
                        lane_proto.is_merging(),
                        incoming_lane_proto_ptr->is_merging()),
              mapping::LaneInteractionProto::MERGE,
              incoming_lane_proto_ptr->type());
        }
      }
    }
  }

  if (VLOG_IS_ON(4)) {
    for (const auto& [lane_id, lane_interactions] : path_lane_interaction_map) {
      VLOG(4) << "AV path lane: " << lane_id;
      for (const auto& lane_interaction : lane_interactions) {
        VLOG(4) << "Fraction: " << lane_interaction.fraction
                << ", other lane point: "
                << lane_interaction.other_lane_point.DebugString()
                << ", AV precedence: "
                << StOverlapMetaProto::OverlapPriority_Name(
                       lane_interaction.priority);
      }
    }
  }

  // TODO(renjie): See if need to parallelize this loop.
  for (auto& st_boundary : *st_boundaries) {
    if (!IsAnalyzableStBoundary(st_boundary)) continue;
    // All st-object-generated st-boundaries must have a traj_id.
    QCHECK(st_boundary->traj_id().has_value());
    const auto& traj_id = *st_boundary->traj_id();
    const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

    ASSIGN_OR_CONTINUE(
        StOverlapMetaProto overlap_meta,
        AnalyzeStOverlap(st_boundary, *traj, psmm, path, path_semantics,
                         path_lane_interaction_map, drive_passage,
                         target_frenet_frame, vehicle_geometry_params,
                         av_shapes, init_v, set_low_priority_for_all));

    st_boundary->set_overlap_meta(std::move(overlap_meta));
  }
}

}  // namespace planner
}  // namespace qcraft
