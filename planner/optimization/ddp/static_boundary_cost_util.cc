#include "onboard/planner/optimization/ddp/static_boundary_cost_util.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_set>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/aabox3d.pb.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/proto/piecewise_linear_function.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/optimization/problem/mfob_path_boundary_cost.h"
#include "onboard/planner/optimization/problem/msd_static_boundary_cost.h"
#include "onboard/planner/optimization/problem/msd_static_boundary_cost_v2.h"
#include "onboard/planner/optimization/problem/static_boundary_cost.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/min_segment_distance_problem.h"
#include "onboard/planner/util/qtfm_segment_matcher_v2.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DEFINE_int32(static_boundary_canvas_level, 0,
             "Boundary points canvas verbosity level.");

DEFINE_bool(trajectory_optimizer_ignore_u_turn_right_boundary, true,
            "Whether to ignore right boundary of u turn.");

namespace qcraft {
namespace planner {
namespace optimizer {
namespace {

struct CurbCostInfo {
  std::vector<double> dists_to_curb;
  std::vector<double> curb_extra_buffer;  // Only consider mirrors here.
  std::vector<double> curb_ref_gain;
};

struct PathBoundaryCostInfo {
  std::vector<double> dists_to_path_boundary;
  std::vector<double>
      dists_to_target_path_boundary;  // Only consider mirrors here.
};

void ClampCurbBufferAndExtraBuffer(
    const PathSlBoundary& path_sl_boundary,
    const PathTimeCorridor& path_time_corridor,
    const FrenetCoordinate& av_frenet_pt, double vehicle_width,
    double vehicle_half_width, double static_boundary_buffer,
    bool is_curb_drivable, const FrenetCoordinate& cur_sl,
    const FrenetCoordinate& prev_sl, double* clamped_max_curb_buffer,
    double* extra_buffer) {
  int narrowest_idx = 0;
  const auto boundary_pair = path_time_corridor.QueryBoundaryL(
      std::min(cur_sl.s, prev_sl.s), std::max(cur_sl.s, prev_sl.s),
      /*t=*/0.0, &narrowest_idx);
  const double narrowest_l =
      narrowest_idx >= path_sl_boundary.size()
          ? 0.0
          : path_sl_boundary.reference_center_l_vector()[narrowest_idx];
  const double curb_l = cur_sl.l > 0.0 && prev_sl.l > 0.0
                            ? std::min(prev_sl.l, cur_sl.l)
                            : std::max(prev_sl.l, cur_sl.l);
  constexpr double kClampBufferWithAvLThreshold = 2.0;
  *clamped_max_curb_buffer = std::numeric_limits<double>::infinity();
  if (is_curb_drivable) {
    double buffer_clamp_boundary = narrowest_l;
    const bool clamp_with_av_frenet_box =
        std::abs(av_frenet_pt.l - narrowest_l) > kClampBufferWithAvLThreshold;
    if (clamp_with_av_frenet_box) {
      buffer_clamp_boundary = curb_l > 0.0
                                  ? std::max(narrowest_l, av_frenet_pt.l)
                                  : std::min(narrowest_l, av_frenet_pt.l);
    }
    *clamped_max_curb_buffer = (curb_l > 0.0 ? curb_l - buffer_clamp_boundary
                                             : buffer_clamp_boundary - curb_l) -
                               vehicle_half_width;
  }
  if (boundary_pair.second->type == PathTimeCorridor::BoundaryInfo::kCurb &&
      boundary_pair.first->type == PathTimeCorridor::BoundaryInfo::kCurb) {
    const double boundary_width =
        boundary_pair.second->l_curb - boundary_pair.first->l_curb;
    const double max_buffer = (boundary_width - vehicle_width) * 0.5;
    *extra_buffer =
        max_buffer - std::min(static_boundary_buffer, *clamped_max_curb_buffer);
  }
}

absl::Status ClampMaxCurbBufferAndNarrowestExtraCurbBuffer(
    const Segment2d& curb_segment, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary,
    const PathTimeCorridor& path_time_corridor,
    const FrenetCoordinate& av_frenet_pt, double vehicle_width,
    double vehicle_half_width, double static_boundary_buffer,
    bool is_curb_drivable, double clamp_front_s, double clamp_end_s,
    const Segment2d& dp_end_vertical_segment,
    const Segment2d& dp_front_vertical_segment, double* clamped_max_curb_buffer,
    double* extra_buffer) {
  const auto cur_sl =
      drive_passage.QueryUnboundedFrenetCoordinateAt(curb_segment.start());
  const auto prev_sl =
      drive_passage.QueryUnboundedFrenetCoordinateAt(curb_segment.end());
  if (!cur_sl.ok() || !prev_sl.ok()) {
    return absl::InternalError("Curb End or Start frenet to xy failed.");
  }
  if ((cur_sl->s > clamp_end_s && prev_sl->s > clamp_end_s) ||
      (cur_sl->s < clamp_front_s && prev_sl->s < clamp_front_s)) {
    *clamped_max_curb_buffer = std::numeric_limits<double>::infinity();
    *extra_buffer = std::numeric_limits<double>::infinity();
  } else if ((cur_sl->s >= clamp_front_s && cur_sl->s <= clamp_end_s) &&
             (prev_sl->s >= clamp_front_s && prev_sl->s <= clamp_end_s)) {
    ClampCurbBufferAndExtraBuffer(
        path_sl_boundary, path_time_corridor, av_frenet_pt, vehicle_width,
        vehicle_half_width, static_boundary_buffer, is_curb_drivable, *cur_sl,
        *prev_sl, clamped_max_curb_buffer, extra_buffer);
  } else {
    FrenetCoordinate clamped_start_pt =
        cur_sl->s < prev_sl->s ? *cur_sl : *prev_sl;
    FrenetCoordinate clamped_end_pt =
        cur_sl->s > prev_sl->s ? *cur_sl : *prev_sl;
    Vec2d end_intersection_pt, start_intersection_pt;
    if (curb_segment.GetIntersect(dp_end_vertical_segment,
                                  &end_intersection_pt)) {
      ASSIGN_OR_RETURN(
          clamped_end_pt,
          drive_passage.QueryUnboundedFrenetCoordinateAt(end_intersection_pt));
    }
    if (curb_segment.GetIntersect(dp_front_vertical_segment,
                                  &start_intersection_pt)) {
      ASSIGN_OR_RETURN(clamped_start_pt,
                       drive_passage.QueryUnboundedFrenetCoordinateAt(
                           start_intersection_pt));
    }
    ClampCurbBufferAndExtraBuffer(
        path_sl_boundary, path_time_corridor, av_frenet_pt, vehicle_width,
        vehicle_half_width, static_boundary_buffer, is_curb_drivable,
        clamped_start_pt, clamped_end_pt, clamped_max_curb_buffer,
        extra_buffer);
  }
  return absl::OkStatus();
}

// NOLINTNEXTLINE
void CollectCurbSegmentsAroundDrivePassage(
    bool consider_mirrors_by_default,
    const VehicleGeometryParamsProto& veh_geo_params,
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary,
    const PathTimeCorridor& path_time_corridor, double static_boundary_buffer,
    const std::function<double(const Vec2d&)>& query_extra_buffer,
    double mirror_extra_buffer, const std::optional<double>& u_turn_middle_s,
    const TrajectoryPoint& plan_start_point, double* clamped_max_curb_buffer,
    std::vector<MsdProblemWithBuffer::SegmentType>* curb_named_segments,
    std::vector<int>* curb_start_segment_ids) {
  QCHECK_NOTNULL(curb_named_segments);
  QCHECK_NOTNULL(curb_start_segment_ids);

  // Collect curb boundaries around drive passage without repeated ones.
  std::unordered_set<mapping::ElementId> curb_boundaries;
  for (int i = 1; i < drive_passage.size(); ++i) {
    // Don't collect curb for extended drive passage.
    if (i > drive_passage.last_real_station_index().value()) {
      break;
    }
    const Vec2d p0 = drive_passage.station(StationIndex(i - 1)).xy();
    const Vec2d& p1 = drive_passage.station(StationIndex(i)).xy();
    const double search_radius = kMaxLateralOffset + (p1 - p0).Length() * 0.5;
    const Vec2d search_center = 0.5 * (p0 + p1);
    const auto candidate_curbs = psmm.GetImpassableBoundariesInfoAtLevel(
        psmm.GetLevel(), search_center, search_radius);
    for (const auto& candidate_curb : candidate_curbs) {
      curb_boundaries.insert(candidate_curb.lane_boundary_id);
    }
  }

  const auto is_on_uturn_right = [&drive_passage,
                                  &u_turn_middle_s](const Segment2d& boundary) {
    if (!u_turn_middle_s.has_value()) return false;
    const auto start_frenet_or =
        drive_passage.QueryUnboundedFrenetCoordinateAt(boundary.start());
    if (!start_frenet_or.ok()) return false;
    const auto end_frenet_or =
        drive_passage.QueryUnboundedFrenetCoordinateAt(boundary.end());
    if (!end_frenet_or.ok()) return false;
    if ((start_frenet_or->s >= *u_turn_middle_s && start_frenet_or->l < 0.0) ||
        (end_frenet_or->s >= *u_turn_middle_s && end_frenet_or->l < 0.0)) {
      return true;
    }
    return false;
  };

  // Mirror info.
  const bool has_mirror =
      veh_geo_params.has_left_mirror() && veh_geo_params.has_right_mirror();
  const double mirror_height_avg =
      ((veh_geo_params.left_mirror().z() -
        veh_geo_params.left_mirror().height() * 0.5) +
       (veh_geo_params.right_mirror().z() -
        veh_geo_params.right_mirror().height() * 0.5)) *
      0.5;

  const auto last_real_station_index = drive_passage.last_real_station_index();
  const auto& last_real_station =
      drive_passage.station(last_real_station_index);
  const auto& first_station = drive_passage.stations().front();
  const Segment2d dp_end_vertical_segment(
      last_real_station.lat_point(-kMaxLateralOffset),
      last_real_station.lat_point(kMaxLateralOffset));
  const Segment2d dp_front_vertical_segment(
      first_station.lat_point(-kMaxLateralOffset),
      first_station.lat_point(kMaxLateralOffset));
  const auto av_frenet_pt =
      drive_passage.QueryUnboundedFrenetCoordinateAt(plan_start_point.pos());
  if (!av_frenet_pt.ok()) return;

  // Emplace to outputs.
  curb_named_segments->clear();
  curb_start_segment_ids->clear();
  for (const auto& id : curb_boundaries) {
    const auto* lane_boundary = psmm.FindLaneBoundaryByIdOrNull(id);
    if (lane_boundary == nullptr) continue;
    const mapping::LaneBoundaryInfo& boundary_info = *lane_boundary;
    const bool consider_mirrors =
        has_mirror && (boundary_info.proto->has_height()
                           ? (boundary_info.proto->height() > mirror_height_avg)
                           : consider_mirrors_by_default);
    const std::vector<Vec2d>& points = boundary_info.points_smooth;
    if (points.empty()) {
      continue;
    }

    bool has_no_prev_segment = true;
    Vec2d prev = points.front();
    for (int i = 1; i < points.size(); ++i) {
      const auto& cur_pt = points[i];
      // Skip short segments.
      if (prev.DistanceTo(cur_pt) <= QtfmSegmentMatcherV2::kMinSegmentLength) {
        continue;
      }

      const Segment2d cur_seg(prev, cur_pt);
      const double extra_buffer = std::min(query_extra_buffer(cur_seg.start()),
                                           query_extra_buffer(cur_seg.end()));
      const double vehicle_width =
          consider_mirrors ? 2.0 * mirror_extra_buffer + veh_geo_params.width()
                           : veh_geo_params.width();
      const double vehicle_half_width = 0.5 * vehicle_width;

      double current_clamped_max_curb_buffer, extra_buffer_curb;
      double buffer =
          consider_mirrors ? extra_buffer + mirror_extra_buffer : extra_buffer;
      const bool is_curb_drivable =
          boundary_info.proto->has_is_drivable_boundary() &&
          boundary_info.proto->is_drivable_boundary();

      if (ClampMaxCurbBufferAndNarrowestExtraCurbBuffer(
              cur_seg, drive_passage, path_sl_boundary, path_time_corridor,
              *av_frenet_pt, vehicle_width, vehicle_half_width,
              static_boundary_buffer, is_curb_drivable, drive_passage.front_s(),
              last_real_station.accumulated_s(), dp_end_vertical_segment,
              dp_front_vertical_segment, &current_clamped_max_curb_buffer,
              &extra_buffer_curb)
              .ok()) {
        *clamped_max_curb_buffer =
            std::min(*clamped_max_curb_buffer, current_clamped_max_curb_buffer);
        buffer = std::min(buffer, extra_buffer_curb);
      }
      const double uturn_gain =
          is_on_uturn_right(cur_seg) ? kUTurnCurbGain : 1.0;

      // Append a segment.
      curb_named_segments->push_back(
          {absl::StrCat("id:", id, ",seg:", i), cur_seg, buffer, uturn_gain});
      if (has_no_prev_segment) {
        curb_start_segment_ids->push_back(curb_named_segments->size() - 1);
        has_no_prev_segment = false;
      }
      prev = points[i];
    }
  }
}

void CollectExtendSolidLinesWithinPathBoundary(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const ConstraintManager& constraint_manager,
    const std::vector<TrajectoryPoint>& smooth_init_traj,
    double solid_line_buffer, double extend_distance,
    std::vector<int>* solid_line_start_segment_ids,
    std::vector<MsdProblemWithBuffer::SegmentType>* solid_line_segments) {
  // Check if solid line has overlap with trajectory.
  constexpr double kVehicleBoxHalfWidth = 0.2;  // m.
  const double lateral_buffer =
      kVehicleBoxHalfWidth - 0.5 * vehicle_geometry_params.width();
  std::vector<Box2d> av_boxes;
  av_boxes.reserve(smooth_init_traj.size());
  for (const auto& pt : smooth_init_traj) {
    av_boxes.push_back(
        ComputeAvBoxWithBuffer(pt.pos(), pt.theta(), vehicle_geometry_params,
                               /*length_buffer=*/0.0, lateral_buffer));
  }
  const auto has_overlap_with_solid_line =
      [&av_boxes](const std::vector<Vec2d>& points) {
        for (int i = 0; i + 1 < points.size(); ++i) {
          const Segment2d segment(points[i], points[i + 1]);
          for (const auto& box : av_boxes) {
            if (box.HasOverlap(segment)) return true;
          }
        }
        return false;
      };

  const Vec2d start_pos = smooth_init_traj.front().pos();
  const double start_theta = smooth_init_traj.front().theta();

  solid_line_start_segment_ids->clear();
  solid_line_segments->clear();
  for (const auto& avoid_line : constraint_manager.AvoidLine()) {
    if (!avoid_line.source().has_solid_line_within_boundary()) continue;
    std::vector<Vec2d> solid_line;
    solid_line.reserve(avoid_line.xy_points().size() + 2);
    for (const auto& pt : avoid_line.xy_points()) {
      solid_line.emplace_back(pt.x(), pt.y());
    }
    if (has_overlap_with_solid_line(solid_line)) continue;

    // Extend at start and end.
    QCHECK_GT(solid_line.size(), 1);
    const Segment2d seg1(solid_line[1], solid_line[0]);
    solid_line.insert(solid_line.begin(),
                      seg1.end() + seg1.unit_direction() * extend_distance);
    const Segment2d seg2(solid_line[solid_line.size() - 2], solid_line.back());
    solid_line.insert(solid_line.end(),
                      seg2.end() + seg2.unit_direction() * extend_distance);

    bool has_no_prev_segment = true;
    Vec2d prev = solid_line.front();
    for (int i = 1; i < solid_line.size(); ++i) {
      // Skip short segments.
      if (prev.DistanceTo(solid_line[i]) <=
          QtfmSegmentMatcherV2::kMinSegmentLength) {
        continue;
      }
      const Segment2d line(prev, solid_line[i]);
      double extra_buffer = 0.0;  // m.
      for (const auto& circle :
           trajectory_optimizer_vehicle_model_params.circles()) {
        const Vec2d center =
            start_pos +
            Vec2d::FastUnitFromAngleN12(start_theta + circle.angle_to_axis()) *
                circle.dist_to_rac();
        const double dist = line.DistanceTo(center) - circle.radius();
        extra_buffer = std::min(extra_buffer, dist - solid_line_buffer);
      }
      for (const auto& circle :
           trajectory_optimizer_vehicle_model_params.mirror_circles()) {
        const Vec2d center =
            start_pos +
            Vec2d::FastUnitFromAngleN12(start_theta + circle.angle_to_axis()) *
                circle.dist_to_rac();
        const double dist = line.DistanceTo(center) - circle.radius();
        extra_buffer = std::min(extra_buffer, dist - solid_line_buffer);
      }

      // Append a segment.
      solid_line_segments->push_back(
          {absl::StrCat("id:", avoid_line.id(), ",seg:", i), line,
           extra_buffer});
      prev = solid_line[i];

      if (has_no_prev_segment) {
        solid_line_start_segment_ids->push_back(solid_line_segments->size() -
                                                1);
        has_no_prev_segment = false;
      }
    }
  }
}

void MaybeUpdateBreakPoint(double dist, double alpha,
                           StaticBoundaryCost<Mfob>::BreakPoint* break_point) {
  if (alpha <= 0.0 && dist < break_point->dists.front()) {
    break_point->has_break_point = true;
    break_point->dists.front() = dist;
    return;
  }
  if (alpha >= 1.0 && dist < break_point->dists.back()) {
    break_point->has_break_point = true;
    break_point->dists.back() = dist;
    return;
  }
  if (alpha <= 0.0 || alpha >= 1.0) return;
  const auto inner_index_pair = break_point->GetIndexPair(alpha);
  const double old_dist =
      break_point->ComputeBoundaryDist(inner_index_pair, alpha);
  // Update only when curb is closer than currrent break points.
  constexpr double kMinDiff = 0.01;  // m.
  if (dist + kMinDiff > old_dist) return;
  break_point->has_break_point = true;
  // Insert new point.
  const auto iter = std::upper_bound(break_point->alphas.begin(),
                                     break_point->alphas.end(), alpha);
  QCHECK(iter != break_point->alphas.begin() &&
         iter != break_point->alphas.end());
  // Only change dist if alphas are very close.
  constexpr double kEps = 0.001;
  bool is_close = false;
  for (int i = 0; i < break_point->alphas.size(); ++i) {
    if (std::abs(break_point->alphas[i] - alpha) < kEps) {
      break_point->dists[i] = dist;
      is_close = true;
      break;
    }
  }
  // Insert one more alpha and dist.
  if (!is_close) {
    const int index = std::distance(break_point->alphas.begin(), iter);
    break_point->alphas.insert(iter, alpha);
    break_point->dists.insert(break_point->dists.begin() + index, dist);
  }
  // Update alpha_interval_inv.
  break_point->alpha_interval_inv.clear();
  break_point->alpha_interval_inv.reserve(break_point->alphas.size() - 1);
  for (int i = 0; i + 1 < break_point->alphas.size(); ++i) {
    break_point->alpha_interval_inv.push_back(
        1.0 / (break_point->alphas[i + 1] - break_point->alphas[i]));
  }
}

void CollectStaticBoundaryBreakPoints(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const FrenetFrame& path, const std::vector<double>& left_dists_to_curb,
    const std::vector<double>& right_dists_to_curb,
    std::vector<StaticBoundaryCost<Mfob>::BreakPoint>* left_curb_break_points,
    std::vector<StaticBoundaryCost<Mfob>::BreakPoint>*
        right_curb_break_points) {
  const int break_points_size = left_curb_break_points->size();
  QCHECK_EQ(break_points_size + 1, left_dists_to_curb.size());
  // Init break points.
  for (int i = 0; i < break_points_size; ++i) {
    (*left_curb_break_points)[i].alphas = {0.0, 1.0};
    (*left_curb_break_points)[i].alpha_interval_inv = {1.0};
    (*left_curb_break_points)[i].dists = {left_dists_to_curb[i],
                                          left_dists_to_curb[i + 1]};
    (*right_curb_break_points)[i].alphas = {0.0, 1.0};
    (*right_curb_break_points)[i].alpha_interval_inv = {1.0};
    (*right_curb_break_points)[i].dists = {right_dists_to_curb[i],
                                           right_dists_to_curb[i + 1]};
  }
  // Collect curb boundaries around drive passage without repeated ones.
  std::unordered_set<mapping::ElementId> curb_boundaries;
  for (int i = 1; i < drive_passage.size(); ++i) {
    // Don't collect curb for extended drive passage.
    if (i > drive_passage.last_real_station_index().value()) {
      break;
    }
    const Vec2d p0 = drive_passage.station(StationIndex(i - 1)).xy();
    const Vec2d& p1 = drive_passage.station(StationIndex(i)).xy();
    const double search_radius = kMaxLateralOffset + (p1 - p0).Length() * 0.5;
    const Vec2d search_center = 0.5 * (p0 + p1);
    const auto candidate_curbs = psmm.GetImpassableBoundariesInfoAtLevel(
        psmm.GetLevel(), search_center, search_radius);
    for (const auto& candidate_curb : candidate_curbs) {
      curb_boundaries.insert(candidate_curb.lane_boundary_id);
    }
  }

  // Only consider start and end of curb as potential break points.
  for (const auto& id : curb_boundaries) {
    const auto* lane_boundary = psmm.FindLaneBoundaryByIdOrNull(id);
    if (lane_boundary == nullptr) continue;
    const std::vector<Vec2d>& points = lane_boundary->points_smooth;
    FrenetCoordinate sl;
    Vec2d normal;
    std::pair<int, int> index_pair;
    double alpha = 0.0;
    // Check start.
    path.XYToSL(points.front(), &sl, &normal, &index_pair, &alpha);
    if (index_pair.first < break_points_size &&
        !(index_pair.first == 0 && alpha < 0.0) &&
        !(index_pair.first == break_points_size - 1 && alpha > 1.0)) {
      if (sl.l > 0.0) {
        MaybeUpdateBreakPoint(sl.l, alpha,
                              &(*left_curb_break_points)[index_pair.first]);
      } else {
        MaybeUpdateBreakPoint(-sl.l, alpha,
                              &(*right_curb_break_points)[index_pair.first]);
      }
    }
    // Check end.
    path.XYToSL(points.back(), &sl, &normal, &index_pair, &alpha);
    if (index_pair.first < break_points_size &&
        !(index_pair.first == 0 && alpha < 0.0) &&
        !(index_pair.first == break_points_size - 1 && alpha > 1.0)) {
      if (sl.l > 0.0) {
        MaybeUpdateBreakPoint(sl.l, alpha,
                              &(*left_curb_break_points)[index_pair.first]);
      } else {
        MaybeUpdateBreakPoint(-sl.l, alpha,
                              &(*right_curb_break_points)[index_pair.first]);
      }
    }
  }
}

void CanvasDrawSegments(
    const std::string& name, const vis::Color& color,
    const std::vector<MsdProblemWithBuffer::SegmentType>& named_segments) {
  vis::Canvas* canvas = nullptr;
  canvas = &vis::vantage::GetCanvasClient()->GetCanvas(name);
  canvas->SetGroundZero(1);
  if (canvas == nullptr) {
    return;
  }
  constexpr double kLineZ = 0.2;
  constexpr int kLineWidth = 3;
  for (const auto& named_segment : named_segments) {
    // Distinguish uturn curb.
    constexpr double kEps = 1.0e-6;
    auto draw_color = color;
    if (std::abs(named_segment.gain - kUTurnCurbGain) < kEps) {
      draw_color = vis::Color(0.8, 0.7, 0.7);
    }
    const Segment2d& seg = named_segment.segment;
    canvas->DrawLine({seg.start().x(), seg.start().y(), kLineZ},
                     {seg.end().x(), seg.end().y(), kLineZ}, draw_color,
                     kLineWidth);
    // Draw extra buffer.
    constexpr double kMinDrawBuffer = 0.01;  // m.
    const double buffer = named_segment.buffer;
    if (buffer > kMinDrawBuffer) {
      canvas->DrawLine(
          {seg.center() + buffer * seg.unit_direction().Perp(), kLineZ},
          {seg.center() - buffer * seg.unit_direction().Perp(), kLineZ},
          vis::Color::kRed, kLineWidth);
    } else if (buffer < -kMinDrawBuffer) {
      canvas->DrawLine(
          {seg.center() + buffer * seg.unit_direction().Perp(), kLineZ},
          {seg.center() - buffer * seg.unit_direction().Perp(), kLineZ},
          vis::Color::kWhite, kLineWidth);
    }
  }
}

void CanvasDrawStaticBoundary(
    const std::string& name, const vis::Color& color,
    const DrivePassage& drive_passage, const CurbCostInfo& left_cci,
    const std::vector<StaticBoundaryCost<Mfob>::BreakPoint>&
        left_curb_break_points,
    const CurbCostInfo& right_cci,
    const std::vector<StaticBoundaryCost<Mfob>::BreakPoint>&
        right_curb_break_points) {
  auto* canvas = &vis::vantage::GetCanvasClient()->GetCanvas(name);
  auto* break_points_canvas =
      &vis::vantage::GetCanvasClient()->GetCanvas(name + "_break_points");
  canvas->SetGroundZero(1);
  break_points_canvas->SetGroundZero(1);
  if (canvas == nullptr || break_points_canvas == nullptr) {
    return;
  }

  constexpr double kLineZ = 0.2;
  constexpr double kBreakPointsZ = 0.25;
  constexpr int kLineWidth = 3;
  std::vector<Vec2d> left_curb_points;
  left_curb_points.reserve(left_cci.dists_to_curb.size());
  std::vector<Vec2d> right_curb_points;
  left_curb_points.reserve(right_cci.dists_to_curb.size());
  std::vector<Vec2d> normals;
  normals.reserve(right_cci.dists_to_curb.size());
  const auto& stations = drive_passage.stations();
  for (int i = 0; i < stations.size(); ++i) {
    left_curb_points.push_back(
        stations[StationIndex(i)].lat_point(left_cci.dists_to_curb[i]));
    right_curb_points.push_back(
        stations[StationIndex(i)].lat_point(-right_cci.dists_to_curb[i]));
    normals.push_back(stations[StationIndex(i)].tangent().Perp());
  }
  for (int i = 0; i + 1 < stations.size(); ++i) {
    // Distinguish uturn curb.
    constexpr double kEps = 1.0e-6;
    auto left_draw_color = color;
    auto right_draw_color = color;
    if (std::abs(left_cci.curb_ref_gain[i] - kUTurnCurbGain) < kEps) {
      left_draw_color = vis::Color(0.8, 0.7, 0.7);
    }
    if (std::abs(right_cci.curb_ref_gain[i] - kUTurnCurbGain) < kEps) {
      right_draw_color = vis::Color(0.8, 0.7, 0.7);
    }
    canvas->DrawLine({left_curb_points[i], kLineZ},
                     {left_curb_points[i + 1], kLineZ}, left_draw_color,
                     kLineWidth);
    canvas->DrawLine({right_curb_points[i], kLineZ},
                     {right_curb_points[i + 1], kLineZ}, right_draw_color,
                     kLineWidth);
    // Draw break points.
    if (left_curb_break_points[i].has_break_point) {
      std::vector<Vec3d> left_break_points;
      left_break_points.reserve(left_curb_break_points[i].alphas.size() - 1);
      for (int j = 0; j < left_curb_break_points[i].alphas.size(); ++j) {
        const double point_s = stations[StationIndex(i)].accumulated_s() +
                               (stations[StationIndex(i + 1)].accumulated_s() -
                                stations[StationIndex(i)].accumulated_s()) *
                                   left_curb_break_points[i].alphas[j];
        left_break_points.emplace_back(
            drive_passage
                .QueryPointXYAtSL(point_s, left_curb_break_points[i].dists[j])
                .value(),
            kBreakPointsZ);
      }
      break_points_canvas->DrawLineStrip(left_break_points, vis::Color::kViolet,
                                         kLineWidth);
    }
    if (right_curb_break_points[i].has_break_point) {
      std::vector<Vec3d> right_break_points;
      right_break_points.reserve(right_curb_break_points[i].alphas.size() - 1);
      for (int j = 0; j < right_curb_break_points[i].alphas.size(); ++j) {
        const double point_s = stations[StationIndex(i)].accumulated_s() +
                               (stations[StationIndex(i + 1)].accumulated_s() -
                                stations[StationIndex(i)].accumulated_s()) *
                                   right_curb_break_points[i].alphas[j];
        right_break_points.emplace_back(
            drive_passage
                .QueryPointXYAtSL(point_s, -right_curb_break_points[i].dists[j])
                .value(),
            kBreakPointsZ);
      }
      break_points_canvas->DrawLineStrip(right_break_points,
                                         vis::Color::kViolet, kLineWidth);
    }
    // Draw extra buffer.
    constexpr double kMinDrawBuffer = 0.01;  // m.
    const vis::Color left_color = left_cci.curb_extra_buffer[i] > 0.0
                                      ? vis::Color::kRed
                                      : vis::Color::kWhite;
    if (left_cci.curb_extra_buffer[i] > kMinDrawBuffer ||
        left_cci.curb_extra_buffer[i] < -kMinDrawBuffer) {
      canvas->DrawLine(
          {left_curb_points[i] + left_cci.curb_extra_buffer[i] * normals[i],
           kLineZ},
          {left_curb_points[i] - left_cci.curb_extra_buffer[i] * normals[i],
           kLineZ},
          left_color, kLineWidth);
    }
    const vis::Color right_color = right_cci.curb_extra_buffer[i] > 0.0
                                       ? vis::Color::kRed
                                       : vis::Color::kWhite;
    if (right_cci.curb_extra_buffer[i] > kMinDrawBuffer ||
        right_cci.curb_extra_buffer[i] < -kMinDrawBuffer) {
      canvas->DrawLine(
          {right_curb_points[i] + right_cci.curb_extra_buffer[i] * normals[i],
           kLineZ},
          {right_curb_points[i] - right_cci.curb_extra_buffer[i] * normals[i],
           kLineZ},
          right_color, kLineWidth);
    }
  }
}

void GeneratePathBoundariesWithRightBoundaryAfterUTurnIgnored(
    const VehicleGeometryParamsProto& veh_geo_params,
    double mirror_extra_buffer, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary, CurbCostInfo* left_cci,
    CurbCostInfo* right_cci, PathBoundaryCostInfo* left_pbci,
    PathBoundaryCostInfo* right_pbci,
    std::vector<double>* reference_center_l_offsets,
    std::optional<double>* uturn_middle_s) {
  FUNC_QTRACE();

  const bool has_mirror =
      veh_geo_params.has_left_mirror() && veh_geo_params.has_right_mirror();
  const double mirror_height_avg =
      ((veh_geo_params.left_mirror().z() -
        veh_geo_params.left_mirror().height() * 0.5) +
       (veh_geo_params.right_mirror().z() -
        veh_geo_params.right_mirror().height() * 0.5)) *
      0.5;

  // Get first uturn middle s.
  bool u_turn_passed = false;
  double u_turn_start_s = 0.0;
  double u_turn_end_s = 0.0;
  for (const auto& station : drive_passage.stations()) {
    if (station.direction() == mapping::LaneProto::UTURN) {
      if (!u_turn_passed) {
        u_turn_start_s = station.accumulated_s();
        u_turn_end_s = station.accumulated_s();
        u_turn_passed = true;
      } else {
        u_turn_end_s = station.accumulated_s();
      }
    } else if (u_turn_passed) {
      break;
    }
  }
  if (u_turn_passed) {
    *uturn_middle_s = 0.5 * (u_turn_start_s + u_turn_end_s);
  }

  // Collect curb.
  constexpr double kEps = 1e-6;
  const auto drive_passage_size = drive_passage.stations().size();
  left_cci->dists_to_curb.reserve(drive_passage_size);
  left_cci->curb_extra_buffer.reserve(drive_passage_size);
  left_cci->curb_ref_gain.resize(drive_passage_size, 1.0);
  right_cci->dists_to_curb.reserve(drive_passage_size);
  right_cci->curb_extra_buffer.reserve(drive_passage_size);
  right_cci->curb_ref_gain.reserve(drive_passage_size);
  for (int i = 0; i < drive_passage.stations().size(); ++i) {
    const auto& station = drive_passage.stations()[StationIndex(i)];
    const auto curb_info_or =
        station.QueryCurbOffsetAndHeightAt(/*signed_lat=*/0.0);
    QCHECK_OK(curb_info_or.status());
    left_cci->dists_to_curb.push_back(curb_info_or->offset.second);
    left_cci->curb_extra_buffer.push_back(
        (has_mirror && curb_info_or->height.second) > mirror_height_avg
            ? mirror_extra_buffer
            : 0.0);
    right_cci->dists_to_curb.push_back(-curb_info_or->offset.first);
    right_cci->curb_extra_buffer.push_back(
        (has_mirror && curb_info_or->height.first) > mirror_height_avg
            ? mirror_extra_buffer
            : 0.0);
    if (uturn_middle_s->has_value() &&
        station.accumulated_s() + kEps >= **uturn_middle_s) {
      right_cci->curb_ref_gain.push_back(kUTurnCurbGain);
    } else {
      right_cci->curb_ref_gain.push_back(1.0);
    }
  }

  // Ignore right path boundary from uturn middle s.
  constexpr double kBoundaryExtendDistOnUTurn = 20.0;  // m.
  const auto path_boundary_size = path_sl_boundary.size();
  left_pbci->dists_to_path_boundary.reserve(path_boundary_size);
  left_pbci->dists_to_target_path_boundary.reserve(path_boundary_size);
  right_pbci->dists_to_path_boundary.reserve(path_boundary_size);
  right_pbci->dists_to_target_path_boundary.reserve(path_boundary_size);
  reference_center_l_offsets->reserve(path_boundary_size);
  QCHECK_GE(drive_passage.stations().size(), path_boundary_size);
  for (int i = 0; i < path_boundary_size; ++i) {
    left_pbci->dists_to_path_boundary.push_back(
        path_sl_boundary.left_l_vector()[i]);
    left_pbci->dists_to_target_path_boundary.push_back(
        path_sl_boundary.target_left_l_vector()[i]);
    if (uturn_middle_s->has_value() &&
        drive_passage.stations()[StationIndex(i)].accumulated_s() + kEps >=
            **uturn_middle_s) {
      right_pbci->dists_to_path_boundary.push_back(
          -path_sl_boundary.right_l_vector()[i] + kBoundaryExtendDistOnUTurn);
      right_pbci->dists_to_target_path_boundary.push_back(
          -path_sl_boundary.target_right_l_vector()[i]);
    } else {
      right_pbci->dists_to_path_boundary.push_back(
          -path_sl_boundary.right_l_vector()[i]);
      right_pbci->dists_to_target_path_boundary.push_back(
          -path_sl_boundary.target_right_l_vector()[i]);
    }
    reference_center_l_offsets->push_back(
        path_sl_boundary.reference_center_l_vector()[i]);
  }
}

void GeneratePathBoundaries(const VehicleGeometryParamsProto& veh_geo_params,
                            double mirror_extra_buffer,
                            const DrivePassage& drive_passage,
                            const PathSlBoundary& path_sl_boundary,
                            CurbCostInfo* left_cci, CurbCostInfo* right_cci,
                            PathBoundaryCostInfo* left_pbci,
                            PathBoundaryCostInfo* right_pbci,
                            std::vector<double>* reference_center_l_offsets) {
  FUNC_QTRACE();

  const bool has_mirror =
      veh_geo_params.has_left_mirror() && veh_geo_params.has_right_mirror();
  const double mirror_height_avg =
      ((veh_geo_params.left_mirror().z() -
        veh_geo_params.left_mirror().height() * 0.5) +
       (veh_geo_params.right_mirror().z() -
        veh_geo_params.right_mirror().height() * 0.5)) *
      0.5;

  const auto drive_passage_size = drive_passage.stations().size();
  left_cci->dists_to_curb.reserve(drive_passage_size);
  left_cci->curb_extra_buffer.reserve(drive_passage_size);
  left_cci->curb_ref_gain.resize(drive_passage_size, 1.0);
  right_cci->dists_to_curb.reserve(drive_passage_size);
  right_cci->curb_extra_buffer.reserve(drive_passage_size);
  right_cci->curb_ref_gain.resize(drive_passage_size, 1.0);
  for (int i = 0; i < drive_passage.stations().size(); ++i) {
    const auto& station = drive_passage.stations()[StationIndex(i)];
    const auto curb_info_or =
        station.QueryCurbOffsetAndHeightAt(/*signed_lat=*/0.0);
    QCHECK_OK(curb_info_or.status());
    left_cci->dists_to_curb.push_back(curb_info_or->offset.second);
    left_cci->curb_extra_buffer.push_back(
        (has_mirror && curb_info_or->height.second > mirror_height_avg)
            ? mirror_extra_buffer
            : 0.0);
    right_cci->dists_to_curb.push_back(-curb_info_or->offset.first);
    right_cci->curb_extra_buffer.push_back(
        (has_mirror && curb_info_or->height.first > mirror_height_avg)
            ? mirror_extra_buffer
            : 0.0);
  }

  const auto path_boundary_size = path_sl_boundary.size();
  left_pbci->dists_to_path_boundary.reserve(path_boundary_size);
  left_pbci->dists_to_target_path_boundary.reserve(path_boundary_size);
  right_pbci->dists_to_path_boundary.reserve(path_boundary_size);
  right_pbci->dists_to_target_path_boundary.reserve(path_boundary_size);
  reference_center_l_offsets->reserve(path_boundary_size);
  QCHECK_GE(drive_passage.stations().size(), path_boundary_size);
  for (int i = 0; i < path_boundary_size; ++i) {
    left_pbci->dists_to_path_boundary.push_back(
        path_sl_boundary.left_l_vector()[i]);
    left_pbci->dists_to_target_path_boundary.push_back(
        path_sl_boundary.target_left_l_vector()[i]);
    right_pbci->dists_to_path_boundary.push_back(
        -path_sl_boundary.right_l_vector()[i]);
    right_pbci->dists_to_target_path_boundary.push_back(
        -path_sl_boundary.target_right_l_vector()[i]);
    reference_center_l_offsets->push_back(
        path_sl_boundary.reference_center_l_vector()[i]);
  }
}

void ComputePathTimeCorridorAndCurvatureCurbBuffer(
    const VehicleGeometryParamsProto& veh_geo_params,
    double mirror_extra_buffer, const DrivePassage& drive_passage,
    const PathTimeCorridor& path_time_corridor, double static_boundary_buffer,
    const std::function<double(const Vec2d&)>& query_extra_buffer,
    const std::vector<double>& left_dists_to_curb,
    const std::vector<double>& right_dists_to_curb,
    std::vector<double>* left_curb_extra_buffer,
    std::vector<double>* right_curb_extra_buffer) {
  constexpr double kEps = 1.0e-6;
  const auto& stations = drive_passage.stations();
  for (int i = 0; i < stations.size(); ++i) {
    const Vec2d left_curb_pos =
        stations[StationIndex(i)].lat_point(left_dists_to_curb[i]);
    // Assume that the input left_curb_extra_buffer only consider mirrors.
    const bool left_consider_mirror = (*left_curb_extra_buffer)[i] > kEps;
    (*left_curb_extra_buffer)[i] += query_extra_buffer(left_curb_pos);
    const Vec2d right_curb_pos =
        stations[StationIndex(i)].lat_point(-right_dists_to_curb[i]);
    // Assume that the input right_curb_extra_buffer only consider mirrors.
    const bool right_consider_mirror = (*right_curb_extra_buffer)[i] > kEps;
    (*right_curb_extra_buffer)[i] += query_extra_buffer(right_curb_pos);

    double vehicle_width = veh_geo_params.width();
    if (left_consider_mirror) {
      vehicle_width += mirror_extra_buffer;
    }
    if (right_consider_mirror) {
      vehicle_width += mirror_extra_buffer;
    }
    // Skip the last station.
    if (i + 1 == stations.size()) continue;
    const auto boundary_pair = path_time_corridor.QueryBoundaryL(
        stations[StationIndex(i)].accumulated_s(),
        stations[StationIndex(i + 1)].accumulated_s(),
        /*t=*/0.0, /*min_lane_width_idx=*/nullptr);
    if (boundary_pair.second->type == PathTimeCorridor::BoundaryInfo::kCurb &&
        boundary_pair.first->type == PathTimeCorridor::BoundaryInfo::kCurb) {
      const double boundary_width =
          boundary_pair.second->l_boundary - boundary_pair.first->l_boundary;
      const double max_buffer = (boundary_width - vehicle_width) * 0.5;
      const double max_extra_buffer = max_buffer - static_boundary_buffer;
      const double buffer_offset = (*left_curb_extra_buffer)[i] +
                                   (*right_curb_extra_buffer)[i] -
                                   max_extra_buffer;
      if (buffer_offset > 0.0) {
        (*left_curb_extra_buffer)[i] -= buffer_offset;
        (*right_curb_extra_buffer)[i] -= buffer_offset;
      }
    }
  }
}

void AddMsdStaticBoundaryCost(
    int trajectory_steps, std::string_view base_name,
    std::string_view source_name,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::vector<int>& start_segment_ids,
    std::vector<MsdProblemWithBuffer::SegmentType> named_segments,
    std::vector<std::string> sub_names, std::vector<double> cascade_buffers,
    std::vector<double> cascade_gains,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  FUNC_QTRACE();

  constexpr double kCutoffDistExtraBuffer = 0.1;
  const double cutoff_distance =
      *std::max_element(cascade_buffers.begin(), cascade_buffers.end()) +
      0.5 * veh_geo_params.width() + kCutoffDistExtraBuffer;
  MsdProblemWithBuffer msd(std::move(named_segments), cutoff_distance,
                           start_segment_ids);

  if (FLAGS_static_boundary_canvas_level >= 1) {
    vis::Canvas* canvas = &vis::vantage::GetCanvasClient()->GetCanvas(
        absl::StrFormat("%s/qtfm_for_%s", base_name, source_name));
    msd.DrawQtfmSegmentMatcher(canvas);
  }

  std::vector<Vec2d> circle_center_offsets;
  std::vector<double> circle_radiuses;
  const int circle_size =
      trajectory_optimizer_vehicle_model_params.circles_size();
  circle_center_offsets.reserve(circle_size);
  circle_radiuses.reserve(circle_size);
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    double cos_sin[2];
    fast_math::CosAndSin<7>(circle.angle_to_axis(), cos_sin);
    circle_center_offsets.push_back(Vec2d(circle.dist_to_rac() * cos_sin[0],
                                          circle.dist_to_rac() * cos_sin[1]));
    circle_radiuses.push_back(circle.radius());
  }

  costs->emplace_back(std::make_unique<MsdStaticBoundaryCost<Mfob>>(
      trajectory_steps, veh_geo_params, std::move(msd),
      stations_query_helper.get(), std::move(sub_names),
      std::move(cascade_buffers), std::move(cascade_gains),
      circle_center_offsets, circle_radiuses,
      /*effect_index=*/trajectory_steps,
      /*ignore_invasion_second_order_derivative=*/true, "MsdStaticBoundaryCost",
      cost_weight_params.msd_static_boundary_cost_weight(),
      /*cost_type=*/source_name == "solid_line"
          ? Cost<Mfob>::CostType::SOLID_LINE_MSD_STATIC_BOUNDARY
          : Cost<Mfob>::CostType::MUST_HAVE));
}

// Don't use this function for solid line!
// It is not supposed to do so.
void AddMsdStaticBoundaryCostV2(
    int trajectory_steps, std::string_view base_name,
    std::string_view source_name, double source_scale,
    double clamped_max_curb_buffer,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::vector<int>& start_segment_ids,
    std::vector<MsdProblemWithBuffer::SegmentType> named_segments,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  QCHECK_GT(trajectory_steps, 0);

  // Hard layer.
  MsdStaticBoundaryCostV2<Mfob>::Layer hard_layer{
      .gain_of_weight = cost_weight_params.static_boundary_hard_cost_weight(),
      .buffer_at_speed = PiecewiseLinearFunctionFromProto(
          cost_weight_params.speed_rel_hard_curb_clearance_plf())};

  // Soft layer.
  const auto& soft_plf = cost_weight_params.speed_rel_soft_curb_clearance_plf();
  std::vector<double> soft_buffer_at_speed_x(soft_plf.x().begin(),
                                             soft_plf.x().end());
  std::vector<double> soft_buffer_at_speed_y(soft_plf.y().begin(),
                                             soft_plf.y().end());
  for (int idx = 0; idx < soft_buffer_at_speed_y.size(); ++idx) {
    soft_buffer_at_speed_y[idx] =
        std::min(soft_buffer_at_speed_y[idx], clamped_max_curb_buffer);
  }
  MsdStaticBoundaryCostV2<Mfob>::Layer soft_layer{
      .gain_of_weight = cost_weight_params.static_boundary_soft_cost_weight(),
      .buffer_at_speed = PiecewiseLinearFunction<double>{
          soft_buffer_at_speed_x, soft_buffer_at_speed_y}};

  // Vehicle shape.
  std::vector<Vec2d> circle_center_offsets;
  std::vector<double> circle_radiuses;
  const int circle_size =
      trajectory_optimizer_vehicle_model_params.circles_size();
  circle_center_offsets.reserve(circle_size);
  circle_radiuses.reserve(circle_size);
  double max_circle_radius = 0.0;
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    double cos_sin[2];
    fast_math::CosAndSin<7>(circle.angle_to_axis(), cos_sin);
    circle_center_offsets.push_back(Vec2d(circle.dist_to_rac() * cos_sin[0],
                                          circle.dist_to_rac() * cos_sin[1]));
    circle_radiuses.push_back(circle.radius());
    max_circle_radius = std::max(max_circle_radius, circle.radius());
  }

  // Build msd.
  constexpr double kCutoffDistExtraBuffer = 0.1;
  double max_buffer = std::max(hard_layer.buffer_at_speed.y().back(),
                               soft_layer.buffer_at_speed.y().back());
  const double cutoff_distance =
      max_buffer + max_circle_radius + kCutoffDistExtraBuffer;
  MsdProblemWithBuffer msd(std::move(named_segments), cutoff_distance,
                           start_segment_ids);

  // msd debug canvas.
  if (FLAGS_static_boundary_canvas_level >= 1) {
    vis::Canvas* canvas = &vis::vantage::GetCanvasClient()->GetCanvas(
        absl::StrFormat("%s/qtfm_for_%s", base_name, source_name));
    msd.DrawQtfmSegmentMatcher(canvas);
  }

  costs->emplace_back(std::make_unique<MsdStaticBoundaryCostV2<Mfob>>(
      trajectory_steps, std::move(msd), stations_query_helper.get(),
      std::vector<std::string>{SoftNameString, HardNameString},
      std::vector<MsdStaticBoundaryCostV2<Mfob>::Layer>{std::move(soft_layer),
                                                        std::move(hard_layer)},
      std::move(circle_center_offsets), std::move(circle_radiuses),
      /*use_hessian_approximate=*/true,
      /*name=*/absl::StrCat("MsdStaticBoundaryCostV2_", source_name),
      cost_weight_params.msd_static_boundary_cost_weight() * source_scale,
      /*cost_type=*/source_name == "uturn_right_curb"
          ? Cost<Mfob>::CostType::UTURN_RIGHT_CURB_MSD_STATIC_BOUNDARY_V2
          : Cost<Mfob>::CostType::CURB_MSD_STATIC_BOUNDARY_V2));
}

void AddCurbStaticBoundaryCost(
    int horizon, const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::vector<Vec2d>& path_points,
    const CenterLineQueryHelper<Mfob>* center_line_helper,
    CurbCostInfo left_cci, CurbCostInfo right_cci,
    std::vector<StaticBoundaryCost<Mfob>::BreakPoint> left_curb_break_points,
    std::vector<StaticBoundaryCost<Mfob>::BreakPoint> right_curb_break_points,
    const std::vector<double>& cascade_buffers,
    const std::vector<double>& cascade_gains,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  // Get vehicle model info.
  std::vector<double> dists_to_rac;
  std::optional<VehicleCircleModelParamsProto::CircleParams> left_corner =
      std::nullopt;
  std::optional<VehicleCircleModelParamsProto::CircleParams> right_corner =
      std::nullopt;
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    switch (circle.type()) {
      case VehicleCircleModelParamsProto::REAR_AXIS_CENTER:
      case VehicleCircleModelParamsProto::MID_AXIS_CENTER:
      case VehicleCircleModelParamsProto::FRONT_AXIS_CENTER:
        dists_to_rac.push_back(circle.dist_to_rac());
        break;
      case VehicleCircleModelParamsProto::FRONT_LEFT_CORNER:
        left_corner = circle;
        break;
      case VehicleCircleModelParamsProto::FRONT_RIGHT_CORNER:
        right_corner = circle;
        break;
      default:
        break;
    }
  }
  // Add extra buffer.
  for (int i = 0; i < left_cci.dists_to_curb.size(); ++i) {
    left_cci.dists_to_curb[i] -= left_cci.curb_extra_buffer[i];
    right_cci.dists_to_curb[i] -= right_cci.curb_extra_buffer[i];
    if (i + 1 == left_cci.dists_to_curb.size()) continue;
    if (left_curb_break_points[i].has_break_point) {
      for (int j = 0; j < left_curb_break_points[i].alphas.size(); ++j) {
        const double extra_buffer = left_cci.curb_extra_buffer[i] +
                                    (left_cci.curb_extra_buffer[i + 1] -
                                     left_cci.curb_extra_buffer[i]) *
                                        left_curb_break_points[i].alphas[j];
        left_curb_break_points[i].dists[j] -= extra_buffer;
      }
    }
    if (right_curb_break_points[i].has_break_point) {
      for (int j = 0; j < right_curb_break_points[i].alphas.size(); ++j) {
        const double extra_buffer = right_cci.curb_extra_buffer[i] +
                                    (right_cci.curb_extra_buffer[i + 1] -
                                     right_cci.curb_extra_buffer[i]) *
                                        right_curb_break_points[i].alphas[j];
        right_curb_break_points[i].dists[j] -= extra_buffer;
      }
    }
  }
  // Left curb.
  costs->emplace_back(std::make_unique<StaticBoundaryCost<Mfob>>(
      horizon, vehicle_geometry_params, path_points, center_line_helper,
      std::move(left_cci.dists_to_curb), std::move(left_curb_break_points),
      std::move(left_cci.curb_ref_gain),
      /*left=*/true, dists_to_rac, left_corner,
      /*use_qtfm=*/true,
      /*sub_names=*/std::vector<std::string>({"Soft", "Hard"}), cascade_gains,
      cascade_buffers,
      /*using_hessian_approximate=*/false,
      "MfobStaticBoundaryCost: left curb"));
  // Right curb.
  costs->emplace_back(std::make_unique<StaticBoundaryCost<Mfob>>(
      horizon, vehicle_geometry_params, path_points, center_line_helper,
      std::move(right_cci.dists_to_curb), std::move(right_curb_break_points),
      std::move(right_cci.curb_ref_gain),
      /*left=*/false, dists_to_rac, right_corner,
      /*use_qtfm=*/true,
      /*sub_names=*/std::vector<std::string>({"Soft", "Hard"}), cascade_gains,
      cascade_buffers,
      /*using_hessian_approximate=*/false,
      "MfobStaticBoundaryCost: right curb"));
}

inline double ComputeMirrorExtraBuffer(
    const VehicleGeometryParamsProto& veh_geo_params) {
  double mirror_extra_buffer = 0.0;
  if (veh_geo_params.has_left_mirror() && veh_geo_params.has_right_mirror()) {
    const double left_buffer = veh_geo_params.left_mirror().y() +
                               0.5 * veh_geo_params.left_mirror().length() -
                               0.5 * veh_geo_params.width();
    const double right_buffer = -veh_geo_params.right_mirror().y() +
                                0.5 * veh_geo_params.right_mirror().length() -
                                0.5 * veh_geo_params.width();
    mirror_extra_buffer = std::max(0.0, 0.5 * (left_buffer + right_buffer));
  }

  return mirror_extra_buffer;
}

std::vector<PathPoint> ComputeRawCenterLinePathPoints(
    absl::Span<const Vec2d> ref_center_vector,
    const DrivePassage& drive_passage) {
  const int points_num = ref_center_vector.size();
  std::vector<PathPoint> raw_path_points;
  raw_path_points.reserve(points_num);
  for (const auto& pt : ref_center_vector) {
    auto& path_point = raw_path_points.emplace_back();
    path_point.set_x(pt.x());
    path_point.set_y(pt.y());
  }
  for (int i = 0; i + 2 < points_num; ++i) {
    // Evaluate kappa and s, please refer to
    // https://qcraft.feishu.cn/docx/ClSBdEyjtowU14xi6B1cM2QVn3c.
    const Vec2d dxy =
        -0.5 * ref_center_vector[i] + 0.5 * ref_center_vector[i + 2];
    const Vec2d d2xy = ref_center_vector[i] - 2.0 * ref_center_vector[i + 1] +
                       ref_center_vector[i + 2];
    const double tmp = 1.0 / (Sqr(dxy.x()) + Sqr(dxy.y()));
    const double kappa =
        (dxy.x() * d2xy.y() - d2xy.x() * dxy.y()) * tmp * std::sqrt(tmp);
    // Use absolute value to compute max value in a range.
    raw_path_points[i + 1].set_kappa(std::abs(kappa));
    raw_path_points[i + 1].set_s(
        raw_path_points[i].s() +
        ref_center_vector[i + 1].DistanceTo(ref_center_vector[i]));
  }
  QCHECK_GT(points_num, 1);
  raw_path_points[0].set_kappa(raw_path_points[1].kappa());
  raw_path_points.back().set_s(raw_path_points[points_num - 2].s() +
                               ref_center_vector[points_num - 1].DistanceTo(
                                   ref_center_vector[points_num - 2]));
  // Set curvature to the max value in a range.
  constexpr double kMaxCurvatureLookBackDist = 5.0;       // m.
  constexpr double kLookBackDistExtensionInUTurn = 15.0;  // m.
  constexpr double kUTurnCheckMaxDist = 40.0;             // m.
  double look_back_dist = kMaxCurvatureLookBackDist;
  for (const auto& station : drive_passage.stations()) {
    if (station.direction() == mapping::LaneProto::UTURN) {
      look_back_dist += kLookBackDistExtensionInUTurn;
      break;
    }
    if (station.accumulated_s() > kUTurnCheckMaxDist) {
      break;
    }
  }
  for (int i = points_num - 1; i >= 0; --i) {
    double max_kappa = raw_path_points[i].kappa();
    for (int j = i; j >= 0; --j) {
      if (raw_path_points[i].s() - raw_path_points[j].s() > look_back_dist) {
        break;
      }
      max_kappa = std::max(max_kappa, raw_path_points[j].kappa());
    }
    raw_path_points[i].set_kappa(max_kappa);
  }

  return raw_path_points;
}

void GetBreakPointsAndAddStaticBoundaryCurbCost(
    int trajectory_steps, std::string_view base_name,
    const DrivePassage& drive_passage, const PlannerSemanticMapManager& psmm,
    const PathTimeCorridor& path_time_corridor,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    double mirror_extra_buffer, const std::vector<double>& cascade_gains,
    double soft_curb_clearance, double hard_curb_clearance,
    const std::function<double(const Vec2d&)>& query_curb_extra_buffer,
    CurbCostInfo left_cci, CurbCostInfo right_cci,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  const std::vector<Vec2d>& station_points = stations_query_helper->points();
  const int curb_size = left_cci.dists_to_curb.size();
  std::vector<StaticBoundaryCost<Mfob>::BreakPoint> left_curb_break_points(
      curb_size - 1);
  std::vector<StaticBoundaryCost<Mfob>::BreakPoint> right_curb_break_points(
      curb_size - 1);

  // Compute extra buffer for static boundary cost.
  ComputePathTimeCorridorAndCurvatureCurbBuffer(
      veh_geo_params, mirror_extra_buffer, drive_passage, path_time_corridor,
      soft_curb_clearance, query_curb_extra_buffer, left_cci.dists_to_curb,
      right_cci.dists_to_curb, &left_cci.curb_extra_buffer,
      &right_cci.curb_extra_buffer);
  // Collect break points if not in vision map.
  if (!psmm.IsOnVisionMap()) {
    const FrenetFrame& path = stations_query_helper->path();
    CollectStaticBoundaryBreakPoints(
        psmm, drive_passage, path, left_cci.dists_to_curb,
        right_cci.dists_to_curb, &left_curb_break_points,
        &right_curb_break_points);
  }
  if (FLAGS_static_boundary_canvas_level >= 1) {
    CanvasDrawStaticBoundary(
        absl::StrFormat("%s/curb_for_static_boundary_cost", base_name),
        vis::Color(1.0, 0.7, 0.7), drive_passage, left_cci,
        left_curb_break_points, right_cci, right_curb_break_points);
  }

  AddCurbStaticBoundaryCost(
      trajectory_steps, veh_geo_params,
      trajectory_optimizer_vehicle_model_params, station_points,
      stations_query_helper.get(), std::move(left_cci), std::move(right_cci),
      std::move(left_curb_break_points), std::move(right_curb_break_points),
      /*cascade_buffers=*/{soft_curb_clearance, hard_curb_clearance},
      cascade_gains, costs);
}

void GetCurbSegmentsAndAddMsdCurbCost(
    int trajectory_steps, std::string_view base_name,
    const TrajectoryPoint& plan_start_point, const DrivePassage& drive_passage,
    const PlannerSemanticMapManager& psmm,
    const PathSlBoundary& path_sl_boundary,
    const PathTimeCorridor& path_time_corridor,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    CurbCostType curb_cost_type, double mirror_extra_buffer,
    const std::optional<double>& uturn_middle_s,
    const std::vector<double>& cascade_gains, double soft_curb_clearance,
    double hard_curb_clearance,
    const std::function<double(const Vec2d&)>& query_curb_extra_buffer,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  std::vector<int> curb_start_segment_ids;
  std::vector<MsdProblemWithBuffer::SegmentType> curb_segments;
  double clamped_max_curb_buffer = std::numeric_limits<double>::infinity();

  CollectCurbSegmentsAroundDrivePassage(
      trajectory_optimizer_vehicle_model_params.consider_mirrors_by_default(),
      veh_geo_params, psmm, drive_passage, path_sl_boundary, path_time_corridor,
      curb_cost_type == CurbCostType::kMsdV2
          ? cost_weight_params.speed_rel_soft_curb_clearance_plf().y(0)
          : soft_curb_clearance,
      query_curb_extra_buffer, mirror_extra_buffer, uturn_middle_s,
      plan_start_point, &clamped_max_curb_buffer, &curb_segments,
      &curb_start_segment_ids);
  if (FLAGS_static_boundary_canvas_level >= 1) {
    CanvasDrawSegments(
        absl::StrFormat("%s/curb_segments_for_static_boundary_cost", base_name),
        vis::Color(1.0, 0.7, 0.7), curb_segments);
  }

  if (!curb_segments.empty() && curb_cost_type == CurbCostType::kMsdV1) {
    AddMsdStaticBoundaryCost(
        trajectory_steps, base_name, /*source_name=*/"curb", cost_weight_params,
        veh_geo_params, trajectory_optimizer_vehicle_model_params,
        curb_start_segment_ids, std::move(curb_segments),
        /*sub_names=*/{"CurbSoft", "CurbHard"},
        /*cascade_buffers=*/
        {std::min(clamped_max_curb_buffer, soft_curb_clearance),
         hard_curb_clearance},
        cascade_gains, stations_query_helper, costs);
  } else if (!curb_segments.empty() && curb_cost_type == CurbCostType::kMsdV2) {
    AddMsdStaticBoundaryCostV2(
        trajectory_steps, base_name, /*source_name=*/"curb",
        /*source_scale=*/kCurbGain, clamped_max_curb_buffer, cost_weight_params,
        trajectory_optimizer_vehicle_model_params, curb_start_segment_ids,
        std::move(curb_segments), stations_query_helper, costs);
  }
}

}  // namespace

// NOLINTNEXTLINE
void AddStaticBoundaryCosts(
    int trajectory_steps, std::string_view base_name,
    bool enable_three_point_turn, const TrajectoryPoint& plan_start_point,
    const DrivePassage& drive_passage, const PlannerSemanticMapManager& psmm,
    const PathSlBoundary& path_sl_boundary,
    const PathTimeCorridor& path_time_corridor,
    const std::vector<double>& left_l_boundary_for_nudge,
    const std::vector<double>& right_l_boundary_for_nudge,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    CurbCostType curb_cost_type, std::optional<double>* extra_curb_buffer_opt,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  FUNC_QTRACE();
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(drive_passage.stations().size(), 0);
  QCHECK_NOTNULL(stations_query_helper.get());

  // Compute mirror extra buffer for curb.
  const double mirror_extra_buffer = ComputeMirrorExtraBuffer(veh_geo_params);

  // Get curb and path boundary info.
  const std::vector<Vec2d>& station_points = stations_query_helper->points();
  CurbCostInfo left_cci, right_cci;
  PathBoundaryCostInfo left_pbci, right_pbci;
  std::vector<double> reference_center_l_offsets;
  std::optional<double> uturn_middle_s = std::nullopt;
  if (FLAGS_trajectory_optimizer_ignore_u_turn_right_boundary &&
      enable_three_point_turn) {
    GeneratePathBoundariesWithRightBoundaryAfterUTurnIgnored(
        veh_geo_params, mirror_extra_buffer, drive_passage, path_sl_boundary,
        &left_cci, &right_cci, &left_pbci, &right_pbci,
        &reference_center_l_offsets, &uturn_middle_s);
  } else {
    GeneratePathBoundaries(veh_geo_params, mirror_extra_buffer, drive_passage,
                           path_sl_boundary, &left_cci, &right_cci, &left_pbci,
                           &right_pbci, &reference_center_l_offsets);
  }

  // Build curb extra curvature buffer querier.
  const auto& ref_center_vector = path_sl_boundary.reference_center_xy_vector();
  auto raw_path_points =
      ComputeRawCenterLinePathPoints(ref_center_vector, drive_passage);

  // Build querier.
  const DiscretizedPath ref_center_path(std::move(raw_path_points));
  const auto ref_center_frenet_status = BuildKdTreeFrenetFrame(
      ref_center_vector, /*down_sample_raw_points = */ false);
  QCHECK(ref_center_frenet_status.ok());
  const auto kappa_buffer_plf = PiecewiseLinearFunctionFromProto(
      cost_weight_params.kappa_rel_curb_clearance_buffer_plf());

  const auto query_curb_extra_buffer =
      [&ref_center_path, &ref_center_frenet_status,
       &kappa_buffer_plf](const Vec2d& pos) -> double {
    const auto sl = ref_center_frenet_status->XYToSL(pos);
    return kappa_buffer_plf(std::abs(ref_center_path.Evaluate(sl.s).kappa()));
  };

  // Set extra curb buffer at plan start point.
  *extra_curb_buffer_opt = query_curb_extra_buffer(plan_start_point.pos());

  // Curb boundary cost.
  const auto av_sl_pos_or =
      drive_passage.QueryFrenetCoordinateAt(plan_start_point.pos());
  if (!av_sl_pos_or.ok()) return;
  const auto av_curb_pair_or =
      drive_passage.QueryCurbOffsetAtS(av_sl_pos_or->s);
  if (!av_curb_pair_or.ok()) return;

  const PiecewiseLinearFunction<double> speed_rel_soft_curb_clearance_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.speed_rel_soft_curb_clearance_plf());
  const PiecewiseLinearFunction<double> speed_rel_hard_curb_clearance_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.speed_rel_hard_curb_clearance_plf());

  const std::vector<double> cascade_gains = {
      cost_weight_params.static_boundary_soft_cost_weight(),
      cost_weight_params.static_boundary_hard_cost_weight()};
  // Add curb.
  const double soft_curb_clearance =
      speed_rel_soft_curb_clearance_plf(plan_start_point.v());
  const double hard_curb_clearance =
      speed_rel_hard_curb_clearance_plf(plan_start_point.v());

  // Compute necessary info and add curb costs according to curb cost type.
  switch (curb_cost_type) {
    case CurbCostType::kStaticBoundary:
      GetBreakPointsAndAddStaticBoundaryCurbCost(
          trajectory_steps, base_name, drive_passage, psmm, path_time_corridor,
          veh_geo_params, trajectory_optimizer_vehicle_model_params,
          stations_query_helper, mirror_extra_buffer, cascade_gains,
          soft_curb_clearance, hard_curb_clearance, query_curb_extra_buffer,
          std::move(left_cci), std::move(right_cci), costs);
      break;
    case CurbCostType::kMsdV1:
    case CurbCostType::kMsdV2:
      GetCurbSegmentsAndAddMsdCurbCost(
          trajectory_steps, base_name, plan_start_point, drive_passage, psmm,
          path_sl_boundary, path_time_corridor, cost_weight_params,
          veh_geo_params, trajectory_optimizer_vehicle_model_params,
          stations_query_helper, curb_cost_type, mirror_extra_buffer,
          uturn_middle_s, cascade_gains, soft_curb_clearance,
          hard_curb_clearance, query_curb_extra_buffer, costs);
      break;
  }

  // Path boundary costs.
  const std::vector<double> rear_gain = {
      cost_weight_params.path_boundary_cost_params()
          .rear_path_boundary_cost_weight(),
      cost_weight_params.target_path_boundary_cost_params()
          .rear_path_boundary_cost_weight()};
  const std::vector<double> front_gain = {
      cost_weight_params.path_boundary_cost_params()
          .front_path_boundary_cost_weight(),
      cost_weight_params.target_path_boundary_cost_params()
          .front_path_boundary_cost_weight()};
  const std::vector<double> buffers_min = {
      cost_weight_params.path_boundary_cost_params().buffer_min(),
      cost_weight_params.target_path_boundary_cost_params().buffer_min()};
  const std::vector<double> rear_buffers_max = {
      cost_weight_params.path_boundary_cost_params().rear_buffer_max(),
      cost_weight_params.target_path_boundary_cost_params().rear_buffer_max()};
  const std::vector<double> front_buffers_max = {
      cost_weight_params.path_boundary_cost_params().front_buffer_max(),
      cost_weight_params.target_path_boundary_cost_params().front_buffer_max()};
  std::vector<double> clamped_buffer_offset = {
      cost_weight_params.path_boundary_cost_params().clamped_buffer_offset(),
      cost_weight_params.target_path_boundary_cost_params()
          .clamped_buffer_offset()};

  std::vector<double> dist_to_rac;
  int rac_index = -1;
  for (int i = 0;
       i < trajectory_optimizer_vehicle_model_params.circles().size(); ++i) {
    const auto& circle = trajectory_optimizer_vehicle_model_params.circles()[i];
    if (circle.type() == VehicleCircleModelParamsProto::REAR_AXIS_CENTER ||
        circle.type() == VehicleCircleModelParamsProto::MID_AXIS_CENTER ||
        circle.type() == VehicleCircleModelParamsProto::FRONT_AXIS_CENTER) {
      dist_to_rac.push_back(circle.dist_to_rac());
    }
    if (circle.type() == VehicleCircleModelParamsProto::REAR_AXIS_CENTER) {
      rac_index = i;
    }
  }

  for (const bool left : {false, true}) {
    std::vector<std::vector<double>> dists_to_path_boundary;
    dists_to_path_boundary.reserve(2);
    std::vector<double> cascade_gains;
    cascade_gains.reserve(2);
    if (left) {
      dists_to_path_boundary.push_back(left_pbci.dists_to_path_boundary);
      dists_to_path_boundary.push_back(left_pbci.dists_to_target_path_boundary);
      cascade_gains.push_back(cost_weight_params.path_boundary_cost_params()
                                  .left_path_boundary_cost_weight());
      cascade_gains.push_back(
          cost_weight_params.target_path_boundary_cost_params()
              .left_path_boundary_cost_weight());
    } else {
      dists_to_path_boundary.push_back(right_pbci.dists_to_path_boundary);
      dists_to_path_boundary.push_back(
          right_pbci.dists_to_target_path_boundary);
      cascade_gains.push_back(cost_weight_params.path_boundary_cost_params()
                                  .right_path_boundary_cost_weight());
      cascade_gains.push_back(
          cost_weight_params.target_path_boundary_cost_params()
              .right_path_boundary_cost_weight());
    }
    std::vector<std::vector<double>> ref_gains;
    ref_gains.reserve(cascade_gains.size());
    std::vector<double> outer_path_boundary_gains(station_points.size(), 1.0);
    std::vector<double> inner_path_boundary_gains(station_points.size(), 1.0);

    ref_gains.push_back(std::move(outer_path_boundary_gains));
    ref_gains.push_back(std::move(inner_path_boundary_gains));

    std::vector<std::vector<double>> l_dists;
    std::vector<double> outer_l_dists(station_points.size(), 0.0);
    l_dists.push_back(std::move(outer_l_dists));
    l_dists.push_back(left ? left_l_boundary_for_nudge
                           : right_l_boundary_for_nudge);

    // TODO(runbing): Consider decay weight based on av dist to boundary.
    auto& gains = ref_gains.back();
    const double decay_gain_look_ahead_s =
        plan_start_point.v() *
            cost_weight_params.target_path_boundary_cost_params()
                .decay_gain_look_ahead_time() +
        veh_geo_params.front_edge_to_center();
    const auto& stations = drive_passage.stations();
    const int s_index =
        std::distance(stations.begin(),
                      std::upper_bound(stations.begin(), stations.end(),
                                       decay_gain_look_ahead_s,
                                       [](double val, const Station& station) {
                                         return val < station.accumulated_s();
                                       }));
    const double decay_gain =
        cost_weight_params.target_path_boundary_cost_params().decay_gain();

    for (int i = 0; i < gains.size(); ++i) {
      if (i > s_index) break;
      gains[i] = std::min(gains[i], decay_gain);
    }
    std::vector<std::string> sub_names = {"Outer", "Inner"};
    costs->emplace_back(std::make_unique<MfobPathBoundaryCost<Mfob>>(
        trajectory_steps, veh_geo_params, station_points,
        stations_query_helper.get(), reference_center_l_offsets,
        dists_to_path_boundary, l_dists, left,
        /*using_hessian_approximate=*/true, dist_to_rac, rac_index,
        std::move(ref_gains), std::move(sub_names), /*use_qtfm=*/true,
        buffers_min, rear_buffers_max, front_buffers_max, clamped_buffer_offset,
        cascade_gains, rear_gain, front_gain,
        absl::StrCat("PathBoundaryCost: ",
                     left ? "left path boundary" : "right path boundary"),
        /*scale=*/1.0,
        /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  }
}

void AddSolidWhiteLineCost(
    int trajectory_steps, std::string_view base_name,
    const std::vector<TrajectoryPoint>& solver_init_traj,
    const ConstraintManager& constraint_manager,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_NOTNULL(costs);

  // Add solid line within path boundary.
  constexpr double kSolidLineExtendDistance = 0.5;  // m.
  constexpr double kSolidLineBuffer = 0.2;          // m.
  std::vector<int> solid_line_start_segment_ids;
  std::vector<MsdProblemWithBuffer::SegmentType> solid_line_segments;
  CollectExtendSolidLinesWithinPathBoundary(
      veh_geo_params, trajectory_optimizer_vehicle_model_params,
      constraint_manager, solver_init_traj, kSolidLineBuffer,
      kSolidLineExtendDistance, &solid_line_start_segment_ids,
      &solid_line_segments);
  if (FLAGS_static_boundary_canvas_level >= 1) {
    CanvasDrawSegments(
        absl::StrFormat("%s/solid_line_segments_for_static_boundary_cost",
                        base_name),
        vis::Color(1.0, 0.7, 0.8), solid_line_segments);
  }
  if (!solid_line_segments.empty()) {
    AddMsdStaticBoundaryCost(
        trajectory_steps, base_name, /*source_name=*/"solid_line",
        cost_weight_params, veh_geo_params,
        trajectory_optimizer_vehicle_model_params, solid_line_start_segment_ids,
        std::move(solid_line_segments),
        /*sub_names=*/{"SolidLine"},
        /*cascade_buffers=*/{kSolidLineBuffer},
        /*cascade_gains=*/
        {cost_weight_params.static_boundary_solid_line_cost_weight()},
        stations_query_helper, costs);
  }
}

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft
