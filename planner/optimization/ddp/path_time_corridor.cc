#include "onboard/planner/optimization/ddp/path_time_corridor.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DEFINE_bool(traj_opt_draw_nudge_buffer, false,
            "If send nudge buffer manager results to canvas.");

namespace qcraft {
namespace planner {
namespace optimizer {

namespace {
using BoundaryInfo = PathTimeCorridor::BoundaryInfo;
void DrawPathTimeCorridorBoundary(
    std::string_view base_name, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary,
    const std::vector<std::vector<BoundaryInfo>>& left_boundary,
    const std::vector<std::vector<BoundaryInfo>>& right_boundary,
    const std::vector<double>& time_points) {
  for (int i = 0; i < time_points.size(); ++i) {
    const double t = time_points[i];
    auto& canvas_boundary = vis::vantage::GetCanvasClient()->GetCanvas(
        absl::StrFormat("%s/nudge_buffer_manager/boundary_%f", base_name, t));
    auto& canvas_object = vis::vantage::GetCanvasClient()->GetCanvas(
        absl::StrFormat("%s/nudge_buffer_manager/object_%f", base_name, t));
    const auto reference_center_xy_vector =
        path_sl_boundary.reference_center_xy_vector();
    const auto& stations = drive_passage.stations();
    const auto& left_boundary_i = left_boundary[i];
    const auto& right_boundary_i = right_boundary[i];
    for (int i = 0; i < path_sl_boundary.size(); ++i) {
      const auto& station = stations[StationIndex(i)];
      canvas_boundary.DrawPoint(Vec3d(reference_center_xy_vector[i]),
                                vis::Color::kLightBlue, 10);
      canvas_boundary.DrawPoint(
          Vec3d(station.lat_point(left_boundary_i[i].l_boundary)),
          vis::Color(0.7, 0.7, 0.7, 0.1), 8);
      canvas_boundary.DrawPoint(
          Vec3d(station.lat_point(right_boundary_i[i].l_boundary)),
          vis::Color(0.7, 0.7, 0.7, 0.1), 8);
      canvas_boundary.DrawLine(
          Vec3d(station.lat_point(left_boundary_i[i].l_boundary)),
          Vec3d(station.lat_point(right_boundary_i[i].l_boundary)),
          vis::Color(0.8, 0.8, 0.8, 0.2), 1);
      // Draw object boundary.
      canvas_object.DrawPoint(Vec3d(reference_center_xy_vector[i]),
                              vis::Color::kLightBlue, 10);
      if (left_boundary_i[i].object_ptr != nullptr) {
        canvas_object.DrawPoint(
            Vec3d(station.lat_point(left_boundary_i[i].l_object)),
            vis::Color::kLightRed, 8);
        canvas_object.DrawText(
            std::string(left_boundary_i[i].object_ptr->traj_id()),
            Vec3d(station.lat_point(left_boundary_i[i].l_object)),
            station.tangent().FastAngle(), 0.1, vis::Color::kLightRed);
      }
      if (right_boundary_i[i].object_ptr != nullptr) {
        canvas_object.DrawPoint(
            Vec3d(station.lat_point(right_boundary_i[i].l_object)),
            vis::Color::kLightRed, 8);
        canvas_object.DrawText(
            std::string(right_boundary_i[i].object_ptr->traj_id()),
            Vec3d(station.lat_point(right_boundary_i[i].l_object)),
            station.tangent().FastAngle(), 0.1, vis::Color::kLightRed);
      }
    }
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

void SampleBoundaryForObjectSegment(bool left, double init_start_s,
                                    double init_end_s, double sample_step,
                                    const Segment2d& line,
                                    const FrenetFrame& dp_frenet,
                                    const SpacetimeObjectTrajectory& traj,
                                    std::vector<BoundaryInfo>* boundary) {
  using InnerComparator = std::function<bool(double, double)>;

  const int indices = std::ceil(line.length() / sample_step);
  const double actual_step = line.length() / static_cast<double>(indices);
  const int max_idx = static_cast<int>(boundary->size() - 1);
  InnerComparator inner_comparator;
  if (left) {
    inner_comparator = std::less_equal<double>();
  } else {
    inner_comparator = std::greater_equal<double>();
  }

  const auto update_boundary_info_if_inner =
      [&traj](double l, const InnerComparator& inner_comparator,
              BoundaryInfo* boundary_info) {
        if (inner_comparator(l, boundary_info->l_boundary)) {
          boundary_info->type = BoundaryInfo::Type::kObject;
        }
        if (inner_comparator(l, boundary_info->l_object)) {
          boundary_info->l_object = l;
          boundary_info->object_ptr = &traj;
        }
      };

  const auto step_vec = actual_step * line.unit_direction();
  for (int i = 0; i <= indices; ++i) {
    const Vec2d pt = line.start() + static_cast<double>(i) * step_vec;
    FrenetCoordinate sl;
    Vec2d normal;
    std::pair<int, int> index_pair;
    double alpha;
    dp_frenet.XYToSL(pt, &sl, &normal, &index_pair, &alpha);
    if (sl.s > init_end_s) continue;
    if (sl.s < init_start_s) continue;
    const int cur_idx = std::clamp(index_pair.first, 0, max_idx);
    const int next_idx = std::clamp(index_pair.second, 0, max_idx);
    const double l = sl.l;
    update_boundary_info_if_inner(l, inner_comparator, &(*boundary)[cur_idx]);
    update_boundary_info_if_inner(l, inner_comparator, &(*boundary)[next_idx]);
  }
}

void SampleBoundaryForContour(double init_start_s, double init_end_s,
                              const Vec2d& contour_pos,
                              const Polygon2d& contour,
                              const FrenetFrame& init_frenet,
                              const FrenetFrame& dp_frenet,
                              const SpacetimeObjectTrajectory& traj,
                              std::vector<BoundaryInfo>* left_boundary,
                              std::vector<BoundaryInfo>* right_boundary) {
  constexpr double kSegmentSampleStep = 1.0;  // m
  // Get nudge direction for object.
  FrenetCoordinate frenet_center;
  Vec2d center_normal;
  init_frenet.XYToSL(contour_pos, &frenet_center, &center_normal);
  // Find half contour based on nudge direction.
  Vec2d front, back;
  int front_index, back_index;
  contour.ExtremePoints(-center_normal.Perp(), &back_index, &front_index, &back,
                        &front);
  const bool left = frenet_center.l > 0.0;
  const auto& contour_lines = contour.line_segments();
  const auto [begin_index, end_index] =
      left ? std::make_pair(back_index, front_index)
           : std::make_pair(front_index, back_index);
  auto* boundary = left ? left_boundary : right_boundary;
  for (int i = begin_index; i != end_index; i = contour.Next(i)) {
    SampleBoundaryForObjectSegment(left, init_start_s, init_end_s,
                                   kSegmentSampleStep, contour_lines[i],
                                   dp_frenet, traj, boundary);
  }
}

std::vector<SpacetimeObjectState> SampleObjectStates(
    const std::vector<int>& time_indices,
    absl::Span<const SpacetimeObjectState> states) {
  // Make sure time_indices is in ascending order.
  std::vector<SpacetimeObjectState> sampled_states;
  sampled_states.reserve(time_indices.size());
  for (const auto k : time_indices) {
    if (k < states.size()) {
      sampled_states.push_back(states[k]);
    } else {
      break;
    }
  }
  return sampled_states;
}
}  // namespace

PathTimeCorridor::PathTimeCorridor(
    const PathSlBoundary* path_sl_boundary,
    std::vector<std::vector<PathTimeCorridor::BoundaryInfo>> left_boundary,
    std::vector<std::vector<PathTimeCorridor::BoundaryInfo>> right_boundary,
    std::vector<double> time_points)
    : path_sl_boundary_(path_sl_boundary),
      left_boundary_(std::move(left_boundary)),
      right_boundary_(std::move(right_boundary)),
      time_points_(std::move(time_points)) {
  QCHECK_NOTNULL(path_sl_boundary_);
  QCHECK_EQ(time_points_.size(), left_boundary_.size());
  QCHECK_EQ(time_points_.size(), right_boundary_.size());
  for (int i = 0; i < time_points_.size(); ++i) {
    QCHECK_EQ(path_sl_boundary->size(), left_boundary_[i].size());
    QCHECK_EQ(path_sl_boundary->size(), right_boundary_[i].size());
  }
}

std::pair<const PathTimeCorridor::BoundaryInfo*,
          const PathTimeCorridor::BoundaryInfo*>
PathTimeCorridor::QueryBoundaryL(double s_start, double s_end, double t,
                                 int* min_lane_width_idx) const {
  const auto& s_vec = path_sl_boundary_->s_vector();
  const auto it_start = std::upper_bound(s_vec.begin(), s_vec.end(), s_start);
  const auto it_end = std::upper_bound(s_vec.begin(), s_vec.end(), s_end);

  const int idx_start =
      std::clamp(static_cast<int>(it_start - s_vec.begin()) - 1, 0,
                 path_sl_boundary_->size() - 1);
  const int idx_end = std::clamp(static_cast<int>(it_end - s_vec.begin()), 0,
                                 path_sl_boundary_->size() - 1);

  int time_idx = time_points_.size() - 1;
  for (int i = 0; i < time_points_.size() - 1; ++i) {
    if (time_points_[i] <= t && time_points_[i + 1] > t) {
      time_idx = i;
    }
  }
  const auto& left_boundary = left_boundary_[time_idx];
  const auto& right_boundary = right_boundary_[time_idx];
  double min_width = std::numeric_limits<double>::infinity();
  int min_idx = idx_start;
  for (int i = idx_start; i <= idx_end; ++i) {
    const auto& left = left_boundary[i];
    const auto& right = right_boundary[i];
    const double lane_width =
        std::min(left.l_boundary, std::min(left.l_curb, left.l_object)) -
        std::max(right.l_boundary, std::max(right.l_curb, right.l_object));
    if (lane_width < min_width) {
      min_width = lane_width;
      min_idx = i;
    }
  }

  if (min_lane_width_idx != nullptr) {
    (*min_lane_width_idx) = min_idx;
  }
  return std::make_pair(&right_boundary[min_idx], &left_boundary[min_idx]);
}

std::pair<PathTimeCorridor::BoundaryInfo, PathTimeCorridor::BoundaryInfo>
PathTimeCorridor::QueryNarrowestBoundaryAllTypes(double s_start, double s_end,
                                                 double t) const {
  const auto& s_vec = path_sl_boundary_->s_vector();
  const auto it_start = std::upper_bound(s_vec.begin(), s_vec.end(), s_start);
  const auto it_end = std::upper_bound(s_vec.begin(), s_vec.end(), s_end);
  const int idx_start =
      std::clamp(static_cast<int>(it_start - s_vec.begin()) - 1, 0,
                 path_sl_boundary_->size() - 1);
  const int idx_end = std::clamp(static_cast<int>(it_end - s_vec.begin()), 0,
                                 path_sl_boundary_->size() - 1);

  int time_idx = time_points_.size() - 1;
  for (int i = 0; i < time_points_.size() - 1; ++i) {
    if (time_points_[i] <= t && time_points_[i + 1] > t) {
      time_idx = i;
    }
  }
  const auto& left_boundary = left_boundary_[time_idx];
  const auto& right_boundary = right_boundary_[time_idx];

  const auto update_narrowest_boundary_info =
      [](const BoundaryInfo& boundary_info,
         const std::function<bool(double, double)>& inner_comparator,
         BoundaryInfo* narrowest_boundary_info) {
        if (inner_comparator(boundary_info.l_boundary,
                             narrowest_boundary_info->l_boundary)) {
          narrowest_boundary_info->l_boundary = boundary_info.l_boundary;
        }
        if (inner_comparator(boundary_info.l_curb,
                             narrowest_boundary_info->l_curb)) {
          narrowest_boundary_info->l_curb = boundary_info.l_curb;
        }
        if (inner_comparator(boundary_info.l_object,
                             narrowest_boundary_info->l_object)) {
          narrowest_boundary_info->l_object = boundary_info.l_object;
          narrowest_boundary_info->object_ptr = boundary_info.object_ptr;
        }
      };

  BoundaryInfo left_boundary_info =
      BoundaryInfo::CreateDefaultLeftBoundaryInfo();
  BoundaryInfo right_boundary_info =
      BoundaryInfo::CreateDefaultRightBoundaryInfo();
  for (int i = idx_start; i <= idx_end; ++i) {
    update_narrowest_boundary_info(left_boundary[i], std::less<double>(),
                                   &left_boundary_info);
    update_narrowest_boundary_info(right_boundary[i], std::greater<double>(),
                                   &right_boundary_info);
  }
  return std::make_pair(right_boundary_info, left_boundary_info);
}

absl::StatusOr<PathTimeCorridor> BuildPathTimeCorridor(
    std::string_view base_name, const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const FrenetFrame& init_traj_frenet_frame,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& veh_geo_params) {
  using BoundaryInfo = PathTimeCorridor::BoundaryInfo;
  std::vector<BoundaryInfo> left_boundary_static;
  std::vector<BoundaryInfo> right_boundary_static;
  left_boundary_static.reserve(path_sl_boundary.size());
  right_boundary_static.reserve(path_sl_boundary.size());
  const auto& reference_center_l_vector =
      path_sl_boundary.reference_center_l_vector();

  int idx = 0;
  for (const auto& station : drive_passage.stations()) {
    if (idx >= path_sl_boundary.size()) break;
    const auto boundary =
        station.QueryEnclosingLaneBoundariesAt(reference_center_l_vector[idx]);
    if (!boundary.ok()) break;
    const auto& boundary_vec = station.boundaries();
    // Update right boundary info.
    auto& right_boundary = right_boundary_static.emplace_back(
        BoundaryInfo::CreateDefaultRightBoundaryInfo());
    if (boundary->right.has_value() &&
        BoundaryInfo::GetType(boundary->right->type) ==
            BoundaryInfo::Type::kLaneBoundary) {
      right_boundary.l_boundary = boundary->right->lat_offset;
      right_boundary.type = BoundaryInfo::Type::kLaneBoundary;
    }
    right_boundary.l_curb = boundary_vec.front().lat_offset;
    if (right_boundary.l_curb > right_boundary.l_boundary) {
      right_boundary.type = BoundaryInfo::Type::kCurb;
    }
    // Update left boundary info.
    auto& left_boundary = left_boundary_static.emplace_back(
        BoundaryInfo::CreateDefaultLeftBoundaryInfo());
    if (boundary->left.has_value() &&
        BoundaryInfo::GetType(boundary->left->type) ==
            BoundaryInfo::Type::kLaneBoundary) {
      left_boundary.l_boundary = boundary->left->lat_offset;
      left_boundary.type = BoundaryInfo::Type::kLaneBoundary;
    }
    left_boundary.l_curb = boundary_vec.back().lat_offset;
    if (left_boundary.l_curb < left_boundary.l_boundary) {
      left_boundary.type = BoundaryInfo::Type::kCurb;
    }
    idx++;
  }

  // Use frenet frame in drive passage to query point.
  const auto& dp_frenet_frame = *drive_passage.frenet_frame();

  const auto init_end_frenet_dp =
      dp_frenet_frame.XYToSL(init_traj.back().pos());
  const double init_end_s =
      init_end_frenet_dp.s + veh_geo_params.front_edge_to_center();

  const auto init_start_frenet_dp =
      dp_frenet_frame.XYToSL(init_traj.front().pos());
  const double init_start_s =
      init_start_frenet_dp.s - veh_geo_params.back_edge_to_center();

  // Firstly, Sample boundary for static or stationary object.
  const auto& spacetime_trajs = st_planner_object_traj.trajectories;
  const auto& trajectory_infos = st_planner_object_traj.trajectory_infos;
  const int num_trajs = spacetime_trajs.size();
  for (const auto& traj : spacetime_trajs) {
    if ((IsStaticObjectType(traj.object_type()) || traj.is_stationary()) &&
        leading_trajs.find(std::string(traj.traj_id())) ==
            leading_trajs.end()) {
      SampleBoundaryForContour(init_start_s, init_end_s, traj.pose().pos(),
                               traj.contour(), init_traj_frenet_frame,
                               dp_frenet_frame, traj, &left_boundary_static,
                               &right_boundary_static);
    }
  }

  // Secondly, Sample boundary for dynamic object.
  std::vector<double> time_points = {0.0, 1.0, 2.0, 3.0};
  std::vector<std::vector<BoundaryInfo>> left_boundary;
  std::vector<std::vector<BoundaryInfo>> right_boundary;

  left_boundary.resize(time_points.size(), left_boundary_static);
  right_boundary.resize(time_points.size(), right_boundary_static);

  std::vector<int> time_indices;
  time_indices.reserve(time_points.size());
  for (const auto& time_point : time_points) {
    time_indices.push_back(std::round(time_point / kTrajectoryTimeStep));
  }

  for (int i = 0; i < num_trajs; ++i) {
    const auto& traj = spacetime_trajs[i];
    const auto& trajectory_info = trajectory_infos[i];
    if (!IsStaticObjectType(traj.object_type()) && !traj.is_stationary() &&
        leading_trajs.find(std::string(traj.traj_id())) ==
            leading_trajs.end() &&
        (trajectory_info.reason ==
         SpacetimePlannerObjectTrajectoryReason::SIDE)) {
      const auto states = SampleObjectStates(time_indices, traj.states());
      for (int k = 0; k < states.size(); ++k) {
        SampleBoundaryForContour(init_start_s, init_end_s,
                                 states[k].traj_point->pos(), states[k].contour,
                                 init_traj_frenet_frame, dp_frenet_frame, traj,
                                 &left_boundary[k], &right_boundary[k]);
      }
    }
  }

  if (FLAGS_traj_opt_draw_nudge_buffer) {
    DrawPathTimeCorridorBoundary(base_name, drive_passage, path_sl_boundary,
                                 left_boundary, right_boundary, time_points);
  }

  return PathTimeCorridor(&path_sl_boundary, std::move(left_boundary),
                          std::move(right_boundary), std::move(time_points));
}

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft
