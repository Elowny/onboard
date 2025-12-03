#include "onboard/planner/util/planner_semantic_map_util.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
constexpr double kEpsilon = 1e-6;
bool GetPointToLaneSignedDistanceAtLevel(
    mapping::LevelId /*level_id*/, const Vec2d& smooth_coord,
    const PlannerSemanticMapManager& semantic_map_manager,
    const mapping::ElementId& lane_id, double* const signed_dist,
    double* const fraction, Vec2d* const point, Segment2d* const segment) {
  Segment2d tmp_segment;
  Vec2d tmp_point;
  double tmp_fraction;
  // TODO(lidong): Get accurate object level.
  if (!semantic_map_manager.GetLaneProjection(smooth_coord, lane_id,
                                              &tmp_fraction, &tmp_point,
                                              signed_dist, &tmp_segment)) {
    return false;
  }
  if (tmp_segment.length() < kEpsilon) {
    QLOG(WARNING) << absl::StrFormat(
        "The semgnet on lane %d is too short to have a reliable estimate",
        lane_id);
    return false;
  }
  *signed_dist = tmp_segment.SignedDistanceTo(smooth_coord);
  if (fraction) *fraction = tmp_fraction;
  if (point) *point = tmp_point;
  if (segment) *segment = tmp_segment;
  return true;
}

bool IsSolidBoundaryType(mapping::LaneBoundaryProto::Type type) {
  switch (type) {
    case mapping::LaneBoundaryProto::UNKNOWN_TYPE:
    case mapping::LaneBoundaryProto::VIRTUAL:
    case mapping::LaneBoundaryProto::BROKEN_WHITE:
    case mapping::LaneBoundaryProto::BROKEN_DOUBLE_WHITE:
    case mapping::LaneBoundaryProto::BROKEN_YELLOW:
    case mapping::LaneBoundaryProto::BROKEN_DOUBLE_YELLOW:
    case mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE:
    case mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE:
      return false;
    case mapping::LaneBoundaryProto::SOLID_WHITE:
    case mapping::LaneBoundaryProto::SOLID_DOUBLE_WHITE:
    case mapping::LaneBoundaryProto::SOLID_YELLOW:
    case mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW:
    case mapping::LaneBoundaryProto::CURB:
      return true;
  }
}

}  // namespace

bool IsOutgoingLane(
    const PlannerSemanticMapManager& /*planner_semantic_map_manager*/,
    const mapping::LaneInfo& source_lane, mapping::ElementId out_lane_id) {
  for (const auto& id : source_lane.outgoing_lanes()) {
    if (out_lane_id == id) {
      return true;
    }
  }
  return false;
}

bool IsRightMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, double s) {
  double accum_len = 0.0;
  for (const auto& seg : lane_path) {
    accum_len += seg.length();
    if (accum_len >= s) {
      SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, planner_semantic_map_manager,
                                      seg.lane_id, false);
      return lane_info.lane_neighbors_on_right.empty();
    }
  }
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(last_lane_info, planner_semantic_map_manager,
                                  lane_path.back().lane_id(), false);
  return last_lane_info.lane_neighbors_on_right.empty();
}

bool IsRightMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id) {
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, planner_semantic_map_manager,
                                  lane_id, false);
  return lane_info.lane_neighbors_on_right.empty();
}

bool IsLeftMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, double s) {
  double accum_len = 0.0;
  for (const auto& seg : lane_path) {
    accum_len += seg.length();
    if (accum_len >= s) {
      SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, planner_semantic_map_manager,
                                      seg.lane_id, false);
      return lane_info.lane_neighbors_on_left.empty();
    }
  }
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(last_lane_info, planner_semantic_map_manager,
                                  lane_path.back().lane_id(), false);
  return last_lane_info.lane_neighbors_on_left.empty();
}

bool IsLeftMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id) {
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, planner_semantic_map_manager,
                                  lane_id, false);
  return lane_info.lane_neighbors_on_left.empty();
}

bool IsTurningLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id) {
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, planner_semantic_map_manager,
                                  lane_id, false);
  if (!lane_info.is_in_intersection) return false;

  switch (lane_info.direction) {
    case mapping::LaneProto::LEFT_TURN:
    case mapping::LaneProto::RIGHT_TURN:
      return true;
    case mapping::LaneProto::STRAIGHT:
    case mapping::LaneProto::UTURN:
      return false;
  }
}

bool IsLanePathBlockedByBox2dAtLevel(
    mapping::LevelId /*level_id*/,
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const Box2d& box, const mapping::LanePath& lane_path, double lat_thres) {
  ASSIGN_OR_DIE(const auto ff, BuildBruteForceFrenetFrame(
                                   SampleLanePathPoints(
                                       planner_semantic_map_manager, lane_path),
                                   /*down_sample_raw_points=*/true));
  bool reached_left = false, reached_right = false;
  for (const auto& pt : box.GetCornersCounterClockwise()) {
    const double lat_offset = ff.XYToSL(pt).l;
    if (lat_offset >= lat_thres) reached_left = true;
    if (lat_offset <= -lat_thres) reached_right = true;

    if (reached_left && reached_right) return true;
  }
  return false;
}

std::vector<Vec2d> SampleLanePathPoints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path) {
  std::vector<Vec2d> sample_points;
  for (const auto& seg : lane_path) {
    if (seg.end_fraction <= seg.start_fraction + kEpsilon) continue;

    const auto* lane_info_ptr =
        planner_semantic_map_manager.FindLaneInfoOrNull(seg.lane_id);
    if (lane_info_ptr == nullptr) {
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
          QIssueSubType::QIST_SEMANTIC_MAP,
          "SemanticMap does not match current lane path.",
          absl::StrCat(lane_path.DebugString(), ", ", QCRAFT_LOC.ToString()));
      break;
    }

    auto tmp_points =
        (seg.start_fraction == 0.0 && seg.end_fraction == 1.0)
            ? lane_info_ptr->points_smooth
            : mapping::ResampleLanePoints(*lane_info_ptr, seg.start_fraction,
                                          seg.end_fraction,
                                          /*cumulative_lengths=*/nullptr);

    if (!sample_points.empty()) sample_points.pop_back();
    sample_points.insert(sample_points.end(), tmp_points.begin(),
                         tmp_points.end());
  }
  return sample_points;
}

// Sample points by smooth coordinates. Can be unified with the above function.
absl::StatusOr<SamplePathPointsResult> SampleLanePathProtoPoints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePathProto& lane_path) {
  std::vector<Vec2d> sample_points;
  bool is_partial = false;
  std::string message = "";
  constexpr auto kFractionEpsilon = 1E-6;
  for (int i = 0; i < lane_path.lane_ids().size(); i++) {
    const auto lane_id = lane_path.lane_ids()[i];
    const auto* lane_info_ptr = planner_semantic_map_manager.FindLaneInfoOrNull(
        mapping::ElementId(lane_id));
    if (lane_info_ptr == nullptr) {
      is_partial = true;
      message =
          absl::StrCat("Cannot find lane_id:", lane_id, QCRAFT_LOC.ToString());
      break;
    }
    const double start_fraction = (i == 0) ? lane_path.start_fraction() : 0.0;
    const double end_fraction =
        (i == lane_path.lane_ids().size() - 1) ? lane_path.end_fraction() : 1.0;
    if (end_fraction <= start_fraction + kFractionEpsilon) continue;
    auto tmp_points =
        mapping::TruncatePoints(lane_info_ptr->proto->polyline().points(),
                                start_fraction, end_fraction, kFractionEpsilon);
    if (!sample_points.empty()) sample_points.pop_back();
    sample_points.insert(sample_points.end(), tmp_points.begin(),
                         tmp_points.end());
  }

  return SamplePathPointsResult{.points = std::move(sample_points),
                                .is_partial = is_partial,
                                .message = std::move(message)};
}

absl::StatusOr<mapping::LanePath> ClampLanePathFromPos(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, const Vec2d& pos) {
  if (lane_path.IsEmpty()) return lane_path;

  const auto points =
      SampleLanePathPoints(planner_semantic_map_manager, lane_path);
  ASSIGN_OR_RETURN(
      const auto ff,
      BuildBruteForceFrenetFrame(points, /*down_sample_raw_points=*/true),
      _.SetPrepend() << "Failed to build frenet frame from lane path "
                     << lane_path.DebugString());
  const auto sl = ff.XYToSL(pos);

  if (sl.s >= lane_path.length()) {
    return absl::NotFoundError(
        absl::StrFormat("Queried pos %s beyond lane path %s.",
                        pos.DebugString(), lane_path.DebugString()));
  }

  return lane_path.AfterArclength(sl.s);
}

absl::StatusOr<mapping::LanePoint> FindOutgoingLanePointWithMinimumHeadingDiff(
    const PlannerSemanticMapManager& psmm, mapping::ElementId id,
    bool allow_virtual) {
  SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, id);

  if (lane_info.IsVirtual() && !allow_virtual) {
    return absl::InternalError(
        absl::StrFormat("Start lane %d is virtual.", lane_info.id.value()));
  }

  constexpr double kSampleLen = 4.0;  // m.
  const Vec2d origin_pt = lane_info.points_smooth.back();
  const Vec2d prev_pt = lane_info.LerpPointFromFraction(
      std::max(0.0, 1.0 - kSampleLen / lane_info.length()));
  const Vec2d heading = (origin_pt - prev_pt).normalized();

  struct OutgoingLaneData {
    mapping::ElementId lane_id;
    double project_value;
    bool is_virtual;
  };

  std::vector<OutgoingLaneData> outgoing_lane_data;
  outgoing_lane_data.reserve(lane_info.outgoing_lanes().size());

  // Calculate proj value for each outgoing lane.
  for (int i = 0; i < lane_info.outgoing_lanes().size(); ++i) {
    const auto temp_id = lane_info.outgoing_lanes()[i];
    SMM_ASSIGN_LANE_OR_CONTINUE(temp_lane_info, psmm, temp_id);
    if (!allow_virtual && temp_lane_info.IsVirtual()) continue;
    const Vec2d next_pt = temp_lane_info.LerpPointFromFraction(
        std::min(1.0, kSampleLen / temp_lane_info.length()));
    const Vec2d tmp_heading = (next_pt - origin_pt).normalized();
    const double proj = heading.Dot(tmp_heading);
    outgoing_lane_data.push_back({temp_id, proj, temp_lane_info.IsVirtual()});
  }

  // Sort according proj value from large to small.
  std::stable_sort(outgoing_lane_data.begin(), outgoing_lane_data.end(),
                   [](const auto& lhs, const auto& rls) {
                     return lhs.project_value > rls.project_value;
                   });

  if (outgoing_lane_data.empty()) {
    return absl::NotFoundError(
        absl::StrFormat("Can not find outgoing lanes for lane id: %d. Search "
                        "allow virtual lane? %d.",
                        lane_info.id.value(), allow_virtual));
  }

  // Select first non virtual lane as extend lane. If all lanes are
  // virtual, select first lane from lane_index_set (after sort).
  auto opt_outgoing_id = outgoing_lane_data.front().lane_id;
  for (const auto& mgr : outgoing_lane_data) {
    if (mgr.is_virtual == false) {
      opt_outgoing_id = mgr.lane_id;
      break;
    }
  }

  return mapping::LanePoint(opt_outgoing_id, 0.0);
}

std::vector<mapping::ElementId> RecomputeStartLanesByPosWithLaneIds(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const mapping::ElementId> start_search_lane_ids,
    const Vec2d& pos, double proj_thres, double max_backward_thres) {
  FUNC_QTRACE();
  std::queue<mapping::ElementId> start_lane_search_queue;
  for (const auto& id : start_search_lane_ids) {
    start_lane_search_queue.push(id);
  }
  std::vector<mapping::ElementId> starting_lane_ids;
  starting_lane_ids.reserve(start_search_lane_ids.size());

  absl::flat_hash_set<mapping::ElementId> visited_id;
  while (!start_lane_search_queue.empty()) {
    const auto lane_id = start_lane_search_queue.front();
    start_lane_search_queue.pop();

    if (visited_id.contains(lane_id)) {
      continue;
    }
    visited_id.insert(lane_id);

    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, lane_id);

    const auto ego_sl = lane_info.SmoothXYToSL(pos);
    if (ego_sl.s > lane_info.length() + proj_thres) {
      for (const auto id : lane_info.outgoing_lanes()) {
        if (visited_id.contains(id)) {
          continue;
        }
        start_lane_search_queue.push(id);
      }
      continue;
    } else if (ego_sl.s < -proj_thres) {
      bool has_incoming = false;
      for (const auto id : lane_info.incoming_lanes()) {
        if (visited_id.contains(id)) {
          continue;
        }
        start_lane_search_queue.push(id);
        has_incoming = true;
      }
      if (has_incoming || ego_sl.s < -max_backward_thres) {
        continue;
      }
    }

    starting_lane_ids.push_back(mapping::ElementId(lane_id));
  }
  return starting_lane_ids;
}

double GetDistOfPointInvasionLaneSupport(
    const Vec2d& smooth_coord,
    const PlannerSemanticMapManager& semantic_map_manager,
    const mapping::ElementId& lane_id, double default_lane_width) {
  const auto level_id = semantic_map_manager.GetLevel();
  double signed_dis;
  double fraction;
  Segment2d segment;
  if (!GetPointToLaneSignedDistanceAtLevel(
          level_id, smooth_coord, semantic_map_manager, lane_id, &signed_dis,
          &fraction, /*point=*/nullptr, &segment)) {
    return -std::numeric_limits<double>::infinity();
  }

  if (fraction == 0.0 || fraction == 1.0) {
    return -std::numeric_limits<double>::infinity();
  }

  // Get the half lane width as the lane support region boundary
  //
  // TODO(weixin):
  //  This should be replaced by a constant-time query to the static info in
  //  the semantic map.
  const auto lane_to_closest_side_boundary_distance =
      [&semantic_map_manager, &segment](
          const mapping::ElementId& lane_id, double fraction, bool right_side,
          double* dist, mapping::ElementId* boundary_id = nullptr) {
        const auto* curr_lane_ptr =
            semantic_map_manager.FindLaneByIdOrNull(lane_id);
        if (curr_lane_ptr == nullptr) {
          QISSUEX_WITH_ARGS(
              QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
              QIssueSubType::QIST_SEMANTIC_MAP, "Can not find lane proto.",
              absl::StrCat("id: ", lane_id, ", loc: ", QCRAFT_LOC.ToString()));
          return false;
        }
        const auto& current_lane = *curr_lane_ptr;
        const auto& neighbor_lanes =
            right_side ? current_lane.lane_neighbors_on_right()
                       : current_lane.lane_neighbors_on_left();
        bool has_valid_dist = false;
        *dist = std::numeric_limits<double>::infinity();
        auto lane_boundary_id = mapping::kInvalidElementId;
        for (const auto& neig : neighbor_lanes) {
          if (neig.this_start_fraction() > fraction ||
              neig.this_end_fraction() < fraction) {
            continue;
          }

          lane_boundary_id = mapping::ElementId(neig.lane_boundary_id());
          if (lane_boundary_id == mapping::kInvalidElementId) {
            break;
          }

          const mapping::LaneBoundaryInfo* lane_boundary =
              semantic_map_manager.FindLaneBoundaryByIdOrNull(lane_boundary_id);
          if (lane_boundary == nullptr) {
            lane_boundary_id = mapping::kInvalidElementId;
            continue;
          }
          const auto boundary_points = VecGeoPointsToVecSmoothPos(
              lane_boundary->proto->polyline().points(),
              semantic_map_manager.coordinate_converter());

          *dist =
              std::accumulate(boundary_points.begin(), boundary_points.end(),
                              std::numeric_limits<double>::infinity(),
                              [&segment](const double dis, const auto& point) {
                                return std::min(dis, segment.DistanceTo(point));
                              });
          has_valid_dist = *dist < std::numeric_limits<double>::infinity();

          // Assuming there is at most one valid lane boundary.
          break;
        }

        if (boundary_id) *boundary_id = lane_boundary_id;
        return has_valid_dist;
      };

  // Use the default lane width if no lane boundaries exist.
  const bool point_on_lane_right = signed_dis > 0.0;
  double half_lane_width = std::numeric_limits<double>::infinity();
  auto lane_boundary_id = mapping::kInvalidElementId;
  const bool has_valid_lane_width = lane_to_closest_side_boundary_distance(
      lane_id, fraction, point_on_lane_right, &half_lane_width,
      &lane_boundary_id);
  const double lane_support_threshold =
      has_valid_lane_width ? half_lane_width : default_lane_width / 2.0;

  return lane_support_threshold - std::abs(signed_dis);
}

bool HasSmoothPointsCrossedBoundary(const PlannerSemanticMapManager& psmm,
                                    absl::Span<const Vec2d> points,
                                    const Range1d<int>& ignore_range,
                                    bool solid_only) {
  constexpr double kMaxLatSearchDist = 3.0;  // m.
  for (int i = 1; i < points.size(); ++i) {
    if (ignore_range.Contains(i - 1) && ignore_range.Contains(i)) {
      continue;
    }

    const auto& curr_xy = points[i];
    const auto& prev_xy = points[i - 1];
    const Segment2d lane_segment(prev_xy, curr_xy);

    const auto candidate_boundaries = psmm.GetLaneBoundariesInfoAtLevel(
        psmm.GetLevel(), curr_xy, kMaxLatSearchDist);

    for (const auto& boundary : candidate_boundaries) {
      if (solid_only && !IsSolidBoundaryType(boundary->type)) continue;

      for (int k = 0; k + 1 < boundary->points_smooth.size(); ++k) {
        const Vec2d& p0 = boundary->points_smooth[k];
        const Vec2d& p1 = boundary->points_smooth[k + 1];
        const Segment2d boundary_segment(p0, p1);

        if (lane_segment.HasIntersect(boundary_segment)) return true;
      }
    }
  }

  return false;
}

}  // namespace qcraft::planner
