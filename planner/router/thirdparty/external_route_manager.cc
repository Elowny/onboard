#include "onboard/planner/router/thirdparty/external_route_manager.h"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/clock.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/router/geometry/gfc.h"
#include "onboard/planner/router/noa/sd_route_util.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/route_navigation.pb.h"

namespace qcraft::planner::route {
namespace {

void UpdateHmiSdRouteMatchDistRange(
    const std::vector<SdRouteMatchState::DistRange>& dist_ranges,
    RouteContentProto* route_content_proto) {
  auto* ranges_proto = route_content_proto->mutable_nca_odc()
                           ->mutable_sd_route_match_dist_ranges();
  ranges_proto->Clear();
  ranges_proto->Reserve(dist_ranges.size());
  std::for_each(dist_ranges.begin(), dist_ranges.end(),
                [ranges_proto](const auto& interval) {
                  auto* proto = ranges_proto->Add();
                  proto->set_start(static_cast<int>(interval.start));
                  proto->set_end(static_cast<int>(interval.end));
                });
}

// Go backwards not allowed yet.
absl::StatusOr<std::pair<int, ProjectToLineStringResult>>
ComputeCoordIndexAndProjectResult(const Vec2d& global2d,
                                  const std::vector<Vec2d>& coords,
                                  int last_coord_index, bool reset,
                                  const double max_project_distance) {
  constexpr int kMinBestProjectDist = 40;  // meters
  constexpr int kLookAheadPoints = 4;
  constexpr int kMaxJumpPoints = 8;

  auto segment_pred_fn = [](const Vec2d&, const Vec2d&) { return true; };
  ProjectToLineStringResult project_result;
  // TODO(xiang): Use distance instead of the points count.
  auto quick_project_result = ProjectToLineString(
      coords.begin() + last_coord_index,
      coords.begin() + std::min(static_cast<int>(coords.size()),
                                last_coord_index + kLookAheadPoints),
      global2d, segment_pred_fn);

  if (HaversineDistance(quick_project_result.match_point, global2d) <
      kMinBestProjectDist) {
    project_result = quick_project_result;
  } else {
    QLOG_EVERY_N_SEC(INFO, 3)
        << "Cannot match points: " << kLookAheadPoints
        << ", match all points from index:" << last_coord_index;
    project_result =
        ProjectToLineString(coords.begin() + last_coord_index, coords.end(),
                            global2d, segment_pred_fn);
  }
  auto project_dist = HaversineDistance(project_result.match_point, global2d);
  if (project_result.segment_index >= 0 &&
      project_dist < max_project_distance &&
      (project_result.segment_index < kMaxJumpPoints || reset)) {
    return std::pair{last_coord_index + project_result.segment_index,
                     project_result};
  }
  return absl::InternalError(absl::StrFormat(
      "Match sd route failed. project_dist: %f, max_project_distance: %f, "
      "segment_index: %d, last_coord_index: %d",
      project_dist, max_project_distance, project_result.segment_index,
      last_coord_index));
}

int ComputeNaviSegmentIndexByCoordsIndex(
    const RoutingResultProto& routing_result_proto, int cur_coord_idx) {
  const auto& navi_segs = routing_result_proto.navi_preview().navi_segments();
  for (int seg_idx = 0; seg_idx < navi_segs.size(); ++seg_idx) {
    if (cur_coord_idx < navi_segs[seg_idx].coord_idx(1)) {
      return seg_idx;
    }
  }
  return navi_segs.size() - 1;
}

struct RouteDistanceResult {
  int distance_to_end = 0;              // To sd route end.
  int distance_to_navi_end = 0;         // To cur navi segment end.
  double distance_to_next_point = 0.0;  // To next point, may be zero.
};

RouteDistanceResult ComputeDistance(
    const Vec2d& match_point, const RoutingResultProto& routing_result_proto,
    const SdRouteMatchState& route_match_state, int seg_idx,
    int cur_match_coord_index) {
  double next_navi_seg_to_end = 0.0;
  double distance_to_next_point = 0.0;
  double distance_to_navi_end = 0.0;
  const auto& segs = routing_result_proto.navi_preview().navi_segments();
  if (cur_match_coord_index + 1 < route_match_state.coords.size()) {
    distance_to_next_point = HaversineDistance(
        match_point, route_match_state.coords[cur_match_coord_index + 1]);
  }
  for (int i = cur_match_coord_index + 2;
       i <= segs[seg_idx].coord_idx()[1] && i < route_match_state.coords.size();
       ++i) {
    distance_to_navi_end += route_match_state.segment_dists[i];
  }
  for (int i = seg_idx + 1; i < segs.size(); i++) {
    next_navi_seg_to_end += segs[i].length();
  }
  distance_to_navi_end += distance_to_next_point;
  const double distance_to_end = next_navi_seg_to_end + distance_to_navi_end;

  VLOG(3) << ", distance_to_end: " << distance_to_end
          << ", distance_to_navi_end: " << distance_to_navi_end
          << ", distance_to_next_point: " << distance_to_next_point
          << ", cur_match_coord_index: " << cur_match_coord_index
          << ", match_point: "
          << absl::StrFormat("%.8f %.8f", match_point.x(), match_point.y());
  return {
      .distance_to_end = static_cast<int>(distance_to_end),
      .distance_to_navi_end = static_cast<int>(distance_to_navi_end),
      .distance_to_next_point = distance_to_next_point,
  };
}

}  // namespace

absl::Status UpdateSdRouteMatchState(const Vec2d& match_point,
                                     int cur_coord_index,
                                     double dist_to_sd_route_end,
                                     SdRouteMatchState* sd_route_match_state) {
  if (sd_route_match_state == nullptr) {
    return absl::FailedPreconditionError(
        "The sd_route_match_state cannot be null.");
  }
  sd_route_match_state->cur_coord_index = cur_coord_index;
  sd_route_match_state->av_project_point = match_point;
  sd_route_match_state->s =
      sd_route_match_state->sd_route_length - dist_to_sd_route_end;

  if (!sd_route_match_state->link_points_size_accumulator.empty()) {
    const auto& points_index =
        sd_route_match_state->link_points_size_accumulator;
    const auto it = std::lower_bound(points_index.begin(), points_index.end(),
                                     cur_coord_index);
    if (it == points_index.end()) {
      return absl::OutOfRangeError("The points index exceeds the points size.");
    }
    const int link_coord_index = *it;
    sd_route_match_state->cur_match_link_index =
        std::distance(points_index.begin(), it);
    sd_route_match_state->cur_match_point_on_link_index =
        cur_coord_index - link_coord_index;
  }
  sd_route_match_state->match_dist_ranges_from_cur =
      sd_route_match_state->match_dist_ranges_from_start;
  for (auto& dist_range : sd_route_match_state->match_dist_ranges_from_cur) {
    dist_range.start -= sd_route_match_state->s;
    dist_range.end -= sd_route_match_state->s;
  }
  return absl::OkStatus();
}

void UpdateRoutingResultProto(const Vec2d& match_point, int cur_seg_dist,
                              int seg_index, const std::vector<Vec2d>& coords,
                              int cur_match_coord_index,
                              RoutingResultProto* routing_result_proto) {
  auto* current_navi = routing_result_proto->mutable_current_navi_instruction();
  current_navi->Clear();
  if (seg_index ==
      routing_result_proto->navi_preview().navi_segments().size() - 1) {
    current_navi->set_is_last_navi_action(true);
  }
  current_navi->set_navi_index(seg_index);
  current_navi->set_distance(cur_seg_dist);
  *current_navi->mutable_navi_action() = routing_result_proto->navi_preview()
                                             .navi_segments()[seg_index]
                                             .navi_actions()[0];
  auto* points_from_current =
      routing_result_proto->mutable_path_points_from_current();
  points_from_current->Clear();
  points_from_current->Reserve(coords.size() - cur_match_coord_index);
  match_point.ToProto(points_from_current->Add());
  for (int i = cur_match_coord_index + 1; i < static_cast<int>(coords.size());
       i++) {
    coords[i].ToProto(points_from_current->Add());
  }
}

absl::Status ExternalRouteManager::InitSdRoute(
    const std::shared_ptr<const SDRouteProto>& sd_route,
    const std::shared_ptr<const SdRouteMatchResultProto>&
        sd_route_match_result_proto) {
  Clear();
  is_first_match_ = true;
  return noa::FillRouteManagerState(sd_route, sd_route_match_result_proto,
                                    &data_);
}

void ExternalRouteManager::DeleteSdRoute(
    const std::shared_ptr<const SDRouteProto>& sd_route) {
  if (data_.sd_route_match_state.sd_route_proto == nullptr) {
    QLOG(ERROR) << "Delete sd route, but NOT found a sd route yet.";
  }
  Clear();
  data_.sd_route_match_state.sd_route_proto = sd_route;
}

bool ExternalRouteManager::UpdateRoute(const Vec2d& global2d,
                                       const double max_project_distance) {
  std::int64_t cur_time_us = absl::ToUnixMicros(Clock::Now());
  if (data_.routing_result_proto.navi_preview().navi_segments().empty() ||
      coords().empty()) {
    QLOG_EVERY_N_SEC(ERROR, 3) << "The points is invalid.";
    return false;
  }
  constexpr int kMatchFailureTimesBeforeReset = 10;
  const bool is_reset =
      (consecutive_failure_times_ > kMatchFailureTimesBeforeReset) ||
      is_first_match_;
  auto coord_index_and_project_result_or = ComputeCoordIndexAndProjectResult(
      global2d, coords(),
      std::clamp(data_.sd_route_match_state.cur_coord_index, 0,
                 static_cast<int>(coords().size() - 1)),
      is_reset, max_project_distance);
  if (!coord_index_and_project_result_or.ok()) {
    QLOG_EVERY_N_SEC(ERROR, 3)
        << "Project to line string failed"
        << ", last cur_coord_index: " << cur_coord_idx()
        << ", max_project_distance: " << max_project_distance
        << ", consecutive_failure_times_: " << consecutive_failure_times_
        << ", is_first_match_: " << is_first_match_
        << ", status: " << coord_index_and_project_result_or.status();
    ++consecutive_failure_times_;
    if (consecutive_failure_times_ > kMatchFailureTimesBeforeReset) {
      // Reset match state
      set_cur_coord_idx(0);
    }
    return false;
  } else {
    is_first_match_ = false;
  }

  consecutive_failure_times_ = 0;
  const auto [cur_match_coord_index, project_result] =
      std::move(coord_index_and_project_result_or).value();
  const int new_seg_idx = ComputeNaviSegmentIndexByCoordsIndex(
      data_.routing_result_proto, cur_match_coord_index);
  auto route_distance_result = ComputeDistance(
      project_result.match_point, data_.routing_result_proto,
      data_.sd_route_match_state, new_seg_idx, cur_match_coord_index);
  data_.rms_debug.set_distance_to_next_stop(
      route_distance_result.distance_to_end);
  data_.route_content_proto.set_distance_to_next_station(
      route_distance_result.distance_to_end);
  data_.routing_result_proto.set_distance(
      route_distance_result.distance_to_end);

  // Calc Navi distance, but now without any navi segments.
  const int cur_seg_dist = route_distance_result.distance_to_end;
  UpdateRoutingResultProto(project_result.match_point, cur_seg_dist,
                           new_seg_idx, coords(), cur_match_coord_index,
                           &data_.routing_result_proto);
  // update route manage state
  const auto update_match_status = UpdateSdRouteMatchState(
      project_result.match_point, cur_match_coord_index,
      data_.route_content_proto.distance_to_next_station(),
      &data_.sd_route_match_state);
  if (!update_match_status.ok()) {
    QLOG_EVERY_N_SEC(ERROR, 3)
        << " UpdateSdRouteMatchState failed. " << update_match_status;
  }
  if (data_.sd_route_match_state.sd_route_match_result_proto != nullptr) {
    UpdateHmiSdRouteMatchDistRange(
        data_.sd_route_match_state.match_dist_ranges_from_cur,
        &data_.route_content_proto);
  }
  // Update rms_debug
  project_result.match_point.ToProto(
      data_.rms_debug.mutable_start_next_route_pos());
  data_.rms_debug.set_last_update_time(cur_time_us);
  return true;
}

void ExternalRouteManager::Clear() {
  consecutive_failure_times_ = 0;
  data_.Clear();
}

}  // namespace qcraft::planner::route
