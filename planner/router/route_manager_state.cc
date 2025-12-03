#include "onboard/planner/router/route_manager_state.h"

#include <sstream>

namespace qcraft::planner {

std::string SdRouteMatchState::ShortDebugString() const {
  std::stringstream ss;
  ss << "coords size: " << coords.size()
     << ", segment_dists size : " << segment_dists.size()
     << ", link_points_size_accumulator size: "
     << link_points_size_accumulator.size()
     << ", match_dist_ranges_from_start size: "
     << match_dist_ranges_from_start.size()
     << ", sd_route_length: " << sd_route_length
     << ", cur_match_link_index: " << cur_match_link_index
     << ", cur_match_point_on_link_index: " << cur_match_point_on_link_index
     << ", cur_coord_index: " << cur_coord_index << ", s: " << s;
  ss << "match_dist_ranges_from_cur: {";
  for (const auto& dist_range : match_dist_ranges_from_cur) {
    ss << "[" << static_cast<int>(dist_range.start) << ","
       << static_cast<int>(dist_range.end) << "]";
  }
  ss << "}";
  return ss.str();
}

void SdRouteMatchState::Clear() {
  sd_route_proto = nullptr;
  sd_route_match_result_proto = nullptr;
  coords.clear();
  link_points_size_accumulator.clear();
  link_s_accumulator.clear();
  segment_dists.clear();
  match_dist_ranges_from_start.clear();
  sd_route_length = 0;
  cur_match_link_index = 0;
  cur_match_point_on_link_index = 0;
  cur_coord_index = 0;
  s = 0;
  av_project_point = {0.0, 0.0};
  match_dist_ranges_from_cur.clear();
}

void RouteManagerState::Clear() {
  rms_debug.set_route_s(0.0);
  rms_debug.set_reached_destination_index(-1);
  rms_debug.set_is_new_route(true);
  rms_debug.set_goto_next_stop(false);
  route.Clear();
  route_from_current.Clear();
  routing_result_proto.Clear();
  route_content_proto.Clear();
  sd_route_match_state.Clear();
}

void RouteResetRequest(RouteManagerState* state) {
  state->Clear();
  ++state->request_id;
}

}  // namespace qcraft::planner
