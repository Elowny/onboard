#include "onboard/planner/router/noa/sd_route_util.h"

#include <algorithm>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include "google/protobuf/repeated_ptr_field.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/route_navigation.pb.h"
#include "onboard/utils/status_macros.h"
namespace qcraft::planner::route::noa {

absl::Status FillRoutingResultBySdRoute(const SDRouteProto& sd_route,
                                        RoutingResultProto* routing_result) {
  if (routing_result == nullptr) {
    return absl::InvalidArgumentError("routing_result cannot be null.");
  }
  if (sd_route.status() != SDRouteProto::CREATE) {
    routing_result->set_success(false);
    return absl::InvalidArgumentError("Can only process the CREATE status.");
  } else {
    routing_result->Clear();
  }
  routing_result->set_update_id(sd_route.route_id());
  auto* preview = routing_result->mutable_navi_preview();
  int size = 0;
  for (const auto& section : sd_route.sd_sections()) {
    auto* points = routing_result->mutable_path_points_from_current();

    for (const auto& link : section.sd_links()) {
      size += link.points().size();
    }
    points->Reserve(size);
    for (const auto& link : section.sd_links()) {
      std::for_each(
          link.points().begin(), link.points().end(), [points](const auto& pt) {
            Vec2dToProto({pt.longitude(), pt.latitude()}, points->Add());
          });
    }
  }
  preview->set_update_id(sd_route.route_id());
  preview->clear_navi_segments();
  auto* navi_segment = preview->add_navi_segments();
  navi_segment->set_length(sd_route.length());
  navi_segment->add_coord_idx(0);
  navi_segment->add_coord_idx(size - 1);
  auto* nav_action = navi_segment->add_navi_actions();
  nav_action->set_navi_action_type(NaviActionProto::DESTINATION);

  auto* cur_nav_aciton = routing_result->mutable_current_navi_instruction();
  cur_nav_aciton->set_distance(sd_route.length());
  cur_nav_aciton->set_is_last_navi_action(true);
  cur_nav_aciton->set_update_id(sd_route.route_id());
  cur_nav_aciton->mutable_navi_action()->set_navi_action_type(
      NaviActionProto::STRAIGHT);
  auto* stop = routing_result->mutable_routing_request()
                   ->mutable_multi_stops()
                   ->add_stops();
  stop->set_stop_name("");
  auto* stop_global_pt = stop->mutable_stop_point()->mutable_global_point();
  if (size > 0) {
    const auto& destination_point =
        routing_result->path_points_from_current(size - 1);
    stop_global_pt->set_longitude(destination_point.x());
    stop_global_pt->set_latitude(destination_point.x());
  }

  routing_result->set_success(true);
  routing_result->set_distance(sd_route.length());
  return absl::OkStatus();
}

absl::Status InitSdRouteMatchResult(
    const std::shared_ptr<const SdRouteMatchResultProto>&
        sd_route_match_result_proto,
    SdRouteMatchState* /*In&Out*/ sd_route_match_state) {
  if (sd_route_match_state == nullptr) {
    return absl::InvalidArgumentError(
        "The sd_route_match_state cannot be null.");
  }
  sd_route_match_state->sd_route_match_result_proto =
      sd_route_match_result_proto;
  if (sd_route_match_state->sd_route_match_result_proto == nullptr) {
    return absl::InvalidArgumentError(
        "The sd_route_match_result_proto cannot be null.");
  }
  const auto cc =
      CoordinateConverter::FromMapOriginCoord(sd_route_match_state->coords[0]);

  sd_route_match_state->match_dist_ranges_from_start.reserve(
      sd_route_match_state->sd_route_match_result_proto->segments_size());

  for (const auto& segment :
       sd_route_match_state->sd_route_match_result_proto->segments()) {
    sd_route_match_state->match_dist_ranges_from_start.push_back({
        .start =
            DistanceToSdMatchPoint(cc, *sd_route_match_state, segment.start()),
        .end = DistanceToSdMatchPoint(cc, *sd_route_match_state, segment.end()),
    });
  }
  sd_route_match_state->match_dist_ranges_from_cur =
      sd_route_match_state->match_dist_ranges_from_start;
  return absl::OkStatus();
}

absl::StatusOr<SdRouteMatchState> BuildSdRouteMatchState(
    const std::shared_ptr<const SDRouteProto>& sd_route,
    const std::shared_ptr<const SdRouteMatchResultProto>&
        sd_route_match_result_proto) {
  if (sd_route == nullptr || sd_route->sd_sections().empty()) {
    return absl::InvalidArgumentError("The sd_route cannot be null.");
  }
  SdRouteMatchState state;
  state.sd_route_proto = sd_route;
  state.sd_route_length = sd_route->length();

  int points_size = 0;
  int links_size = 0;
  for (const auto& section : state.sd_route_proto->sd_sections()) {
    for (const auto& link : section.sd_links()) {
      points_size += link.points().size();
    }
    links_size += section.sd_links_size();
  }

  state.link_points_size_accumulator.reserve(links_size);
  state.link_s_accumulator.reserve(links_size);
  state.coords.reserve(points_size);

  state.link_points_size_accumulator.push_back(0);
  state.link_s_accumulator.push_back(0);
  for (const auto& section : state.sd_route_proto->sd_sections()) {
    for (const auto& link : section.sd_links()) {
      state.link_points_size_accumulator.push_back(
          state.link_points_size_accumulator.back() + link.points().size());
      state.link_s_accumulator.push_back(state.link_s_accumulator.back() +
                                         link.length());

      std::for_each(link.points().begin(), link.points().end(),
                    [&coords = state.coords](const auto& pt) {
                      coords.push_back({pt.longitude(), pt.latitude()});
                    });
    }
  }
  if (state.coords.empty()) {
    return absl::InvalidArgumentError(
        "sd_route_proto maybe invalid, global points are empty!");
  }

  auto cc = CoordinateConverter::FromMapOriginCoord(state.coords[0]);
  state.segment_dists.reserve(points_size);

  auto last_smooth_pt = cc.GlobalToSmooth(state.coords[0]);
  for (const auto& pt : state.coords) {
    auto smooth_pt = cc.GlobalToSmooth(pt);
    auto dist = smooth_pt.DistanceTo(last_smooth_pt);
    state.segment_dists.push_back(dist);
    last_smooth_pt = smooth_pt;
  }
  if (sd_route_match_result_proto != nullptr) {
    RETURN_IF_ERROR(
        InitSdRouteMatchResult(sd_route_match_result_proto, &state));
  }
  return state;
}

absl::Status FillRouteManagerState(
    const std::shared_ptr<const SDRouteProto>& sd_route,
    const std::shared_ptr<const SdRouteMatchResultProto>&
        sd_route_match_result_proto,
    RouteManagerState* rms) {
  ASSIGN_OR_RETURN(
      auto sd_route_match_state,
      BuildSdRouteMatchState(sd_route, sd_route_match_result_proto));
  rms->sd_route_match_state = std::move(sd_route_match_state);
  return FillRoutingResultBySdRoute(*rms->sd_route_match_state.sd_route_proto,
                                    &rms->routing_result_proto);
}

double DistanceToSdMatchPoint(const CoordinateConverter& cc,
                              const SdRouteMatchState& state,
                              const SdRouteMatchPoint& sd_route_match_point) {
  // Invalid  input, return 0
  if (state.link_points_size_accumulator.size() !=
      state.link_s_accumulator.size()) {
    return 0.0;
  }

  if (sd_route_match_point.link_index() > state.link_s_accumulator.size() - 1) {
    return 0.0;
  }

  const double links_dist =
      state.link_s_accumulator[sd_route_match_point.link_index()];
  const int link_coord_beg_index =
      state.link_points_size_accumulator[sd_route_match_point.link_index()];
  if (link_coord_beg_index + sd_route_match_point.link_point_index() >
      state.coords.size() - 1) {
    return 0.0;
  }
  const auto link_index_point =
      cc.GlobalToSmooth(state.coords[link_coord_beg_index +
                                     sd_route_match_point.link_point_index()]);
  const auto link_match_point =
      cc.GlobalToSmooth(Vec2d(sd_route_match_point.point().longitude(),
                              sd_route_match_point.point().latitude()));
  const auto match_interval_dist =
      link_index_point.DistanceTo(link_match_point);
  const auto dist_to_link_beg =
      match_interval_dist +
      std::accumulate(state.segment_dists.begin() + link_coord_beg_index,
                      state.segment_dists.begin() + link_coord_beg_index +
                          sd_route_match_point.link_point_index(),
                      0.0);
  return links_dist + dist_to_link_beg;
}

}  // namespace qcraft::planner::route::noa
