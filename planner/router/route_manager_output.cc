#include "onboard/planner/router/route_manager_output.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <utility>

#include "absl/container/flat_hash_map.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

void RouteManagerOutput::ToProto(RouteManagerOutputProto* proto) const {
  proto->Clear();

  if (!route_sections_from_current.empty()) {
    route_sections_from_current.ToProto(
        proto->mutable_route_sections_from_current());
  }

  if (!avoid_lanes.empty()) {
    proto->mutable_avoid_lanes()->Reserve(avoid_lanes.size());
    for (const auto& avoid_lane : avoid_lanes) {
      proto->mutable_avoid_lanes()->Add(avoid_lane.value());
    }
  }

  proto->set_signal(signal);
  proto->set_rerouted(rerouted);

  *proto->mutable_destination_stop() = destination_stop;
  proto->set_update_id(update_id);

  if (recommend_max_speed_limit.has_value()) {
    proto->set_recommend_lane_speed_limit(*recommend_max_speed_limit);
  }

  *proto->mutable_routing_request() = routing_request_proto;
  route_navi_info.ToProto(proto->mutable_route_navi_info());

  if (alter_route_msg.has_value()) {
    alter_route_msg->route_sections_from_current.ToProto(
        proto->mutable_alternative_path_proto()
            ->mutable_route_sections_from_current());
    alter_route_msg->route_navi_info.ToProto(
        proto->mutable_alternative_path_proto()->mutable_route_navi_info());
    proto->mutable_alternative_path_proto()->set_roundabout_distance(
        alter_route_msg->roundabout_distance);
  }

  proto->set_route_status(status);

  if (!avoid_lane_segments.IsEmpty()) {
    avoid_lane_segments.ToProto(proto->mutable_avoid_lane_segments());
  }

  proto->mutable_overlap_sections()->Reserve(overlap_sections.size());
  for (const auto& overlap_section : overlap_sections) {
    *proto->mutable_overlap_sections()->Add() = overlap_section;
  }
  proto->set_overlap_sections_precomputed(overlap_sections_precomputed);
}

void RouteManagerOutput::FromProto(const RouteManagerOutputProto& proto) {
  SCOPED_QTRACE("RouteManagerOutput::FromProto");

  avoid_lanes.clear();
  if (!proto.avoid_lanes().empty()) {
    for (const auto& avoid_lane : proto.avoid_lanes()) {
      avoid_lanes.insert(mapping::ElementId(avoid_lane));
    }
  }

  if (proto.has_route_sections_from_current()) {
    route_sections_from_current =
        RouteSections::BuildFromProto(proto.route_sections_from_current());
  } else if (proto.has_route_from_current()) {
    route_sections_from_current = RouteSections::BuildFromProto(
        proto.route_from_current().route_section_sequence());

    if (avoid_lanes.empty()) {
      for (const auto& avoid_lane : proto.route_from_current().avoid_lanes()) {
        avoid_lanes.insert(mapping::ElementId(avoid_lane));
      }
    }
  }

  signal = proto.signal();
  rerouted = proto.rerouted();

  destination_stop = proto.destination_stop();
  update_id = proto.update_id();

  if (proto.has_recommend_lane_speed_limit()) {
    recommend_max_speed_limit = proto.recommend_lane_speed_limit();
  }

  if (proto.has_routing_request()) {
    routing_request_proto = proto.routing_request();
  } else if (proto.has_route_from_current()) {
    routing_request_proto = proto.route_from_current().routing_request();
  }

  route_navi_info.FromProto(proto.route_navi_info());

  if (proto.has_alternative_path_proto()) {
    RouteNaviInfo route_navi_info;
    route_navi_info.FromProto(proto.alternative_path_proto().route_navi_info());
    alter_route_msg = {
        .route_sections_from_current = RouteSections::BuildFromProto(
            proto.alternative_path_proto().route_sections_from_current()),
        .route_navi_info = std::move(route_navi_info),
        .roundabout_distance =
            proto.alternative_path_proto().has_roundabout_distance()
                ? proto.alternative_path_proto().roundabout_distance()
                : 0.0};
  }
  if (proto.has_route_status()) {
    status = proto.route_status();
  } else if (proto.has_reset()) {
    status = RouteManagerOutputProto::RESET;
  } else if (proto.has_is_valid()) {
    status = proto.is_valid() ? RouteManagerOutputProto::NORMAL
                              : RouteManagerOutputProto::INVALID;
  }

  avoid_lane_segments.lane_segments_map.clear();
  if (proto.has_avoid_lane_segments()) {
    avoid_lane_segments.FromProto(proto.avoid_lane_segments());
  }

  overlap_sections.clear();
  overlap_sections.reserve(proto.overlap_sections().size());
  for (const auto& overlap_section : proto.overlap_sections()) {
    overlap_sections.push_back(overlap_section);
  }
  overlap_sections_precomputed = proto.has_overlap_sections_precomputed()
                                     ? proto.overlap_sections_precomputed()
                                     : false;
}

RouteManagerOutputProto CompatibilityProcessForRouteManagerOutput(
    const RouteManagerOutputProto& rm_out) {
  RouteManagerOutputProto rm_output = rm_out;
  if (!rm_output.has_routing_request() && rm_out.has_route_from_current() &&
      rm_out.route_from_current().has_routing_request()) {
    *rm_output.mutable_routing_request() =
        rm_out.route_from_current().routing_request();
  }

  if (!rm_output.has_route_sections_from_current() &&
      rm_out.has_route_from_current() &&
      rm_out.route_from_current().has_route_section_sequence()) {
    *rm_output.mutable_route_sections_from_current() =
        rm_out.route_from_current().route_section_sequence();
  }

  return rm_output;
}

}  // namespace planner
}  // namespace qcraft
