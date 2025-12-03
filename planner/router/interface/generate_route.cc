#include "onboard/planner/router/interface/generate_route.h"

#include <stddef.h>

#include <algorithm>
#include <iterator>
#include <memory>
#include <ostream>
#include <string>

#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "google/protobuf/message.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/compatibility_layer.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_util.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/smm_proto_util.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_definition.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/route_core_searcher.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner::route {

namespace {
std::shared_ptr<mapping::v2::SemanticMapManager> SmmV1ToV2(
    const mapping::SemanticMapManager& smm_v1) {
  auto semantic_map_proto = smm_v1.semantic_map();
  auto semantic_map_meta_proto = smm_v1.semantic_map_meta();
  const auto& converter = smm_v1.coordinate_converter();
  const mapping::v2::UpdateId update_id(smm_v1.get_route_update_id());
  auto smm_v2 = mapping::v2::SemanticMapManager::MakeShared(
      update_id, {}, converter, std::move(semantic_map_proto),
      std::move(semantic_map_meta_proto));
  return smm_v2;
}
}  // namespace

absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
SearchRouteResultsBetweenLanePoints(
    const mapping::SemanticMapManager& smm, const mapping::LanePoint& origin,
    absl::Span<const mapping::LanePoint> destinations, bool is_bus) {
  return SearchRouteResultsBetweenLanePoints(*SmmV1ToV2(smm), origin,
                                             destinations, is_bus);
}

absl::StatusOr<CompositeLanePath> SearchLanePathBetweenLanePoints(
    const mapping::SemanticMapManager& smm, const mapping::LanePoint& origin,
    absl::Span<const mapping::LanePoint> destinations, bool is_bus) {
  return SearchLanePathBetweenLanePoints(*SmmV1ToV2(smm), origin, destinations,
                                         is_bus);
}

absl::StatusOr<CompositeLanePath> SearchLanePathBetweenVec3ds(
    const mapping::SemanticMapManager& smm, const MapIndex* map_index,
    const Vec3d& origin, const Vec3d& destination, bool is_bus) {
  return SearchLanePathBetweenVec3ds(*SmmV1ToV2(smm), map_index, origin,
                                     destination, is_bus);
}

absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
SearchRouteResultsBetweenLanePoints(
    const mapping::v2::SemanticMapManager& smm,
    const mapping::LanePoint& origin,
    absl::Span<const mapping::LanePoint> destinations, bool is_bus) {
  RouteParamProto route_params = planner::CreateDefaultRouteParam();
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_precond = {
      .is_bus = is_bus,
      .use_time = true,
  };

  SMM_LANE_PROTO_OR_RETURN(origin_lane_proto, smm, origin.lane_id(),
                           absl::NotFoundError(absl::StrCat(
                               "Can not find origin: ", origin.DebugString())));
  const mapping::PointToLane origin_ptl = {.lane_proto = origin_lane_proto,
                                           .dist = 0.0,
                                           .fraction = origin.fraction()};

  std::vector<std::vector<mapping::PointToLane>> dests;
  dests.reserve(destinations.size());
  for (const auto& dest : destinations) {
    SMM_LANE_PROTO_OR_CONTINUE(dest_proto, smm, dest.lane_id());
    dests.push_back({mapping::PointToLane{
        .lane_proto = dest_proto, .dist = 0.0, .fraction = dest.fraction()}});
  }
  return route_core.SearchForRoutePathAlongLanePoints(route_precond,
                                                      {origin_ptl}, dests);
}

absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
SearchRouteResultsBetweenDestinations(
    const mapping::v2::SemanticMapManager& smm, const MapIndex* map_index,
    const google::protobuf::RepeatedPtrField<RoutingDestinationProto>&
        destinations,
    const RouteRestrictDistrict& route_restrict, bool is_bus) {
  if (destinations.size() < 2) {
    return absl::InvalidArgumentError(
        "Input destinations' size is less than 2");
  }
  RouteParamProto route_params = planner::CreateDefaultRouteParam();
  RouteCore route_core(&smm, &route_params);
  RouteCorePrecondition route_precond = {
      .is_bus = is_bus,
      .use_time = true,
      .route_restrict_district = route_restrict,
  };

  ASSIGN_OR_RETURN(const auto origin, ParseDestinationProtoToLanePoint(
                                          smm, map_index, destinations[0]));
  const mapping::PointToLane origin_ptl = {
      .lane_proto = mapping::FindLaneProto(smm, origin.lane_id()),
      .dist = 0.0,
      .fraction = origin.fraction(),
  };

  std::vector<std::vector<mapping::PointToLane>> dests;
  dests.reserve(destinations.size() - 1);
  for (int i = 1; i < destinations.size(); ++i) {
    ASSIGN_OR_RETURN(const auto dest, ParseDestinationProtoToLanePoint(
                                          smm, map_index, destinations[i]));
    dests.push_back({mapping::PointToLane{
        .lane_proto = mapping::FindLaneProto(smm, dest.lane_id()),
        .dist = 0.0,
        .fraction = dest.fraction()}});
  }
  return route_core.SearchForRoutePathAlongLanePoints(route_precond,
                                                      {origin_ptl}, dests);
}

absl::StatusOr<std::vector<CompositeLanePath>> GenerateRoutePathFromStops(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    absl::Span<const mapping::LanePoint> destinations,
    absl::Span<const int> stops_index, bool infinite_loop, bool is_bus) {
  std::vector<CompositeLanePath> total_lane_paths;
  for (size_t i = 0; i + 1 < stops_index.size(); ++i) {
    std::vector<mapping::LanePoint> current_destinations;
    for (int j = stops_index[i] + 1; j <= stops_index[i + 1]; ++j) {
      current_destinations.push_back(destinations[j]);
    }
    auto current_lane_path_or = SearchLanePathBetweenLanePoints(
        semantic_map_manager, destinations[stops_index[i]],
        current_destinations, is_bus);
    if (!current_lane_path_or.ok()) {
      total_lane_paths.emplace_back();
      continue;
    }
    total_lane_paths.emplace_back(std::move(current_lane_path_or).value());
  }

  if (infinite_loop) {
    std::vector<mapping::LanePoint> current_destinations;
    for (int i = 0; i <= stops_index[0]; ++i) {
      current_destinations.push_back(destinations[i]);
    }
    auto current_lane_path_or = SearchLanePathBetweenLanePoints(
        semantic_map_manager, destinations.back(), current_destinations,
        is_bus);
    if (!current_lane_path_or.ok()) {
      total_lane_paths.emplace_back();
    } else {
      total_lane_paths.emplace_back(std::move(current_lane_path_or).value());
    }
  }
  return total_lane_paths;
}

absl::StatusOr<std::vector<CompositeLanePath>>
GenerateRoutePathByRoutingRequest(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const MapIndex* map_index, const RoutingRequestProto& routing_request_proto,
    bool is_bus) {
  auto multiple_request_or = BuildMultipleStopsRequest(
      semantic_map_manager, map_index, routing_request_proto);
  if (!multiple_request_or.ok()) {
    QLOG(ERROR) << "Cannot transform to MultipleStopsRequest from:"
                << routing_request_proto.DebugString();
    return multiple_request_or.status();
  }

  auto destination_stop_or = route::GetDestinations(
      semantic_map_manager, map_index, *multiple_request_or);

  if (!destination_stop_or.ok()) {
    return absl::NotFoundError(absl::StrCat(
        "Cannot get destination.", destination_stop_or.status().ToString()));
  }
  const auto destinations = std::move(destination_stop_or->first);
  const auto stops_index = std::move(destination_stop_or->second);
  if (stops_index.empty() || destinations.empty()) {
    return absl::NotFoundError("Invalid route request, no stops.");
  }

  auto paths_or = GenerateRoutePathFromStops(
      semantic_map_manager, destinations, stops_index,
      multiple_request_or->infinite_loop(), is_bus);
  return paths_or;
}

absl::StatusOr<CompositeLanePath> SearchLanePathBetweenLanePoints(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const mapping::LanePoint& origin,
    absl::Span<const mapping::LanePoint> destinations, bool is_bus) {
  ASSIGN_OR_RETURN(const auto sections_lanes,
                   SearchRouteResultsBetweenLanePoints(
                       semantic_map_manager, origin, destinations, is_bus));
  return sections_lanes.second;
}

absl::StatusOr<CompositeLanePath> SearchLanePathBetweenVec3ds(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const MapIndex* map_index, const Vec3d& origin, const Vec3d& destination,
    bool is_bus) {
  mapping::GeoPointProto origin_proto;
  origin_proto.set_longitude(origin.x());
  origin_proto.set_latitude(origin.y());
  origin_proto.set_altitude(origin.z());

  mapping::GeoPointProto destination_proto;
  destination_proto.set_longitude(destination.x());
  destination_proto.set_latitude(destination.y());
  destination_proto.set_altitude(destination.z());
  return SearchLanePathBetweenGeoPoints(
      semantic_map_manager, map_index, origin_proto, destination_proto, is_bus);
}

absl::StatusOr<CompositeLanePath> SearchLanePathBetweenGeoPoints(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const MapIndex* map_index, const mapping::GeoPointProto& origin,
    const mapping::GeoPointProto& destination, bool is_bus) {
  auto origin_lane_or =
      GetNearestLanePoint(semantic_map_manager, map_index, origin.longitude(),
                          origin.latitude(), origin.altitude());
  auto destination_or = GetNearestLanePoint(
      semantic_map_manager, map_index, destination.longitude(),
      destination.latitude(), destination.altitude());

  if (!origin_lane_or.ok()) {
    return origin_lane_or.status();
  }
  if (!destination_or.ok()) {
    return destination_or.status();
  }
  VLOG(3) << "origin:" << origin.ShortDebugString() << " -> "
          << origin_lane_or->DebugString()
          << " destination:" << destination.ShortDebugString() << " -> "
          << destination_or->DebugString();
  return SearchLanePathBetweenLanePoints(
      semantic_map_manager, std::move(origin_lane_or).value(),
      {std::move(destination_or).value()}, is_bus);
}

absl::StatusOr<CompositeLanePath> SearchRoutingRequest(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const MapIndex& map_index,
    const RoutingRequestProto& routing_request, const PoseProto& pose,
    bool is_bus, bool use_time) {
  const auto param = planner::CreateDefaultRouteParam();
  RoutingRequestContext context = {
      .semantic_map_manager = &semantic_map_manager,
      .map_index = &map_index,
      .route_param_proto = &param,
  };
  RouteCore route_core;
  route_core.set_semantic_map_manager(&semantic_map_manager);
  route_core.set_route_param_proto(&param);
  RouteCorePrecondition cond = {
      .is_bus = is_bus,
      .use_time = use_time,
  };
  double heading = cc.SmoothYawToGlobal(pose.yaw());
  const Vec2d global_pose =
      cc.SmoothToGlobal({pose.pos_smooth().x(), pose.pos_smooth().y()});
  auto sections_lanes_or = SearchForRouteCorePathFromGlobalPose(
      context, cc, routing_request, global_pose, cond, heading, &route_core);
  if (!sections_lanes_or.ok()) {
    return sections_lanes_or.status();
  }
  return sections_lanes_or->second;
}

absl::StatusOr<std::vector<RouteSections>> GenerateRouteSectionsFromStops(
    const mapping::v2::SemanticMapManager& smm,
    absl::Span<const mapping::LanePoint> destinations,
    absl::Span<const int> stops_index, bool infinite_loop, bool is_bus) {
  std::vector<planner::RouteSections> total_section_seqs;
  for (size_t i = 0; i + 1 < stops_index.size(); ++i) {
    std::vector<mapping::LanePoint> current_destinations;
    for (int j = stops_index[i] + 1; j <= stops_index[i + 1]; ++j) {
      current_destinations.push_back(destinations[j]);
    }
    auto current_section_lanes_or = SearchRouteResultsBetweenLanePoints(
        smm, destinations[stops_index[i]], current_destinations, is_bus);
    if (!current_section_lanes_or.ok()) {
      total_section_seqs.emplace_back();
      continue;
    }
    total_section_seqs.push_back(std::move(current_section_lanes_or)->first);
  }

  if (infinite_loop) {
    std::vector<mapping::LanePoint> current_destinations;
    for (int i = 0; i <= stops_index[0]; ++i) {
      current_destinations.push_back(destinations[i]);
    }
    auto current_section_lanes_or = SearchRouteResultsBetweenLanePoints(
        smm, destinations.back(), current_destinations, is_bus);
    if (!current_section_lanes_or.ok()) {
      total_section_seqs.emplace_back();
    } else {
      total_section_seqs.push_back(std::move(current_section_lanes_or)->first);
    }
  }
  return total_section_seqs;
}

absl::StatusOr<RouteSections> GenerateTotalRouteSections(
    absl::Span<const RouteSections> vec_route_sections) {
  const double start_fraction = vec_route_sections.front().start_fraction();
  const double end_fraction = vec_route_sections.back().end_fraction();
  const auto destination = vec_route_sections.back().destination();
  std::vector<mapping::SectionId> all_section_ids;
  int num_sections = 0;
  for (const auto& route_sections : vec_route_sections) {
    num_sections += route_sections.size();
  }
  all_section_ids.reserve(num_sections);

  for (const auto& route_sections : vec_route_sections) {
    for (const auto& section_id : route_sections.section_ids()) {
      if (!all_section_ids.empty() && all_section_ids.back() == section_id) {
        continue;
      }
      all_section_ids.push_back(section_id);
    }
  }

  return RouteSections(start_fraction, end_fraction, std::move(all_section_ids),
                       destination);
}

absl::StatusOr<std::pair<std::vector<mapping::LanePoint>, std::vector<int>>>
GetDestinations(const mapping::v2::SemanticMapManager& semantic_map_manager,
                const MapIndex* map_index,
                const MultipleStopsRequest& multiple_stops) {
  std::vector<int> stops_index;
  std::vector<mapping::LanePoint> destinations;
  stops_index.reserve(multiple_stops.stop_size());
  stops_index.push_back(0);
  destinations.reserve(multiple_stops.destination_size());
  if (multiple_stops.destination_size() < 2) {
    return absl::InvalidArgumentError(
        "you should input at least 2 destinations.");
  }

  for (int i = 0; i < multiple_stops.destination_size(); ++i) {
    const auto lane_point =
        multiple_stops.ComputeDestinationLanePointByTotalIndex(
            semantic_map_manager, map_index, i);
    if (!lane_point.ok()) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Failed to find corresponding lane point. The error message is: ",
          lane_point.status().message()));
    }
    destinations.push_back(lane_point.value());
    if (multiple_stops.GetStopIndex(i) != stops_index.back()) {
      stops_index.push_back(multiple_stops.GetStopIndex(i));
    }
  }
  return std::make_pair(destinations, stops_index);
}

absl::StatusOr<mapping::LanePoint> GetNearestLanePoint(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const MapIndex* map_index, double x, double y, double z) {
  RoutingDestinationProto destination;
  destination.mutable_global_point()->set_longitude(x);
  destination.mutable_global_point()->set_latitude(y);
  destination.mutable_global_point()->set_altitude(z);
  auto result_or = ParseDestinationProtoToLanePoint(semantic_map_manager,
                                                    map_index, destination);
  if (!result_or.ok()) {
    return absl::NotFoundError(absl::StrFormat(
        "Could not found lane id from destination proto,rad: %.8f,%.8f,%.2f", x,
        y, z));
  }
  return result_or;
}

template <typename T, typename Container>
void ProtoAppend(const Container& c,
                 google::protobuf::RepeatedField<T>* target) {
  target->Reserve(target->size() + std::distance(std::begin(c), std::end(c)));
  for (const auto& e : c) {
    target->Add(e.value());
  }
}

absl::StatusOr<PathStatInfoProto> StatPathInfo(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CompositeLanePath& composite_lane_path) {
  PathStatInfoProto proto;

  std::vector<mapping::ElementId> lane_ids;
  auto last_lane_id = mapping::kInvalidElementId;
  lane_ids.reserve(composite_lane_path.size());

  for (const auto& lane_path : composite_lane_path.lane_paths()) {
    for (const auto& lane_id : lane_path.lane_ids()) {
      if (last_lane_id != lane_id) {
        lane_ids.push_back(lane_id);
      }
      last_lane_id = lane_id;
    }
  }
  if (lane_ids.empty()) {
    return absl::NotFoundError("Lane id is emtpy.");
  }
  if (lane_ids.size() == 1) {
    const auto& lane_id = composite_lane_path.lane_path(0).lane_id(0);
    const auto& start_fraction =
        composite_lane_path.lane_path(0).start_fraction();
    const auto& end_fraction = composite_lane_path.lane_path(0).end_fraction();

    SMM_ASSIGN_LANE_OR_ERROR(lane, semantic_map_manager, lane_id);
    proto.set_length(lane.length() * (end_fraction - start_fraction));
    return proto;
  }
  const double start_fraction = composite_lane_path.front().fraction();
  const double end_fraction = composite_lane_path.back().fraction();

  double sum_len = 0.0;
  int left_turn = 0;
  int right_turn = 0;
  int u_turn = 0;
  int intersection = 0;
  int light = 0;
  double non_motor_len = 0.0;
  std::vector<mapping::ElementId> non_motor_ids;
  non_motor_ids.reserve(4);
  std::vector<mapping::ElementId> left_turn_ids;
  left_turn_ids.reserve(4);
  std::vector<mapping::ElementId> u_turn_ids;
  u_turn_ids.reserve(4);

  for (int i = 0; i < lane_ids.size(); ++i) {
    SMM_ASSIGN_LANE_OR_CONTINUE(lane, semantic_map_manager, lane_ids[i]);
    const auto* lane_proto = &lane.Proto();
    switch (lane_proto->direction()) {
      case mapping::LaneProto::LEFT_TURN:
        ++left_turn;
        left_turn_ids.push_back(mapping::ElementId(lane_proto->id()));
        break;
      case mapping::LaneProto::RIGHT_TURN:
        ++right_turn;
        break;
      case mapping::LaneProto::UTURN:
        ++u_turn;
        u_turn_ids.push_back(mapping::ElementId(lane_proto->id()));
        break;
      default: {
      };
    }

    if (lane_proto->is_in_intersection()) {
      ++intersection;
    }

    if (lane_proto->startpoint_associated_traffic_lights_size() != 0) {
      ++light;
    }

    // LENGTH
    if (i == 0) {
      sum_len += lane.length() * (1.0 - start_fraction);
    } else if (i == lane_ids.size() - 1) {
      sum_len += lane.length() * end_fraction;
    } else {
      sum_len += lane.length();
      if (mapping::compat::IsPassengerVehicleAvoidLaneType(
              lane_proto->type())) {
        non_motor_ids.push_back(mapping::ElementId(lane.Proto().id()));
        non_motor_len += lane.length();
      }
    }
  }
  // Check lane change
  std::vector<double> lange_change_lengths;
  lange_change_lengths.reserve(composite_lane_path.lane_paths().size());
  for (const auto& lane_path : composite_lane_path.lane_paths()) {
    lange_change_lengths.push_back(
        mapping::GetLanePathLength(semantic_map_manager, lane_path));
  }

  // Only check lane change at origin.
  bool is_feasilbe_lane_change_at_origin = true;
  const std::vector<double> allow_lane_change_length = {20.0, 50.0, 80, 120.0,
                                                        160.0};
  double sum_origin_len = 0.0;
  for (int i = 0; i < lange_change_lengths.size(); i++) {
    sum_origin_len += lange_change_lengths[i];
    if (i > 4) {
      break;
    }
    if (sum_origin_len < allow_lane_change_length[i]) {
      LOG(WARNING) << "Cannot lane change in path, sum_origin_len:"
                   << sum_origin_len << ", lane_change_count:" << (i + 1);
      is_feasilbe_lane_change_at_origin = false;
      break;
    }
  }

  proto.set_length(sum_len);
  proto.set_non_motor_len(non_motor_len);
  proto.set_traffic_light_num(light);
  proto.set_left_turn_num(left_turn);
  proto.set_right_turn_num(right_turn);
  proto.set_u_turn_num(u_turn);
  proto.set_intersection_num(intersection);
  proto.set_feasible_lc_at_origin(is_feasilbe_lane_change_at_origin);
  proto.mutable_non_motor_ids()->Reserve(non_motor_ids.size());
  ProtoAppend(non_motor_ids, proto.mutable_non_motor_ids());
  proto.mutable_u_turn_ids()->Reserve(u_turn_ids.size());
  ProtoAppend(u_turn_ids, proto.mutable_u_turn_ids());
  proto.mutable_left_turn_ids()->Reserve(left_turn_ids.size());
  ProtoAppend(left_turn_ids, proto.mutable_u_turn_ids());
  return proto;
}

RouteProto CompositeLanePathToRouteProto(const CompositeLanePath& route_path) {
  RouteProto route;

  for (const auto& seg : route_path.lane_paths()) {
    for (const auto id : seg.lane_ids()) {
      route.mutable_lane_ids()->Add(id.value());
    }
  }
  route.set_start_fraction(route_path.front().fraction());
  route.set_end_fraction(route_path.back().fraction());
  route_path.ToProto(route.mutable_lane_path());
  return route;
}

}  // namespace qcraft::planner::route
