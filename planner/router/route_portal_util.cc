#include "onboard/planner/router/route_portal_util.h"

#include <ostream>
#include <utility>

#include "absl/status/status.h"
#include "common/proto/map_geometry.pb.h"
#include "glog/logging.h"
#include "google/protobuf/stubs/port.h"
#include "google/protobuf/stubs/status.h"
#include "google/protobuf/util/json_util.h"

namespace qcraft::planner {

absl::StatusOr<QCommand::DriveRoute> ConvertToDriverRoute(
    const std::string& pnc_arg) {
  QCommand::DriveRoute drive_route;
  google::protobuf::util::JsonParseOptions parse_options;
  parse_options.ignore_unknown_fields = true;
  const auto status = google::protobuf::util::JsonStringToMessage(
      pnc_arg, &drive_route, parse_options);
  if (!status.ok()) {
    LOG(ERROR) << "Parse Json DriveRoute failed: " << status.ToString();
    return absl::InvalidArgumentError(status.ToString());
  }
  return drive_route;
}

absl::StatusOr<RoutingRequestProto> ConverterToRoutingRequestProto(
    const QCommand::DriveRoute& drive_route) {
  RoutingRequestProto routing_request_proto;
  MultipleStopsRequestProto multi_stops;
  multi_stops.set_infinite_loop(drive_route.infinite_loop());
  multi_stops.set_skip_past_stops(drive_route.skip_past_stops());
  for (const auto& stop : drive_route.stops()) {
    auto* new_stop = multi_stops.add_stops();
    if (stop.has_stop_name()) {
      new_stop->set_stop_name(stop.stop_name());
    }
    if (stop.has_stop_point() && stop.stop_point().has_geo_point()) {
      *new_stop->mutable_stop_point()->mutable_global_point() =
          stop.stop_point().geo_point();
    }
    if (stop.via_points_size() > 0) {
      for (const auto& via_point : stop.via_points()) {
        if (!via_point.has_geo_point()) {
          return absl::InvalidArgumentError(
              "Invalid via point,no field: global_point");
        }
        *new_stop->add_via_points()->mutable_global_point() =
            via_point.geo_point();
      }
    }
  }
  const auto& limited_zone = drive_route.limited_zone();
  for (const auto& lane : limited_zone.lanes()) {
    routing_request_proto.add_avoid_lanes(lane);
  }
  for (const auto& region : limited_zone.regions()) {
    auto* avoid_regions = routing_request_proto.add_avoid_regions();
    avoid_regions->CopyFrom(region);
  }
  *routing_request_proto.mutable_multi_stops() = std::move(multi_stops);
  return routing_request_proto;
}

}  // namespace qcraft::planner
